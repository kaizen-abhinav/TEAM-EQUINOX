#!/usr/bin/env python3
"""
Lane Detection ROS2 Node

Subscribes to camera images, runs UFLDv2 lane detection,
and publishes detected lane coordinates + annotated visualization.

Subscriptions:
    /carla/ego_vehicle/rgb_front/image  (sensor_msgs/Image)

Publications:
    /lka/lane_detection  (lka_interfaces/LaneDetection)
    /lka/lane_image      (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time

from endurance.msg import LaneDetection
from lka.ufldv2_detector import UFLDv2Detector
from lka.kalman_filter import DualLaneTracker


class LaneDetectionNode(Node):
    """ROS2 node for UFLDv2 lane detection."""

    def __init__(self):
        super().__init__('lane_detection_node')

        # Declare parameters
        self.declare_parameter('model_type', 'culane')
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('camera_topic', '/carla/ego_vehicle/rgb_front/image')

        model_type = self.get_parameter('model_type').get_parameter_value().string_value
        use_fp16 = self.get_parameter('use_fp16').get_parameter_value().bool_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        # Initialize UFLDv2 detector
        self.get_logger().info(f'Loading UFLDv2 model (type={model_type}, fp16={use_fp16})...')
        self.detector = UFLDv2Detector(model_type=model_type, use_fp16=use_fp16)
        self.detector.load_model()
        self.get_logger().info('UFLDv2 model loaded successfully!')

        # Initialize Kalman Filter tracker
        self.lane_tracker = DualLaneTracker()

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Publishers
        self.lane_pub = self.create_publisher(LaneDetection, '/lka/lane_detection', 10)
        self.image_pub = self.create_publisher(Image, '/lka/lane_image', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # Performance tracking
        self.frame_count = 0
        self.fps_smooth = 0.0

        self.get_logger().info(f'Lane detection node ready. Listening on: {camera_topic}')

    def image_callback(self, msg: Image):
        """Process incoming camera image and detect lanes."""
        start_time = time.time()

        # Convert ROS Image to OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = cv_image.shape[:2]

        # Run lane detection
        lane_result = self.detector.detect(cv_image)
        primary_lanes = lane_result['primary']

        # Build LaneDetection message
        lane_msg = LaneDetection()
        lane_msg.header = msg.header
        lane_msg.image_width = width
        lane_msg.image_height = height
        lane_msg.num_lanes = len(primary_lanes)

        all_x = []
        all_y = []
        sizes = []
        for lane in primary_lanes:
            sizes.append(len(lane))
            for (x, y) in lane:
                all_x.append(float(x))
                all_y.append(float(y))

        lane_msg.lane_points_x = all_x
        lane_msg.lane_points_y = all_y
        lane_msg.lane_sizes = sizes

        self.lane_pub.publish(lane_msg)

        # Publish annotated image for visualization
        vis_frame = self.detector.draw_lanes(cv_image, lane_result)

        # Add FPS overlay
        inference_time = time.time() - start_time
        current_fps = 1.0 / inference_time if inference_time > 0 else 0
        self.fps_smooth = 0.9 * self.fps_smooth + 0.1 * current_fps
        self.frame_count += 1

        info_text = f"FPS: {self.fps_smooth:.1f} | Lanes: {len(primary_lanes)} | Inference: {inference_time*1000:.1f}ms"
        import cv2
        cv2.putText(vis_frame, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        vis_msg = self.bridge.cv2_to_imgmsg(vis_frame, encoding='bgr8')
        vis_msg.header = msg.header
        self.image_pub.publish(vis_msg)

        # Log periodically
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: {len(primary_lanes)} lanes, '
                f'{self.fps_smooth:.1f} FPS, {inference_time*1000:.1f}ms inference'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down lane detection node.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
