#!/usr/bin/env python3
"""
Stanley Controller ROS2 Node

Standalone node implementing the Stanley lateral controller for lane-following.
Subscribes to lane detections and vehicle speed, publishes steering commands.

This node has NO dependency on CARLA or the detector — it is purely a controller.

Subscriptions:
    /lka/lane_detection              (lka_interfaces/LaneDetection)
    /VehicleSpeed                    (feedback/Velocity)

Publications:
    /lka/steering_cmd                (std_msgs/Float32)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from endurance.msg import Velocity
import numpy as np
import math

from endurance.msg import LaneDetection


class StanleyControllerNode(Node):
    """
    ROS2 node implementing the Stanley lateral controller.

    The Stanley controller computes steering based on:
    1. Cross-track error (lateral offset from lane center)
    2. Speed-dependent gain (steering reduces at higher speeds)

    Formula:
        steering = k * error / (k_soft + speed)
    """

    def __init__(self):
        super().__init__('stanley_controller_node')

        # Declare parameters
        self.declare_parameter('k', 0.8)
        self.declare_parameter('k_soft', 1.0)
        self.declare_parameter('steering_smoothing', 0.3)
        self.declare_parameter('image_width', 960)
        self.declare_parameter('image_height', 540)
        self.declare_parameter('ref_y_ratio', 0.75)
        self.declare_parameter('odometry_topic', '/carla/ego_vehicle/odometry')

        # Read parameters
        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.k_soft = self.get_parameter('k_soft').get_parameter_value().double_value
        self.steering_smoothing = self.get_parameter('steering_smoothing').get_parameter_value().double_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        ref_y_ratio = self.get_parameter('ref_y_ratio').get_parameter_value().double_value
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value

        # Derived
        self.ref_y = int(self.image_height * ref_y_ratio)
        self.last_steer = 0.0
        self.current_speed_ms = 0.0

        # Publisher
        self.steer_pub = self.create_publisher(Float32, '/lka/steering_cmd', 10)

        # Subscribers
        self.lane_sub = self.create_subscription(
            LaneDetection,
            '/lka/lane_detection',
            self.lane_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Velocity,
            odometry_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Stanley controller node ready (k={self.k}, k_soft={self.k_soft}, '
            f'smoothing={self.steering_smoothing}, ref_y={self.ref_y})'
        )

    def odom_callback(self, msg: Velocity):
        """Update current vehicle speed from velocity msg."""
        self.current_speed_ms = msg.vehicle_velocity

    def _unpack_lanes(self, msg: LaneDetection):
        """Unpack the flattened LaneDetection message into list of lane point lists."""
        lanes = []
        offset = 0
        for size in msg.lane_sizes:
            lane = []
            for j in range(size):
                x = msg.lane_points_x[offset + j]
                y = msg.lane_points_y[offset + j]
                lane.append((x, y))
            lanes.append(lane)
            offset += size
        return lanes

    def _get_lane_center(self, lanes):
        """
        Calculate the lane center x-coordinate at the reference y position.

        Args:
            lanes: List of lanes, each lane is a list of (x, y) tuples

        Returns:
            center_x at reference point, or None if insufficient data
        """
        if len(lanes) < 2:
            return None

        left_lane = lanes[0]
        right_lane = lanes[1]

        if len(left_lane) < 2 or len(right_lane) < 2:
            return None

        def find_closest_point(lane, target_y):
            closest = None
            min_dist = float('inf')
            for x, y in lane:
                dist = abs(y - target_y)
                if dist < min_dist:
                    min_dist = dist
                    closest = (x, y)
            return closest

        left_point = find_closest_point(left_lane, self.ref_y)
        right_point = find_closest_point(right_lane, self.ref_y)

        if left_point is None or right_point is None:
            return None

        center_x = (left_point[0] + right_point[0]) / 2.0
        return center_x

    def _get_curvature_feedforward(self, lanes):
        """
        Calculate the look-ahead curvature feedforward from the lane polynomials.
        x = a*y^2 + b*y + c -> 'a' represents curvature.
        """
        if len(lanes) < 2:
            return 0.0
            
        left_lane = lanes[0]
        right_lane = lanes[1]
        
        if len(left_lane) < 3 or len(right_lane) < 3:
            return 0.0
            
        def get_curvature(lane):
            pts = np.array(lane)
            x = pts[:, 0]
            y = pts[:, 1]
            try:
                # Fit x = a*y^2 + b*y + c
                a, b, c = np.polyfit(y, x, 2)
                return a
            except np.linalg.LinAlgError:
                return 0.0
                
        left_a = get_curvature(left_lane)
        right_a = get_curvature(right_lane)
        
        # Average curvature
        avg_curvature = (left_a + right_a) / 2.0
        
        # Tune this feedforward gain. 
        # Disable feedforward by setting it to 0.0 to rely purely on cross-track error for smoother steering.
        k_feedforward = 0.0  
        
        # Clamp the feedforward contribution so it only "assists" rather than completely taking over
        raw_feedforward = avg_curvature * k_feedforward
        clamped_feedforward = float(np.clip(raw_feedforward, -0.3, 0.3))
        
        return clamped_feedforward

    def _calculate_steering(self, lanes):
        """
        Calculate steering using an Advanced Stanley controller.
        Includes cross-track error and curvature feedforward.
        """
        center_x = self._get_lane_center(lanes)

        if center_x is None:
            # No valid lanes detected, gradually return to center
            steer = self.last_steer * 0.95
        else:
            # 1. Cross-Track Error
            image_center_x = self.image_width / 2.0
            error = (center_x - image_center_x) / (self.image_width / 2.0)

            # 2. Dynamic Speed Gain Scheduling
            speed_ms = max(self.current_speed_ms, 0.5)
            # Reduce steering gain automatically at higher speeds
            dynamic_k = self.k * (5.0 / (speed_ms + 5.0))

            # 3. Curvature Feedforward
            feedforward = self._get_curvature_feedforward(lanes)

            # Advanced Stanley: Feedback + Feedforward
            steer = (dynamic_k * error / (self.k_soft + speed_ms * 0.1)) + feedforward

            # IPG CarMaker steering convention: Positive = Left, Negative = Right
            # A positive error means lane center is to the right, requiring a right turn (negative steer).
            steer = -steer

            # Clamp steering
            steer = float(np.clip(steer, -1.0, 1.0))

        # Apply smoothing to prevent oscillation
        steer = self.steering_smoothing * self.last_steer + (1 - self.steering_smoothing) * steer
        self.last_steer = steer

        return steer

    def lane_callback(self, msg: LaneDetection):
        """Process lane detection and publish steering command."""
        # Update image dimensions from message if they differ
        if msg.image_width > 0 and msg.image_height > 0:
            if msg.image_width != self.image_width or msg.image_height != self.image_height:
                self.image_width = msg.image_width
                self.image_height = msg.image_height
                ref_y_ratio = self.get_parameter('ref_y_ratio').get_parameter_value().double_value
                self.ref_y = int(self.image_height * ref_y_ratio)
                self.get_logger().info(
                    f'Updated image dimensions: {self.image_width}x{self.image_height}, ref_y={self.ref_y}'
                )

        # Unpack lanes from message
        lanes = self._unpack_lanes(msg)

        # Calculate steering
        steer = self._calculate_steering(lanes)

        # Publish
        steer_msg = Float32()
        steer_msg.data = steer
        self.steer_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Stanley controller node.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
