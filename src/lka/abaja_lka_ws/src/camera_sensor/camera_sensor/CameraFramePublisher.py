#!/usr/bin/env python3
#import all the necessary libraries
import socket
import re
import numpy as np
import cv2
import os
import time
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image #import message

class RSDSCameraPublisher(Node):
    def __init__(self):
        super().__init__('Camera_Publisher')
        self.get_logger().info("Starting Camera Publisher Node...")

        # Connect to Camera RSDS stream
        self.TCP_IP = "172.23.128.1"
        self.TCP_PORT = 2210
        self.sock = None
        self.connected = False
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2.0  # seconds

        # Connect initially
        self.connect_to_camera()

        # RSDS stream contains two parts - first part is the header and second part is the image frame - detailed explanation is available in the reference manual
        self.expected_header_bytes = 64 # this is the header size - default
        self.bridge = CvBridge()

        # Uncomment the below lines if you want to save the image frames in a folder
        # self.image_dir = "/home/vasanth/Images" # make sure to specify ur folder directory
        # os.makedirs(self.image_dir, exist_ok=True)

        # edit this based on the topic name and message name - Create a publisher (here Image is the message name and RGBImage is the topic)
        self.publisher_ = self.create_publisher(Image, 'RGBImage', 10)
        self.get_logger().info("Publishing created for topic RGBImage")

        # Create a named window for displaying the video stream
        # cv2.namedWindow("RSDS Camera Stream", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("RSDS Camera Stream", 640, 480)
        # self.get_logger().info("Video display window created")

        # running the camera at 30fps - change it as per need
        self.timer = self.create_timer(0.0333, self.timer_callback)

    def connect_to_camera(self):
        """Connect to the camera RSDS stream with timeout and retry logic"""
        # Close existing socket if any
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None

        # Attempt connection with retries
        for attempt in range(self.max_reconnect_attempts):
            try:
                self.get_logger().info(f"Attempting to connect to RSDS server at {self.TCP_IP}:{self.TCP_PORT} (attempt {attempt + 1}/{self.max_reconnect_attempts})")

                # Create socket with timeout
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(10.0)  # 10 second timeout for connection

                self.sock.connect((self.TCP_IP, self.TCP_PORT))

                # Read and handle the initial banner (could be IPGMovie or RSDS)
                banner = self.sock.recv(64)
                banner_str = banner.decode("utf-8", errors="ignore")
                self.get_logger().info(f"Received banner: {banner_str.strip()}")

                # Check if we got the IPGMovie banner - if so, we're connected
                if "*IPGMovie" in banner_str:
                    self.get_logger().info("IPG Movie banner detected - connection established")
                # Otherwise, we might have gotten an RSDS header directly - we'll handle it in timer_callback
                elif "*RSDS" in banner_str or "*CameraRSI" in banner_str:
                    self.get_logger().info("RSDS/CameraRSI banner detected - connection established")
                else:
                    self.get_logger().warn(f"Unknown banner format: {banner_str}")

                # Set timeout for subsequent operations
                self.sock.settimeout(5.0)  # 5 second timeout for read operations

                self.get_logger().info(f"Connected to RSDS TCP/IP server at {self.TCP_IP}:{self.TCP_PORT}")
                self.connected = True
                return True

            except socket.timeout:
                self.get_logger().warn(f"Connection timeout (attempt {attempt + 1}/{self.max_reconnect_attempts})")
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None

            except Exception as e:
                self.get_logger().error(f"Failed to connect to RSDS server: {e}")
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None

            # Wait before retrying (except on last attempt)
            if attempt < self.max_reconnect_attempts - 1:
                time.sleep(self.reconnect_delay)

        self.get_logger().error(f"Failed to connect to RSDS server after {self.max_reconnect_attempts} attempts")
        self.connected = False
        return False

    def timer_callback(self):
        # Check if we are connected, if not try to reconnect
        if not self.connected:
            self.get_logger().warn("Camera not connected, attempting to reconnect...")
            self.connect_to_camera()
            # If still not connected, skip this cycle
            if not self.connected:
                return

        # Check if node is still OK before proceeding
        if not rclpy.ok():
            return

        try:
            # Read the header (64 bytes) for the movie stream
            header_data = b""
            while len(header_data) < self.expected_header_bytes:
                packet = self.sock.recv(self.expected_header_bytes - len(header_data))
                if not packet:
                    self.get_logger().error("Socket closed while reading header.")
                    self.connected = False
                    return
                header_data += packet

            header_str = header_data.decode("utf-8", errors="ignore")
            splitdata = header_str.split(" ")

            # Check if we got the expected format for the movie header
            if len(splitdata) < 6:
                self.get_logger().error(f"Failed to parse movie header: {header_str}")
                self.connected = False
                return

            imgtype = splitdata[2]
            img_size = splitdata[4]
            data_len = int(splitdata[5])

            # Parse image size (format: WIDTHxHEIGHT)
            try:
                width, height = map(int, img_size.split('x'))
            except ValueError:
                self.get_logger().error(f"Invalid image size format: {img_size}")
                self.connected = False
                return

            # Read the image data
            raw_data = b""
            while len(raw_data) < data_len:
                packet = self.sock.recv(data_len - len(raw_data))
                if not packet:
                    self.get_logger().error("Socket closed while reading image data.")
                    self.connected = False
                    return
                raw_data += packet

            # Convert to numpy array based on image type
            if imgtype == "rgb":
                image_np = np.frombuffer(raw_data, dtype=np.uint8).reshape((height, width, 3))
            elif imgtype == "grey":
                image_np = np.frombuffer(raw_data, dtype=np.uint8).reshape((height, width))
            else:
                self.get_logger().error(f"Unsupported image type: {imgtype}")
                self.connected = False
                return

            # Save image to disk - Uncomment the below lines if you want to save the image frames in a folder
            # filename = f"{self.image_dir}/frame_{int(sim_time * 1000)}.png"
            # success = cv2.imwrite(filename, image_np)
            # if success:
            #    self.get_logger().info(f"Image saved to {filename}")
            # else:
            #    self.get_logger().error(f"Failed to save image to {filename}")

            # Display the image
            # Convert from RGB to BGR for OpenCV display
            bgr_image = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
            cv2.imshow("RSDS Camera Stream", bgr_image)

            # Process any OpenCV window events (allows window to refresh and accept key inputs)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.get_logger().info("ESC pressed, shutting down...")
                rclpy.shutdown()

            # Convert and publish
            ros_image = self.bridge.cv2_to_imgmsg(image_np, encoding="rgb8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"

            self.publisher_.publish(ros_image)
            # self.get_logger().info(f"Published frame at time {sim_time}, channel {channel}") # uncomment if you want a feedback when the publisher is running

        except socket.timeout:
            self.get_logger().error("Socket timeout while reading from camera.")
            self.connected = False
        except Exception as e:
            self.get_logger().error(f"Timer callback failed: {e}")
            self.connected = False

    def __del__(self):
        # Clean up OpenCV windows when the node is destroyed
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RSDSCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Camera Publisher Node...")
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()