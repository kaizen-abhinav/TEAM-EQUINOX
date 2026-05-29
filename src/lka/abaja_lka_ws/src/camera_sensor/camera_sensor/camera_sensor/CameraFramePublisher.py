#!/usr/bin/env python3
import socket
import re
import numpy as np
import cv2
import os
import threading
import queue
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image

class RSDSCameraPublisher(Node):
    def __init__(self):
        super().__init__('Camera_Publisher')
        self.get_logger().info("Starting Camera Publisher Node...")

        self.TCP_IP = "172.23.128.1"
        self.TCP_PORT = 2210
        self.expected_header_bytes = 64
        self.bridge = CvBridge()
        
        self.publisher_ = self.create_publisher(Image, 'RGBImage', 10)
        self.get_logger().info("Publishing created for topic RGBImage")

        self.camera_data_queue = queue.Queue(maxsize=2)
        
        self._connect()

        self.reader_thread = threading.Thread(target=self._camera_reader_worker, daemon=True)
        self.reader_thread.start()

        # Publish at 30fps
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def _connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        try:
            self.sock.connect((self.TCP_IP, self.TCP_PORT))
            self.get_logger().info(f"Connected to RSDS TCP/IP server at {self.TCP_IP}:{self.TCP_PORT}")
            
            # Read and discard the initial connection banner (*IPGMovie...)
            banner = self.sock.recv(64)
            self.get_logger().info(f"Received banner: {banner.decode('utf-8', errors='ignore').strip()}")
            self.sock.settimeout(None) # Remove timeout for continuous blocking read
        except Exception as e:
            self.get_logger().error(f"Failed to connect to RSDS server: {e}")

    def _camera_reader_worker(self):
        self.get_logger().info("Camera reader thread started.")
        while rclpy.ok():
            try:
                header_data = b""
                while len(header_data) < self.expected_header_bytes:
                    packet = self.sock.recv(self.expected_header_bytes - len(header_data))
                    if not packet:
                        raise ConnectionError("Socket closed while reading header")
                    header_data += packet
                
                header_str = header_data.decode("utf-8", errors="ignore")
                pattern = r"\*(?:RSDS|CameraRSI)\s+(\d+)\s+(\S+)\s+([\d\.]+)\s+(\d+)x(\d+)\s+(\d+)"
                match = re.search(pattern, header_str)
                if not match:
                    self.get_logger().error(f"Failed to parse CameraRSI/RSDS header:\n{header_str}")
                    continue

                width = int(match.group(4))
                height = int(match.group(5))
                img_len = int(match.group(6))

                expected_len = width * height * 3
                if img_len != expected_len:
                    self.get_logger().warn(f"Image length mismatch: expected {expected_len}, got {img_len}")
                    continue

                raw_data = b""
                while len(raw_data) < img_len:
                    packet = self.sock.recv(img_len - len(raw_data))
                    if not packet:
                        raise ConnectionError("Socket closed while reading image data")
                    raw_data += packet
                
                image_np = np.frombuffer(raw_data, dtype=np.uint8).reshape((height, width, 3))

                if self.camera_data_queue.full():
                    self.camera_data_queue.get_nowait()
                self.camera_data_queue.put(image_np)

            except Exception as e:
                self.get_logger().error(f"Reader worker failed: {e}. Reconnecting in 2s...")
                try:
                    self.sock.close()
                except:
                    pass
                import time
                time.sleep(2)
                self._connect()
                
    def timer_callback(self):
        try:
            image_np = self.camera_data_queue.get_nowait()
            ros_image = self.bridge.cv2_to_imgmsg(image_np, encoding="rgb8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            self.publisher_.publish(ros_image)
        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

    def __del__(self):
        try:
            self.sock.close()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RSDSCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Camera Publisher Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()