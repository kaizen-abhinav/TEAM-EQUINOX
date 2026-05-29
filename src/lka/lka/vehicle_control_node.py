#!/usr/bin/env python3
"""
Vehicle Control ROS2 Node — Steering-Only Relay

Relays Stanley controller steering commands to the CarMaker bridge.
Throttle and brake are handled entirely by IPG Driver.

Subscriptions:
    /lka/steering_cmd                           (std_msgs/Float32)

Publications:
    /vehicle_control                            (vehiclecontrol/Control)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from endurance.msg import Control


class VehicleControlNode(Node):
    """
    ROS2 node that relays Stanley steering to the CarMaker bridge.

    This node is intentionally simple:
    - Receives steering from Stanley controller (/lka/steering_cmd)
    - Publishes to /vehicle_control with latswitch=1 (steering override ON)
    - Sets longswitch=0 (throttle/brake stay with IPG Driver)
    - No keyboard teleop, no throttle/brake logic
    """

    def __init__(self):
        super().__init__('vehicle_control_node')

        # Declare parameters
        self.declare_parameter('control_topic', '/vehicle_control')
        control_topic = self.get_parameter('control_topic').get_parameter_value().string_value

        # Current steering value from Stanley
        self.steer = 0.0

        # Publisher
        self.control_pub = self.create_publisher(Control, control_topic, 10)

        # Subscriber: steering command from Stanley controller
        self.steer_sub = self.create_subscription(
            Float32, '/lka/steering_cmd', self.steer_callback, 10
        )

        # Timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Vehicle control node ready (steering-only relay).\n'
            f'  Steering: /lka/steering_cmd → {control_topic}\n'
            f'  Throttle/Brake: IPG Driver (not touched)'
        )

    def steer_callback(self, msg: Float32):
        """Receive steering command from Stanley controller."""
        self.steer = msg.data

    def control_loop(self):
        """Publish steering command to CarMaker bridge at 20 Hz."""
        control_msg = Control()
        control_msg.steering = float(self.steer)
        control_msg.throttle = 0.0   # Not used — IPG Driver handles throttle
        control_msg.brake = 0.0      # Not used — IPG Driver handles brake
        control_msg.latswitch = 1    # Lateral steering override: ON (Stanley controls)
        control_msg.longswitch = 0   # Longitudinal override: OFF (IPG Driver controls)

        self.control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info('Shutting down vehicle control node.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
