#!/usr/bin/env python3
"""
Performance Logger ROS2 Node

Subscribes to vehicle state, lane detection, and steering topics.
Logs data to CSV and generates a BAJA SAEINDIA 2026 Article K.2 dashboard on shutdown.

Subscriptions:
    /VehicleSpeed                    (feedback/Velocity)
    /InertialData                    (inertial_msgs/Pose)
    /lka/lane_detection              (lka_interfaces/LaneDetection)
    /lka/steering_cmd                (std_msgs/Float32)
    /lka/tRoad                       (std_msgs/Float32)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from feedback.msg import Velocity
from inertial_msgs.msg import Pose
import numpy as np
import math
import time
import os
from datetime import datetime

# Use non-interactive backend for matplotlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from lka_interfaces.msg import LaneDetection


class PerformanceLoggerNode(Node):
    """
    Lightweight performance logger for the LKA system.

    Logs data to memory and saves final dashboard at session end.
    """

    def __init__(self):
        super().__init__('performance_logger_node')

        # Declare parameters
        self.declare_parameter('update_interval', 5)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('odometry_topic', 'VehicleSpeed')
        self.declare_parameter('imu_topic', 'InertialData')

        self.update_interval = self.get_parameter('update_interval').get_parameter_value().integer_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        if not output_dir:
            self.output_dir = os.path.join(os.getcwd(), 'lka_logs')
        else:
            self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # Session data storage
        self.start_time = None
        self.frame_count = 0
        self.times = []
        self.accel_angles = []
        self.speeds = []
        self.steerings = []
        self.troads = []
        self.lane_counts = []

        # Latest values (updated by callbacks)
        self._latest_speed_kmh = 0.0
        self._latest_accel = None
        self._latest_steering = 0.0
        self._latest_troad = 0.0
        self._latest_lane_count = 0

        # Subscribers — using feedback/Velocity and inertial_msgs/Pose
        # (published by carmakercamera.py)
        self.odom_sub = self.create_subscription(
            Velocity, odometry_topic, self.velocity_callback, qos_profile_sensor_data
        )
        self.imu_sub = self.create_subscription(
            Pose, imu_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.lane_sub = self.create_subscription(
            LaneDetection, '/lka/lane_detection', self.lane_callback, 10
        )
        self.steer_sub = self.create_subscription(
            Float32, '/lka/steering_cmd', self.steer_callback, 10
        )
        self.troad_sub = self.create_subscription(
            Float32, '/lka/tRoad', self.troad_callback, 10
        )

        # Timer for data sampling (20 Hz)
        self.sample_timer = self.create_timer(0.05, self.sample_data)

        self.get_logger().info(f'Performance logger ready. Output dir: {self.output_dir}')

    def velocity_callback(self, msg: Velocity):
        """Update speed from Velocity message (Vhcl.v in m/s)."""
        self._latest_speed_kmh = 3.6 * msg.vehicle_velocity

    def imu_callback(self, msg: Pose):
        """Update acceleration from inertial Pose message."""
        self._latest_accel = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )

    def lane_callback(self, msg: LaneDetection):
        self._latest_lane_count = msg.num_lanes

    def steer_callback(self, msg: Float32):
        self._latest_steering = msg.data

    def troad_callback(self, msg: Float32):
        self._latest_troad = msg.data

    def sample_data(self):
        """Sample current state at fixed rate."""
        if self._latest_speed_kmh == 0.0 and self._latest_lane_count == 0 and self.frame_count == 0:
            # Don't start logging until we have data
            return

        if self.start_time is None:
            self.start_time = time.time()

        self.frame_count += 1
        current_time = time.time() - self.start_time

        # Calculate acceleration angle
        accel_angle = 0.0
        if self._latest_accel is not None:
            ax, ay, _az = self._latest_accel
            accel_angle = math.degrees(math.atan2(ay, ax))

        self.times.append(current_time)
        self.accel_angles.append(accel_angle)
        self.speeds.append(self._latest_speed_kmh)
        self.steerings.append(self._latest_steering)
        self.troads.append(self._latest_troad)
        self.lane_counts.append(self._latest_lane_count)

        # Periodic log
        if self.frame_count % (self.update_interval * 20) == 0:
            self.get_logger().info(
                f'[Logger] t={current_time:.1f}s, speed={self._latest_speed_kmh:.1f} km/h, '
                f'tRoad={self._latest_troad:.3f}m, steer={self._latest_steering:.3f}'
            )
            # Generate dashboard periodically to avoid ROS2 shutdown signal issues
            try:
                self.finalize()
            except Exception as e:
                pass

    def finalize(self):
        """Generate BAJA Article K.2 dashboard and save CSV."""
        MAX_SPEED_KMH = 33.0
        NOMINAL_SPEED_KMH = 30.0
        LANE_WIDTH_M = 3.0
        VEHICLE_WIDTH_M = 1.5

        if len(self.times) < 2:
            self.get_logger().warn('Not enough data to generate graphs.')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        times = np.array(self.times)
        speeds = np.array(self.speeds)
        speeds_ms = speeds / 3.6

        dt = np.diff(times)
        avg_speed = (speeds_ms[1:] + speeds_ms[:-1]) / 2
        distance = np.concatenate([[0], np.cumsum(avg_speed * dt)])

        troad_array = np.array(self.troads)
        max_cte_allowed = 0.9  # e.g., 0.9m deviation allowed

        # Compute Metrics
        rmse = np.sqrt(np.mean(troad_array**2))
        ilc_percentage = (np.abs(troad_array) < max_cte_allowed).sum() / len(troad_array) * 100

        steering_deg = np.array(self.steerings) * 25
        lane_counts = np.array(self.lane_counts)

        # Create dashboard
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle(
            'BAJA SAEINDIA 2026 - LKA Performance Analysis (Article K.2)\nROS2 System Data',
            fontsize=14, fontweight='bold'
        )

        # Plot 1: Lateral Deviation (tRoad) vs Time
        ax1 = fig.add_subplot(2, 2, 1)
        ax1.plot(times, troad_array, 'b-', linewidth=1, alpha=0.8, label='tRoad (Lateral Deviation)')
        ax1.axhline(y=max_cte_allowed, color='r', linestyle='--', linewidth=2,
                     label=f'Lane Limit (+{max_cte_allowed}m)')
        ax1.axhline(y=-max_cte_allowed, color='r', linestyle='--', linewidth=2,
                     label=f'Lane Limit (-{max_cte_allowed}m)')
        ax1.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
        ax1.fill_between(times, -max_cte_allowed, max_cte_allowed, alpha=0.1, color='green')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Lateral Deviation tRoad (m)')
        ax1.set_title('Lateral Deviation (tRoad) vs Time', fontweight='bold')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-1.5, 1.5)

        # Plot 2: Steering vs Distance
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(distance, steering_deg, 'g-', linewidth=1, alpha=0.8)
        ax2.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
        ax2.set_xlabel('Distance (m)')
        ax2.set_ylabel('Steering Angle (deg)')
        ax2.set_title('Steering Angle vs Distance', fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # Plot 3: Speed vs Distance
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(distance, speeds, 'purple', linewidth=1.5, label='Vehicle Speed', alpha=0.8)
        ax3.axhline(y=MAX_SPEED_KMH, color='r', linestyle='--', linewidth=2,
                     label=f'DQ Limit ({MAX_SPEED_KMH:.0f} km/h)')
        ax3.axhline(y=NOMINAL_SPEED_KMH, color='orange', linestyle=':', linewidth=1.5,
                     label=f'Target ({NOMINAL_SPEED_KMH:.0f} ± 3 km/h)')
        ax3.fill_between(distance, 0, MAX_SPEED_KMH, alpha=0.1, color='green')
        ax3.set_xlabel('Distance (m)')
        ax3.set_ylabel('Speed (km/h)')
        ax3.set_title('Speed vs Distance', fontweight='bold')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim(0, 40)

        # Plot 4: Lane Detection State vs Time
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.fill_between(times, 0, lane_counts, step='mid', alpha=0.7, color='green',
                          label='Lanes Detected')
        ax4.axhline(y=2, color='blue', linestyle='--', linewidth=1.5, label='Min Required (2)')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Lane Count')
        ax4.set_title('Lane Detection State vs Time', fontweight='bold')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)
        ax4.set_ylim(0, 4)

        plt.tight_layout(rect=[0, 0.1, 1, 0.95])

        # Summary footer
        total_time = times[-1]
        total_distance = distance[-1]
        max_speed = np.max(speeds)
        detection_rate = (lane_counts >= 2).sum() / len(lane_counts) * 100

        summary = (
            f"Session: {total_time:.1f}s | Distance: {total_distance:.0f}m | "
            f"Avg Speed: {np.mean(speeds):.1f} km/h | Max Speed: {max_speed:.1f} km/h\n"
            f"RMSE: {rmse:.4f}m | ILC: {ilc_percentage:.1f}%"
        )
        fig.text(0.5, 0.02, summary, ha='center', fontsize=10,
                 bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))

        dq_violations = (speeds > MAX_SPEED_KMH).sum()
        if dq_violations > 0:
            result = "DISQUALIFIED - Speed exceeded 33 km/h"
            result_color = 'red'
        elif detection_rate < 95:
            result = f"WARNING - Lane detection rate: {detection_rate:.1f}%"
            result_color = 'orange'
        elif ilc_percentage < 90.0:
            result = f"WARNING - Low ILC percentage: {ilc_percentage:.1f}%"
            result_color = 'orange'
        else:
            result = "PASS - All criteria met"
            result_color = 'green'

        fig.text(0.5, 0.06, f"RESULT: {result}", ha='center', fontsize=12,
                 fontweight='bold', color=result_color)

        dashboard_file = os.path.join(self.output_dir, f'baja_lka_dashboard_{timestamp}.png')
        fig.savefig(dashboard_file, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'Dashboard saved: {dashboard_file}')

        # Save CSV
        csv_file = os.path.join(self.output_dir, f'performance_data_{timestamp}.csv')
        with open(csv_file, 'w') as f:
            f.write('time_s,accel_angle_deg,speed_kmh,steering,troad,lane_count\n')
            for i in range(len(self.times)):
                f.write(
                    f'{self.times[i]:.3f},{self.accel_angles[i]:.2f},{self.speeds[i]:.2f},'
                    f'{self.steerings[i]:.4f},{self.troads[i]:.4f},'
                    f'{self.lane_counts[i]}\n'
                )
        self.get_logger().info(f'CSV saved: {csv_file}')

        # Print summary
        self.get_logger().info(
            f'\n{"="*60}\n'
            f'BAJA SAEINDIA 2026 - LKA PERFORMANCE SUMMARY\n'
            f'{"="*60}\n'
            f'Total Time:          {total_time:.2f} s\n'
            f'Total Distance:      {total_distance:.0f} m\n'
            f'Average Speed:       {np.mean(speeds):.1f} km/h\n'
            f'Maximum Speed:       {max_speed:.1f} km/h (Limit: {MAX_SPEED_KMH:.0f})\n'
            f'Lane Detection Rate: {detection_rate:.1f}%\n'
            f'RMSE (tRoad):        {rmse:.4f} m\n'
            f'ILC Percentage:      {ilc_percentage:.1f}%\n'
            f'RESULT: {result}\n'
            f'{"="*60}'
        )

    def destroy_node(self):
        """Generate dashboard before shutdown."""
        self.finalize()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down performance logger.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
