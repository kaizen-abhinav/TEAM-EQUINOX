#!/usr/bin/env python3
"""
Performance Logger ROS2 Node

Subscribes to vehicle state, lane detection, AEB, and telemetry topics.
Logs data to CSV and generates an Endurance Performance Dashboard on shutdown.

Subscriptions:
    /VehicleSpeed                    (endurance/Velocity)
    /InertialData                    (endurance/Pose)
    /lka/lane_detection              (endurance/LaneDetection)
    /lka/steering_cmd                (std_msgs/Float32)
    /aeb/distance                    (std_msgs/Float32)
    /endurance/sRoad                 (std_msgs/Float32)
    /endurance/tRoad                 (std_msgs/Float32)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from endurance.msg import Velocity
from endurance.msg import Pose
import numpy as np
import math
import time
import os
from datetime import datetime

# Use non-interactive backend for matplotlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from endurance.msg import LaneDetection


class PerformanceLoggerNode(Node):
    """
    Lightweight performance logger for the Endurance system.

    Logs data to memory and saves final dashboard at session end.
    """

    def __init__(self):
        super().__init__('performance_logger_node')

        # Declare parameters
        self.declare_parameter('update_interval', 5)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('stage1_end_m', 200.0) # End of straight section
        self.declare_parameter('stage2_end_m', 400.0) # End of curved section

        self.update_interval = self.get_parameter('update_interval').get_parameter_value().integer_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.stage1_end = self.get_parameter('stage1_end_m').get_parameter_value().double_value
        self.stage2_end = self.get_parameter('stage2_end_m').get_parameter_value().double_value

        if not output_dir:
            self.output_dir = os.path.join(os.getcwd(), 'endurance_logs')
        else:
            self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # Session data storage
        self.start_time = None
        self.frame_count = 0
        
        self.times = []
        self.speeds = []
        self.steerings = []
        self.troads = []
        self.sroads = []
        self.aeb_distances = []
        self.lane_counts = []

        # Latest values (updated by callbacks)
        self._latest_speed_kmh = 0.0
        self._latest_steering = 0.0
        self._latest_troad = 0.0
        self._latest_sroad = 0.0
        self._latest_aeb_dist = 300.0
        self._latest_lane_count = 0

        # Subscribers
        self.odom_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, qos_profile_sensor_data
        )
        self.lane_sub = self.create_subscription(
            LaneDetection, '/lka/lane_detection', self.lane_callback, 10
        )
        self.steer_sub = self.create_subscription(
            Float32, '/lka/steering_cmd', self.steer_callback, 10
        )
        self.troad_sub = self.create_subscription(
            Float32, '/endurance/tRoad', self.troad_callback, 10
        )
        self.sroad_sub = self.create_subscription(
            Float32, '/endurance/sRoad', self.sroad_callback, 10
        )
        self.aeb_dist_sub = self.create_subscription(
            Float32, '/aeb/distance', self.aeb_dist_callback, 10
        )

        # Timer for data sampling (20 Hz)
        self.sample_timer = self.create_timer(0.05, self.sample_data)

        self.get_logger().info(f'Performance logger ready. Output dir: {self.output_dir}')

    def velocity_callback(self, msg: Velocity):
        self._latest_speed_kmh = 3.6 * msg.vehicle_velocity

    def lane_callback(self, msg: LaneDetection):
        self._latest_lane_count = msg.num_lanes

    def steer_callback(self, msg: Float32):
        self._latest_steering = msg.data

    def troad_callback(self, msg: Float32):
        self._latest_troad = msg.data

    def sroad_callback(self, msg: Float32):
        self._latest_sroad = msg.data

    def aeb_dist_callback(self, msg: Float32):
        self._latest_aeb_dist = msg.data

    def sample_data(self):
        """Sample current state at fixed rate."""
        # Wait for the vehicle to start moving before logging to avoid recording setup times
        if self._latest_speed_kmh < 0.1 and self.frame_count == 0:
            return

        if self.start_time is None:
            self.start_time = time.time()

        self.frame_count += 1
        current_time = time.time() - self.start_time

        self.times.append(current_time)
        self.speeds.append(self._latest_speed_kmh)
        self.steerings.append(self._latest_steering)
        self.troads.append(self._latest_troad)
        self.sroads.append(self._latest_sroad)
        self.aeb_distances.append(self._latest_aeb_dist)
        self.lane_counts.append(self._latest_lane_count)

        # Periodic log
        if self.frame_count % (self.update_interval * 20) == 0:
            self.get_logger().info(
                f'[Logger] t={current_time:.1f}s, speed={self._latest_speed_kmh:.1f} km/h, '
                f'sRoad={self._latest_sroad:.1f}m, tRoad={self._latest_troad:.3f}m, AEB dist={self._latest_aeb_dist:.1f}m'
            )
            try:
                self.finalize()
            except Exception as e:
                pass

    def finalize(self):
        """Generate Endurance Dashboard and save CSV."""
        if len(self.times) < 2:
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        t = np.array(self.times)
        v = np.array(self.speeds)
        troad = np.array(self.troads)
        sroad = np.array(self.sroads)
        aeb_dist = np.array(self.aeb_distances)

        # Identify stages based on sRoad
        stage1_mask = sroad < self.stage1_end
        stage2_mask = (sroad >= self.stage1_end) & (sroad < self.stage2_end)
        stage3_mask = sroad >= self.stage2_end

        # Identify time boundaries for the shaded regions
        t_stage1_end = t[stage1_mask][-1] if np.any(stage1_mask) else 0
        t_stage2_end = t[stage2_mask][-1] if np.any(stage2_mask) else t_stage1_end
        t_end = t[-1]

        # Create dashboard
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle(
            'Abaja 2026 - Endurance Performance Dashboard',
            fontsize=16, fontweight='bold'
        )

        # Plot 1: Speed profile vs. Time for the complete run, annotated by stage
        ax1 = fig.add_subplot(3, 1, 1)
        ax1.plot(t, v, 'b-', linewidth=2, label='Ego Speed (km/h)')
        
        # Shade stages
        if t_stage1_end > 0:
            ax1.axvspan(0, t_stage1_end, color='lightgreen', alpha=0.3, label='Stage 1 (Straight)')
        if t_stage2_end > t_stage1_end:
            ax1.axvspan(t_stage1_end, t_stage2_end, color='lightcoral', alpha=0.3, label='Stage 2 (Curve)')
        if t_end > t_stage2_end:
            ax1.axvspan(t_stage2_end, t_end, color='lightblue', alpha=0.3, label='Stage 3 (Cut-in)')

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Speed (km/h)')
        ax1.set_title('Speed Profile vs. Time (Annotated by Stage)', fontweight='bold')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.4)

        # Plot 2: Lateral deviation (tRoad) vs. Time during the straight and curved sections (Stages 1 & 2)
        ax2 = fig.add_subplot(3, 1, 2)
        
        # Filter data for stages 1 and 2
        stages_1_2_mask = stage1_mask | stage2_mask
        t_1_2 = t[stages_1_2_mask]
        troad_1_2 = troad[stages_1_2_mask]

        if len(t_1_2) > 0:
            ax2.plot(t_1_2, troad_1_2, 'g-', linewidth=2, label='Lateral Deviation (tRoad)')
            if t_stage1_end > 0:
                ax2.axvspan(0, t_stage1_end, color='lightgreen', alpha=0.3)
            if t_stage2_end > t_stage1_end:
                ax2.axvspan(t_stage1_end, t_stage2_end, color='lightcoral', alpha=0.3)
            
            ax2.axhline(0, color='gray', linestyle='--', linewidth=1)
            ax2.axhline(0.9, color='red', linestyle=':', label='+0.9m Limit')
            ax2.axhline(-0.9, color='red', linestyle=':')
            
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('tRoad (m)')
        ax2.set_title('Lateral Deviation (tRoad) vs. Time (Straight & Curved Sections)', fontweight='bold')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.4)
        ax2.set_ylim(-1.5, 1.5)

        # Plot 3: Distance vs. Time to the cut-in target vehicle during Stage 3
        ax3 = fig.add_subplot(3, 1, 3)
        
        t_3 = t[stage3_mask]
        aeb_dist_3 = aeb_dist[stage3_mask]

        if len(t_3) > 0:
            ax3.plot(t_3, aeb_dist_3, 'r-', linewidth=2, label='Distance to Target (m)')
            ax3.axvspan(t_stage2_end, t_end, color='lightblue', alpha=0.3)
            
            # AEB Activation Threshold marker (assumption based on 5-10m typical stop)
            ax3.axhline(5.0, color='orange', linestyle='--', label='Critical Stop Distance (5m)')
            
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Distance (m)')
        ax3.set_title('Distance to Cut-In Target vs. Time (Stage 3)', fontweight='bold')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.4)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        # Save Dashboard
        dashboard_file = os.path.join(self.output_dir, f'endurance_dashboard_{timestamp}.png')
        fig.savefig(dashboard_file, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'Dashboard saved: {dashboard_file}')

        # Save CSV
        csv_file = os.path.join(self.output_dir, f'endurance_data_{timestamp}.csv')
        with open(csv_file, 'w') as f:
            f.write('time_s,speed_kmh,troad_m,sroad_m,aeb_dist_m,steering,lane_count\n')
            for i in range(len(self.times)):
                f.write(
                    f'{self.times[i]:.3f},{self.speeds[i]:.2f},'
                    f'{self.troads[i]:.4f},{self.sroads[i]:.2f},'
                    f'{self.aeb_distances[i]:.2f},{self.steerings[i]:.4f},'
                    f'{self.lane_counts[i]}\n'
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
