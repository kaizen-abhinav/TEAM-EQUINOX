#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from endurance.msg import Velocity, Pose
import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import numpy as np
import os
import time
from datetime import datetime

class AEBPlotter(Node):
    def __init__(self):
        super().__init__('aeb_plotter')
        
        # Data storage
        self.times = []
        self.x = []
        self.y = []
        self.v = []
        self.dist = []
        self.brake = []
        
        self.start_time = None
        self.scenario_active = False
        
        # Subscriptions
        self.create_subscription(Pose, '/InertialData', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(Velocity, '/VehicleSpeed', self.vel_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, '/aeb/distance', self.dist_callback, 10)
        self.create_subscription(Float32, '/aeb/brake_cmd', self.brake_callback, 10)
        
        # Latest values
        self._curr_x = 0.0
        self._curr_y = 0.0
        self._curr_v = 0.0
        self._curr_dist = 300.0
        self._curr_brake = 0.0
        
        # Timer for sampling (20Hz)
        self.timer = self.create_timer(0.05, self.sample_data)
        
        self.output_dir = '../../results/AEB'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('AEB Plotter started. Waiting for data...')

    def pose_callback(self, msg):
        self._curr_x = msg.position.x
        self._curr_y = msg.position.y

    def vel_callback(self, msg):
        self._curr_v = msg.vehicle_velocity * 3.6 # km/h

    def dist_callback(self, msg):
        self._curr_dist = msg.data

    def brake_callback(self, msg):
        self._curr_brake = msg.data

    def sample_data(self):
        # Detect scenario start
        if not self.scenario_active:
            if self._curr_v > 0.1 or self._curr_dist < 290.0:
                self.scenario_active = True
                self.start_time = time.time()
                self.get_logger().info('Scenario STARTED. Logging data...')
                # Reset buffers
                self.times.clear()
                self.x.clear()
                self.y.clear()
                self.v.clear()
                self.dist.clear()
                self.brake.clear()
            else:
                return
        
        # Detect scenario end (target disappeared and we are slow/stopped, or time reset)
        if self.scenario_active:
            if self._curr_dist > 290.0 and self._curr_v < 0.1 and len(self.times) > 50:
                self.get_logger().info('Scenario ENDED. Saving plots...')
                self.save_plots()
                self.scenario_active = False
                return

        t = time.time() - self.start_time
        self.times.append(t)
        self.x.append(self._curr_x)
        self.y.append(self._curr_y)
        self.v.append(self._curr_v)
        self.dist.append(self._curr_dist)
        self.brake.append(self._curr_brake)
        
        if len(self.times) % 100 == 0:
            self.get_logger().info(f'Logged {t:.1f}s of data...')

    def save_plots(self):
        if not self.times:
            self.get_logger().warn('No data collected. Not saving plots.')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Convert to numpy arrays
        t = np.array(self.times)
        x = np.array(self.x)
        y = np.array(self.y)
        v = np.array(self.v)
        dist = np.array(self.dist)
        brake = np.array(self.brake)
        
        # 1. BEV Trajectory Plot
        fig1, ax1 = plt.subplots(figsize=(10, 8))
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = mcoll.LineCollection(segments, cmap='jet', norm=plt.Normalize(v.min(), v.max()))
        lc.set_array(v)
        lc.set_linewidth(2)
        line = ax1.add_collection(lc)
        fig1.colorbar(line, label='Velocity (km/h)')
        ax1.set_xlim(x.min() - 5, x.max() + 5)
        ax1.set_ylim(y.min() - 5, y.max() + 5)
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Bird\'s Eye View (BEV) Trajectory')
        ax1.grid(True)
        bev_path = os.path.join(self.output_dir, f'aeb_bev_{timestamp}.png')
        fig1.savefig(bev_path)
        plt.close(fig1)

        # 2. Distance vs. Time Plot
        fig2, ax2 = plt.subplots(figsize=(10, 6))
        ax2.plot(t, dist, 'r-', label='Distance to Target')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Distance (m)')
        ax2.set_title('Distance to Cut-in Target vs. Time')
        ax2.legend()
        ax2.grid(True)
        dist_path = os.path.join(self.output_dir, f'aeb_distance_{timestamp}.png')
        fig2.savefig(dist_path)
        plt.close(fig2)

        # 3. Brake Force / Deceleration Profile
        # Deceleration (m/s^2)
        v_ms = v / 3.6
        accel = np.diff(v_ms) / np.diff(t)
        decel = -accel
        decel = np.insert(decel, 0, 0.0) # Match length
        
        fig3, ax3_1 = plt.subplots(figsize=(10, 6))
        ax3_1.plot(t, brake, 'b-', label='Brake Force (normalized)')
        ax3_1.set_xlabel('Time (s)')
        ax3_1.set_ylabel('Brake Force', color='b')
        ax3_1.tick_params(axis='y', labelcolor='b')
        
        ax3_2 = ax3_1.twinx()
        ax3_2.plot(t, decel, 'g-', label='Deceleration (m/s²)')
        ax3_2.set_ylabel('Deceleration (m/s²)', color='g')
        ax3_2.tick_params(axis='y', labelcolor='g')
        
        plt.title('Brake Force and Deceleration Profile')
        fig3.tight_layout()
        brake_path = os.path.join(self.output_dir, f'aeb_brake_profile_{timestamp}.png')
        fig3.savefig(brake_path)
        plt.close(fig3)

        self.get_logger().info(f'Plots saved to {self.output_dir}')

def main(args=None):
    rclpy.init(args=args)
    plotter = AEBPlotter()
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.get_logger().info('KeyboardInterrupt, saving plots...')
    finally:
        plotter.save_plots()
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
