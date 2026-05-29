#!/usr/bin/env python3
# Combined CarMaker Interface Publisher (Endurance: LKA + AEB)
# ==========================================================
# Steering: controlled by ROS2 Stanley controller via VC.Steer.Ang (DVA write)
# Throttle/Brake: 
#   - If AEB is active, AEB fully overrides to brake.
#   - If LKA is active, cruise control maintains SPEED_LIMIT_KMH.

import time
import socket
import re
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import queue

# Import pycarmaker and handle potential import error for testing
try:
    from CarMaker import CarMaker, Quantity
except ImportError:
    from endurance_core.CarMaker import CarMaker, Quantity

# Message imports
from geometry_msgs.msg import Point, Vector3
from endurance.msg import Pose
from endurance.msg import RadarTrack, RadarTrackList
from endurance.msg import Control
from endurance.msg import Velocity
from std_msgs.msg import Float32


def safe_float(q: Quantity):
    """Safely convert CarMaker Quantity to float"""
    return float(q.data) if hasattr(q, 'data') and q.data is not None else 0.0


class CarMakerCompleteInterface(Node):
    MAX_STEER_DEG = 180.0
    SPEED_LIMIT_KMH = 10.0

    def __init__(self):
        super().__init__('carmaker_complete_interface')
        self.get_logger().info("=" * 70)
        self.get_logger().info("🚗 CarMaker Interface (Endurance: LKA + AEB)")
        self.get_logger().info("=" * 70)

        # --- Configuration Parameters ---
        self.CARMAKER_IP = "172.23.128.1"
        self.CARMAKER_PORT = 16660

        self._log_counter = 0

        self.init_carmaker_connection()
        self.init_vehicle_control()
        self.init_publishers()
        
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.init_subscribers()

        # --- Create High-Frequency ROS Timers ---
        self.carmaker_timer = self.create_timer(0.01, self.carmaker_callback, callback_group=self.timer_cb_group)

        self.get_logger().info("✅ CarMaker Complete Interface initialized successfully!")

    def init_carmaker_connection(self):
        """Initialize CarMaker connection and subscribe to all required quantities."""
        try:
            self.cm = CarMaker(self.CARMAKER_IP, self.CARMAKER_PORT)
            self.cm.connect()
            self.get_logger().info(f"🔗 CarMaker connected at {self.CARMAKER_IP}:{self.CARMAKER_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ CarMaker connection failed: {e}")
            raise e

        # Control Quantities
        self.q_steering = Quantity("VC.Steer.Ang", Quantity.FLOAT)
        self.q_vc_gas = Quantity("VC.Gas", Quantity.FLOAT)
        self.q_vc_brake = Quantity("VC.Brake", Quantity.FLOAT)
        self.q_lat_passive = Quantity("Driver.Lat.passive", Quantity.INT)
        self.q_long_passive = Quantity("Driver.Long.passive", Quantity.INT)

        # Telemetry Quantities
        self.inertial_quantities = {
            'position': [Quantity(f"Sensor.Inertial.Vhcl.IN00.Pos_0.{axis}", Quantity.FLOAT) for axis in "xyz"],
            'velocity': [Quantity(f"Sensor.Inertial.Vhcl.IN00.Vel_0.{axis}", Quantity.FLOAT) for axis in "xyz"],
            'orientation': [Quantity(f"Car.{angle}", Quantity.FLOAT) for angle in ["Pitch", "Roll", "Yaw"]],
            'angular_velocity': [Quantity(f"Sensor.Inertial.Vhcl.IN00.Omega_0.{axis}", Quantity.FLOAT) for axis in "xyz"],
            'linear_acceleration': [Quantity(f"Sensor.Inertial.Vhcl.IN00.Acc_0.{axis}", Quantity.FLOAT) for axis in "xyz"]
        }
        for q_group in self.inertial_quantities.values():
            for q in q_group: self.cm.subscribe(q)

        self.radar_quantities = []
        for i in range(32):
            qs = [Quantity(f"Sensor.Radar.Vhcl.RAD00.Obj{i}.{val}", Quantity.FLOAT) for val in ["DistX", "DistY", "VrelX", "VrelY"]]
            for q in qs: self.cm.subscribe(q)
            self.radar_quantities.append(tuple(qs))

        self.velocity = Quantity("Vhcl.v", Quantity.FLOAT)
        self.wheelfl = Quantity("Car.WheelSpd_FL", Quantity.FLOAT)
        self.wheelfr = Quantity("Car.WheelSpd_FR", Quantity.FLOAT)
        self.wheelrl = Quantity("Car.WheelSpd_RL", Quantity.FLOAT)
        self.wheelrr = Quantity("Car.WheelSpd_RR", Quantity.FLOAT)
        
        self.q_sroad = Quantity("Car.Road.sRoad", Quantity.FLOAT)
        self.q_troad = Quantity("Car.Road.tRoad", Quantity.FLOAT)
        
        for q in [self.velocity, self.wheelfl, self.wheelfr, self.wheelrl, self.wheelrr, self.q_sroad, self.q_troad]: self.cm.subscribe(q)

        self.cm.read()  # Initial read to populate values
        self.get_logger().info("📊 All CarMaker quantities subscribed.")

    def init_vehicle_control(self):
        self.steering = 0.0  # normalized [-1, 1] from Stanley controller
        self.latswitch = 0   # 1 = we are steering, 0 = not yet
        self.aeb_brake_cmd = 0.0
        self._speed_override_active = False

    def init_publishers(self):
        self.inertial_pub = self.create_publisher(Pose, 'InertialData', 10)
        self.radar_pub = self.create_publisher(RadarTrackList, 'RadarObjects', 10)
        self.speed_pub = self.create_publisher(Velocity, 'VehicleSpeed', 10)
        
        self.aeb_dist_pub = self.create_publisher(Float32, '/aeb/distance', 10)
        self.aeb_closing_pub = self.create_publisher(Float32, '/aeb/closing_speed', 10)
        self.aeb_ego_pub = self.create_publisher(Float32, '/aeb/ego_speed', 10)
        self.sroad_pub = self.create_publisher(Float32, '/endurance/sRoad', 10)
        self.troad_pub = self.create_publisher(Float32, '/endurance/tRoad', 10)

    def init_subscribers(self):
        self.control_sub = self.create_subscription(
            Control, '/vehicle_control', self.control_callback, 10, callback_group=self.sub_cb_group
        )
        self.aeb_brake_sub = self.create_subscription(
            Float32, '/aeb/brake_cmd', self.aeb_brake_callback, 10, callback_group=self.sub_cb_group
        )

    def control_callback(self, msg: Control):
        self.steering = msg.steering
        self.latswitch = msg.latswitch if msg.latswitch is not None else 0

    def aeb_brake_callback(self, msg: Float32):
        self.aeb_brake_cmd = max(0.0, min(1.0, float(msg.data)))

    def _normalized_to_radians(self, normalized_steer):
        max_steer_rad = self.MAX_STEER_DEG * math.pi / 180.0
        return float(normalized_steer) * max_steer_rad

    def carmaker_callback(self):
        try:
            # --- Read telemetry from CarMaker ---
            self.cm.read()

            current_speed_ms = safe_float(self.velocity)
            current_speed_kmh = current_speed_ms * 3.6

            # --- AEB Radar logic ---
            best_dist = None
            best_vrel = 0.0
            
            for i, qs in enumerate(self.radar_quantities):
                raw_dist = safe_float(qs[0]) # DistX
                dist = abs(raw_dist)
                if dist < 0.1 or dist > 300.0:
                    continue
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_vrel = safe_float(qs[2]) # VrelX
            
            if best_dist is None:
                best_dist = 300.0
                best_vrel = 0.0

            closing_speed = max(0.0, -1.0 * best_vrel)

            # Publish AEB Topics
            self.aeb_dist_pub.publish(Float32(data=float(best_dist)))
            self.aeb_closing_pub.publish(Float32(data=float(closing_speed)))
            self.aeb_ego_pub.publish(Float32(data=float(current_speed_kmh)))
            
            self.sroad_pub.publish(Float32(data=float(safe_float(self.q_sroad))))
            self.troad_pub.publish(Float32(data=float(safe_float(self.q_troad))))

            # --- End AEB Radar logic ---

            aeb_active = self.aeb_brake_cmd >= 0.05
            steer_rad = self._normalized_to_radians(self.steering)

            if aeb_active:
                # 🚨 AEB ACTIVE 🚨
                self.cm.DVA_write(self.q_lat_passive, 1)
                self.cm.DVA_write(self.q_long_passive, 1)
                
                self.cm.DVA_write(self.q_vc_gas, 0.0)
                self.cm.DVA_write(self.q_vc_brake, float(self.aeb_brake_cmd))
                
                # Still steer if LKA is on, else steer straight or IPG default
                if self.latswitch == 1:
                    self.cm.DVA_write(self.q_steering, steer_rad)

                self._speed_override_active = True
                
            elif self.latswitch == 1:
                # 🛣️ LKA ACTIVE 🛣️
                self.cm.DVA_write(self.q_lat_passive, 1)
                self.cm.DVA_write(self.q_long_passive, 1)
                
                target_speed = self.SPEED_LIMIT_KMH
                speed_error = target_speed - current_speed_kmh
                
                if speed_error > 0:
                    gas = min(0.3, speed_error * 0.05)
                    brake = 0.0
                else:
                    gas = 0.0
                    brake = min(0.3, -speed_error * 0.05)
                    
                self.cm.DVA_write(self.q_vc_gas, float(gas))
                self.cm.DVA_write(self.q_vc_brake, float(brake))
                
                self.cm.DVA_write(self.q_steering, steer_rad)
                
                self._speed_override_active = True
                
            else:
                # LKA and AEB are both OFF
                self.cm.DVA_write(self.q_lat_passive, 1) 
                self.cm.DVA_write(self.q_long_passive, 0)
                if self._speed_override_active:
                    self.cm.DVA_write(self.q_vc_gas, 0.0, mode="Off")
                    self.cm.DVA_write(self.q_vc_brake, 0.0, mode="Off")
                    self._speed_override_active = False

            # --- Publish sensor data to ROS ---
            self.publish_inertial_data()
            self.publish_radar_data()
            self.publish_velocity_data()

            self._log_counter += 1
            if self._log_counter % 200 == 0:
                steer_deg = steer_rad * 180.0 / math.pi
                status_str = "AEB" if aeb_active else ("LKA" if self.latswitch == 1 else "OFF")
                self.get_logger().info(
                    f"🎮 DVA: steer={steer_deg:.1f}° | speed={current_speed_kmh:.1f} km/h | Mode: {status_str}"
                )

        except Exception as e:
            self.get_logger().warn(f"⚠️ CarMaker callback error: {str(e)}")
            time.sleep(0.1)
            try:
                if hasattr(self, 'cm'):
                    self.cm.disconnect()
                self.init_carmaker_connection()
            except Exception as reconnect_e:
                self.get_logger().error(f"Reconnect failed: {reconnect_e}")

    def publish_inertial_data(self):
        msg = Pose()
        q = self.inertial_quantities
        msg.position.x, msg.position.y, msg.position.z = safe_float(q['position'][0]), safe_float(q['position'][1]), safe_float(q['position'][2])
        msg.velocity.x, msg.velocity.y, msg.velocity.z = safe_float(q['velocity'][0]), safe_float(q['velocity'][1]), safe_float(q['velocity'][2])
        msg.orientation.x, msg.orientation.y, msg.orientation.z = safe_float(q['orientation'][0]), safe_float(q['orientation'][1]), safe_float(q['orientation'][2])
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = safe_float(q['angular_velocity'][0]), safe_float(q['angular_velocity'][1]), safe_float(q['angular_velocity'][2])
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = safe_float(q['linear_acceleration'][0]), safe_float(q['linear_acceleration'][1]), safe_float(q['linear_acceleration'][2])
        self.inertial_pub.publish(msg)

    def publish_radar_data(self):
        msg = RadarTrackList()
        for i, qs in enumerate(self.radar_quantities):
            track = RadarTrack()
            track.tracking_id, track.x_distance, track.y_distance, track.vx, track.vy = i + 1, safe_float(qs[0]), safe_float(qs[1]), safe_float(qs[2]), safe_float(qs[3])
            msg.objects.append(track)
        self.radar_pub.publish(msg)

    def publish_velocity_data(self):
        msg = Velocity()
        msg.vehicle_velocity = safe_float(self.velocity)
        msg.wheelrpm_fl, msg.wheelrpm_fr, msg.wheelrpm_rl, msg.wheelrpm_rr = safe_float(self.wheelfl), safe_float(self.wheelfr), safe_float(self.wheelrl), safe_float(self.wheelrr)
        self.speed_pub.publish(msg)

    def __del__(self):
        if hasattr(self, 'cm'): self.cm.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CarMakerCompleteInterface()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, Exception) as e:
        print(f"Shutting down due to: {e}")
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        print("✅ CarMaker Interface shutdown successful")

if __name__ == '__main__':
    main()
