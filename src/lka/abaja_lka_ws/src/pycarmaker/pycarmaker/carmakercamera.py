# Combined CarMaker Interface Publisher
# ======================================
# Steering: controlled by ROS2 Stanley controller via VC.Steer.Ang (DVA write)
# Throttle/Brake: controlled entirely by IPG Driver (we do NOT override)
# Speed Limit: 30 km/h hard cap enforced by overriding gas/brake when exceeded

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
    try:
        from .CarMaker import CarMaker, Quantity
    except ImportError:
        try:
            from pycarmaker.CarMaker import CarMaker, Quantity
        except ImportError:
            try:
                from pycarmaker.pycarmaker.CarMaker import CarMaker, Quantity
            except ImportError:
                print("CRITICAL: pycarmaker library not found. Cannot connect to CarMaker.")

                # Define dummy classes so the script doesn't crash on syntax, but it won't connect.
                class Quantity:
                    def __init__(self, name, type): self.data = 0.0

                class CarMaker:
                    def __init__(self, ip, port): pass
                    def connect(self): raise ConnectionError("pycarmaker not found")
                    def DVA_write(self, q, val): pass
                    def read(self): pass
                    def subscribe(self, q): pass

# Message imports
from geometry_msgs.msg import Point, Vector3
from inertial_msgs.msg import Pose
from radar_msgs.msg import RadarTrack, RadarTrackList
from vehiclecontrol.msg import Control
from feedback.msg import Velocity


def safe_float(q: Quantity):
    """Safely convert CarMaker Quantity to float"""
    return float(q.data) if hasattr(q, 'data') and q.data is not None else 0.0


class CarMakerCompleteInterface(Node):
    # Maximum steering wheel angle in degrees.
    # 540 degrees is too sensitive for an aggressive controller. 
    # Reducing to 180 degrees mapping makes the normalized [-1, 1] output much smoother.
    MAX_STEER_DEG = 180.0

    # Target speed for autonomous cruise control
    SPEED_LIMIT_KMH = 10.0

    def __init__(self):
        super().__init__('carmaker_complete_interface')
        self.get_logger().info("=" * 70)
        self.get_logger().info("🚗 CarMaker Interface (Stanley Steering Override + IPG Longitudinal)")
        self.get_logger().info("=" * 70)

        # --- Configuration Parameters ---
        self.CARMAKER_IP = "172.23.128.1"
        self.CARMAKER_PORT = 16660

        # Log counter for periodic debug output
        self._log_counter = 0

        # --- Initialization ---
        self.init_carmaker_connection()
        self.init_vehicle_control()
        self.init_publishers()
        
        # We need separate callback groups so the blocking CarMaker timer doesn't starve the ROS subscriber
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.init_subscribers()

        # --- Create High-Frequency ROS Timers ---
        self.carmaker_timer = self.create_timer(0.01, self.carmaker_callback, callback_group=self.timer_cb_group)  # 100Hz for high-rate sensors

        self.get_logger().info("✅ CarMaker Complete Interface initialized successfully!")
        self.get_logger().info(f"   Steering: ROS2 Stanley → VC.Steer.Ang (DVA)")
        self.get_logger().info(f"   Throttle/Brake: IPG Driver (passive=0)")
        self.get_logger().info(f"   Speed Limit: {self.SPEED_LIMIT_KMH} km/h")

    def init_carmaker_connection(self):
        """Initialize CarMaker connection and subscribe to all required quantities."""
        try:
            self.cm = CarMaker(self.CARMAKER_IP, self.CARMAKER_PORT)
            self.cm.connect()
            self.get_logger().info(f"🔗 CarMaker connected at {self.CARMAKER_IP}:{self.CARMAKER_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ CarMaker connection failed: {e}")
            raise e

        # --- Control Quantities ---
        # Steering: external steering command via VC.Steer.Ang (radians)
        self.q_steering = Quantity("VC.Steer.Ang", Quantity.FLOAT)

        # Throttle/Brake: only used for speed-limiting override
        self.q_gas = Quantity("DM.Gas", Quantity.FLOAT)
        self.q_brake = Quantity("DM.Brake", Quantity.FLOAT)

        # Driver passivity flags:
        #   Driver.Lat.passive  = 1 → IPG Driver does NOT steer (we steer via DVA)
        #   Driver.Long.passive = 0 → IPG Driver DOES handle throttle/brake
        self.q_lat_passive = Quantity("Driver.Lat.passive", Quantity.INT)
        self.q_long_passive = Quantity("Driver.Long.passive", Quantity.INT)

        max_steer_rad = self.MAX_STEER_DEG * math.pi / 180.0
        self.get_logger().info(f"🎯 Steering config: MAX_STEER_DEG={self.MAX_STEER_DEG}° → ±{max_steer_rad:.2f} rad")

        # --- Telemetry Quantities ---
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
        for q in [self.velocity, self.wheelfl, self.wheelfr, self.wheelrl, self.wheelrr]: self.cm.subscribe(q)

        self.cm.read()  # Initial read to populate values
        self.get_logger().info("📊 All CarMaker quantities subscribed.")

    def init_vehicle_control(self):
        self.steering = 0.0  # normalized [-1, 1] from Stanley controller
        self.latswitch = 0   # 1 = we are steering, 0 = not yet
        self._speed_override_active = False  # True when we're overriding gas/brake for speed limit

    def init_publishers(self):
        self.inertial_pub = self.create_publisher(Pose, 'InertialData', 10)
        self.radar_pub = self.create_publisher(RadarTrackList, 'RadarObjects', 10)
        self.speed_pub = self.create_publisher(Velocity, 'VehicleSpeed', 10)

    def init_subscribers(self):
        self.control_sub = self.create_subscription(
            Control, '/vehicle_control', self.control_callback, 10, callback_group=self.sub_cb_group
        )

    def control_callback(self, msg: Control):
        """Receive steering command from vehicle_control_node.
        
        We ONLY use the steering value and latswitch.
        Throttle/brake are entirely handled by IPG Driver.
        """
        self.steering = msg.steering
        self.latswitch = msg.latswitch if msg.latswitch is not None else 0

    def _normalized_to_radians(self, normalized_steer):
        """Convert normalized steering [-1, 1] to steering wheel angle in radians.

        CarMaker's VC.Steer.Ang expects the steering angle in radians.
        A typical passenger car has ±540° (±9.42 rad) of steering wheel travel.
        """
        max_steer_rad = self.MAX_STEER_DEG * math.pi / 180.0
        return float(normalized_steer) * max_steer_rad

    def carmaker_callback(self):
        """Main CarMaker data processing loop (100Hz).
        
        Control strategy:
        1. Lateral (Steering): Driver.Lat.passive = 1 always → IPG Driver never steers.
           When latswitch=1, we write our Stanley steering angle via VC.Steer.Ang.
        2. Longitudinal (Throttle/Brake): Driver.Long.passive = 0 always → IPG Driver 
           handles throttle/brake. EXCEPTION: if speed > 30 km/h, we temporarily 
           override to cut gas and apply brake (speed limiter).
        """
        try:
            # --- Step 1: Set Driver Passivity ---
            # IPG Driver is ALWAYS passive on lateral (steering) — we handle it
            self.cm.DVA_write(self.q_lat_passive, 1)
            # IPG Driver is ACTIVE on longitudinal (throttle/brake) — it drives
            # (we only override when speed limit is exceeded)
            current_speed_ms = safe_float(self.velocity)
            current_speed_kmh = current_speed_ms * 3.6

            # --- Step 2 & 3: LKA Speed and Steering Control ---
            if self.latswitch == 1:
                # 1. Graceful Cruise Control (10 km/h)
                target_speed = self.SPEED_LIMIT_KMH
                speed_error = target_speed - current_speed_kmh
                
                if speed_error > 0:
                    # Need to accelerate gently
                    gas = min(0.3, speed_error * 0.05)
                    brake = 0.0
                else:
                    # Need to decelerate gently
                    gas = 0.0
                    brake = min(0.3, -speed_error * 0.05)
                    
                self.cm.DVA_write(self.q_gas, gas)
                self.cm.DVA_write(self.q_brake, brake)
                
                if not self._speed_override_active:
                    self._speed_override_active = True
                    self.get_logger().info(f"🛣️ LKA Connected: Gracefully holding {target_speed} km/h")

                # 2. Lateral (Steering) Control
                steer_rad = self._normalized_to_radians(self.steering)
                self.cm.DVA_write(self.q_steering, steer_rad)
            else:
                # LKA is off, release everything so IPG Driver handles it
                if self._speed_override_active:
                    self.cm.DVA_write(self.q_gas, 0.0, mode="Off")
                    self.cm.DVA_write(self.q_brake, 0.0, mode="Off")
                    self._speed_override_active = False
                    self.get_logger().info(f"✅ LKA Disconnected: IPG Driver resumed control.")

            # --- Step 4: Read telemetry from CarMaker ---
            self.cm.read()

            # --- Step 5: Publish sensor data to ROS ---
            self.publish_inertial_data()
            self.publish_radar_data()
            self.publish_velocity_data()

            # --- Periodic debug logging (every ~2 seconds at 100Hz) ---
            self._log_counter += 1
            if self._log_counter % 200 == 0:
                steer_rad = self._normalized_to_radians(self.steering)
                steer_deg = steer_rad * 180.0 / math.pi
                self.get_logger().info(
                    f"🎮 DVA: steer_norm={self.steering:.3f} → {steer_deg:.1f}° ({steer_rad:.3f} rad) | "
                    f"speed={current_speed_kmh:.1f} km/h | "
                    f"lat={'STANLEY' if self.latswitch else 'NONE'} long={'SPEED_LIM' if self._speed_override_active else 'IPG'}"
                )

        except Exception as e:
            self.get_logger().warn(f"⚠️ CarMaker callback error: {str(e)}")
            time.sleep(0.1)  # Brief pause, not 1s — avoid stalling the control loop
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