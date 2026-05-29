#!/usr/bin/env python3
# common imports for all
import time
import rclpy
from rclpy.node import Node
from pycarmaker import CarMaker, Quantity

# geometry_msgs used inside inertial_msgs
from geometry_msgs.msg import Point, Vector3

# inertial sensor message
from inertial_msgs.msg import Pose

# radar sensor message
from radar_msgs.msg import RadarTrack, RadarTrackList

# road sensor message
from road_sensor.msg import RoadGroundTruth

# vehicle controller message
from vehiclecontrol.msg import Control

# vehicle speed message
from feedback.msg import Velocity

def safe_float(q: Quantity):
    try:
        if q.data is None:
            return 0.0
        if isinstance(q.data, str) and "no such element" in q.data:
            return 0.0
        return float(q.data)
    except:
        return 0.0

class CarMakerInterface(Node):
    def __init__(self):
        super().__init__('CarMakerInterface')
        self.get_logger().info("Starting CarMaker Publisher Node...")

        # Connect to CarMaker
        IP_ADDRESS = "172.21.64.1" # change the IP Address as per your PC
        PORT = 16660
        self.cm = CarMaker(IP_ADDRESS, PORT)
        self.cm.connect()
        self.get_logger().info(f"Connected to CarMaker at {IP_ADDRESS}:{PORT}")
        
        # Latest received values (default to zero)
        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.latswitch = 0
        self.longswitch = 0
        
        # Error tracking dictionary - to ensure errors are only displayed once
        self.reported_errors = {}

        # Vehicle Control UAQs - for more details refer reference manual
        self.q_gas = Quantity("VC.Gas", Quantity.FLOAT) # throttle
        self.q_brake = Quantity("VC.Brake", Quantity.FLOAT) # brake
        self.q_steering = Quantity("VC.Steer.Ang", Quantity.FLOAT) # steering angle at steering wheel
        self.q_lat_passive = Quantity("Driver.Lat.passive", Quantity.INT)  
        self.q_long_passive = Quantity("Driver.Long.passive", Quantity.INT)
        
        # Subscribing to Inertial Sensor output UAQs - for more details refer reference manual
        self.inertial_quantities = {
            'position': [
                Quantity("Sensor.Inertial.Vhcl.IN00.Pos_0.x", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Pos_0.y", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Pos_0.z", Quantity.FLOAT)
            ],
            'velocity': [
                Quantity("Sensor.Inertial.Vhcl.IN00.Vel_0.x", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Vel_0.y", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Vel_0.z", Quantity.FLOAT)
            ],
            'orientation': [
                Quantity("Car.Pitch", Quantity.FLOAT),
                Quantity("Car.Roll", Quantity.FLOAT),
                Quantity("Car.Yaw", Quantity.FLOAT)
            ],
            'angular_velocity': [
                Quantity("Sensor.Inertial.Vhcl.IN00.Omega_0.x", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Omega_0.y", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Omega_0.z", Quantity.FLOAT)
            ],
            'linear_acceleration': [
                Quantity("Sensor.Inertial.Vhcl.IN00.Acc_0.x", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Acc_0.y", Quantity.FLOAT),
                Quantity("Sensor.Inertial.Vhcl.IN00.Acc_0.z", Quantity.FLOAT)
            ]
        }

        for q_group in self.inertial_quantities.values():
            for q in q_group:
                self.cm.subscribe(q)

        # Subscribing to RADAR sensor output UAQs - for more details refer reference manual
        self.radar_quantities = []
        for i in range(32):
            prefix = f"Sensor.Radar.Vhcl.RAD00.Obj{i}"
            q_x = Quantity(f"{prefix}.DistX", Quantity.FLOAT)
            q_y = Quantity(f"{prefix}.DistY", Quantity.FLOAT)
            q_vx = Quantity(f"{prefix}.VrelX", Quantity.FLOAT)
            q_vy = Quantity(f"{prefix}.VrelY", Quantity.FLOAT)

            self.cm.subscribe(q_x)
            self.cm.subscribe(q_y)
            self.cm.subscribe(q_vx)
            self.cm.subscribe(q_vy)

            self.radar_quantities.append((q_x, q_y, q_vx, q_vy))
        
        # Subscribing to Road sensor output UAQs - for more details refer reference manual
        self.q_onroad   = Quantity("Sensor.Road.Vhcl.RD00.onRoad", Quantity.INT)
        self.q_tRoad    = Quantity("Vhcl.tRoad", Quantity.FLOAT)
        self.q_sRoad    = Quantity("Vhcl.sRoad", Quantity.FLOAT)
        self.q_rCurve   = Quantity("Sensor.Road.Vhcl.RD00.Route.CurveXY", Quantity.FLOAT)
        self.q_pCurve   = Quantity("Sensor.Road.Vhcl.RD00.Path.CurveXY", Quantity.FLOAT)
        self.q_rDevAng  = Quantity("Sensor.Road.Vhcl.RD00.Route.DevAng", Quantity.FLOAT)
        self.q_pDevAng  = Quantity("Sensor.Road.Vhcl.RD00.Path.DevAng", Quantity.FLOAT)
        self.q_rDevDist  = Quantity("Sensor.Road.Vhcl.RD00.Route.DevDist", Quantity.FLOAT)
        self.q_pDevDist  = Quantity("Sensor.Road.Vhcl.RD00.Path.DevDist", Quantity.FLOAT)
        
        for q in [self.q_onroad, self.q_tRoad, self.q_sRoad]:
            self.cm.subscribe(q)
        
        # Subscription to road curvature quantities - with error checking
        try:
            for q in [self.q_rCurve, self.q_pCurve, self.q_rDevAng, 
                     self.q_pDevAng, self.q_rDevDist, self.q_pDevDist]:
                self.cm.subscribe(q)
        except Exception as e:
            self.log_once(f"Error subscribing to road curvature quantities: {e}")
            self.log_once("Your simulation may not have a defined route or the required quantities.")
        
        # Subscribing to other required UAQs - for more details refer reference manual
        self.velocity = Quantity("Vhcl.v", Quantity.FLOAT)
        self.wheelfl = Quantity("Car.WheelSpd_FL", Quantity.FLOAT)
        self.wheelfr = Quantity("Car.WheelSpd_FR", Quantity.FLOAT)
        self.wheelrl = Quantity("Car.WheelSpd_RL", Quantity.FLOAT)
        self.wheelrr = Quantity("Car.WheelSpd_RR", Quantity.FLOAT)
        for r in [self.velocity,self.wheelfl,self.wheelfr,self.wheelrl,self.wheelrr]:
            self.cm.subscribe(r)
        
        # Subscribing to the vehicle control topic from which the control commands are read
        self.subscription = self.create_subscription(Control,'/vehicle_control',self.control_callback,10)   
        self.get_logger().info("Subscribed to /vehicle_control topic.")
        
        # Perform initial read
        try:
            self.cm.read()
        except Exception as e:
            self.log_once("Initial read warning: " + str(e))
        time.sleep(0.1)

        # Publishers for various topics
        self.inertial_pub = self.create_publisher(Pose, 'InertialData', 10) # update based on your message name and topic (here Pose is the message name and InertialTopic is the topic)
        self.radar_pub = self.create_publisher(RadarTrackList, 'RadarObjects', 10) # update based on your message name and topic (here RadarTrackList is the message name and radarobjects is the topic)
        self.road_pub = self.create_publisher(RoadGroundTruth, 'RoadData', 10) # update based on your message name and topic (here RoadGroundTruth is the message name and RoadData is the topic)
        self.speed_pub = self.create_publisher(Velocity, 'VehicleSpeed', 10)
        self.get_logger().info("Publishers created for '/InertialData' , '/RadarObjects' , '/VehicleSpeed' and '/RoadData'. ")

        # Display initial zero speed
        self.get_logger().info("Vehicle Speed: 0.00 km/h | Wheel Speeds (RPM) - FL: 0.0, FR: 0.0, RL: 0.0, RR: 0.0")

        # Timer for publishing at 100 Hz
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Carmaker Interface Node Initialized - Timer set to 100 Hz.")
    
    def log_once(self, message, level="warn"):
        """Log a message only once to avoid repetition"""
        if message not in self.reported_errors:
            if level == "warn":
                self.get_logger().warn(message)
            elif level == "info":
                self.get_logger().info(message)
            elif level == "error":
                self.get_logger().error(message)
            else:
                self.get_logger().debug(message)
            # Mark this message as already reported
            self.reported_errors[message] = True
        
    def control_callback(self, msg: Control):
        # Assigning the data that we get from vehicle control topic to the relevant variables
        self.steering = msg.steering
        self.throttle = msg.throttle
        self.brake = msg.brake
        self.latswitch = msg.latswitch if msg.latswitch is not None else 0.0
        self.longswitch = msg.longswitch if msg.longswitch is not None else 0.0

    def timer_callback(self):
        try:
            # check if longitudinal switch is enabled in order to start controlling the throttle and brakes - similar to how you will have a switch in the car
            if self.longswitch == 1:
                self.cm.DVA_write(self.q_gas, self.throttle)
                self.cm.DVA_write(self.q_brake, self.brake)
                self.cm.DVA_write(self.q_long_passive, 1)
                # self.get_logger().info(f"Throttle: {self.throttle:.2f}, Brake: {self.brake:.2f}") 
            
            # check if lateral switch is enabled in order to start controlling the steering - similar to how you will have a switch in the car
            if self.latswitch == 1:
                self.cm.DVA_write(self.q_steering, self.steering)
                self.cm.DVA_write(self.q_lat_passive, 1)
                # self.get_logger().info(f"Steering: {self.steering:.2f}")

            self.cm.read()

            # ----------- Inertial Sensor Data Publisher -----------
            # Assigining all the quantities we subscribed from Carmaker to the ROS message
            inertial_msg = Pose()
            inertial_msg.position.x = safe_float(self.inertial_quantities['position'][0])
            inertial_msg.position.y = safe_float(self.inertial_quantities['position'][1])
            inertial_msg.position.z = safe_float(self.inertial_quantities['position'][2])

            inertial_msg.velocity.x = safe_float(self.inertial_quantities['velocity'][0])
            inertial_msg.velocity.y = safe_float(self.inertial_quantities['velocity'][1])
            inertial_msg.velocity.z = safe_float(self.inertial_quantities['velocity'][2])

            inertial_msg.orientation.x = safe_float(self.inertial_quantities['orientation'][0])
            inertial_msg.orientation.y = safe_float(self.inertial_quantities['orientation'][1])
            inertial_msg.orientation.z = safe_float(self.inertial_quantities['orientation'][2])

            inertial_msg.angular_velocity.x = safe_float(self.inertial_quantities['angular_velocity'][0])
            inertial_msg.angular_velocity.y = safe_float(self.inertial_quantities['angular_velocity'][1])
            inertial_msg.angular_velocity.z = safe_float(self.inertial_quantities['angular_velocity'][2])

            inertial_msg.linear_acceleration.x = safe_float(self.inertial_quantities['linear_acceleration'][0])
            inertial_msg.linear_acceleration.y = safe_float(self.inertial_quantities['linear_acceleration'][1])
            inertial_msg.linear_acceleration.z = safe_float(self.inertial_quantities['linear_acceleration'][2])

            # Publishing the ROS message
            self.inertial_pub.publish(inertial_msg)

            # ----------- Radar Sensor Data Publisher -----------
            # Assigining all the quantities we subscribed from Carmaker to the ROS message
            radar_msg = RadarTrackList()
            
            for i, (q_x, q_y, q_vx, q_vy) in enumerate(self.radar_quantities):
                t = RadarTrack()
                t.tracking_id = i + 1
                t.x_distance = safe_float(q_x)
                t.y_distance = safe_float(q_y)
                t.vx = safe_float(q_vx)
                t.vy = safe_float(q_vy)
                radar_msg.objects.append(t)
                
            # Publishing the ROS message
            self.radar_pub.publish(radar_msg)

            # ----------- Road Sensor Data Publisher -----------
            # Assigining all the quantities we subscribed from Carmaker to the ROS message
            road_msg = RoadGroundTruth()
            
            # Basic road information (should always be available)
            road_msg.preview_point = int(self.q_onroad.data) if self.q_onroad.data is not None else 0
            road_msg.lane_deviation = safe_float(self.q_tRoad)
            road_msg.distance_traveled = safe_float(self.q_sRoad)
            
            # Route/path information (might not be available if no route is defined)
            # Only attempt to access route/path parameters with proper error handling
            road_msg.r_curvature = safe_float(self.q_rCurve)
            road_msg.p_curvature = safe_float(self.q_pCurve)
            road_msg.r_deviation_angle = safe_float(self.q_rDevAng)
            road_msg.p_deviation_angle = safe_float(self.q_pDevAng)
            road_msg.r_deviation_dist = safe_float(self.q_rDevDist)
            road_msg.p_deviation_dist = safe_float(self.q_pDevDist)
            
            # Publishing the ROS message
            self.road_pub.publish(road_msg)
            
            # ----------- Velocity Data Publisher -----------
            # Assigining all the quantities we subscribed from Carmaker to the ROS message
            velocity_msg = Velocity()
            
            velocity_msg.vehicle_velocity = safe_float(self.velocity)
            velocity_msg.wheelrpm_fl = safe_float(self.wheelfl)
            velocity_msg.wheelrpm_fr = safe_float(self.wheelfr)
            velocity_msg.wheelrpm_rl = safe_float(self.wheelrl)
            velocity_msg.wheelrpm_rr = safe_float(self.wheelrr)
            
            # Publishing the ROS message
            self.speed_pub.publish(velocity_msg)
            
            # CONTINUOUSLY display vehicle speed and wheel speeds in every callback
            vehicle_speed_kph = safe_float(self.velocity) * 3.6  # Convert m/s to km/h
            fl_wheel = safe_float(self.wheelfl)
            fr_wheel = safe_float(self.wheelfr)
            rl_wheel = safe_float(self.wheelrl)
            rr_wheel = safe_float(self.wheelrr)
            
            self.get_logger().info(f"Vehicle Speed: {vehicle_speed_kph:.2f} km/h | "
                                  f"Wheel Speeds (RPM) - FL: {fl_wheel:.1f}, "
                                  f"FR: {fr_wheel:.1f}, "
                                  f"RL: {rl_wheel:.1f}, "
                                  f"RR: {rr_wheel:.1f}")

        except Exception as e:
            self.log_once(f"Data read/write error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CarMakerInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CarMaker Interface Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()