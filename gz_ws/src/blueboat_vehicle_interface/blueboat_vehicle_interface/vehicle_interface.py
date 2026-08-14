#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from blueboat_interfaces.srv import SetTarget
from std_msgs.msg import Bool
from .my_boat_controller import MyBoatController
from .blueboat_telemetry import BlueboatTelemetryCSV
import sys
sys.path.insert(0, '/home/blueboat_sitl/gz_ws/src/extras_boat')
# from my_boat_controller import MyBoatController
import math
import time


class VehicleInterface(Node):
    def __init__(self):
        super().__init__('blueboat_vehicle_interface')

        # ROS Parameters
        self.declare_parameter('use_gps', False) # False for Gazebo sim, true for real boat
        self.declare_parameter('distance_tolerance', 0.2)  # meters
        self.declare_parameter(
            'mavlink_connection_string', 'udp:127.0.0.1:14550'  # default to Gazebo SITL
        )
        self.use_gps = self.get_parameter('use_gps').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.mavlink_connection_string = self.get_parameter(
            'mavlink_connection_string'
        ).value

        # Target position (XY or GPS coordinates, depending on the mode)
        self.goal = [0.0, 0.0]  # Initial Goal

        # Boat origin in lat/long, change as needed
        # self.initial_lat = 48.284864 # edge of lake
        # self.initial_lon = 11.606720 # edge of lake
        self.initial_lat = 48.214611
        self.initial_lon = 11.720278

        # Current boat position
        self.x = None
        self.y = None
        self.lat = None
        self.lon = None

        # MAVLink Connection and vehicle configuration
        self.vehicle = MyBoatController(self.mavlink_connection_string)
        self.vehicle._auto_disconnect_enabled = False  # Prevent auto-disconnection during operations
        self.vehicle.connect()
        print("Start setting vehicle params")
        self.vehicle.set_boat_param("FRAME_CLASS", 2) # Sets vehicle as boat
        self.vehicle.set_boat_param("PILOT_STEER_TYPE", 1) # Set steering to two paddle input
        self.vehicle.set_boat_param("WP_RADIUS", 0.2)
        # self.vehicle.set_boat_param("GUID_OPTIONS", 64)
        # print("Finish setting vehicle commands, now checking them")
        # frame = self.vehicle.read_boat_param("FRAME_CLASS") # should be 2
        # steering = self.vehicle.read_boat_param("PILOT_STEER_TYPE") # should be 1
        # wp_radius = self.vehicle.read_boat_param("WP_RADIUS")
        # print(f"Boat frame class: {frame}")
        # print(f"Boat steering type: {steering}")
        # print(f"Waypoint radius: {wp_radius}")

        # More params to mimic old PID controller
        # TODO: edit all of these, driving is still v bad
        # self.vehicle.set_boat_param("NAVL1_PERIOD", 0) # lower for aggressive turning, higher for smoother but laggy
        # self.vehicle.set_boat_param("CRUISE_SPEED", 0.7) # boat speed when moving forwards
        # self.vehicle.set_boat_param("CRUISE_THROTTLE", 0.4) # baseline throttle to maintain cruising speed
        # self.vehicle.set_boat_param("ATC_STR_RAT_P", 0.35) # like heading P value (from PID)
        # self.vehicle.set_boat_param("ATC_STR_RAT_I", 0.05) # like heading I value (from PID)
        # self.vehicle.set_boat_param("ATC_STR_RAT_D", 0.05) # like heading D value (from PID)
        # self.vehicle.set_boat_param("ATC_TURN_MAX_G", 45) # higher for sharp turns
        # self.vehicle.set_boat_param("WP_PIVOT_ANGLE", 60) 
        # self.vehicle.set_boat_param("WP_PIVOT_RATE", 60)
        # self.vehicle.set_boat_param("ATC_STR_ANG_P", 2.5)
        # self.vehicle.set_boat_param("ATC_STR_RAT_MAX", 45)
        # self.vehicle.set_boat_param("ATC_STR_ACC_MAX", 30)
        # self.vehicle.set_boat_param("ATC_STR_DEC_MAX", 30)

        self.vehicle.set_guided_mode()
        self.vehicle.arm_vehicle()
        self.vehicle._auto_disconnect_enabled = True
        print("Finished vehicle configuration")

        # Odometry subscription (from Gazebo?)
        self.create_subscription(Odometry, '/model/blueboat/odometry', self.odom_callback, 10)
        
        # Service for receiving target from planner
        self.target_service = self.create_service(
            SetTarget, 'set_target', self.set_target_callback
        )

        # Publisher to indicate when target reached
        self.target_reached_pub = self.create_publisher(Bool, '/target_reached', 10)
        self.goal_reached = False  # check whether it has already been reported

        # Thread and telemetry initialization
        self.logger = BlueboatTelemetryCSV()
        self.logger.start_logger_thread(self.vehicle, self)
        print("Initialization finished")

    def set_target_callback(self, request, response):
        self.goal = [request.x, request.y]
        if self.use_gps:
            self.get_logger().info(f"Received new GPS destination: lat={request.x}, lon={request.y}")
        else:
            self.get_logger().info(f"Received new XY destination: x={request.x}, y={request.y}")
        self.goal_reached = False  # Reset for next target
        response.accepted = True

        lat, long = self.cartesian_to_gps(self.goal[0], self.goal[1])
        # self.vehicle.send_target_position(lat, long)
        self.vehicle.send_waypoint(lat, long)

        time.sleep(5)  # To modify later, simulates data collection
        self.get_logger().info("...completed.")
        return response

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = self.get_yaw(msg.pose.pose.orientation)

        # TODO: edit self.use_gps inconsistencies
        if self.use_gps:
            # In GPS mode, assume pos.x = latitude, pos.y = longitude
            self.lat = pos.x
            self.lon = pos.y
            distance, heading_error = self.compute_control_errors_gps(self.lat, self.lon, yaw)
        else:
            # In XY mode, use standard XY coordinates
            self.x = pos.x
            self.y = pos.y
            distance, heading_error = self.compute_control_errors(self.x, self.y, yaw)
        
        if distance < self.distance_tolerance and not self.goal_reached:
            self.goal_reached = True
            self.target_reached_pub.publish(
                Bool(data=True)
            ) # Publish that target reached
            self.get_logger().info("Goal reached -> Message sent")
        # else:
        #     lat, long = self.cartesian_to_gps(self.goal[0], self.goal[1])
        #     self.vehicle.send_target_position(lat, long)

        # Current output
        if self.use_gps:
            self.get_logger().info(f"GPS: lat={pos.x:.6f}, lon={pos.y:.6f}, yaw={math.degrees(yaw):.1f}°")
        else:
            self.get_logger().info(f"XY: x={pos.x:.2f}, y={pos.y:.2f}, yaw={math.degrees(yaw):.1f}°")
        self.get_logger().info(f"Distance: {distance:.2f} m | Heading_Error: {math.degrees(heading_error):.1f}°")
        #self.get_logger().info(f"PWM Left: {pwm_left}, PWM Right: {pwm_right}")

        # print("----------------")
        # ardu_msg = self.vehicle.conn.recv_match(
        #     type='LOCAL_POSITION_NED',
        #     blocking=False
        # )

        # if ardu_msg is not None:
        #     print(
        #         f"North: {ardu_msg.x:.2f}, "
        #         f"East: {ardu_msg.y:.2f}, "
        #         f"Down: {ardu_msg.z:.2f}"
        #     )

            # print(self.vehicle.read_boat_param("WP_RADIUS"))
            # print(self.vehicle.read_boat_param("NAVL1_PERIOD"))
            # print(self.vehicle.read_boat_param("CRUISE_SPEED"))
            # print(self.vehicle.read_boat_param("CRUISE_THROTTLE"))
            # print(self.vehicle.read_boat_param("ATC_STR_RAT_P"))
            # print(self.vehicle.read_boat_param("ATC_STR_RAT_I"))
            # print(self.vehicle.read_boat_param("ATC_STR_RAT_D"))
            # msg = self.vehicle.conn.recv_match(
            #     type='SERVO_OUTPUT_RAW',
            #     blocking=True,
            #     timeout=1
            # )
            # print(msg)
            # print(self.vehicle.read_boat_param("ATC_TURN_MAX_G"))
            # print(self.vehicle.read_boat_param("GUID_OPTIONS"))
            # print(self.vehicle.read_boat_param("WP_RADIUS"))
            # print(self.vehicle.read_boat_param("NAVL1_PERIOD"))
            # print(self.vehicle.read_boat_param("CRUISE_SPEED"))
            # print(self.vehicle.read_boat_param("CRUISE_THROTTLE"))
            # print(self.vehicle.read_boat_param("ATC_STR_RAT_P"))
            # print(self.vehicle.read_boat_param("ATC_STR_RAT_I"))
            # print(self.vehicle.read_boat_param("ATC_STR_RAT_D"))
            # msg = self.vehicle.conn.recv_match(
            #     type='SERVO_OUTPUT_RAW',
            #     blocking=True,
            #     timeout=1
            # )
            # print(msg)
            # self.vehicle.read_boat_param("ATC_TURN_MAX_G")
            # self.vehicle.read_boat_param("GUID_OPTIONS")
            # print(self.vehicle.read_boat_param("SERVO1_FUNCTION"))
            # print(self.vehicle.read_boat_param("SERVO3_FUNCTION"))
        print("================")

    def compute_control_errors(self, x, y, yaw):
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        heading_error = (target_angle - yaw + math.pi) % (2 * math.pi) - math.pi
        return distance, heading_error

    def compute_control_errors_gps(self, current_lat, current_lon, yaw):
        """Calculate control errors using GPS coordinates (latitude, longitude)"""
        target_lat = self.goal[0]  # latitude
        target_lon = self.goal[1]  # longitude
        
        distance = self.haversine_distance(current_lat, current_lon, target_lat, target_lon)
        target_bearing = self.bearing_to_target(current_lat, current_lon, target_lat, target_lon)
        heading_error = (target_bearing - yaw + math.pi) % (2 * math.pi) - math.pi
        
        return distance, heading_error
    
    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate the great circle distance in meters between two GPS points"""
        R = 6371000  # Earth's radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    def bearing_to_target(self, lat1, lon1, lat2, lon2):
        """Calculate the bearing (angle) from point 1 to point 2 in radians"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.atan2(y, x)
        return bearing
    
    # Do not use, has coordinate mismatch
    def send_cartesian_target_to_ardupilot(self, x, y, z=0.0):
        self.vehicle.conn.mav.set_position_target_local_ned_send(
            0,
            self.vehicle.conn.target_system,
            self.vehicle.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,

            # ignore everything except position
            0b0000111111111000, # type mask
            x, y, z, # position
            0, 0, 0, # velocity
            0, 0, 0, # acceleration
            0, 0 # yaw, yaw_rate
        )

    def get_target_position_cartesian(self):
        return self.goal
    
    def get_target_position_gps(self):
        return self.cartesian_to_gps(*self.goal)
    
    def get_current_position_cartesian(self):
        return (self.x, self.y)
    
    def get_current_position_gps(self):
        return self.cartesian_to_gps(self.x, self.y)

    def cartesian_to_gps(self, x, y):
        lat = self.initial_lat + (y / 111320)  # Latitude degree per meter
        lon = self.initial_lon + (
            x / (40075000 * (1 / 360) * math.cos(
                math.radians(self.initial_lat)
            ))
        )  # Longitude degree per meter
        return (lat, lon)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # pass #TODO: add function to close csv and join thread
        node.logger.stop_telemetry()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()