#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from blueboat_interfaces.srv import SetTarget
from std_msgs.msg import Bool
import math
import time


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error):
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        self.last_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class BlueboatPIDNode(Node):
    def __init__(self):
        super().__init__('blueboat_pid_regler')

        # ROS Parameter für GPS Modus
        self.declare_parameter('use_gps', False)
        self.use_gps = self.get_parameter('use_gps').value
        
        # Zielposition (XY oder GPS Koordinaten je nach Modus)
        self.goal = [0.0, 0.0]  # initiales Ziel
        # PID-Regler initialisieren
        self.heading_pid = PID(kp=3.0, ki=0.0, kd=0.4)
        self.speed_pid = PID(kp=0.1, ki=0.0, kd=0.8)

        # MAVLink-Verbindung
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()
        mode = "GPS" if self.use_gps else "XY"
        self.get_logger().info(f"Mit MAVLink verbunden - Modus: {mode}")

        # Odometrie abonnieren
        self.create_subscription(Odometry, '/model/blueboat/odometry', self.odom_callback, 10)
        
        self.target_service = self.create_service(SetTarget, 'set_target', self.set_target_callback)
        # Publisher für Ziel-Erreichung
        self.target_reached_pub = self.create_publisher(Bool, '/target_reached', 10)
        self.goal_reached = False  # merken, ob schon gemeldet

    def set_target_callback(self, request, response):
        self.goal = [request.x, request.y]
        if self.use_gps:
            self.get_logger().info(f"Neues GPS Ziel empfangen: lat={request.x}, lon={request.y}")
        else:
            self.get_logger().info(f"Neues XY Ziel empfangen: x={request.x}, y={request.y}")
        self.goal_reached = False  # zurücksetzen!
        response.accepted = True
        time.sleep(5)
        self.get_logger().info("...abgeschlossen.")
        return response

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = self.get_yaw(msg.pose.pose.orientation)

        if self.use_gps:
            # In GPS mode, assume pos.x = latitude, pos.y = longitude
            distance, heading_error = self.compute_control_errors_gps(pos.x, pos.y, yaw)
        else:
            # In XY mode, use standard XY coordinates
            distance, heading_error = self.compute_control_errors(pos.x, pos.y, yaw)
        speed_cmd = self.compute_speed_cmd(distance, heading_error)
        steer_cmd = self.compute_steer_cmd(heading_error)

        pwm_left, pwm_right = self.compute_motor_pwms(speed_cmd, steer_cmd)

        self.send_pwm(pwm_left, pwm_right)
        #Zielerreichung publizieren (nur einaml)
        if distance < 0.3 and not self.goal_reached:
            self.goal_reached = True
            self.target_reached_pub.publish(Bool(data=True))
            self.get_logger().info("Ziel erreicht -> Nachricht gesendet")

        # Ausgabe der aktuellen Werte
        if self.use_gps:
            self.get_logger().info(f"GPS: lat={pos.x:.6f}, lon={pos.y:.6f}, yaw={math.degrees(yaw):.1f}°")
        else:
            self.get_logger().info(f"XY: x={pos.x:.2f}, y={pos.y:.2f}, yaw={math.degrees(yaw):.1f}°")
        self.get_logger().info(f"Distanz: {distance:.2f} m | Heading_Fehler: {math.degrees(heading_error):.1f}°")
        #self.get_logger().info(f"PWM Left: {pwm_left}, PWM Right: {pwm_right}")

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

    def compute_speed_cmd(self, distance, heading_error):
        if distance < 0.1:
            self.speed_pid.integral = 0.0
            return -0.2  # leicht rückwärts zum Bremsen
        if distance > 0.3 and abs(heading_error) > math.radians(90):
            return 0.0  # auf der Stelle drehen
        return max(min(-self.speed_pid.update(distance), 1.0), -0.5)

    def compute_steer_cmd(self, heading_error):
        steer = self.heading_pid.update(heading_error)
        return max(min(steer, 1.0), -1.0)


    def compute_motor_pwms(self, speed_cmd, steer_cmd):
        base_pwm = 1500
        power = int(400 * speed_cmd)
        turn = int(400 * steer_cmd)
        left = max(min(base_pwm - power - turn, 1900), 1100)
        right = max(min(base_pwm - power + turn, 1900), 1100)
        return left, right

    def send_pwm(self, pwm_left, pwm_right):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            pwm_left, 0, pwm_right, 0,
            0,# CH5
            0, # CH6
            0, 0
        )

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


def main(args=None):
    rclpy.init(args=args)
    node = BlueboatPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
