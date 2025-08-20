#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from pymavlink import mavutil
import math
import time

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
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

class PIDReglerNode(Node):
    def __init__(self):
        super().__init__('blueboat_pid_regler')

        # Zielkoordinate in Gazebo-Frame
        self.goal = [0.0, 0.0]  # x, y

        # PID für Heading (Lenkung)
        self.pid_heading = PID(kp=5.5, ki=0.0, kd=0.2)
        self.pid_speed = PID(kp=0.3, ki=0.0, kd=1) 
        # MAVLink starten
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info("MAVLink verbunden")

        # Odometry abonnieren
        self.subscription = self.create_subscription(
            Odometry,
            '/model/blueboat/odometry',
            self.odom_callback,
            10
        )
        #Log Datei
        #self.logfile = open("blueboat_log.txt", "w")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

        # Zielrichtung berechnen
        dx = self.goal[0] - pos.x
        dy = self.goal[1] - pos.y
        target_angle = math.atan2(dy, dx)
        distance = math.hypot(dx, dy)

        # Kursfehler (zwischen -pi und pi)
        heading_error = (target_angle - yaw + math.pi) % (2 * math.pi) - math.pi

        # PID-Output -> Lenkung [-1, 1] → PWM 1500 ± 500
        steer_cmd = -self.pid_heading.update(heading_error)
        steer_cmd = max(min(steer_cmd, 1.0), -1.0)
        steer_pwm = int(1500 + 500 * steer_cmd)
        # Distanz -> Geschwindigkeit (Throttle)
        speed_cmd = self.pid_speed.update(distance)
        speed_cmd = max(min(speed_cmd, 1.0), -0.5)
        throttle_pwm = int(1500 + 500 * speed_cmd)
        

        if distance < 0.5:
            throttle_pwm = 1400
            self.get_logger().info("Ziel erreicht → Stop")
            
        self.get_logger().info(f"Distanz: {distance:.2f} m | Throttle_PWM: {throttle_pwm}")
        self.get_logger().info(f"Yaw={math.degrees(yaw):.1f}°, TargetAngle={math.degrees(target_angle):.1f}°, HeadingErr={math.degrees(heading_error):.1f}°")

        
        
        # RC Override senden (CH1=Steering, CH3=Throttle)
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            steer_pwm,     # CH1
            0,             # CH2 unused
            throttle_pwm,  # CH3
            0, 0, 0, 0, 0, 0
        )

        self.get_logger().info(f" Dist={distance:.2f}, HeadingErr={math.degrees(heading_error):.1f}°, PWM={steer_pwm}")
        #self.logfile.write(f"{time.time():.2f}, {distance:.2f}, {math.degrees(heading_error):.2f}, {steer_pwm}, {throttle_pwm}\n")
        #self.logfile.flush()


    def get_yaw_from_quat(self, q):
        # Quaternion → Yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = PIDReglerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #node.logfile.close()  # Log-Datei schließen
        node.destroy_node()
        rclpy.shutdown()