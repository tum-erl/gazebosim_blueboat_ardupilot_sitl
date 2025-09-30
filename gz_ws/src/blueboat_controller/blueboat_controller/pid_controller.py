#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
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

        # Festes Ziel (XY)
        self.goal = [0.0, 0.0]

        # PID-Regler
        self.heading_pid = PID(kp=3.0, ki=0.0, kd=0.4)
        self.speed_pid = PID(kp=0.1, ki=0.0, kd=0.8)

        # MAVLink-Verbindung
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.master.wait_heartbeat()
        self.get_logger().info("Mit MAVLink verbunden - Modus: XY, Ziel=(0,0)")

        # Odometrie abonnieren
        self.create_subscription(Odometry, '/model/blueboat/odometry', self.odom_callback, 10)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = self.get_yaw(msg.pose.pose.orientation)

        distance, heading_error = self.compute_control_errors(pos.x, pos.y, yaw)

        speed_cmd = self.compute_speed_cmd(distance, heading_error)
        steer_cmd = self.compute_steer_cmd(heading_error)

        pwm_left, pwm_right = self.compute_motor_pwms(speed_cmd, steer_cmd)
        self.send_pwm(pwm_left, pwm_right)

        # Kompaktes Logging
        self.get_logger().info(
            f"XY: x={pos.x:.2f}, y={pos.y:.2f}, yaw={math.degrees(yaw):.1f}° | "
            f"Distanz: {distance:.2f} m | Heading-Error: {math.degrees(heading_error):.1f}° | "
            f"PWM L/R: {pwm_left}/{pwm_right}"
        )

    def compute_control_errors(self, x, y, yaw):
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx) if distance > 1e-6 else yaw
        heading_error = (target_angle - yaw + math.pi) % (2 * math.pi) - math.pi
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
            0, 0, 0, 0
        )

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


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

