#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

def quaternion_to_yaw(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class PositionHoldController(Node):
    def __init__(self):
        super().__init__('position_hold_controller')
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/model/blueboat/odometry',
            self.odometry_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/blueboat/cmd_vel', 10)

        # PID controller gains
        self.Kp = 0.5
        self.Ki = 0.01
        self.Kd = 0.1

        # Target position (latitude, longitude) - using initial position for now
        self.target_position = None
        self.initial_position_set = False


        # PID controller state
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0

    def odometry_callback(self, data):
        current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        
        if not self.initial_position_set:
            self.target_position = current_position
            self.initial_position_set = True
            self.get_logger().info(f"Initial position set to: {self.target_position}")

        current_yaw = quaternion_to_yaw(data.pose.pose.orientation)
        self.control_robot(current_position, current_yaw)

    def control_robot(self, current_position, current_yaw):
        if self.target_position is None:
            return

        error_x = self.target_position[0] - current_position[0]
        error_y = self.target_position[1] - current_position[1]

        # PID controller calculations
        self.integral_error_x += error_x
        self.integral_error_y += error_y

        derivative_error_x = error_x - self.prev_error_x
        derivative_error_y = error_y - self.prev_error_y

        # Calculate the velocity commands in the world frame
        vx_world = self.Kp * error_x + self.Ki * self.integral_error_x + self.Kd * derivative_error_x
        vy_world = self.Kp * error_y + self.Ki * self.integral_error_y + self.Kd * derivative_error_y

        # Rotate the velocity commands to the robot's frame
        vx_robot = vx_world * math.cos(current_yaw) + vy_world * math.sin(current_yaw)
        vy_robot = -vx_world * math.sin(current_yaw) + vy_world * math.cos(current_yaw)


        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Create and publish the Twist message
        twist_msg = Twist()
        twist_msg.linear.x = vx_robot
        twist_msg.linear.y = vy_robot
        self.cmd_vel_publisher.publish(twist_msg)

        self.get_logger().info(f"Target: {self.target_position}, Current: {current_position}, Error: ({error_x}, {error_y}), Command: ({vx_robot}, {vy_robot})")


def main(args=None):
    rclpy.init(args=args)
    position_hold_controller = PositionHoldController()
    rclpy.spin(position_hold_controller)
    position_hold_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
