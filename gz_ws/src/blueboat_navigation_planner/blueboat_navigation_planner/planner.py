#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from blueboat_interfaces.srv import SetTarget
import time
import subprocess


class NavPlanner(Node):
    def __init__(self):
        super().__init__('blueboat_navigation_planner')

        # Service client for setting targets
        self.client = self.create_client(SetTarget, 'set_target')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller service...')

        # Destinations
        self.targets = [[0.0, 0.0], [4.0, 8.0], [8.0, 0.0]]
        self.current_target = 0
        self.waiting_for_target = False

        # Subscribers for goal achievement
        # TODO: create target_reached publisher in an interface node (was in PID b4)
        self.create_subscription(Bool, '/target_reached', self.target_callback, 10)

        # Start with the first goal
        self.send_next_target()

    def send_next_target(self):
        if self.current_target >= len(self.targets):
            self.get_logger().info("All goals achieved.")
            return

        x, y = self.targets[self.current_target]
        req = SetTarget.Request()  # Service request, uses ROS service client to send target
        req.x = x
        req.y = y
        self.waiting_for_target = True  # Target retrieved, waiting to send
        self.get_logger().info("Trying to send a destination...")
        marker_name = f"marker_{self.current_target}"
        self.spawn_marker(marker_name, x, y)
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_target_response)

    def handle_target_response(self, future):
        print("Entered target response callback")
        try:
            result = future.result()
            if result.accepted:
                self.get_logger().info(f"Target {self.current_target + 1} sent: ({self.targets[self.current_target][0]}, {self.targets[self.current_target][1]})")
            else:
                self.get_logger().warn("Target was not accepted.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        print("Exited target response callback")

    def target_callback(self, msg):
        self.get_logger().info(f"Callback received data={msg.data}, waiting={self.waiting_for_target}")
        if msg.data and self.waiting_for_target:
            self.get_logger().info("Goal reached")

            self.waiting_for_target = False

            # # Action upon goal achievement (z. B. Pause, Logging, Kamera)
            # self.perform_action_at_target()

            self.current_target += 1
            self.send_next_target()

    # def perform_action_at_target(self):
    #     # Simulate an action (z. B. Messung, Kamera etc.)
    #     self.get_logger().info("Warte 5 Sekunden als Aktion...")
    #     time.sleep(5)
    #     self.get_logger().info("Aktion abgeschlossen.")

    # Useful visually for Gazebo, remove when going to hardware
    def spawn_marker(self, name, x, y, z=0.0):
        sdf_path = "/home/blueboat_sitl/gz_ws/src/marker_model/model.sdf"
        cmd = [
            "gz", "service", "-s", "/world/waves/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'sdf_filename: "{sdf_path}", name: "{name}", pose: {{ position: {{ x: {x}, y: {y}, z: {z} }} }}'
        ]
        subprocess.run(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
