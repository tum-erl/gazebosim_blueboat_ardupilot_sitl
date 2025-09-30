#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from blueboat_interfaces.srv import SetTarget
import time
import subprocess


class Planner(Node):
    def __init__(self):
        super().__init__('blueboat_planner')

        # Service-Client für Zielsetzung
        self.client = self.create_client(SetTarget, 'set_target')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Warte auf Regler-Service...')

        # Zielpunkte
        self.targets = [[5.0, 0.0], [5.0, 5.0], [0.0, 5.0], [0.0, 0.0]]
        self.current_target = 0
        self.waiting_for_target = False

        # Subscriber für Zielerreichung
        self.create_subscription(Bool, '/target_reached', self.target_callback, 10)

        # Starte mit dem ersten Ziel
        self.send_next_target()

    def send_next_target(self):
        if self.current_target >= len(self.targets):
            self.get_logger().info("Alle Ziele erreicht.")
            return

        x, y = self.targets[self.current_target]
        req = SetTarget.Request()
        req.x = x
        req.y = y
        self.waiting_for_target = True
        self.get_logger().info("moechte Ziel senden...")
        marker_name = f"marker_{self.current_target}"
        self.spawn_marker(marker_name, x, y)
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_target_response)

    def handle_target_response(self, future):
        try:
            result = future.result()
            if result.accepted:
                self.get_logger().info(f"Ziel {self.current_target + 1} gesendet: ({self.targets[self.current_target][0]}, {self.targets[self.current_target][1]})")
            else:
                self.get_logger().warn("Ziel wurde nicht akzeptiert.")
        except Exception as e:
            self.get_logger().error(f"Service-Aufruf fehlgeschlagen: {e}")


    def target_callback(self, msg):
        self.get_logger().info(f"Callback empfangen: data={msg.data}, waiting={self.waiting_for_target}")
        if msg.data and self.waiting_for_target:
            self.get_logger().info("Ziel erreicht. Führe Aktion aus...")

            self.waiting_for_target = False

            # Aktion nach Zielerreichung (z. B. Pause, Logging, Kamera)
            self.perform_action_at_target()

            self.current_target += 1
            self.send_next_target()

    def perform_action_at_target(self):
        # Simuliere eine Aktion (z. B. Messung, Kamera etc.)
        self.get_logger().info("Warte 5 Sekunden als Aktion...")
        time.sleep(5)
        self.get_logger().info("Aktion abgeschlossen.")

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
    node = Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
