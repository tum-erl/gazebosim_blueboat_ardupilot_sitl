#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Entfernung in Metern (great-circle)."""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


class GuidedMissionNode(Node):
    def __init__(self):
        super().__init__('guided_mission_node')

        # --- Parameter ---
        self.arrival_radius_m = 2.0     # Radius für "WP erreicht"
        self.hold_seconds = 20.0        # Wartezeit am WP
        self.setpoint_rate_hz = 5.0     # Hz fürs Setpoint-Streaming

        # Wegpunkte (lat, lon, alt)
        self.waypoints: List[Tuple[float, float, float]] = [
            (48.285083, 11.606444, 0.0),
            (48.284583, 11.605833, 0.0),
            (48.284944, 11.606417, 0.0),
            (48.284864, 11.606720, 0.0),
        ]

        # --- State ---
        self.wp_idx = 0
        self.holding = False
        self.hold_until: Time | None = None
        self.have_fix = False
        self.curr_lat = None
        self.curr_lon = None
        self.curr_alt = 0.0

        # Publisher
        self.pub_target = self.create_publisher(
            GeoPoseStamped,
            '/mavros/setpoint_position/global',
            10
        )

        # Subscriber (QoS Best Effort, weil MAVROS so publisht)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_fix = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            qos
        )

        # Timer für Setpoints
        self.timer = self.create_timer(1.0 / self.setpoint_rate_hz, self.tick)

        self.get_logger().info("GuidedMissionNode gestartet – streame Setpoints im GUIDED Mode")

    def gps_cb(self, msg: NavSatFix):
        self.have_fix = msg.status.status >= 0
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude
        self.curr_alt = msg.altitude

    def tick(self):
        if not self.have_fix:
            self.get_logger().info("Warte auf GPS-Fix…")
            return
        if self.wp_idx >= len(self.waypoints):
            self.get_logger().info("Mission abgeschlossen – keine weiteren Wegpunkte.")
            return

        target_lat, target_lon, target_alt = self.waypoints[self.wp_idx]
        dist = haversine_m(self.curr_lat, self.curr_lon, target_lat, target_lon)

        now = self.get_clock().now()

        if self.holding:
            if now >= self.hold_until:
                self.wp_idx += 1
                self.holding = False
                if self.wp_idx < len(self.waypoints):
                    self.get_logger().info(" Weiter zu WP {self.wp_idx+1}/{len(self.waypoints)}")
                else:
                    self.get_logger().info(" Alle Wegpunkte erreicht.")
            else:
                self.publish_target(target_lat, target_lon, target_alt)
                remaining = (self.hold_until - now).nanoseconds / 1e9
                self.get_logger().info(" Halte an WP {self.wp_idx+1}: noch {remaining:.1f}s")
                return
        else:
            if dist <= self.arrival_radius_m:
                self.holding = True
                self.hold_until = now + Duration(seconds=self.hold_seconds)
                self.get_logger().info(
                    "WP {self.wp_idx+1} erreicht (dist={dist:.1f} m). Halte {self.hold_seconds:.0f}s…"
                )

        # immer Setpoints publizieren
        self.publish_target(target_lat, target_lon, target_alt)

        self.get_logger().info(
            f"WP {self.wp_idx+1}/{len(self.waypoints)} | dist={dist:.1f} m | "
            f"target=({target_lat:.6f},{target_lon:.6f}) | "
            f"gps=({self.curr_lat:.6f},{self.curr_lon:.6f})"
        )

    def publish_target(self, lat: float, lon: float, alt: float):
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.latitude = float(lat)
        msg.pose.position.longitude = float(lon)
        msg.pose.position.altitude = float(alt)
        self.pub_target.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GuidedMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
