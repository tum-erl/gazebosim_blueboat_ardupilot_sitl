#!/usr/bin/env python3
import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Entfernung in Metern (great-circle)."""
    R = 6371000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1); dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
    return 2 * R * math.asin(math.sqrt(a))


class GuidedMissionNode(Node):
    def __init__(self):
        super().__init__('guided_mission_node')

        # --- Parameter (können auch als ROS-Parameter deklariert werden) ---
        self.arrival_radius_m = 3.0            # Ab wann gilt Wegpunkt als erreicht
        self.hold_seconds = 10.0               # Haltezeit am Wegpunkt
        self.setpoint_rate_hz = 5.0            # Wie oft Ziel publishen (MAVROS braucht kontinuierliche Setpoints)

        # Wegpunkte (lat, lon, alt_m)
        self.waypoints: List[Tuple[float, float, float]] = [
            (48.2525290, 11.6047148,  0.0),
            (48.2526000, 11.6049000,  0.0),
            (48.2527000, 11.6047000,  0.0),
            (48.2526500, 11.6045000,  0.0),
        ]

        # --- State ---
        self.wp_idx = 0
        self.holding = False
        self.hold_until: Time | None = None
        self.have_fix = False
        self.curr_lat = None
        self.curr_lon = None
        self.curr_alt = 0.0

        # Publisher / Subscriber
        self.pub_target = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        self.sub_fix = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, 10)

        # Timer zum Setpoint-Stream
        self.timer = self.create_timer(1.0 / self.setpoint_rate_hz, self.tick)

        self.get_logger().info("GuidedMissionNode gestartet. Sende kontinuierlich Global-Setpoints.")

    # --- Callbacks ---
    def gps_cb(self, msg: NavSatFix):
        # Nur sinnvolle Fixes werten (fix_status >= 2: 2D/3D)
        self.have_fix = msg.status.status >= 2
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude
        self.curr_alt = msg.altitude

    def tick(self):
        # Keine Setpoints senden, bevor wir GPS haben oder alle WPs abgearbeitet sind
        if not self.have_fix:
            self.get_logger().throttle_log(5000, self.get_clock(), "Warte auf GPS-Fix…")
            return
        if self.wp_idx >= len(self.waypoints):
            self.get_logger().throttle_log(5000, self.get_clock(), "Mission abgeschlossen. (Keine weiteren Setpoints)")
            return

        target_lat, target_lon, target_alt = self.waypoints[self.wp_idx]

        # Entfernung berechnen
        dist = haversine_m(self.curr_lat, self.curr_lon, target_lat, target_lon)

        # Hold-Logik
        now = self.get_clock().now()
        if self.holding:
            # Während Hold weiter das gleiche Ziel publishen (Position Hold)
            if now >= self.hold_until:
                # Weiter zum nächsten Wegpunkt
                self.wp_idx += 1
                self.holding = False
                if self.wp_idx < len(self.waypoints):
                    self.get_logger().info(f"Weiter zu WP {self.wp_idx+1}/{len(self.waypoints)}")
                else:
                    self.get_logger().info("Alle Wegpunkte abgefahren. Mission fertig.")
            else:
                # Noch halten – einfach weiter publishen
                self.publish_target(target_lat, target_lon, target_alt)
                self.get_logger().throttle_log(
                    1000, self.get_clock(),
                    f"Halte an WP {self.wp_idx+1}: noch {(self.hold_until - now).nanoseconds/1e9:.1f}s"
                )
                return
        else:
            # Noch nicht im Hold: prüfen, ob Ziel erreicht
            if dist <= self.arrival_radius_m:
                self.holding = True
                self.hold_until = now + Duration(seconds=self.hold_seconds)
                self.get_logger().info(
                    f"WP {self.wp_idx+1} erreicht (dist={dist:.1f} m). Halte {self.hold_seconds:.0f}s…"
                )

        # In jedem Fall den aktuellen Ziel-Setpoint streamen
        self.publish_target(target_lat, target_lon, target_alt)

        # Status-Log (gedrosselt)
        self.get_logger().throttle_log(
            1000, self.get_clock(),
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