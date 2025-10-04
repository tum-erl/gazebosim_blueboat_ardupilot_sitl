#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.task import Future

from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from stepper_interfaces.action import SetDepth
from std_srvs.srv import Trigger


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1); dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
    return 2 * R * math.asin(math.sqrt(a))


class GuidedMeasureMissionNode(Node):
    def __init__(self):
        super().__init__('guided_measure_mission')

        # -------- Parameter (kannst du bei Bedarf als ROS-Parameter deklarieren) --------
        self.arrival_radius_m = 3.0           # Ab wann gilt der WP als erreicht
        self.hold_seconds = 10.0               # Halten nach Messung
        self.setpoint_rate_hz = 5.0            # Stream-Frequenz der Setpoints (GUIDED braucht kontinuierliche Ziele)
        self.measure_depth_cm = 200            # Messtiefe für die Winde
        self.waypoints: List[Tuple[float, float, float]] = [
            (48.2525290, 11.6047148, 0.0),
            (48.2526000, 11.6049000, 0.0),
            (48.2527000, 11.6047000, 0.0),
            (48.2526500, 11.6045000, 0.0),
        ]

        # -------- State --------
        self.wp_idx = 0
        self.have_fix = False
        self.curr_lat: Optional[float] = None
        self.curr_lon: Optional[float] = None
        self.curr_alt: float = 0.0
        self.state = 'NAVIGATE'  # NAVIGATE -> LOWERING -> MEASURING -> RAISING -> HOLD -> NAVIGATE/FINISHED
        self.hold_until = None

        # QoS: MAVROS /global_position/global ist i.d.R. BestEffort
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publisher / Subscriber
        self.pub_target = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', qos_reliable)
        self.sub_fix = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, qos_best_effort)

        # Action-/Service-Clients
        self.stepper_client = ActionClient(self, SetDepth, '/stepper/set_depth')
        self.temp_client = self.create_client(Trigger, '/read_temp')

        # Timer für Setpoint-Streaming / Missionslogik
        self.timer = self.create_timer(1.0 / self.setpoint_rate_hz, self.tick)

        self.get_logger().info('GuidedMeasureMissionNode gestartet: fahre WPs an, messe Temperatur, hebe Winde, halte, weiter…')

    # --- Callbacks ---
    def gps_cb(self, msg: NavSatFix):
        self.have_fix = msg.status.status >= 0
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude
        self.curr_alt = msg.altitude

    # --- Helpers ---
    def publish_target(self, lat: float, lon: float, alt: float):
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.latitude = float(lat)
        msg.pose.position.longitude = float(lon)
        msg.pose.position.altitude = float(alt)
        self.pub_target.publish(msg)

    def send_stepper_goal(self, depth_cm: int) -> Future:
        if not self.stepper_client.wait_for_server(timeout_sec=0.0):
            # Nicht blockieren – wir melden und versuchen beim nächsten Tick weiter
            self.get_logger().warn('Stepper ActionServer noch nicht verfügbar.')
            f = Future()
            f.set_exception(RuntimeError('Stepper server unavailable'))
            return f
        goal = SetDepth.Goal()
        goal.target_depth_cm = int(depth_cm)
        return self.stepper_client.send_goal_async(goal)

    def call_temp(self) -> Future:
        if not self.temp_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn('Temperatur-Service /read_temp noch nicht verfügbar.')
            f = Future()
            f.set_exception(RuntimeError('Temp service unavailable'))
            return f
        req = Trigger.Request()
        return self.temp_client.call_async(req)

    # --- Main loop ---
    def tick(self):
        # GPS / fertig?
        if not self.have_fix:
            self.get_logger().warn('Warte auf GPS-Fix…', once=True)
            return
        if self.wp_idx >= len(self.waypoints):
            self.get_logger().info('Mission abgeschlossen.', once=True)
            return

        tgt_lat, tgt_lon, tgt_alt = self.waypoints[self.wp_idx]
        # Streame IMMER den aktuellen Ziel-Setpoint (auch im Hold/Lowering/…)
        self.publish_target(tgt_lat, tgt_lon, tgt_alt)

        # Distanz & Statuslog
        dist = haversine_m(self.curr_lat, self.curr_lon, tgt_lat, tgt_lon)
        self.get_logger().debug(f'WP {self.wp_idx+1}/{len(self.waypoints)} | dist={dist:.1f} m | state={self.state}')

        # State Machine
        if self.state == 'NAVIGATE':
            if dist <= self.arrival_radius_m:
                self.get_logger().info(f'WP {self.wp_idx+1} erreicht (dist={dist:.1f} m). Senke Winde auf {self.measure_depth_cm} cm…')
                self.state = 'LOWERING'
                self.lower_future = self.send_stepper_goal(self.measure_depth_cm)
                self.lower_future.add_done_callback(self._on_lower_sent)
        elif self.state == 'LOWERING':
            # Warten, bis Resultat-Callback state umstellt (siehe _on_lower_result)
            pass
        elif self.state == 'MEASURING':
            # Temperatur messen, dann hoch
            if not hasattr(self, 'measured'):
                self.measured = True
                self.get_logger().info('Starte Temperaturmessung…')
                self.temp_future = self.call_temp()
                self.temp_future.add_done_callback(self._on_temp_done)
        elif self.state == 'RAISING':
            # Warte auf Raise-Result (Callback setzt state)
            pass
        elif self.state == 'HOLD':
            now = self.get_clock().now()
            if now >= self.hold_until:
                # Weiter zum nächsten WP
                self.wp_idx += 1
                if self.wp_idx < len(self.waypoints):
                    self.get_logger().info(f'Weiter zu WP {self.wp_idx+1}/{len(self.waypoints)}')
                    self.state = 'NAVIGATE'
                    if hasattr(self, 'measured'):
                        delattr(self, 'measured')
                else:
                    self.get_logger().info('Alle Wegpunkte abgefahren. Mission fertig.')
        # sonst: DONE / FINISHED – passiert oben

    # --- Action/Service Callbacks ---
    def _on_lower_sent(self, future: Future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Winde: Ziel (senken) nicht akzeptiert.')
                self.state = 'NAVIGATE'  # überspringen, weiterfahren
                return
            self.get_logger().info('Winde: Senken akzeptiert – warte auf Abschluss…')
            self.lower_result_future = goal_handle.get_result_async()
            self.lower_result_future.add_done_callback(self._on_lower_result)
        except Exception as e:
            self.get_logger().error(f'Winde: Fehler beim Senden (senken): {e}')
            self.state = 'NAVIGATE'

    def _on_lower_result(self, future: Future):
        try:
            res = future.result()
            if res.status == 4 and res.result.success:
                self.get_logger().info(f'Winde unten: {res.result.final_depth_cm} cm. → Messen')
                self.state = 'MEASURING'
            else:
                self.get_logger().warn(f'Winde Senken fehlgeschlagen: {res.result.message}')
                self.state = 'NAVIGATE'
        except Exception as e:
            self.get_logger().error(f'Winde: Fehler im Senk-Resultat: {e}')
            self.state = 'NAVIGATE'

    def _on_temp_done(self, future: Future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'Temperatur: {resp.message}')
            else:
                self.get_logger().warn(f'Temperaturmessung fehlgeschlagen: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'Temperatur-Service Fehler: {e}')
        # In jedem Fall → Winde hoch
        self.get_logger().info('Hebe Winde auf 0 cm…')
        self.state = 'RAISING'
        self.raise_future = self.send_stepper_goal(0)
        self.raise_future.add_done_callback(self._on_raise_sent)

    def _on_raise_sent(self, future: Future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Winde: Ziel (heben) nicht akzeptiert.')
                # trotzdem kurz halten und weiter
                self._start_hold()
                return
            self.get_logger().info('Winde: Heben akzeptiert – warte auf Abschluss…')
            self.raise_result_future = goal_handle.get_result_async()
            self.raise_result_future.add_done_callback(self._on_raise_result)
        except Exception as e:
            self.get_logger().error(f'Winde: Fehler beim Senden (heben): {e}')
            self._start_hold()

    def _on_raise_result(self, future: Future):
        try:
            res = future.result()
            if res.status == 4 and res.result.success:
                self.get_logger().info('Winde oben. → Halten')
            else:
                self.get_logger().warn(f'Winde Heben fehlgeschlagen: {res.result.message}')
        except Exception as e:
            self.get_logger().error(f'Winde: Fehler im Hebe-Resultat: {e}')
        self._start_hold()

    def _start_hold(self):
        self.state = 'HOLD'
        self.hold_until = self.get_clock().now() + Duration(seconds=self.hold_seconds)
        self.get_logger().info(f'Halte {self.hold_seconds:.0f}s am WP… (Setpoint-Stream läuft weiter)')

def main(args=None):
    rclpy.init(args=args)
    node = GuidedMeasureMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
