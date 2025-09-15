#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from mavros_msgs.msg import GPSRAW, OverrideRCIn
from std_msgs.msg import Bool


def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_deg(a):
    """wrap angle to [-180, 180)"""
    a = (a + 180.0) % 360.0 - 180.0
    return a

def haversine_m(lat1, lon1, lat2, lon2):
    """Entfernung in Metern"""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2):
    """Peilung (0..360) von Punkt1 nach Punkt2 in Grad"""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlmb)
    brg = math.degrees(math.atan2(y, x))
    if brg < 0:
        brg += 360.0
    return brg


class PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_t = None

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_t = None

    def update(self, error):
        now = time.time()
        dt = 0.1 if self.last_t is None else max(1e-3, now - self.last_t)
        self.last_t = now

        self.integral += error * dt
        deriv = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return clamp(out, self.out_min, self.out_max)


class ASVPidRcNode(Node):
    def __init__(self):
        super().__init__('asv_pid_rc_node')

        # --- Parameter ---
        self.declare_parameter('arrival_radius_m', 1.5)           # Ziel erreicht unterhalb dieses Radius
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('steer_span', 400)                  # ± um Neutral für CH1 (Lenkung)
        self.declare_parameter('throttle_span_fwd', 350)           # + um Neutral für Vorwärts
        self.declare_parameter('throttle_span_rev', 300)           # - um Neutral für Rückwärts
        self.declare_parameter('invert_steer', False)              # ggf. true, wenn Lenkung invertiert ist
        self.declare_parameter('invert_throttle', False)           # ggf. true, wenn Gas invertiert ist
        self.declare_parameter('max_throttle_when_large_heading_deg', 70.0)  # bei großem Headingfehler Gas reduzieren
        self.declare_parameter('guided_only', False)               # wenn true, sendet nur wenn mode GUIDED (optional, ausbaufähig)

        # PID auf Heading (Fehler in Grad)
        self.declare_parameter('heading_kp', 0.03)   # 0.03 -> 30° Fehler ~ 0.9 Steer
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 0.01)

        # Schlichtes Speed-“PID”: hier proportional auf Distanz (m)
        self.declare_parameter('speed_kp', 0.15)     # 5 m -> 0.75 “Throttle-Norm”
        self.declare_parameter('speed_ki', 0.0)
        self.declare_parameter('speed_kd', 0.0)

        # Ziel (lat/lon) initial optional als Parameter
        self.declare_parameter('goal_lat', 0.0)
        self.declare_parameter('goal_lon', 0.0)
        self.have_goal = False
        self.goal_lat = float(self.get_parameter('goal_lat').value)
        self.goal_lon = float(self.get_parameter('goal_lon').value)
        if abs(self.goal_lat) > 1e-9 or abs(self.goal_lon) > 1e-9:
            self.have_goal = True

        # Aktuelle Navigation
        self.last_lat = None
        self.last_lon = None
        self.last_cog_deg = None  # aus GPS (centideg)
        self.target_reached = False
        # Stop
        self.stopped = False

        # PID-Instanzen
        self.heading_pid = PID(
            kp=float(self.get_parameter('heading_kp').value),
            ki=float(self.get_parameter('heading_ki').value),
            kd=float(self.get_parameter('heading_kd').value),
            out_min=-1.0, out_max=1.0
        )
        self.speed_pid = PID(
            kp=float(self.get_parameter('speed_kp').value),
            ki=float(self.get_parameter('speed_ki').value),
            kd=float(self.get_parameter('speed_kd').value),
            out_min=-1.0, out_max=1.0
        )

        # Publisher & Subscriber
        self.pub_rc = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.pub_reached = self.create_publisher(Bool, 'asv/target_reached', 10)
        
        # Stop vio Topic
        self.create_subscription(Bool, '/asv/stop', self.stop_callback, 10)

        # Ziel setzen via Topic: geometry_msgs/Point (x=lat, y=lon)
        self.sub_goal = self.create_subscription(Point, 'asv/target', self.cb_target, 10)

        # GPS von MAVROS (dein Topic)
        self.sub_gpsraw = self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.cb_gps, 10)

        # 10 Hz Steuer-Loop
        self.timer = self.create_timer(0.1, self.control_step)

        self.get_logger().info('ASV PID RC node bereit. Ziel via Topic "asv/target" (Point: x=lat, y=lon) setzen.')

    # ---- Callbacks ----
    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Stop-Signal empfangen -> Boot anhalten")
            self.stop()
        else:
            self.stopped = False
            self.get_logger().info("Stop aufgehoben -> Regelung wieder aktiv")

    def stop(self):
        self.stopped = True
        self.speed_pid.integral = 0.0
        self.heading_pid.integral = 0.0
        self.send_rc(1500, 1500)   # neutral

    def cb_target(self, msg: Point):
        self.goal_lat = float(msg.x)
        self.goal_lon = float(msg.y)
        self.have_goal = True
        self.target_reached = False
        self.heading_pid.reset()
        self.speed_pid.reset()
        self.get_logger().info(f'Neues Ziel: lat={self.goal_lat:.7f}, lon={self.goal_lon:.7f}')

    def cb_gps(self, msg: GPSRAW):
        # nur mit Fix arbeiten
        if msg.fix_type < 2:
            return
        self.last_lat = msg.lat / 1e7
        self.last_lon = msg.lon / 1e7
        # COG in centideg -> Grad (0..360), 65535 = unknown
        if msg.cog != 65535:
            self.last_cog_deg = (msg.cog / 100.0) % 360.0

    # ---- Steuerlogik ----

    def control_step(self):
        if self.stopped:
            return
        if not self.have_goal:
            return
        if self.last_lat is None or self.last_lon is None or self.last_cog_deg is None:
            return

        # Distanz & Peilung
        dist = haversine_m(self.last_lat, self.last_lon, self.goal_lat, self.goal_lon)
        brg = bearing_deg(self.last_lat, self.last_lon, self.goal_lat, self.goal_lon)  # 0..360
        heading_err = wrap_deg(brg - self.last_cog_deg)  # in Grad, -180..180

        # Ziel erreicht?
        if dist < float(self.get_parameter('arrival_radius_m').value):
            if not self.target_reached:
                self.target_reached = True
                self.pub_reached.publish(Bool(data=True))
                self.get_logger().info('Ziel erreicht – Stoppe (neutral).')
            self.send_rc(neutral=True)
            return
        else:
            # sobald wir raus sind, wieder False
            if self.target_reached:
                self.target_reached = False

        # Heading-PID (Norm-Steer −1..1)
        steer_norm = self.heading_pid.update(heading_err)  # Fehler in Grad!

        # Einfache Speed-Regel: proportional auf Distanz (Norm −1..1), Bremse bei großem Heading-Fehler
        speed_cmd = self.speed_pid.update(dist)  # >0 => vorwärts
        # Bei großem Kursfehler Gas begrenzen
        max_head = float(self.get_parameter('max_throttle_when_large_heading_deg').value)
        if abs(heading_err) > max_head:
            speed_cmd = min(speed_cmd, 0.2)  # z.B. auf 20% drosseln

        # in PWM wandeln
        pwm1, pwm3 = self.mix_to_pwm(steer_norm, speed_cmd)

        self.send_rc(pwm1=pwm1, pwm3=pwm3)

        # Debug schlank:
        self.get_logger().info(
            f'dist={dist:.1f}m brg={brg:.0f}° cog={self.last_cog_deg:.0f}° '
            f'err={heading_err:.0f}° steer={steer_norm:+.2f} spd={speed_cmd:+.2f} '
            f'CH1={pwm1} CH3={pwm3}'
        )

    def mix_to_pwm(self, steer_norm, speed_norm):
        neutral = int(self.get_parameter('neutral_pwm').value)
        steer_span = int(self.get_parameter('steer_span').value)
        thr_span_f = int(self.get_parameter('throttle_span_fwd').value)
        thr_span_r = int(self.get_parameter('throttle_span_rev').value)
        inv_st = bool(self.get_parameter('invert_steer').value)
        inv_th = bool(self.get_parameter('invert_throttle').value)

        steer_norm = clamp(steer_norm, -1.0, 1.0)
        speed_norm = clamp(speed_norm, -1.0, 1.0)

        # Steering CH1
        steer = steer_norm if not inv_st else -steer_norm
        ch1 = neutral + int(steer * steer_span)

        # Throttle CH3 (asymmetrisch)
        spd = speed_norm if not inv_th else -speed_norm
        if spd >= 0:
            ch3 = neutral + int(spd * thr_span_f)
        else:
            ch3 = neutral + int(spd * thr_span_r)  # spd negativ -> neutral - X

        ch1 = clamp(ch1, 1100, 1900)
        ch3 = clamp(ch3, 1100, 1900)
        return ch1, ch3

    def send_rc(self, pwm1=None, pwm3=None, neutral=False):
        msg = OverrideRCIn()
        ch = [0]*18  # 0 = no override

        if neutral:
            n = int(self.get_parameter('neutral_pwm').value)
            ch[0] = n       # CH1 Lenkung neutral
            ch[2] = n       # CH3 Gas neutral
        else:
            if pwm1 is not None:
                ch[0] = int(pwm1)  # CH1
            if pwm3 is not None:
                ch[2] = int(pwm3)  # CH3

        msg.channels = ch
        self.pub_rc.publish(msg)


def main():
    rclpy.init()
    node = ASVPidRcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
