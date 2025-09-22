#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from mavros_msgs.msg import GPSRAW, OverrideRCIn


def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_deg(a):
    """wrap angle to [-180, 180)"""
    return (a + 180.0) % 360.0 - 180.0

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
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlmb
    )
    brg = math.degrees(math.atan2(y, x))
    return brg + 360.0 if brg < 0.0 else brg


class PID:
    def __init__(self, kp, ki, kd, out_min=-3.0, out_max=3.0):
        # bewusst breiter Clamp als in der alten Version; End-Clamp passiert außerhalb
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

        # --- Parameter (schlank & sim-nah) ---
        self.declare_parameter('arrival_radius_m', 0.6)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('pwm_span', 400)          # 1500 ± 400 -> 1100..1900
        self.declare_parameter('chan_left', 1)           # 1-basiert: CH1
        self.declare_parameter('chan_right', 3)          # 1-basiert: CH3

        # Heading-PID in RAD (wie in deiner Simulation)
        self.declare_parameter('heading_kp', 3.0)
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 0.4)

        # Speed-PID (hier P/PD möglich, wie in der Simulation)
        self.declare_parameter('speed_kp', 0.1)
        self.declare_parameter('speed_ki', 0.0)
        self.declare_parameter('speed_kd', 0.8)

        # Ziel (lat/lon) initial optional
        self.declare_parameter('goal_lat', 48.28454)
        self.declare_parameter('goal_lon', 11.60564)
        self.have_goal = False
        self.goal_lat = float(self.get_parameter('goal_lat').value)
        self.goal_lon = float(self.get_parameter('goal_lon').value)
        if abs(self.goal_lat) > 1e-9 or abs(self.goal_lon) > 1e-9:
            self.have_goal = True
            self.get_logger().info(f'Startziel: lat={self.goal_lat:.7f}, lon={self.goal_lon:.7f}')

        # Aktuelle Navigation
        self.last_lat = None
        self.last_lon = None
        self.last_cog_deg = None  # 0..360 (aus GPS COG); 655.35° (=65535 centideg) bedeutet unknown
        self.target_reached = False

        # PIDs
        self.heading_pid = PID(
            kp=float(self.get_parameter('heading_kp').value),
            ki=float(self.get_parameter('heading_ki').value),
            kd=float(self.get_parameter('heading_kd').value),
            out_min=-3.0, out_max=3.0
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

        self.create_subscription(Bool, '/asv/stop', self.stop_callback, 10)
        self.create_subscription(Point, 'asv/target', self.cb_target, 10)
        self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.cb_gps, 10)

        # 10 Hz Steuer-Loop
        self.timer = self.create_timer(0.1, self.control_step)

        self.get_logger().info('ASV PID RC node (sim-Style Regler) bereit.')

    # ---- Callbacks ----
    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Stop-Signal -> neutral.")
            self.speed_pid.integral = 0.0
            self.heading_pid.integral = 0.0
            self.send_rc(neutral=True)

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

    # ---- Sim-Style Stellgrößen ----
    def compute_speed_cmd(self, distance_m, heading_error_rad):
        """Wie in deiner Simulation."""
        if distance_m < 0.1:
            self.speed_pid.integral = 0.0
            return -0.2  # leicht rückwärts zum Bremsen
        if distance_m > 0.3 and abs(heading_error_rad) > math.radians(90):
            return 0.0  # auf der Stelle drehen
        # Negatives Vorzeichen, damit „vorwärts“ PWM > neutral (siehe compute_motor_pwms)
        return clamp(-self.speed_pid.update(distance_m), -0.5, 1.0)

    def compute_steer_cmd(self, heading_error_rad):
        steer = self.heading_pid.update(heading_error_rad)
        return clamp(steer, -1.0, 1.0)

    def compute_motor_pwms(self, speed_cmd, steer_cmd):
        base_pwm = int(self.get_parameter('neutral_pwm').value)
        span = int(self.get_parameter('pwm_span').value)
        power = int(span * speed_cmd)
        turn = int(span * steer_cmd)
        # links/rechts differenziell – wie in der Simulation:
        # Vorzeichen so gewählt, dass speed_cmd < 0 → PWM > base (vorwärts)
        left = clamp(base_pwm - power - turn, 1100, 1900)
        right = clamp(base_pwm - power + turn, 1100, 1900)
        return left, right

    # ---- Steuerlogik ----
    def control_step(self):
        if not self.have_goal:
            return
        if self.last_lat is None or self.last_lon is None or self.last_cog_deg is None:
            return

        # Distanz & Peilung
        dist = haversine_m(self.last_lat, self.last_lon, self.goal_lat, self.goal_lon)
        brg_deg = bearing_deg(self.last_lat, self.last_lon, self.goal_lat, self.goal_lon)  # 0..360
        heading_err_deg = wrap_deg(brg_deg - self.last_cog_deg)   # -180..180
        heading_err_rad = math.radians(heading_err_deg)

        # Ziel erreicht?
        if dist < float(self.get_parameter('arrival_radius_m').value):
            if not self.target_reached:
                self.target_reached = True
                self.pub_reached.publish(Bool(data=True))
                self.get_logger().info('Ziel erreicht – neutral.')
            self.send_rc(neutral=True)
            return
        else:
            if self.target_reached:
                self.target_reached = False

        
        speed_cmd = self.compute_speed_cmd(dist, heading_err_rad)
        steer_cmd = self.compute_steer_cmd(heading_err_rad)
        pwm_left, pwm_right = self.compute_motor_pwms(speed_cmd, steer_cmd)

        # Senden (CH1=links, CH3=rechts)
        self.send_rc(pwm_left, pwm_right)

        # Debug
        self.get_logger().info(
            f'dist={dist:.2f}m brg={brg_deg:.0f}° cog={self.last_cog_deg:.0f}° '
            f'err={heading_err_deg:.0f}° spd={speed_cmd:+.2f} str={steer_cmd:+.2f} '
            f'L={pwm_left} R={pwm_right}'
        )

    def send_rc(self, pwm_left=None, pwm_right=None, neutral=False):
        msg = OverrideRCIn()
        ch = [0]*18  # 0 = no override

        if neutral:
            n = int(self.get_parameter('neutral_pwm').value)
            # Standard: CH1 (links), CH3 (rechts)
            ch[int(self.get_parameter('chan_left').value) - 1]  = n
            ch[int(self.get_parameter('chan_right').value) - 1] = n
        else:
            if pwm_left is not None:
                ch[int(self.get_parameter('chan_left').value) - 1]  = int(pwm_left)
            if pwm_right is not None:
                ch[int(self.get_parameter('chan_right').value) - 1] = int(pwm_right)

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
