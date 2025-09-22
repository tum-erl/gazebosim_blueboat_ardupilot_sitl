#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from mavros_msgs.msg import GPSRAW, OverrideRCIn
from sensor_msgs.msg import Imu


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
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlmb)
    brg = math.degrees(math.atan2(y, x))
    return brg + 360.0 if brg < 0.0 else brg

# Quaternion -> Yaw (rad), kreissichere Interpolation in Grad
def quat_to_yaw_rad(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)  # [-pi, pi]

def circ_lerp_deg(a_deg, b_deg, k):
    """Zyklische Interpolation a->b in Grad, k in [0,1]."""
    diff = wrap_deg(b_deg - a_deg)
    return (a_deg + k*diff + 360.0) % 360.0


class PID:
    def __init__(self, kp, ki, kd, out_min=-3.0, out_max=3.0):
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
        self.declare_parameter('arrival_radius_m', 0.6)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('pwm_span', 400)          # 1500 ± 400 -> 1100..1900
        self.declare_parameter('chan_left', 1)           # 1-basiert: CH1
        self.declare_parameter('chan_right', 3)          # 1-basiert: CH3

        # Heading-PID in RAD
        self.declare_parameter('heading_kp', 3.0)
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 0.4)

        # Speed-PID
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

        # --- Zustände ---
        self.last_lat = None
        self.last_lon = None
        self.target_reached = False
        self.stop_active = False  # <<< Stop latch

        # IMU-Parameter & Zustände
        self.declare_parameter('imu_yaw_offset_deg', 0.0)
        self.declare_parameter('imu_yaw_lpf_tau_s', 0.5)
        self.last_yaw_deg_imu_raw = None
        self.last_yaw_deg_imu_filt = None
        self.last_imu_t = None

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

        # GPS: Position (QoS BestEffort, KeepLast(10))
        gps_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.cb_gps, gps_qos)

        # IMU: Heading (SensorDataQoS = BestEffort, volatile)
        self.create_subscription(Imu, '/mavros/imu/data', self.cb_imu, qos_profile_sensor_data)

        # 10 Hz Steuer-Loop
        self.timer = self.create_timer(0.1, self.control_step)

        self.get_logger().info('ASV PID RC node (IMU-Heading, QoS fix, Stop latch) bereit.')

    # ---- Callbacks ----
    def stop_callback(self, msg: Bool):
        if msg.data:
            self.stop_active = True
            self.get_logger().info("Stop-Signal erhalten → neutral (latched).")
            self.speed_pid.reset()
            self.heading_pid.reset()
            # optional: Ziel abbrechen
            # self.have_goal = False
        else:
            self.stop_active = False
            self.get_logger().info("Stop aufgehoben → Regelung läuft wieder.")

    def cb_target(self, msg: Point):
        self.goal_lat = float(msg.x)
        self.goal_lon = float(msg.y)
        self.have_goal = True
        self.target_reached = False
        self.heading_pid.reset()
        self.speed_pid.reset()
        self.get_logger().info(f'Neues Ziel: lat={self.goal_lat:.7f}, lon={self.goal_lon:.7f}')

    def cb_gps(self, msg: GPSRAW):
        if msg.fix_type < 2:
            return
        self.last_lat = msg.lat / 1e7
        self.last_lon = msg.lon / 1e7

    def cb_imu(self, msg: Imu):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        if (qx*qx + qy*qy + qz*qz + qw*qw) < 1e-10:
            return
        yaw_rad = quat_to_yaw_rad(qx, qy, qz, qw)
        yaw_deg = (math.degrees(yaw_rad) + float(self.get_parameter('imu_yaw_offset_deg').value)) % 360.0

        now = time.time()
        if self.last_imu_t is None or self.last_yaw_deg_imu_filt is None:
            self.last_yaw_deg_imu_raw = yaw_deg
            self.last_yaw_deg_imu_filt = yaw_deg
            self.last_imu_t = now
            return

        dt = max(1e-3, now - self.last_imu_t)
        self.last_imu_t = now
        tau = max(1e-3, float(self.get_parameter('imu_yaw_lpf_tau_s').value))
        alpha = clamp(dt / (tau + dt), 0.0, 1.0)

        self.last_yaw_deg_imu_raw = yaw_deg
        self.last_yaw_deg_imu_filt = circ_lerp_deg(self.last_yaw_deg_imu_filt, yaw_deg, alpha)

    # ---- Stellgrößen ----
    def compute_speed_cmd(self, distance_m, heading_error_rad):
        if distance_m < 0.1:
            self.speed_pid.integral = 0.0
            return -0.2
        if distance_m > 0.3 and abs(heading_error_rad) > math.radians(90):
            return 0.0
        return clamp(-self.speed_pid.update(distance_m), -0.5, 1.0)

    def compute_steer_cmd(self, heading_error_rad):
        steer = self.heading_pid.update(heading_error_rad)
        return clamp(steer, -1.0, 1.0)

    def compute_motor_pwms(self, speed_cmd, steer_cmd):
        base_pwm = int(self.get_parameter('neutral_pwm').value)
        span = int(self.get_parameter('pwm_span').value)
        power = int(span * speed_cmd)
        turn = int(span * steer_cmd)
        left = clamp(base_pwm - power - turn, 1100, 1900)
        right = clamp(base_pwm - power + turn, 1100, 1900)
        return left, right

    # ---- Steuerlogik ----
    def control_step(self):
        # Latch-Stop: neutral halten solange aktiv
        if self.stop_active:
            self.send_rc(neutral=True)
            return

        if not self.have_goal:
            return
        if self.last_lat is None or self.last_lon is None:
            return
        if self.last_yaw_deg_imu_filt is None:
            return

        dist = haversine_m(self.last_lat, self.last_lon, self.goal_lat, self.goal_lon)
        brg_deg = bearing_deg(self.last_lat, self.last_lon, self.goal_lat, self.goal_lon)
        heading_deg = self.last_yaw_deg_imu_filt
        heading_err_deg = wrap_deg(brg_deg - heading_deg)
        heading_err_rad = math.radians(heading_err_deg)

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
        self.send_rc(pwm_left, pwm_right)

        self.get_logger().info(
            f'dist={dist:.2f}m brg={brg_deg:.0f}° hdg_imu={heading_deg:.0f}° '
            f'err={heading_err_deg:.0f}° spd={speed_cmd:+.2f} str={steer_cmd:+.2f} '
            f'L={pwm_left} R={pwm_right}'
        )

    def send_rc(self, pwm_left=None, pwm_right=None, neutral=False):
        msg = OverrideRCIn()
        ch = [0]*18

        if neutral:
            n = int(self.get_parameter('neutral_pwm').value)
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
