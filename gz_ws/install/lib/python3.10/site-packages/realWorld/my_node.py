""""
ASV Control ROS2 Node
Converts the boat_control.py script into a ROS2 node with services and status publishing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import NavSatFix
import threading
import time
import sys
from pymavlink import mavutil

# Global Coordinate System:
# The script now uses MAV_FRAME_GLOBAL_INT to send global coordinates
# - X-axis: Latitude (in degrees)
# - Y-axis: Longitude (in degrees)
# - Z-axis: Altitude (in meters, 0 = surface level)
# 
# Example: To go to lat=48.284812, lon=11.606132, send x=48.284812, y=11.606132
# Example: To go to lat=48.285000, lon=11.607000, send x=48.285000, y=11.607000

class ASVControlNode(Node):
    """ROS2 node for ASV control via MAVLink"""
    
    def __init__(self):
        super().__init__('asv_control_node')
        
        # Initialize MAVLink connection parameters
        self.connection_string = self.declare_parameter('connection_string', 'udpin:0.0.0.0:14550').value
        #self.connection_string = self.declare_parameter('connection_string', 'udpin:0.0.0.0:14600').value
        self.connection = None
        self.target_system = None
        self.target_component = None
        self.monitoring = False
        self.latest_status = {
            'mode': 'Unknown',
            'armed': 'Unknown',
            'gps': 'No data',
            'battery': 'Unknown',
            'connected': False,
            'gps_lat': 0.0,
            'gps_lon': 0.0,
            'gps_alt': 0.0,
            'gps_fix': 0,
            'battery_percent': 0
        }
        self.status_lock = threading.Lock()
        
        # Position control variables
        self.target_position = None
        self.position_lock = threading.Lock()
        self.position_sending_active = False
        
        # Setup QoS for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, 'asv/gps', qos_profile)
        
        # Subscribers
        self.position_sub = self.create_subscription(
            Pose,
            'asv/set_position',
            self.position_callback,
            qos_profile
        )
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.position_timer = self.create_timer(0.5, self.send_position)  # 2 Hz
        self.monitoring_timer = None
        
        # Start status monitoring thread
        self.start_status_monitoring()
        
        self.get_logger().info('ASV Control Node initialized')
        self.get_logger().info(f'Connection string: {self.connection_string}')
        self.get_logger().info('Using global coordinates (latitude/longitude) for position control')
        
    def connect(self, timeout=10):
        """Connect to the boat and wait for heartbeat"""
        try:
            self.get_logger().info(f"Connecting to {self.connection_string}...")
            self.connection = mavutil.mavlink_connection(self.connection_string)
            
            self.get_logger().info("Waiting for heartbeat...")
            self.connection.wait_heartbeat(timeout=timeout)
            
            self.target_system = self.connection.target_system
            self.target_component = self.connection.target_component
            
            self.get_logger().info(f"Connected to system {self.target_system} component {self.target_component}")
            
            # Update status
            with self.status_lock:
                self.latest_status['connected'] = True
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ Connection failed: {e}")
            
            # Update status
            with self.status_lock:
                self.latest_status['connected'] = False
            
            return False
    
    def set_guided_mode(self, timeout=10):
        """Set the vehicle to GUIDED mode"""
        if not self.connection:
            self.get_logger().error("Not connected to vehicle")
            return False
            
        try:
            self.get_logger().info("Setting mode to GUIDED...")
            
            # Try to set GUIDED mode
            try:
                self.connection.set_mode('GUIDED')
                self.get_logger().info("Sent mode change using set_mode('GUIDED')")
            except Exception as e:
                self.get_logger().warn(f"set_mode failed: {e}")
                # Fallback: try to get mode ID and use set_mode_send
                try:
                    mode_mapping = self.connection.mode_mapping()
                    if 'GUIDED' in mode_mapping:
                        mode_id = mode_mapping['GUIDED']
                        self.connection.set_mode_send(mode_id)
                        self.get_logger().info(f"Sent mode change using set_mode_send({mode_id})")
                    else:
                        self.get_logger().error("✗ GUIDED mode not available")
                        return False
                except Exception as e2:
                    self.get_logger().error(f"set_mode_send failed: {e2}")
                    return False
            
            # Wait for mode change confirmation
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    # Check if mode was set using flightmode property
                    current_flightmode = self.connection.flightmode
                    if current_flightmode == 'GUIDED':
                        self.get_logger().info(f"Mode set to GUIDED (via flightmode)")
                        return True
                    
                    # Also check current mode from heartbeat
                    heartbeat = self.connection.recv_match(type='HEARTBEAT', blocking=False)
                    if heartbeat:
                        current_mode_id = heartbeat.custom_mode
                        # Try to get mode name from ID
                        try:
                            mode_mapping = self.connection.mode_mapping()
                            for name, mid in mode_mapping.items():
                                if mid == current_mode_id:
                                    if name == 'GUIDED':
                                        self.get_logger().info(f"Mode set to GUIDED (via heartbeat)")
                                        return True
                                    else:
                                        self.get_logger().info(f"Current mode: {name} (ID: {current_mode_id})")
                                    break
                        except:
                            self.get_logger().info(f"Current mode ID: {current_mode_id}")
                    
                except Exception as e:
                    self.get_logger().warn(f"⚠ Error checking mode: {e}")
            
            self.get_logger().error("✗ Timeout waiting for mode change to GUIDED")
            return False
            
        except Exception as e:
            self.get_logger().error(f"✗ Error setting mode: {e}")
            return False

    def arm_vehicle(self):
        """Arm the vehicle"""
        if not self.connection:
            self.get_logger().error("Not connected to vehicle")
            return False
            
        try:
            self.get_logger().info("Arming vehicle...")
            
            # Send arm command
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                1,  # Arm (1)
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            
            self.get_logger().info("Arm command sent")
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ Error arming: {e}")
            return False

    def _update_status_thread(self):
        """Background thread to continuously update system status"""
        while self.monitoring:
            try:
                if not self.connection:
                    with self.status_lock:
                        self.latest_status = {
                            'mode': 'Unknown',
                            'armed': 'Unknown',
                            'gps': 'No data',
                            'battery': 'Unknown',
                            'connected': False,
                            'gps_lat': 0.0,
                            'gps_lon': 0.0,
                            'gps_alt': 0.0,
                            'gps_fix': 0,
                            'battery_percent': 0
                        }
                    time.sleep(1.0)
                    continue
                
                # Get current mode
                try:
                    current_mode = self.connection.flightmode
                except:
                    current_mode = 'Unknown'
                
                # Get armed state
                try:
                    armed = self.connection.motors_armed()
                    armed_status = "Armed" if armed else "Disarmed"
                except:
                    armed_status = "Unknown"
                
                # Get GPS info
                gps = self.connection.recv_match(type='GPS_RAW_INT', blocking=False)
                gps_info = "No data"
                gps_lat = 0.0
                gps_lon = 0.0
                gps_alt = 0.0
                gps_fix = 0
                
                if gps:
                    gps_fix = gps.fix_type
                    if gps.fix_type >= 2:
                        gps_lat = gps.lat / 1e7
                        gps_lon = gps.lon / 1e7
                        gps_alt = gps.alt / 1000.0
                        gps_info = f"{gps_lat:.6f}, {gps_lon:.6f}, {gps_alt:.1f}m"
                    else:
                        gps_info = f"No fix (type {gps.fix_type})"
                
                # Get battery info
                battery = self.connection.recv_match(type='SYS_STATUS', blocking=False)
                battery_info = "Unknown"
                battery_percent = 0
                if battery and hasattr(battery, 'battery_remaining'):
                    battery_percent = battery.battery_remaining
                    battery_info = f"{battery.battery_remaining}%"
                
                # Update latest status with thread safety
                with self.status_lock:
                    self.latest_status = {
                        'mode': current_mode,
                        'armed': armed_status,
                        'gps': gps_info,
                        'battery': battery_info,
                        'connected': True,
                        'gps_lat': gps_lat,
                        'gps_lon': gps_lon,
                        'gps_alt': gps_alt,
                        'gps_fix': gps_fix,
                        'battery_percent': battery_percent
                    }
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error updating status: {e}")
                time.sleep(0.1)

    def start_status_monitoring(self):
        """Start monitoring system status in a background thread"""
        self.get_logger().info("Starting system status monitoring...")
        self.monitoring = True
        self.status_thread = threading.Thread(target=self._update_status_thread, daemon=True)
        self.status_thread.start()

    def stop_status_monitoring(self):
        """Stop monitoring system status"""
        self.monitoring = False
        if hasattr(self, 'status_thread'):
            self.status_thread.join(timeout=1)

    def publish_status(self):
        """Publish current status to ROS topics"""
        try:
            with self.status_lock:
                status = self.latest_status.copy()
            
            # Publish GPS data if available
            if status.get('connected', False) and status.get('gps_fix', 0) >= 2:
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = "map"
                gps_msg.latitude = status.get('gps_lat', 0.0)
                gps_msg.longitude = status.get('gps_lon', 0.0)
                gps_msg.altitude = status.get('gps_alt', 0.0)
                gps_msg.status.status = status.get('gps_fix', 0)
                self.gps_pub.publish(gps_msg)
        except Exception as e:
            self.get_logger().error(f"Error in publish_status: {e}")

    def position_callback(self, msg):
        """Callback for position setpoint messages"""
        with self.position_lock:
            # Check if this is a new target (different from current)
            if (self.target_position is None or 
                abs(msg.position.x - self.target_position.position.x) > 1e-6 or 
                abs(msg.position.y - self.target_position.position.y) > 1e-6):
                
                self.target_position = msg
                self.position_sending_active = True
                self.get_logger().info(f'New position target received: lat={msg.position.x:.6f}, lon={msg.position.y:.6f}')
            else:
                # Same target, just update the message but don't log
                self.target_position = msg
        
    def send_position(self):
        """Send position target to the vehicle via MAVLink"""
        if not self.connection or not self.target_system:
            return
            
        with self.position_lock:
            if not self.position_sending_active or self.target_position is None:
                return
            position = self.target_position
        
        try:
            # Use global coordinates (latitude and longitude)
            # Convert to integer format (degrees * 1e7)
            target_lat = int(position.position.x * 1e7)  # latitude in degrees * 1e7
            target_lon = int(position.position.y * 1e7)  # longitude in degrees * 1e7
            target_alt = int(0.0 * 1000)  # altitude in mm (0 = surface level)
            
            self.connection.mav.set_position_target_global_int_send(
                0,  # time_boot_ms (ignored)
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # coordinate frame for global coordinates
                0b0000111111111000,  # type_mask (position only)
                target_lat,  # latitude in degrees * 1e7
                target_lon,  # longitude in degrees * 1e7
                target_alt,  # altitude in mm
                0,  # x velocity in m/s
                0,  # y velocity in m/s
                0,  # z velocity in m/s
                0,  # x acceleration or force (if bit 10 of type_mask is set)
                0,  # y acceleration or force (if bit 10 of type_mask is set)
                0,  # z acceleration or force (if bit 10 of type_mask is set)
                0,  # yaw setpoint
                0   # yaw rate setpoint
            )
                
            # Only log when a new target is first sent
            if self.position_sending_active:
                self.get_logger().info(f"Global position target sent: lat={position.position.x:.6f}, lon={position.position.y:.6f}")
                # Mark that we've sent this target
                self.position_sending_active = False
            
        except Exception as e:
            self.get_logger().error(f"Error sending position target: {e}")


    def close(self):
        """Close the connection"""
        if self.connection:
            self.connection.close()
            self.get_logger().info("Connection closed")
        
        # Update status
        with self.status_lock:
            self.latest_status['connected'] = False


def main(args=None):
    """Main function to run the ASV control node"""
    rclpy.init(args=args)
    
    node = ASVControlNode()
    
    try:
        # Try to connect automatically on startup
        node.connect()
        
        # If connection successful, try to set GUIDED mode and arm
        if node.latest_status['connected']:
            if node.set_guided_mode():
                node.get_logger().info("Successfully set vehicle to GUIDED mode!")
                # Try to arm the vehicle
                node.arm_vehicle()
            else:
                node.get_logger().error("Failed to set GUIDED mode")
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Operation interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.stop_status_monitoring()
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 