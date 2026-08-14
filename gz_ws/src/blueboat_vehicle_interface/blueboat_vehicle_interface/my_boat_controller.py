from pymavlink import mavutil
import sys
import time
sys.path.insert(0, '/home/blueboat_sitl/gz_ws/src/extras_boat')
from changing_modes import BoatController

class MyBoatController(BoatController):
    # Adds extra paramter to control if boat disconnects when functions run
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._auto_disconnect_enabled = True

    # Redefine mavlink connection to avoid multiple connections
    def connect(self):
        if self.conn != None:
            return
        
        super().connect()
        self._connected = True

    # Only disconnects if auto-disconnect is enabled
    def disconnect(self):
        if self._auto_disconnect_enabled:
            return super().disconnect()
    
    # Set boat parameters, replaces QGroundControl interface
    # Ex. self.set_param("FRAME_CLASS", 2) to set vehicle to boat
    def set_boat_param(self, name: str, value: int) -> bool:
        if not self._connected:
            self.get_logger().error("MAVLink connection not established. Cannot set parameter.")
            return False
        
        try:
            self.conn.mav.param_set_send(
                self.conn.target_system,
                self.conn.target_component,
                name.encode('utf-8'),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to set parameter {name}: {e}")
            return False
        

    def read_boat_param(self, name:str):
        self.conn.mav.param_request_read_send(
            self.conn.target_system,
            self.conn.target_component,
            name.encode('utf-8'),
            -1
        )
        msg = self.conn.recv_match(
            type='PARAM_VALUE',
            blocking=True,
            timeout=3
        )
        if msg is None:
            print(f"Failed to read parameter {name}")
            return None
        return msg.param_value
    

    def send_target_position(self, lat, lon):
        # Send the target position to the vehicle
        self.conn.mav.set_position_target_global_int_send(
            0,                         # time_boot_ms
            self.conn.target_system,      # target_system
            self.conn.target_component,   # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000,        # type_mask (only positions enabled)
            int(lat * 1e7),            # lat_int - X Position in WGS84 frame in 1e7 * degrees
            int(lon * 1e7),            # lon_int - Y Position in WGS84 frame in 1e7 * degrees
            0,                         # alt
            0, 0, 0,                   # X, Y, Z velocity in m/s (not used)
            0, 0, 0,                   # afx, afy, afz acceleration (not used)
            0, 0)                      # yaw, yaw rate (not used)
        

    def send_waypoint(self, lat, lon):
        self.conn.mav.mission_item_send(
            self.conn.target_system, self.conn.target_component, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
            0, 0, 0, lat, lon, 0
        )
    
    # frame = mavutil.mavlink.MAV_FRAME_GLOBAL
    # frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    # frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
