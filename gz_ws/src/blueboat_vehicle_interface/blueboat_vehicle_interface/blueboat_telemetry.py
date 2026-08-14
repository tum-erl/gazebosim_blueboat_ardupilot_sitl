from __future__ import annotations
import csv
import time
import datetime as dt
import pytz
import threading
import os
import pandas as pd
import numpy as np
from typing import TextIO
from .my_boat_controller import MyBoatController
# from .vehicle_interface import VehicleInterface
from pathlib import Path

class BlueboatTelemetryCSV():
    def __init__(
        self, field_names=[
            "time",
            "heading",
            "target_bearing",
            "nav_bearing",
            "wp_dist",
            "xtrack_error",
            "servo1",
            "servo2",
            "servo3",
            "servo4",
            "x",
            "y",
            "x_target",
            "y_target"
        ]):
        self.field_names = field_names
        package_dir = Path(__file__).parent
        self.folder = package_dir / "telemetry_data"
        self.file_name = self.generate_file_name()
        self.csv_file = open(
            Path(self.folder, self.file_name), "w+", newline=""
        )
        self.csv_writer = csv.DictWriter(self.csv_file, self.field_names)
        self.csv_writer.writeheader()
        self.telemetry_cache = {}
        self.allow_threading = False
        self.telemetry_thread = None


    def generate_file_name(self):
        time_format = "%Y-%m-%d_%H:%M:%S"
        file_time = dt.datetime.now(
            tz = pytz.utc
        )
        return file_time.strftime(time_format)


    # Kinda hardcoded w/ file format for now
    def save_blueboat_data(self, vehicle: VehicleInterface):
        """
        Save one row of telemetry to csv.
        Assumes file is open.
        File remains open after function is run.
        """

        nav = self.telemetry_cache.get("nav")
        servo = self.telemetry_cache.get("servo")
        hud = self.telemetry_cache.get("hud")

        row = {
            "time": time.time(),
            "heading":
                getattr(hud, "heading", None),
            "target_bearing":
                getattr(nav, "target_bearing", None),
            "nav_bearing":
                getattr(nav, "nav_bearing", None),
            "wp_dist":
                getattr(nav, "wp_dist", None),
            "xtrack_error":
                getattr(nav, "xtrack_error", None),
            "servo1":
                getattr(servo, "servo1_raw", None),
            "servo2":
                getattr(servo, "servo2_raw", None),
            "servo3":
                getattr(servo, "servo3_raw", None),
            "servo4":
                getattr(servo, "servo4_raw", None),
            "x": vehicle.x,
            "y": vehicle.y,
            "x_target": vehicle.goal[0],
            "y_target": vehicle.goal[1]
        }
        self.csv_writer.writerow(row)
        self.csv_file.flush()


    def get_telemetry(
            self, boat: MyBoatController
        ):
        """
        Read available MAVLink messages.
        Store newest values in a cache."""

        msg = boat.conn.recv_match(blocking=False)

        if msg != None:
            msg_type = msg.get_type()

            if msg_type == "NAV_CONTROLLER_OUTPUT":
                self.telemetry_cache["nav"] = msg
            elif msg_type == "SERVO_OUTPUT_RAW":
                self.telemetry_cache["servo"] = msg
            elif msg_type == "VFR_HUD":
                self.telemetry_cache["hud"] = msg
            elif msg_type == "HEARTBEAT":
                self.telemetry_cache["heartbeat"] = msg
        

    # Kinda hardcoded
    def log_telemetry(
            self, reference_time: float,
            boat: MyBoatController,
            vehicle: VehicleInterface
        ):
        while self.allow_threading:
            self.get_telemetry(boat)

            if time.time() - reference_time >= 0.5:
                self.save_blueboat_data(vehicle)
                reference_time = time.time()
            time.sleep(0.02) # High freq to catch param updates


    def start_logger_thread(
            self, boat: MyBoatController, vehicle: VehicleInterface
        ):
        self.allow_threading = True
        self.telemetry_thread = threading.Thread(
            target = self.log_telemetry, args = (time.time(), boat, vehicle)
        )
        self.telemetry_thread.start()

    
    def stop_telemetry(self):
        self.allow_threading = False
        self.telemetry_thread.join()
        self.csv_file.flush()
        self.csv_file.close()

        # Post-processing
        csv_path = Path(self.folder, self.file_name)
        df = pd.read_csv(csv_path)
        df['distance'] = np.sqrt((df.x - df.x_target)**2 + (df.y - df.y_target)**2)
        df.to_csv(csv_path, index = False)

