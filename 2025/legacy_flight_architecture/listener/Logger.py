import csv
import os
from datetime import datetime

class TelemetryLogger:
    def __init__(self, filepath="telemetry_log.csv"):
        self.filepath = filepath
        self.headers = [
            "timestamp", "drone_lat", "drone_lon", "drone_alt_m",
            "pitch_deg", "roll_deg", "yaw_deg", "target_lat", "target_lon"
        ]
        
        # Create the file and write headers if it doesn't exist
        file_exists = os.path.isfile(self.filepath)
        with open(self.filepath, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.headers)
            if not file_exists:
                writer.writeheader()
                print(f"[LOGGER] Created new log file: {self.filepath}")

    def log_data(self, d_lat, d_lon, d_alt, pitch, roll, yaw, t_lat, t_lon):
        """Called by payload_drop.py to instantly append a row of live data"""
        row = {
            "timestamp": datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "drone_lat": round(d_lat, 7),
            "drone_lon": round(d_lon, 7),
            "drone_alt_m": round(d_alt, 3),
            "pitch_deg": round(pitch, 4),
            "roll_deg": round(roll, 4),
            "yaw_deg": round(yaw, 4),
            "target_lat": round(t_lat, 7),
            "target_lon": round(t_lon, 7),
        }
        with open(self.filepath, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.headers)
            writer.writerow(row)
