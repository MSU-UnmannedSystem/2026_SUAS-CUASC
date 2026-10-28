import queue
import time
import math
from competition.drone_simulator.missions.mission_base import MissionBase
from competition.drone_simulator.vehicle import commands

# PATHING FIX: Direct import from jetson_rescue
from competition.jetson_rescue.geolocation import TargetGeolocator
from competition.jetson_rescue.camera import VisionPipeline
from listener.Logger import TelemetryLogger

TELEMETRY_MAX_AGE_S = 1.0  # older position/attitude counts as no data

class PackageDeliveryMission(MissionBase):
    def __init__(self, link, config, desk_test=False):
        super().__init__(link, config)
        self.desk_test = desk_test
        self.geolocator = TargetGeolocator(res_w=1920, res_h=1080, hfov_deg=65.0, vfov_deg=39.4)
        self.vision = VisionPipeline(camera_index=0, target_type='red_bullseye')
        self.logger = TelemetryLogger()
        
        # 10-FOOT TEST FIX: Raised drop floor to 2.0m (6.5 ft) to prevent ground-effect crashes
        self.drop_altitude = 2.0 

    def arm_and_takeoff(self):
        if self.desk_test:
            print("[DELIVERY] DESK TEST MODE: Props disabled. Bypassing climb.")
            commands.set_mode(self.link, "GUIDED")
            self.set_expected_mode("GUIDED")
            commands.arm(self.link)
            time.sleep(2)
        else:
            super().arm_and_takeoff()

    def execute(self):
        print("[DELIVERY] Taking control. Switching to GUIDED mode.")
        commands.set_mode(self.link, "GUIDED")
        self.set_expected_mode("GUIDED")
        
        target_locked = False
        target_lat_int = 0
        target_lon_int = 0
        tolerance_meters = 2.0 
        distance = 999.0 # Default starting distance

        # Opened before the loop so no autopilot message is missed.
        with self.link.subscribe("STATUSTEXT") as statustexts:
            while self.active:
                pixel_x, pixel_y = self.vision.get_target_pixel()
            
                # --- ARDUPILOT ERROR TRAP ---
                while True:
                    try:
                        stat_msg = statustexts.get(timeout=0)
                    except queue.Empty:
                        break
                    if stat_msg.get_srcSystem() == self.link.target_system:
                        print(f"\n[FC ALERT] ArduPilot System Message: {stat_msg.text}")

                pos_msg = self.link.latest_fresh("GLOBAL_POSITION_INT", TELEMETRY_MAX_AGE_S)
                att_msg = self.link.latest_fresh("ATTITUDE", TELEMETRY_MAX_AGE_S)
            
                current_alt = 0.0
                if pos_msg:
                    current_alt = pos_msg.relative_alt / 1000.0

                if pixel_x is not None and not target_locked and pos_msg and att_msg:
                    current_lat = pos_msg.lat / 1e7
                    current_lon = pos_msg.lon / 1e7
                
                    t_lat, t_lon, _, _ = self.geolocator.calculate_target_coordinates(
                        pixel_x, pixel_y, current_lat, current_lon, current_alt, 
                        att_msg.pitch, att_msg.roll, att_msg.yaw
                    )

                    target_lat_int = int(t_lat * 1e7)
                    target_lon_int = int(t_lon * 1e7)
                    target_locked = True
                
                    self.logger.log_data(current_lat, current_lon, current_alt, att_msg.pitch, att_msg.roll, att_msg.yaw, t_lat, t_lon)

                    commands.send_position_target(
                        self.link, target_lat_int / 1e7, target_lon_int / 1e7, self.takeoff_alt)

                if target_locked and pos_msg:
                    lat_dif = (abs(target_lat_int - pos_msg.lat) / 1e7) * 111319
                    lon_dif = (abs(target_lon_int - pos_msg.lon) / 1e7) * 111319 * math.cos(math.radians(35))
                    distance = (lat_dif**2 + lon_dif**2)**(0.5)

                    if distance <= tolerance_meters:
                        print(f"\n[DELIVERY] Target Reached (Dist: {distance:.2f}m)! Descending to {self.drop_altitude}m...")
                    
                        commands.send_position_target(
                            self.link, target_lat_int / 1e7, target_lon_int / 1e7, self.drop_altitude)

                        # Descent Trap
                        while current_alt > self.drop_altitude:
                            wait_msg = self.link.latest_fresh("GLOBAL_POSITION_INT", TELEMETRY_MAX_AGE_S)
                            if wait_msg:
                                current_alt = wait_msg.relative_alt / 1000.0
                                print(f"[DESCENDING] Alt: {current_alt:.2f}m", end='\r')
                        
                            self.check_safety()
                            time.sleep(0.1)

                        print(f"\n[DELIVERY] Descent clear at {current_alt:.2f}m. Actuating Servo.")
                    
                        servo_pin = 6 
                        pwm_open = 1900 
                        commands.set_servo(self.link, servo_pin, pwm_open)
                    
                        print("[DELIVERY] Package Delivered. Ascending to clear airspace.")
                        commands.send_position_target(
                            self.link, target_lat_int / 1e7, target_lon_int / 1e7, self.takeoff_alt)
                        break

                # --- THE TELEMETRY FLOOD HUD ---
                print(f"| SYS: OK | ALT: {current_alt:.2f}m | TGT_PIXEL: ({pixel_x}, {pixel_y}) | LOCK: {target_locked} | DIST: {distance:.2f}m |")

                self.check_safety()
                time.sleep(0.2) 

        self.vision.release()
        print("[DELIVERY] Handing back to Base.")