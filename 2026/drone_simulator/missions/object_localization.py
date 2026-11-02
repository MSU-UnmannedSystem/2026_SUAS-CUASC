import time
import csv
from competition.drone_simulator.missions.mission_base import MissionBase
from competition.drone_simulator.vehicle import commands
from competition.jetson_rescue.geolocation import TargetGeolocator
from inference.camera import VisionPipeline

TELEMETRY_MAX_AGE_S = 1.0  # older position/attitude counts as no data

class ObjectLocalizationMission(MissionBase):
    def __init__(self, link, config, desk_test=False):
        super().__init__(link, config)
        self.desk_test = desk_test
        self.geolocator = TargetGeolocator(res_w=1920, res_h=1080, hfov_deg=65.0, vfov_deg=39.4)
        
        # FIX 1: Utilize the threaded pipeline and set the mode to squares!
        self.vision = VisionPipeline(camera_index=0, target_type='bw_square')
        
        # Non-blocking cooldown tracker
        self.last_log_time = 0
        
        # Setup CSV to hand to the judges
        self.csv_filename = f"target_locations_{int(time.time())}.csv"
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Target_ID", "Latitude", "Longitude"])

    def arm_and_takeoff(self):
        if self.desk_test:
            print("[LOCALIZATION] DESK TEST MODE: Skipping physical takeoff wait.")
            self.set_expected_mode("AUTO")
            commands.arm(self.link)
            time.sleep(2)
        else:
            super().arm_and_takeoff()

    def execute(self):
        print("[LOCALIZATION] Entering Passive Monitoring. Drone is flying AUTO path.")
        if not self.desk_test:
            commands.set_mode(self.link, "AUTO")
        self.set_expected_mode("AUTO")

    def monitor(self):
        # 1. Run the normal base class waypoint check
        super().monitor()

        # 2. Check for squares via the background thread
        pixel_x, pixel_y = self.vision.get_target_pixel()
        
        # FIX 2: Implement a non-blocking 2-second cooldown
        current_time = time.time()
        
        if pixel_x is not None and (current_time - self.last_log_time > 2.0):
            pos_msg = self.link.latest_fresh("GLOBAL_POSITION_INT", TELEMETRY_MAX_AGE_S)
            att_msg = self.link.latest_fresh("ATTITUDE", TELEMETRY_MAX_AGE_S)
            
            if pos_msg and att_msg:
                current_lat = pos_msg.lat / 1e7
                current_lon = pos_msg.lon / 1e7
                alt = pos_msg.relative_alt / 1000.0  
                
                # Rulebook enforcement warning
                if alt < 7.62:
                    print(f"WARNING: Altitude is {alt:.1f}m! Climb above 25ft to avoid penalty!")

                t_lat, t_lon, _, _ = self.geolocator.calculate_target_coordinates(
                    pixel_x, pixel_y, current_lat, current_lon, alt, 
                    att_msg.pitch, att_msg.roll, att_msg.yaw
                )
                
                print(f"[LOCALIZATION] Square Found! Target GPS: {t_lat:.6f}, {t_lon:.6f}")
                
                # Save to CSV
                with open(self.csv_filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Square_Target", t_lat, t_lon])
                
                # Reset the cooldown timer
                self.last_log_time = current_time

    def end_mission(self):
        # Ensure hardware is released when the flight finishes
        self.vision.release()
        super().end_mission()