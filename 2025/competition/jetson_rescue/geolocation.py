import math
import numpy as np

class TargetGeolocator:
    """
    Higher Brain Geolocation Engine: Bridges Vision and ArduPilot MAVLink telemetry.
    Calculates geographic coordinates of a target based on pixel location and drone attitude.
    """
    def __init__(self, res_w, res_h, hfov_deg, vfov_deg):
        self.res_w = res_w
        self.res_h = res_h
        self.hfov_rad = math.radians(hfov_deg)
        self.vfov_rad = math.radians(vfov_deg)

        self.fx = (res_w / 2.0) / math.tan(self.hfov_rad / 2.0)
        self.fy = (res_h / 2.0) / math.tan(self.vfov_rad / 2.0)

        self.cx = res_w / 2.0
        self.cy = res_h / 2.0

        self.EARTH_RADIUS = 6378137.0

    def calculate_target_coordinates(self, pixel_x, pixel_y, drone_lat, drone_lon, alt_agl, pitch, roll, yaw):
        # 1. Image Plane to Camera Ray
        ray_x_cam = (self.cy - pixel_y) / self.fy
        ray_y_cam = (pixel_x - self.cx) / self.fx
        ray_z_cam = 1.0  
        ray_cam = np.array([ray_x_cam, ray_y_cam, ray_z_cam])

        # 2. Rotation Matrices: Body Frame to NED Frame
        R_roll = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll),  math.cos(roll)]
        ])
        R_pitch = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        R_yaw = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw),  math.cos(yaw), 0],
            [0, 0, 1]
        ])

        R_ned = R_yaw @ R_pitch @ R_roll
        ray_ned = R_ned @ ray_cam

        # 3. Ground Plane Intersection
        if ray_ned[2] <= 0:
            raise ValueError("Target calculation failed: Camera ray is parallel to or above the ground plane.")

        scale = alt_agl / ray_ned[2]
        offset_north = ray_ned[0] * scale
        offset_east = ray_ned[1] * scale

        # 4. Convert Physical Offset to Geographic Coordinates
        lat_rad = math.radians(drone_lat)
        dlat = offset_north / self.EARTH_RADIUS
        dlon = offset_east / (self.EARTH_RADIUS * math.cos(lat_rad))

        target_lat = drone_lat + math.degrees(dlat)
        target_lon = drone_lon + math.degrees(dlon)

        return target_lat, target_lon, offset_north, offset_east

if __name__ == '__main__':
    # --- OFFLINE TESTING BLOCK ---
    print("Initializing Geolocator...")
    # Updated for ELP USB12MP02AF camera at 1080p
    geolocator = TargetGeolocator(res_w=1920, res_h=1080, hfov_deg=65.0, vfov_deg=39.4)
    
    # Dummy Inputs: Target at center pixel, Drone level at 50m AGL heading North
    target_pixel_x = 1920 / 2
    target_pixel_y = 1080 / 2
    
    test_lat, test_lon = 38.145123, -76.428543  
    test_alt_agl = 50.0    
    test_pitch, test_roll, test_yaw = 0.0, 0.0, 0.0 
    
    print("Calculating Target Geolocation...\n")
    try:
        t_lat, t_lon, off_n, off_e = geolocator.calculate_target_coordinates(
            target_pixel_x, target_pixel_y, test_lat, test_lon, 
            test_alt_agl, test_pitch, test_roll, test_yaw
        )
        
        print("--- RESULTS ---")
        print(f"Target Physical Offset: North = {off_n:.2f}m, East = {off_e:.2f}m")
        print(f"Drone Coordinate:  Lat {test_lat:.6f}, Lon {test_lon:.6f}")
        print(f"Target Coordinate: Lat {t_lat:.6f}, Lon {t_lon:.6f}")
        
    except ValueError as e:
        print(f"Error: {e}")
