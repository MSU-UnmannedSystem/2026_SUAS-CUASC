from pymavlink import mavutil
import math
import csv
import os
import sys

def analyze_blackbox(bin_filepath, scoring_waypoints, altitudes_in_feet):
    print(f"=== C-UASC 2026: 3D WAYPOINT LOG ANALYZER ===")
    print(f"Targeting DataFlash Log: {bin_filepath}")
    
    if not os.path.exists(bin_filepath):
        print(f"\n[ERROR] File Not Found: {bin_filepath}")
        sys.exit(1)

    try:
        mlog = mavutil.mavlink_connection(bin_filepath)
    except Exception as e:
        print(f"Failed to open log: {e}")
        sys.exit(1)

    converted_waypoints = []
    for lat, lon, alt in scoring_waypoints:
        final_alt = alt * 0.3048 if altitudes_in_feet else alt
        converted_waypoints.append((lat, lon, final_alt))

    closest_approaches = {i: 99999999.0 for i in range(len(converted_waypoints))}
    
    print(f"Scanning GPS & Barometer telemetry against {len(converted_waypoints)} targets...")
    
    gps_count = 0
    while True:
        msg = mlog.recv_match(type=['POS', 'GLOBAL_POSITION_INT'], blocking=False)
        if msg is None:
            break 
        
        lat = msg.Lat if hasattr(msg, 'Lat') else msg.lat / 1e7
        lon = msg.Lng if hasattr(msg, 'Lng') else msg.lon / 1e7
        alt_agl = msg.RelHomeAlt if hasattr(msg, 'RelHomeAlt') else msg.relative_alt / 1000.0
        gps_count += 1

        for i, (t_lat, t_lon, t_alt_m) in enumerate(converted_waypoints):
            lat_dif_m = (abs(t_lat - lat)) * 111319
            lon_dif_m = (abs(t_lon - lon)) * 111319 * math.cos(math.radians(lat))
            alt_dif_m = abs(t_alt_m - alt_agl)
            
            distance_3d = (lat_dif_m**2 + lon_dif_m**2 + alt_dif_m**2)**0.5

            if distance_3d < closest_approaches[i]:
                closest_approaches[i] = distance_3d

    print(f"Parsed {gps_count} positional telemetry points.")

    # --- OUTPUT RESULTS TO TERMINAL ---
    print("\n--- FINAL 3D SCORING RESULTS ---")
    
    # Official rules: Max 10 points for the ENTIRE mission
    total_max_score = 10.0 
    total_penalty = 0.0
    
    for i in range(len(converted_waypoints)):
        dist = closest_approaches[i]
        # Official rules: Deduct 1 point for every meter over 1 meter
        penalty = max(0.0, dist - 1.0)
        total_penalty += penalty
        
        print(f"Waypoint {i+1}: Closest 3D Approach = {dist:.2f}m | Penalty = -{penalty:.2f} pts")
    
    # Prevent negative scores
    estimated_final_score = max(0.0, total_max_score - total_penalty)
    
    print(f"--------------------------------")
    print(f"Total Penalty Incurred: -{total_penalty:.2f} Points")
    print(f"ESTIMATED TOTAL POINTS: {estimated_final_score:.2f} / {total_max_score:.2f} Points")
    
    # --- SAVE TO CSV ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    os.makedirs(output_dir, exist_ok=True)
    
    csv_filename = os.path.join(output_dir, "MSU_CUASC_WAYPOINT_NAVIGATION.csv")
    
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Waypoint_Index", "Target_Lat", "Target_Lon", "Target_Alt_Meters", "Closest_3D_Approach_M"])
        for i in range(len(converted_waypoints)):
            t_lat, t_lon, t_alt_m = converted_waypoints[i]
            writer.writerow([i+1, t_lat, t_lon, round(t_alt_m, 2), round(closest_approaches[i], 2)])
            
    print(f"\nReport saved successfully to: {csv_filename}")

if __name__ == "__main__":
    DOWNLOADS_DIR = r"C:\Users\ishan\Downloads"
    BIN_FILE_NAME = "00000030.BIN" 
    BIN_FILE_PATH = os.path.join(DOWNLOADS_DIR, BIN_FILE_NAME)
    ALTITUDES_ARE_FEET = True 
    
    SCORING_WAYPOINTS = [
        (35.049875, -118.150127, 50),
        (35.050154, -118.149204, 150), 
        (35.052716, -118.150231, 100),
        (35.050101, -118.150687, 50),
        (35.048860, -118.153033, 100),
        (35.047553, -118.151674, 100), 
        (35.048843, -118.151194, 50)
    ]
    
    analyze_blackbox(BIN_FILE_PATH, SCORING_WAYPOINTS, ALTITUDES_ARE_FEET)