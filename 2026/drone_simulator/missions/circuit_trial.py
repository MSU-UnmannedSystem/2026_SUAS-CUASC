from pymavlink import mavutil
import math
import csv
import os
import sys

def analyze_time_trial(bin_filepath, scoring_waypoints, altitudes_in_feet, start_idx, end_idx, comp_fast, comp_slow):
    print(f"=== C-UASC 2026: CIRCUIT TIME TRIAL ANALYZER ===")
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
    wp_hit_times_us = {i: [] for i in range(len(converted_waypoints))}
    
    print(f"Scanning telemetry against {len(converted_waypoints)} targets (4-Meter Validation Zone)...")
    
    gps_count = 0
    while True:
        msg = mlog.recv_match(type=['POS', 'GLOBAL_POSITION_INT'], blocking=False)
        if msg is None:
            break 
        
        time_us = getattr(msg, 'TimeUS', getattr(msg, 'time_boot_ms', 0) * 1000)
        if time_us == 0:
            continue

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
            
            if distance_3d <= 4.0:
                wp_hit_times_us[i].append(time_us)

    # --- VALIDATE RUN & CALCULATE TIME ---
    print("\n--- FINAL CIRCUIT TIME TRIAL RESULTS ---")
    
    run_valid = True
    for i in range(len(converted_waypoints)):
        dist = closest_approaches[i]
        status = "PASS" if dist <= 4.0 else "FAIL (INVALID RUN)"
        if dist > 4.0: run_valid = False
        print(f"Waypoint {i+1}: Closest Approach = {dist:.2f}m | Status: {status}")

    if not run_valid:
        print("\n[!] WARNING: One or more waypoints were not passed within 4 meters. Run is invalid.")
        return # Halt scoring logic if run is invalid

    print("\n[SUCCESS] All 7 Waypoints successfully cleared within 4 meters.")

    if wp_hit_times_us[start_idx] and wp_hit_times_us[end_idx]:
        start_t = wp_hit_times_us[start_idx][0] 
        valid_end_times = [t for t in wp_hit_times_us[end_idx] if t > start_t]
        
        if valid_end_times:
            end_t = valid_end_times[0]
            total_time_seconds = (end_t - start_t) / 1000000.0
            print(f"Mission Clock Started: Waypoint {start_idx + 1}")
            print(f"Mission Clock Stopped: Waypoint {end_idx + 1}")
            print(f"--------------------------------")
            print(f"OFFICIAL CIRCUIT TIME: {total_time_seconds:.2f} Seconds")
            
            # Linear Scoring Matrix (Fastest = 12pts, Slowest = 8pts)
            if comp_fast and comp_slow and comp_slow > comp_fast:
                if total_time_seconds <= comp_fast:
                    score = 12.0
                elif total_time_seconds >= comp_slow:
                    score = 8.0
                else:
                    ratio = (total_time_seconds - comp_fast) / (comp_slow - comp_fast)
                    score = 12.0 - (4.0 * ratio)
                print(f"ESTIMATED TOTAL POINTS: {score:.2f} / 12.00 Points")
            else:
                print("ESTIMATED TOTAL POINTS: [Awaiting Judge Data for Fastest/Slowest Times]")
        else:
            print("\n[ERROR] Drone hit the End Waypoint before the Start Waypoint.")
    else:
         print("\n[ERROR] Start or End Waypoint was never successfully hit.")

    # --- SAVE TO CSV ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "output")
    os.makedirs(output_dir, exist_ok=True)
    
    csv_filename = os.path.join(output_dir, "MSU_CUASC_TIME_TRIAL.csv")
    
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Waypoint_Index", "Target_Lat", "Target_Lon", "Target_Alt_Meters", "Closest_3D_Approach_M", "Zone_4M_Cleared"])
        for i in range(len(converted_waypoints)):
            t_lat, t_lon, t_alt_m = converted_waypoints[i]
            cleared = "YES" if closest_approaches[i] <= 4.0 else "NO"
            writer.writerow([i+1, t_lat, t_lon, round(t_alt_m, 2), round(closest_approaches[i], 2), cleared])
            
    print(f"\nReport saved successfully to: {csv_filename}")

if __name__ == "__main__":
    DOWNLOADS_DIR = r"C:\Users\ishan\Downloads"
    BIN_FILE_NAME = "00000031.BIN" 
    BIN_FILE_PATH = os.path.join(DOWNLOADS_DIR, BIN_FILE_NAME)
    ALTITUDES_ARE_FEET = True 
    
    SCORING_WAYPOINTS = [
        (35.049875, -118.150127, 50),
        (35.050154, -118.149204, 150), 
        (35.052716, -118.150231, 100), # Waypoint 3 (Index 2)
        (35.050101, -118.150687, 50),
        (35.048860, -118.153033, 100), # Waypoint 5 (Index 4)
        (35.047553, -118.151674, 100), 
        (35.048843, -118.151194, 50)
    ]
    
    START_WAYPOINT_INDEX = 2 
    END_WAYPOINT_INDEX = 4 

    # --- JUDGE SCORING VARIABLES ---
    # Update these numbers when the judges announce the fastest and slowest times (in seconds).
    # Leave them as None if you just want to calculate your raw time without points.
    COMPETITION_FASTEST_TIME_SEC = None 
    COMPETITION_SLOWEST_TIME_SEC = None 

    analyze_time_trial(BIN_FILE_PATH, SCORING_WAYPOINTS, ALTITUDES_ARE_FEET, START_WAYPOINT_INDEX, END_WAYPOINT_INDEX, COMPETITION_FASTEST_TIME_SEC, COMPETITION_SLOWEST_TIME_SEC)