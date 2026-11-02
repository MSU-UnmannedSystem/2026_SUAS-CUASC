import os
import sys
import csv
import math
from pymavlink import mavutil

# ==========================================
# C-UASC 2026: MSU UNMANNED SYSTEMS MASTER CONFIG
# ==========================================

DOWNLOADS_DIR = r"C:\Users\ishan\Downloads"
WP_NAV_BIN = os.path.join(DOWNLOADS_DIR, "00000030.BIN")
TIME_TRIAL_BIN = os.path.join(DOWNLOADS_DIR, "00000031.BIN")
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")

# Waypoint Data
ALTITUDES_ARE_FEET = True
SCORING_WAYPOINTS = [
    (35.049875, -118.150127, 50),
    (35.050154, -118.149204, 150),
    (35.052716, -118.150231, 100), # WP 3 (Start)
    (35.050101, -118.150687, 50),
    (35.048860, -118.153033, 100), # WP 5 (End)
    (35.047553, -118.151674, 100),
    (35.048843, -118.151194, 50)
]
START_WP_IDX = 2 
END_WP_IDX = 4

# Time Trial Judge Data (Fastest/Slowest run of the day in seconds)
TT_FASTEST_SEC = None
TT_SLOWEST_SEC = None

# Package Drop (Ball Drop) Data
BALL_DROP_ERROR_FEET = 11.0
BALL_DROP_BELOW_6FT = False # Set to True if drone dipped below 6ft altitude

# Package Delivery (Box Drop) Data
# BOX_DROP_ERROR_FEET = None
# BOX_IMPULSE_EXCEEDED = False
# BOX_DROPPED_ABOVE_6FT_NO_PARACHUTE = False
# CUBE_ADDED_WEIGHT_GRAMS = 0

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def get_converted_waypoints():
    converted = []
    for lat, lon, alt in SCORING_WAYPOINTS:
        final_alt = alt * 0.3048 if ALTITUDES_ARE_FEET else alt
        converted.append((lat, lon, final_alt))
    return converted

def calculate_3d_dist(lat1, lon1, alt1_m, lat2, lon2, alt2_m):
    lat_dif_m = (abs(lat1 - lat2)) * 111319
    lon_dif_m = (abs(lon1 - lon2)) * 111319 * math.cos(math.radians(lat2))
    alt_dif_m = abs(alt1_m - alt2_m)
    return (lat_dif_m**2 + lon_dif_m**2 + alt_dif_m**2)**0.5

def read_log_telemetry(bin_path, waypoints):
    if not os.path.exists(bin_path):
        return None, None
        
    try:
        mlog = mavutil.mavlink_connection(bin_path)
    except Exception:
        return None, None

    closest_approaches = {i: 99999999.0 for i in range(len(waypoints))}
    wp_hit_times_us = {i: [] for i in range(len(waypoints))}
    
    while True:
        msg = mlog.recv_match(type=['POS', 'GLOBAL_POSITION_INT'], blocking=False)
        if msg is None:
            break 
            
        time_us = getattr(msg, 'TimeUS', getattr(msg, 'time_boot_ms', 0) * 1000)
        lat = msg.Lat if hasattr(msg, 'Lat') else msg.lat / 1e7
        lon = msg.Lng if hasattr(msg, 'Lng') else msg.lon / 1e7
        alt_agl = msg.RelHomeAlt if hasattr(msg, 'RelHomeAlt') else msg.relative_alt / 1000.0

        for i, (t_lat, t_lon, t_alt_m) in enumerate(waypoints):
            dist = calculate_3d_dist(t_lat, t_lon, t_alt_m, lat, lon, alt_agl)
            if dist < closest_approaches[i]:
                closest_approaches[i] = dist
            if dist <= 4.0 and time_us > 0:
                wp_hit_times_us[i].append(time_us)
                
    return closest_approaches, wp_hit_times_us

# ==========================================
# SCORING ENGINES
# ==========================================

def score_waypoint_nav():
    print("Processing Waypoint Navigation...")
    wps = get_converted_waypoints()
    approaches, _ = read_log_telemetry(WP_NAV_BIN, wps)
    
    if not approaches:
        return "ERROR: File Not Found", 0.0, []

    total_max_score = 10.0 
    total_penalty = 0.0
    csv_data = []
    
    for i in range(len(wps)):
        dist = approaches[i]
        penalty = max(0.0, dist - 1.0)
        total_penalty += penalty
        csv_data.append([i+1, wps[i][0], wps[i][1], round(wps[i][2], 2), round(dist, 2)])
        
    final_score = max(0.0, total_max_score - total_penalty)
    report = f"Total Penalty: -{total_penalty:.2f} pts\nFinal Score: {final_score:.2f} / 10.00 pts"
    
    return report, final_score, csv_data

def score_time_trial():
    print("Processing Circuit Time Trial...")
    wps = get_converted_waypoints()
    approaches, hit_times = read_log_telemetry(TIME_TRIAL_BIN, wps)
    
    if not approaches:
        return "ERROR: File Not Found", 0.0, []

    csv_data = []
    run_valid = True
    for i in range(len(wps)):
        cleared = "YES" if approaches[i] <= 4.0 else "NO"
        if approaches[i] > 4.0: run_valid = False
        csv_data.append([i+1, wps[i][0], wps[i][1], round(wps[i][2], 2), round(approaches[i], 2), cleared])

    if not run_valid:
        return "STATUS: INVALID RUN (Missed 4m Zone)", 0.0, csv_data

    if hit_times[START_WP_IDX] and hit_times[END_WP_IDX]:
        start_t = hit_times[START_WP_IDX][0]
        valid_ends = [t for t in hit_times[END_WP_IDX] if t > start_t]
        
        if valid_ends:
            total_sec = (valid_ends[0] - start_t) / 1000000.0
            score = 0.0
            score_txt = "[Awaiting Judge Bounds]"
            
            if TT_FASTEST_SEC and TT_SLOWEST_SEC:
                if total_sec <= TT_FASTEST_SEC: score = 12.0
                elif total_sec >= TT_SLOWEST_SEC: score = 8.0
                else:
                    ratio = (total_sec - TT_FASTEST_SEC) / (TT_SLOWEST_SEC - TT_FASTEST_SEC)
                    score = 12.0 - (4.0 * ratio)
                score_txt = f"{score:.2f} / 12.00 pts"
                
            report = f"Flight Time: {total_sec:.2f}s\nEstimated Score: {score_txt}"
            return report, score, csv_data
            
    return "STATUS: Start/End gates not sequentially cleared.", 0.0, csv_data

def score_ball_drop():
    print("Processing Package Drop (Ball)...")
    error_meters = BALL_DROP_ERROR_FEET * 0.3048
    
    max_score = 14.0
    penalty = error_meters * 1.0 # 1 point per meter
    
    score = max(0.0, max_score - penalty)
    if BALL_DROP_BELOW_6FT:
        score -= 6.0
        
    score = max(0.0, score) # No negative scores
    report = f"Distance from center: {BALL_DROP_ERROR_FEET}ft ({error_meters:.2f}m)\nFinal Score: {score:.2f} / 14.00 pts"
    return report, score

# def score_box_drop():
#     print("Processing Package Delivery (Box)...")
#     if BOX_DROP_ERROR_FEET is None:
#         return "STATUS: No Data Provided", 0.0
#     
#     error_meters = BOX_DROP_ERROR_FEET * 0.3048
#     max_score = 13.0
#     penalty = error_meters * 1.0
#     score = max(0.0, max_score - penalty)
#     
#     if BOX_IMPULSE_EXCEEDED: score -= 6.0
#     if BOX_DROPPED_ABOVE_6FT_NO_PARACHUTE: score = 0.0
#     
#     bonus = (CUBE_ADDED_WEIGHT_GRAMS // 500) * 0.25
#     score += bonus
#     
#     report = f"Distance from center: {BOX_DROP_ERROR_FEET}ft\nFinal Score: {score:.2f} pts"
#     return report, score

# ==========================================
# MAIN EXECUTION
# ==========================================

if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    wp_report, wp_score, wp_csv = score_waypoint_nav()
    tt_report, tt_score, tt_csv = score_time_trial()
    ball_report, ball_score = score_ball_drop()
    # box_report, box_score = score_box_drop()
    
    # Write CSVs
    if wp_csv:
        with open(os.path.join(OUTPUT_DIR, "MSU_CUASC_WAYPOINT_NAVIGATION.csv"), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["Waypoint_Index", "Target_Lat", "Target_Lon", "Target_Alt_Meters", "Closest_3D_Approach_M"])
            w.writerows(wp_csv)
            
    if tt_csv:
        with open(os.path.join(OUTPUT_DIR, "MSU_CUASC_TIME_TRIAL.csv"), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["Waypoint_Index", "Target_Lat", "Target_Lon", "Target_Alt_Meters", "Closest_3D_Approach_M", "Zone_4M_Cleared"])
            w.writerows(tt_csv)

    # Write Master Summary
    summary_path = os.path.join(OUTPUT_DIR, "MSU_CUASC_MASTER_SUMMARY.txt")
    with open(summary_path, 'w') as f:
        f.write("==========================================\n")
        f.write(" MSU UNMANNED SYSTEMS: FLIGHT LINE REPORT\n")
        f.write("==========================================\n\n")
        
        f.write("--- 1. WAYPOINT NAVIGATION ---\n")
        f.write(wp_report + "\n\n")
        
        f.write("--- 2. CIRCUIT TIME TRIAL ---\n")
        f.write(tt_report + "\n\n")
        
        f.write("--- 3. PACKAGE DROP (BALL) ---\n")
        f.write(ball_report + "\n\n")
        
        f.write("--- 4. PACKAGE DELIVERY (BOX) ---\n")
        f.write("STATUS: Data Pending\n\n")
        # f.write(box_report + "\n\n")
        
        total_known = wp_score + tt_score + ball_score
        f.write("==========================================\n")
        f.write(f" CURRENT KNOWN TOTAL: {total_known:.2f} Points\n")
        f.write("==========================================\n")

    print(f"\n[SUCCESS] Master analysis complete. Check the 'outputs' folder for CSVs and the Summary Text file.")