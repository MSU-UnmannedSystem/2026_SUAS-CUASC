def parameters():
    return {
        # --- ACTUAL COMPETITION FLIGHT PARAMS ---
        "takeoff_alt": 16.0,  # 52.5 ft hard deck clearance
        "max_alt": 60.0,      # High ceiling for safety margin
        "min_alt": 0.5,       # Allowed to get very close to ground (landing logic overrides this)
        # Takeoff climb rate in m/s = the Cube's WP_SPD_UP (m/s; WPNAV_SPEED_UP in cm/s on older
        # firmware). Only sizes the takeoff timeout. 2.5 matches SITL's default, NOT yet read
        # from our Cube; on the Jetson run:
        # run `python3 competition/drone_simulator/tools/sitl_link_check.py params --device /dev/ttyTHS1`
        "climb_rate_m_s": 2.5,

        # --- 10-FOOT TEST FLIGHT PARAMS (Commented Out) ---
        # "takeoff_alt": 15.24,
        # "max_alt": 16.76,  # 50 ft ceiling for testing

        "waypoints": [
            (35.049875, -118.150127, 50),
            (35.050154, -118.149204, 150), 
            (35.052716, -118.150231, 100),
            (35.050101, -118.150687, 50),
            (35.048860, -118.153033, 100),
            (35.047553, -118.151674, 100), 
            (35.048843, -118.151194, 50)
        ],
        "time_limit": 600,
        "end_mode": "RTL",
        "geofence": (35.04709, 35.05322, -118.15354, -118.14854)
    }