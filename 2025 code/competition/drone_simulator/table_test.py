import sys
import os
import time

# Route to sibling folder
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from competition.drone_simulator.vehicle import commands
from competition.drone_simulator.vehicle.commands import CommandError
from competition.drone_simulator.vehicle.link import Link, LinkConfig
from competition.jetson_rescue.camera import VisionPipeline
from competition.drone_simulator.vehicle.upload import upload_waypoint_mission
from competition.drone_simulator.config.mission_params import parameters

TELEMETRY_MAX_AGE_S = 1.0  # older position counts as no data


def try_set_mode(link, mode):
    """Bench helper: a rejected mode change (e.g. no GPS indoors) is printed, not fatal."""
    try:
        commands.set_mode(link, mode)
    except CommandError as e:
        print(f"\n[WARNING] Mode change to {mode} not confirmed: {e}")


def try_set_servo(link, pin, pwm):
    """Bench helper: a rejected or unconfirmed servo command is printed, not fatal."""
    try:
        commands.set_servo(link, pin, pwm)
    except CommandError as e:
        print(f"\n[WARNING] Servo {pin} -> {pwm} not confirmed: {e}")


def run_table_test():
    print("\n=== MANUAL TABLE TEST (HARDWARE-IN-THE-LOOP) ===")

    # NOTE: Change to /dev/ttyTHS0 if ttyTHS1 throws a permission/not found error
    link = Link.connect(LinkConfig("/dev/ttyTHS1", baud=921600))
    try:
        vision = VisionPipeline(camera_index=0, target_type='red_bullseye')
        config = parameters()

        cube_dropped = False
        ball_dropped = False
        target_lost_timer = None

        # --- PHASE 0: WAYPOINT UPLOAD TEST ---
        print("\n[PHASE 0] Testing MAVLink Waypoint Transfer...")
        try:
            upload_waypoint_mission(link, config["waypoints"])
            print(f"[SUCCESS] {len(config['waypoints'])} Competition Waypoints locked into Cube memory.")
        except Exception as e:
            print(f"[ERROR] Waypoint upload failed: {e}")
            sys.exit(1)

        print("\n[READY] Hold the drone over the target. Monitoring telemetry...\n")

        try:
            while not (cube_dropped and ball_dropped):
                # 1. Read the Camera
                x, y = vision.get_target_pixel()

                # 2. Read the Barometer (Relative Altitude)
                msg = link.latest_fresh("GLOBAL_POSITION_INT", TELEMETRY_MAX_AGE_S)
                alt = msg.relative_alt / 1000.0 if msg else 0.0

                # 3. Check Centering (1080p center is ~960, 540. Allow a +/- 150px window)
                centered = False
                if x is not None and y is not None:
                    if 810 < x < 1110 and 390 < y < 690:
                        centered = True
                        target_lost_timer = None # Reset the panic timer when centered

                # 4. The Active Auto-Failsafe (Vision Loss)
                if not centered:
                    if target_lost_timer is None:
                        target_lost_timer = time.time()
                    elif time.time() - target_lost_timer > 4.0:
                        print("\n\n[CRITICAL ERROR] Vision lock lost for 4 seconds!")
                        print("[AUTO-FAILSAFE] Forcing ArduPilot into LOITER mode...")
                        try_set_mode(link, "LOITER")
                        print("[SAFEGUARD] Terminating companion computer loop immediately.")
                        vision.release()
                        sys.exit(0)

                # 5. Print the Live HUD
                print(f"| ALT: {alt:.2f}m | PIXEL: ({x}, {y}) | CENTERED: {centered} |", end='\r')

                # 6. The Floor Test (Simulating Phase 1 Cube Drop at 0.5m)
                if centered and alt < 0.5 and not cube_dropped:
                    print(f"\n\n[TRIGGER] Cube condition met (Alt < 0.5m). Actuating Pin 6 (1900 PWM)...")
                    try_set_servo(link, 6, 1900)
                    cube_dropped = True
                    target_lost_timer = None
                    time.sleep(2)

                # 7. The Ceiling Test (Simulating Phase 2 Ball Drop at 1.5m)
                if centered and alt > 1.5 and not ball_dropped:
                    print(f"\n\n[TRIGGER] Ball condition met (Alt > 1.5m). Actuating Pin 7 (1900 PWM)...")
                    try_set_servo(link, 7, 1900)
                    ball_dropped = True
                    target_lost_timer = None
                    time.sleep(2)

                time.sleep(0.1)

            # --- PHASE 3: THE AUTO HANDOFF TEST ---
            print("\n\n=== PHASE 3: WAYPOINT HANDOFF TEST ===")
            print("Payloads dropped. Jetson is commanding ArduPilot to enter AUTO mode...")
            try_set_mode(link, "AUTO")
            time.sleep(2) # Give FC time to process the mode change

            if link.latest_fresh("HEARTBEAT", 2.0) is not None:
                current_mode = commands.current_mode(link)
                print(f"[VERIFY] ArduPilot is currently in: {current_mode} mode.")
                if current_mode == "AUTO":
                    print("[SUCCESS] Waypoint logic verified! ArduPilot accepted the route.")
                else:
                    print("[WARNING] ArduPilot rejected AUTO mode. (This is normal indoors without a 3D GPS lock).")

            # --- PHASE 4: THE RC OVERRIDE TEST ---
            print("\n=== PHASE 4: FAILSAFE OVERRIDE TEST ===")
            print("-> SAFETY PILOT: Flip your flight mode switch to LOITER now!")

            while True:
                if link.latest_fresh("HEARTBEAT", 2.0) is not None:
                    current_mode = commands.current_mode(link)
                    print(f"Waiting for override... Current Mode: {current_mode}        ", end='\r')

                    if current_mode == "LOITER":
                        print(f"\n\n[CRITICAL] RC Pilot Takeover Detected! Mode Switched to: {current_mode}")
                        print("[SAFEGUARD] Terminating companion computer loop immediately.")
                        print("-> PILOT HAS 100% MANUAL CONTROL.")
                        vision.release()
                        sys.exit(0)

                time.sleep(0.2)

        except KeyboardInterrupt:
            vision.release()
            print("\n\n[SHUTDOWN] Test terminated safely by user.")
    finally:
        link.close()

if __name__ == "__main__":
    run_table_test()
