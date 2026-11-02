import sys
import os
import time

# Route to sibling folder
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from competition.drone_simulator.vehicle import commands
from competition.drone_simulator.vehicle.commands import CommandError
from competition.drone_simulator.vehicle.link import Link, LinkConfig

def run_barebone_test():
    print("\n=== NAKED CUBE I/O TEST ===")

    # Connect to the Cube
    link = Link.connect(LinkConfig("/dev/ttyTHS1", baud=921600))
    try:
        print("[SUCCESS] Data link established.")

        # --- TEST 1: PIN 6 (CUBE DROP) ---
        print("\n[TEST 1] Firing Pin 6 to 1900 PWM...")
        print(">>> Look at 'ch6out' in Mission Planner! It should jump to 1900.")
        try:
            commands.set_servo(link, 6, 1900)
        except CommandError as e:
            print(f"[WARNING] Pin 6 not confirmed: {e}")
        time.sleep(5)

        # --- TEST 2: PIN 7 (BALL DROP) ---
        print("\n[TEST 2] Firing Pin 7 to 1900 PWM...")
        print(">>> Look at 'ch7out' in Mission Planner! It should jump to 1900.")
        try:
            commands.set_servo(link, 7, 1900)
        except CommandError as e:
            print(f"[WARNING] Pin 7 not confirmed: {e}")
        time.sleep(5)

        # --- TEST 3: THE MODE HANDOFF ---
        print("\n[TEST 3] Forcing ArduPilot into LOITER mode...")
        try:
            commands.set_mode(link, "LOITER")
        except CommandError as e:
            # e.g. LOITER needs a GPS position estimate, which a bench Cube may not have.
            print(f"[WARNING] LOITER not confirmed: {e}")
        print(">>> Look at the Mission Planner HUD! The flight mode should change to LOITER.")

        print("\n[SUCCESS] I/O Test Complete. If all 3 tests matched in Mission Planner, your MAVLink bridge is flawless.")
    finally:
        link.close()

if __name__ == "__main__":
    run_barebone_test()
