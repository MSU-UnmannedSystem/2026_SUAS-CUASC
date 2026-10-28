import os
import queue
import sys
import time
import random as rand
import math

from pymavlink import mavutil

# Run as a plain script: put "2025 code" on sys.path for the package imports.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from competition.drone_simulator.vehicle import commands
from competition.drone_simulator.vehicle.commands import CommandError
from competition.drone_simulator.vehicle.link import Link, LinkConfig
from competition.drone_simulator.vehicle.upload import MissionError, upload_waypoint_mission

mavlink = mavutil.mavlink


options = "a: Make drone fly\n" \
"b: Make drone land\n" \
"c: Return to location\n" \
"d: Make drone fly forward\n" \
"t: Traverse payload drop area\n" \
"e: Terminate program\n" \
"f: Print options\n"

def prompt_user(link):
    """Prompts the user for a choice from the options variable"""

    choice = input("What would you like to do: ")

    if choice == 'a':
        make_drone_fly_up(link)
        return 1
    elif choice == 'b':
        land_drone(link)
        return 1
    elif choice == 'c':
        return_to_location(link)
        return 1
    elif choice == 'd':
        make_drone_fly_forward(link)
        return 1
    elif choice == 'e':
        print("\nProgram terminating...")
        return None
    elif choice == 'f':
        print(options)
        return 1
    elif choice == 't':
        traverse_payload(link)
        return 1
    else:
        return None

def arm_drone(link):
    """"Arms the drone when ready to take off"""

    print("arming.....")
    commands.arm(link)
    print("Motors armed")

def change_mode(mode, link):
    commands.set_mode(link, mode)

def _next_message(sub, link):
    """Blocks until the next message from the vehicle arrives on ``sub``."""
    while True:
        try:
            msg = sub.get(timeout=1.0)
        except queue.Empty:
            continue
        if msg.get_srcSystem() == link.target_system:
            return msg

def make_drone_fly_up(link, flight_height=0):
    """"Makes the drone fly (uses arm and changed mode function)"""

    #changed Mode to guided so it can fly upwards and arms drone
    change_mode("GUIDED", link)
    arm_drone(link)
    #sets the baseline to 0 until first itteration
    base_line = None

    #ask for user input for what height the drone should fly to
    if flight_height == 0:
        flight_height = int(input("Target altitude: "))

    #subscribe before the take off command so no VFR_HUD is missed
    with link.subscribe("VFR_HUD") as hud:
        #starts the take off command
        commands.command_long(link, mavlink.MAV_CMD_NAV_TAKEOFF,
                              [0, 0, 0, 0, 0, 0, flight_height])

        #starting altitude
        print("Taking off")
        print("Altitude: 0.00m")

        #while loop that prints off the current altitude for the drone
        #itterates unitl drone reaches target height
        while True:

            msg = _next_message(hud, link)

            if not msg:
                continue

            if base_line is None:
                base_line = msg.alt
                altitude = msg.alt

            altitude = msg.alt - base_line
            #groundspeed = msg.groundspeed
            if (flight_height - (0.01)) <= altitude:
                print("Altitude Reached!\n")
                break
            elif altitude < 0.01:
                continue
            else:
                print(f"Altitude: {altitude:.2f}m")

def land_drone(link):
    """"Lands the Drone"""
    print("Landing...\n")
    with link.subscribe("GLOBAL_POSITION_INT") as positions:
        change_mode("LAND", link)

        while True:
            msg = _next_message(positions, link)

            alt = msg.relative_alt/1000.0
            if alt < 0.01:
                print("landed!!!\n")
                break

def make_drone_fly_forward(link):
    """Makes the drone fly forward with proper mission upload protocol"""

    print("Starting mission upload...")

    # The upload replaces any existing mission (seq 0 = home, seq 1 = the waypoint).
    num_wp = 1
    print(f"Sending mission count: {num_wp}")

    try:
        flight_height = int(input("Target altitude: "))

        print("Sending mission item...")
        try:
            upload_waypoint_mission(
                link,
                [(42.7782989, -84.5959282, flight_height)],  # lat, lon, alt
                end_action=None,
            )
        except (MissionError, ValueError) as e:
            print(f"Mission upload failed with error: {e}")
            return False

        print("Mission upload successful!")

        # Now set mode and arm
        make_drone_fly_up(link, flight_height)
        change_mode("AUTO", link)

        print("Flying to Waypoint")

    except Exception as e:
        print(f"Error during mission upload: {e}")

def return_to_location(link):
    change_mode("RTL", link)

def traverse_payload(link):
    random_lattitude = rand.randint(int(35.05935 * 1e7), int(35.06012 * 1e7))
    random_longitude = rand.randint(int(-118.160 * 1e7), int(-118.158 * 1e7))

    flight_height = 30
    tolerance_meters = 5
    change_mode("STABILIZE", link)
    arm_drone(link)
    make_drone_fly_up(link, flight_height)
    change_mode("AUTO", link)

    print(random_lattitude, ", ", random_longitude)
    with link.subscribe("GLOBAL_POSITION_INT") as positions:
        while True:
            msg = _next_message(positions, link)

            latitude_dif_meters = (abs(random_lattitude - msg.lat) / 1e7) * 111319
            longitude_dif_meters = (abs(random_longitude - msg.lon) / 1e7) * 111319* math.cos(math.radians(35))

            distance = (latitude_dif_meters**2 + longitude_dif_meters**2)**(1/2)
            print(distance, ", ", msg.lat, ", ", msg.lon)
            if (distance <= tolerance_meters):
                print("Dropping Payload")
                print("Returning to launch")
                return_to_location(link)
                break



#Main Function
#Initalizes Mavlink connection to SITL (Software In The Loop)
def main():
    print("Waiting for heartbeat...")

    link = Link.connect(LinkConfig("udp:127.0.0.1:14553"))
    try:
        print(f"Connected to system {link.target_system}, component {link.target_component}")

        print(options)
        while True:
            try:
                response = prompt_user(link)
            except CommandError as e:
                print(f"Command failed: {e}")
                continue

            if response:
                continue
            else:
                break
    finally:
        link.close()





if __name__ == "__main__":
    main()
