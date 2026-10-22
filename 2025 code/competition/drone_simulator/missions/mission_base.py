from pymavlink import mavutil
import time
import threading
import sys
from competition.drone_simulator.vehicle.modes import change_mode, arm_drone
from competition.drone_simulator.vehicle.upload import upload_waypoints

# DEPRECATED: replaced by vehicle/link.py (Link). Remove when callers migrate (step 4).
class HeartbeatThread(threading.Thread):
    """
    Sends MAVLink HEARTBEAT to the FC at HEARTBEAT_HZ continuously.
    Must be alive for the entire flight. Stopping it arms the GCS failsafe.
    """
    HEARTBEAT_HZ = 2

    def __init__(self, master):
        super().__init__(daemon=True)
        self.master = master
        self._stop_event = threading.Event()

    def run(self):
        period = 1.0 / self.HEARTBEAT_HZ
        while not self._stop_event.is_set():
            self.master.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_GCS,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            time.sleep(period)

    def stop(self):
        self._stop_event.set()


class MissionBase():

    # If a safety officer takes over via RC, we log it and wait rather than abort.
    RC_OVERRIDE_MODES = {"STABILIZE", "ALT_HOLD", "LOITER", "LAND"}

    def __init__(self, master, config):
        self.master = master
        self.config = config
        self.takeoff_alt = None
        self.waypoints = None
        self.active = False
        self.start_time = None
        self.time_limit = self.config.get("time_limit", 600)
        self.last_waypoint = None
        self.last_heartbeat = time.time()
        self.expected_mode = "AUTO"
        self.max_alt = self.config.get("max_alt", 120)
        self.min_alt = self.config.get("min_alt", 5)
        self.geofence = self.config.get("geofence", None)
        self.end_mode = self.config.get("end_mode", "RTL")

        self._landing = False

        self._heartbeat_thread = None

    def run(self):
        """Main mission entry point"""
        try:
            self.setup()
            self.arm_and_takeoff()
            self.execute()

            self.start_time = time.time()
            self.active = True

            while self.active:
                self.update_heartbeat()
                self.monitor()
                self.check_time_limit()
                self.check_safety()
                time.sleep(0.2)

            self.end_mission()

        except Exception as e:
            print(f"Mission failed: {e}")
            self.abort()

    def setup(self):
        """Prepare mission — validates config, starts heartbeat, uploads waypoints."""

        self._heartbeat_thread = HeartbeatThread(self.master)
        self._heartbeat_thread.start()
        print(f"[BASE] GCS heartbeat thread started @ {HeartbeatThread.HEARTBEAT_HZ} Hz")

        if "takeoff_alt" not in self.config:
            raise ValueError("Missing takeoff_alt in mission config")

        self.takeoff_alt = self.config["takeoff_alt"]

        if self.takeoff_alt <= 0:
            raise ValueError("takeoff_alt must be > 0")

        if "waypoints" not in self.config:
            raise ValueError("Missing waypoints in mission config")

        self.waypoints = self.config["waypoints"]

        if not isinstance(self.waypoints, list) or len(self.waypoints) == 0:
            raise ValueError("Waypoints must be a non-empty list")

        upload_waypoints(self.master, self.waypoints)
        print(f"[BASE] Waypoints uploaded. Target alt: {self.takeoff_alt}m | "
              f"Waypoints: {len(self.waypoints)}")

    def execute(self):
        """Switch to AUTO and wait for FC confirmation."""
        print("[BASE] Starting mission — switching to AUTO")
        change_mode(self.master, "AUTO")
        self.set_expected_mode("AUTO")

        start = time.time()
        while True:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb:
                mode_str = mavutil.mode_string_v10(hb)
                if mode_str == "AUTO":
                    print("[BASE] AUTO mode confirmed")
                    break
            if time.time() - start > 10:
                raise RuntimeError("Failed to enter AUTO mode within 10 seconds")

    def monitor(self):
        """
        Monitor waypoint progress.
        """
        msg = self.master.recv_match(type="MISSION_ITEM_REACHED", blocking=False)

        if not msg:
            return

        reached = msg.seq
        final_waypoint = len(self.waypoints) - 1

        print(f"[BASE] Waypoint reached: {reached}/{final_waypoint}")

        if reached >= final_waypoint:
            print("[BASE] All waypoints reached — mission complete")
            self.active = False

    def arm_and_takeoff(self):
        """Standard arming and takeoff sequence."""
        change_mode(self.master, "GUIDED")
        self.set_expected_mode("GUIDED")
        arm_drone(self.master)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.takeoff_alt
        )

        start = time.time()
        timeout = 30

        while True:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)

            if msg:
                alt = msg.relative_alt / 1000.0
                print(f"[BASE] Altitude: {round(alt, 2)} m")

                if alt >= self.takeoff_alt * 0.95:
                    print("[BASE] Target altitude reached")
                    break

            if time.time() - start >= timeout:
                raise RuntimeError("Takeoff timeout")

            time.sleep(0.1)

    def check_time_limit(self):
        """Enforce competition time limit."""
        if self.start_time is None:
            return

        elapsed = time.time() - self.start_time
        if elapsed >= self.time_limit:
            print(f"[BASE] Time limit reached ({self.time_limit}s)")
            raise RuntimeError("Time limit exceeded")

    def abort(self):
        """Failsafe — stop heartbeat then RTL/LAND."""
        print("[BASE] Aborting mission")
        self._stop_heartbeat()
        self.end_mission()

    def check_safety(self):
        """
        Safety checks: link loss, mode guard, altitude bounds, geofence.
        """
        hb = self.master.recv_match(type="HEARTBEAT", blocking=False)

        if hb:
            self.last_heartbeat = time.time()
            current_mode = mavutil.mode_string_v10(hb)

            if current_mode != self.expected_mode:
                if current_mode in self.RC_OVERRIDE_MODES:
                    # FIX: Total script termination to ensure RC safety pilot has 100% control
                    print(f"\n[CRITICAL] RC Pilot Takeover Detected! Mode Switched to: {current_mode}")
                    print("[SAFEGUARD] Terminating companion computer guidance loop immediately.")
                    self.active = False
                    sys.exit(0)
                else:
                    raise RuntimeError(
                        f"Unexpected mode change: expected={self.expected_mode}, "
                        f"got={current_mode}"
                    )
        else:
            # Check for link loss
            if time.time() - self.last_heartbeat > 5:
                raise RuntimeError("Lost vehicle heartbeat — link failure")

        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            alt = msg.relative_alt / 1000.0

            # Skip altitude floor check during landing/RTL descent
            if not self._landing and alt < self.min_alt:
                raise RuntimeError(f"Altitude below minimum ({alt:.1f}m < {self.min_alt}m)")

            if alt > self.max_alt:
                raise RuntimeError(f"Altitude above maximum ({alt:.1f}m > {self.max_alt}m)")

            if self.geofence:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                lat_min, lat_max, lon_min, lon_max = self.geofence
                if not (lat_min <= lat <= lat_max and lon_min <= lon <= lon_max):
                    raise RuntimeError(f"Geofence violation: ({lat:.5f}, {lon:.5f})")

    def end_mission(self):
        """
        Clean shutdown — sets _landing flag, stops heartbeat, changes mode.
        """
        print("[BASE] Ending mission")
        self._landing = True        # suppress min_alt check from here on
        self._stop_heartbeat()

        print(f"[BASE] Switching to {self.end_mode}")
        change_mode(self.master, self.end_mode)
        self.set_expected_mode(self.end_mode)
        self.active = False

    def set_expected_mode(self, mode):
        self.expected_mode = mode

    def update_heartbeat(self):
        hb = self.master.recv_match(type="HEARTBEAT", blocking=False)
        if hb:
            self.last_heartbeat = time.time()

    def _stop_heartbeat(self):
        """Stop the GCS heartbeat thread if it's running."""
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.stop()
            print("[BASE] GCS heartbeat stopped")