import contextlib
import logging
import queue
import time

from competition.drone_simulator.vehicle import commands, params
from competition.drone_simulator.vehicle.upload import upload_waypoint_mission

logger = logging.getLogger(__name__)

# Position checks are skipped when GLOBAL_POSITION_INT is older than this.
POSITION_MAX_AGE_S = 2.0
HEARTBEAT_LOSS_S = 5.0


class PilotTakeover(Exception):
    """The safety pilot switched to a manual mode; the companion must stop commanding."""


class MissionBase():
    """Base flight sequence on a Link: upload, GUIDED, arm, takeoff, execute, monitor, end.

    The Link owns the GCS heartbeat; it keeps running until the caller closes
    the link (after the end mode is confirmed).
    """

    # If a safety officer takes over via RC, we log it and wait rather than abort.
    RC_OVERRIDE_MODES = {"STABILIZE", "ALT_HOLD", "LOITER", "LAND"}

    def __init__(self, link, config):
        self.link = link
        self.config = config
        self.takeoff_alt = None
        self.waypoints = None
        self.last_nav_seq = None
        self.active = False
        self.start_time = None
        self.time_limit = self.config.get("time_limit", 600)
        self.last_waypoint = None
        self.expected_mode = "AUTO"
        self.max_alt = self.config.get("max_alt", 120)
        self.min_alt = self.config.get("min_alt", 5)
        self.geofence = self.config.get("geofence", None)
        self.end_mode = self.config.get("end_mode", "RTL")

        self._landing = False

        self._subscriptions = contextlib.ExitStack()
        self._reached = None

    def run(self):
        """Main mission entry point"""
        try:
            self.setup()
            self.arm_and_takeoff()
            self.execute()

            self.active = True

            while self.active:
                self.monitor()
                self.check_time_limit()
                self.check_safety()
                time.sleep(0.2)

            self.end_mission()

        except PilotTakeover as e:
            # Send nothing more: the pilot is flying.
            self._close_subscriptions()
            logger.critical("Mission stopped, pilot has control (%s)", e)
        except Exception as e:
            logger.error("Mission failed: %s", e)
            self.abort()

    def setup(self):
        """Prepare mission — validates config, uploads and verifies waypoints."""

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

        self.last_nav_seq = upload_waypoint_mission(self.link, self.waypoints)
        logger.info("Waypoints uploaded. Target alt: %sm | Waypoints: %d",
                    self.takeoff_alt, len(self.waypoints))

        # Open now so no MISSION_ITEM_REACHED can be missed; closed in end_mission/abort.
        self._reached = self._subscriptions.enter_context(
            self.link.subscribe("MISSION_ITEM_REACHED", maxsize=200))

    def execute(self):
        """Switch to AUTO; set_mode waits for the autopilot to confirm it."""
        logger.info("Starting mission — switching to AUTO")
        commands.set_mode(self.link, "AUTO", timeout_s=10.0)
        self.set_expected_mode("AUTO")
        logger.info("AUTO mode confirmed")

    def monitor(self):
        """
        Monitor waypoint progress.
        """
        if self._reached is None:
            return
        while True:
            try:
                msg = self._reached.get(timeout=0)
            except queue.Empty:
                return
            if msg.get_srcSystem() != self.link.target_system:
                continue

            reached = msg.seq
            self.last_waypoint = reached
            logger.info("Waypoint reached: %d/%d", reached, self.last_nav_seq)

            if reached >= self.last_nav_seq:
                logger.info("All waypoints reached — mission complete")
                self.active = False

    def arm_and_takeoff(self):
        """Standard arming and takeoff sequence."""
        # Read before arming: ArduPilot disarms again after DISARM_DELAY without a takeoff.
        climb_rate_m_s = params.climb_speed_m_s(self.link)

        commands.set_mode(self.link, "GUIDED")
        self.set_expected_mode("GUIDED")

        # Competition clock starts at takeoff.
        self.start_time = time.monotonic()
        commands.arm(self.link)

        commands.takeoff(self.link, self.takeoff_alt, climb_rate_m_s=climb_rate_m_s)
        logger.info("Target altitude reached")

    def check_time_limit(self):
        """Enforce competition time limit."""
        if self.start_time is None:
            return

        elapsed = time.monotonic() - self.start_time
        if elapsed >= self.time_limit:
            logger.warning("Time limit reached (%ss)", self.time_limit)
            raise RuntimeError("Time limit exceeded")

    def abort(self):
        """Failsafe — switch to the end mode, logging (not raising) any error."""
        logger.warning("Aborting mission")
        try:
            self.end_mission()
        except Exception as e:
            logger.error("Abort could not set %s: %s", self.end_mode, e)

    def check_safety(self):
        """
        Safety checks: link loss, mode guard, altitude bounds, geofence.

        Raises:
            PilotTakeover: The mode changed to one of RC_OVERRIDE_MODES.
            RuntimeError: Any other violation.
        """
        if self.link.age("HEARTBEAT") > HEARTBEAT_LOSS_S:
            raise RuntimeError("Lost vehicle heartbeat — link failure")

        current_mode = commands.current_mode(self.link)
        if current_mode != self.expected_mode:
            if current_mode in self.RC_OVERRIDE_MODES:
                logger.critical("RC Pilot Takeover Detected! Mode Switched to: %s", current_mode)
                logger.critical("Terminating companion computer guidance loop immediately.")
                self.active = False
                raise PilotTakeover(current_mode)
            raise RuntimeError(
                f"Unexpected mode change: expected={self.expected_mode}, "
                f"got={current_mode}"
            )

        if self.link.age("GLOBAL_POSITION_INT") > POSITION_MAX_AGE_S:
            return
        msg = self.link.latest("GLOBAL_POSITION_INT")
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
        Clean shutdown — sets _landing, then switches to end_mode (confirmed).

        Heartbeats keep running; the caller closes the Link once this returns.
        """
        logger.info("Ending mission")
        self._landing = True        # suppress min_alt check from here on
        self._close_subscriptions()

        logger.info("Switching to %s", self.end_mode)
        commands.set_mode(self.link, self.end_mode)
        self.set_expected_mode(self.end_mode)
        self.active = False

    def set_expected_mode(self, mode):
        self.expected_mode = mode

    def _close_subscriptions(self):
        self._subscriptions.close()
        self._reached = None
