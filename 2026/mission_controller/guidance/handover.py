"""The Jetson's part of the flight: horizontal centring during NAV_GUIDED_ENABLE.

The flight controller flies the Mission Planner mission in AUTO, including
the descent to the drop altitude, and stays responsible for stabilisation
and failsafes. When the mission reaches the NAV_GUIDED_ENABLE item,
ArduCopter stays in AUTO but accepts velocity setpoints from the Jetson,
within the DO_GUIDED_LIMITS it enforces itself. This controller then:

1. Checks the drone is within ``drop_alt_tolerance_m`` of ``drop_alt_m``;
   if not, it hands straight back, skipping the release.
2. ALIGN: sends bounded horizontal velocity from the pixel error, vertical
   velocity 0 (altitude stays where the FC put it).
3. Once the target has been within ``center_tolerance_m`` for
   ``center_hold_s``, hands back by making the mission's DO_SET_SERVO
   current (bypassing the guard DO_JUMP before it): the FC fires the
   release and flies the rest of the mission.

Lost lock: hold (zero velocity) and keep looking; after
``lost_lock_timeout_s`` without the target, hand back at the first NAV
item after the release, so the FC skips it and nothing is dropped.

If the mode stops being AUTO (pilot/GCS took over), the mission moves past
NAV_GUIDED_ENABLE (FC limit, or someone else moved it), or the link is lost,
the controller stops sending immediately. When the FC ends the handover
itself (DO_GUIDED_LIMITS timeout or limit, or the Jetson stops and the
timeout expires), the guard DO_JUMP right after NAV_GUIDED_ENABLE makes it
skip the release.
"""

import logging
import math
import time
from typing import Optional, Tuple

from mission_controller.config.flight_params import HandoverConfig
from mission_controller.guidance.camera_geometry import (
    centering_velocity, pixel_to_ground_offset)
from mission_controller.vehicle import commands
from mission_controller.vehicle.commands import CommandError
from mission_controller.vehicle.link import Link
from mission_controller.vehicle.mission import HandoverPlan
from mission_controller.vision.detector import TargetDetector

logger = logging.getLogger(__name__)

HEARTBEAT_LOSS_S = 3.0
# Waiting only ends on a disarm after the vehicle has been above this height:
# ArduPilot auto-disarms on the ground (DISARM_DELAY) and the pilot re-arms.
FLOWN_ALT_M = 2.0

# Outcomes of HandoverController.run()
LOCKED = "locked"                    # centred; handed back at DO_SET_SERVO (FC releases)
GAVE_UP = "gave_up"                  # lost lock for lost_lock_timeout_s; release skipped
WRONG_ALTITUDE = "wrong_altitude"    # not near the drop altitude; release skipped
FC_ENDED = "fc_ended"                # mission moved past NAV_GUIDED_ENABLE without us
PILOT_TAKEOVER = "pilot_takeover"    # mode is no longer AUTO
LINK_LOST = "link_lost"


class HandoverController:
    """Runs one handover. Construct, then call :meth:`wait_for_handover` and :meth:`run`.

    Args:
        link: Connected link (the Link's own heartbeat keeps running throughout).
        detector: Target detector (camera).
        plan: Handover location and FC limits from ``vehicle.mission.check_mission``.
        config: Speeds, tolerances and the drop altitude.
    """

    def __init__(self, link: Link, detector: TargetDetector, plan: HandoverPlan,
                 config: HandoverConfig) -> None:
        self.link = link
        self.detector = detector
        self.plan = plan
        self.config = config
        self._period_s = 1.0 / config.command_rate_hz
        self._last_status = float("-inf")

    # --- state ---------------------------------------------------------------

    def _armed(self) -> bool:
        hb = self.link.latest("HEARTBEAT")
        return bool(hb is not None and hb.base_mode & commands.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def _authority_lost(self) -> Optional[str]:
        """Returns an outcome if the Jetson may no longer guide, else None."""
        if not self.link.is_up or self.link.age("HEARTBEAT") > HEARTBEAT_LOSS_S:
            return LINK_LOST
        if commands.current_mode(self.link) != "AUTO":
            return PILOT_TAKEOVER
        seq = commands.mission_current_seq(self.link)
        if seq is not None and seq != self.plan.guided_seq:
            return FC_ENDED
        return None

    def _telemetry(self) -> Optional[Tuple[float, float, float]]:
        """(relative alt m, roll rad, pitch rad), or None if stale."""
        max_age = self.config.telemetry_max_age_s
        pos = self.link.latest_fresh("GLOBAL_POSITION_INT", max_age)
        att = self.link.latest_fresh("ATTITUDE", max_age)
        if pos is None or att is None:
            return None
        return pos.relative_alt / 1000.0, att.roll, att.pitch

    def _target_offset(self, alt_m: float, roll: float,
                       pitch: float) -> Optional[Tuple[float, float]]:
        det = self.detector.detect()
        if det is None or time.monotonic() - det.captured_at > self.config.detection_max_age_s:
            return None
        return pixel_to_ground_offset(det.x_px, det.y_px, det.width_px, det.height_px, alt_m,
                                      roll, pitch, self.config.camera_hfov_deg,
                                      self.config.camera_vfov_deg)

    def _send(self, forward: float, right: float, down: float) -> None:
        commands.send_velocity_body(self.link, forward, right, down)

    def _status(self, alt_m: float, offset: Optional[Tuple[float, float]]) -> None:
        now = time.monotonic()
        if now - self._last_status < 1.0:
            return
        self._last_status = now
        logger.info("[ALIGN] alt %.2f m | target %s", alt_m,
                    "offset %.2f m (fwd %.2f, right %.2f)" % (math.hypot(*offset), offset[0],
                                                             offset[1])
                    if offset else "NOT SEEN")

    # --- entry points ---------------------------------------------------------

    def wait_for_handover(self, poll_s: float = 0.2, timeout_s: Optional[float] = None) -> bool:
        """Blocks until AUTO reaches the NAV_GUIDED_ENABLE item while armed.

        Sends nothing. Returns False instead if the vehicle disarms after having
        flown (been armed above FLOWN_ALT_M), or ``timeout_s`` passes. A disarm
        on the ground (e.g. ArduPilot's DISARM_DELAY before takeoff) keeps waiting.

        Args:
            poll_s: Polling period, in seconds.
            timeout_s: Give up after this many seconds (None = wait forever).

        Returns:
            True when the handover has started.
        """
        logger.info("Waiting for AUTO to reach NAV_GUIDED_ENABLE (seq %d); sending nothing",
                    self.plan.guided_seq)
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        flew = False
        warned_past = False
        while deadline is None or time.monotonic() < deadline:
            armed = self._armed()
            telemetry = self._telemetry()
            if armed and telemetry is not None and telemetry[0] > FLOWN_ALT_M:
                flew = True
            if flew and not armed:
                logger.info("Vehicle landed and disarmed without reaching the handover")
                return False
            seq = commands.mission_current_seq(self.link)
            if seq is not None and seq > self.plan.guided_seq and not warned_past:
                logger.warning("Mission is already at item %d, past NAV_GUIDED_ENABLE (%d); "
                               "no handover this flight unless it is restarted", seq,
                               self.plan.guided_seq)
                warned_past = True
            if (armed and commands.current_mode(self.link) == "AUTO"
                    and seq == self.plan.guided_seq):
                logger.info("Handover started: AUTO is on NAV_GUIDED_ENABLE (seq %d)",
                            self.plan.guided_seq)
                return True
            time.sleep(poll_s)
        return False

    def run(self) -> str:
        """Centres over the target, then hands control back to the FC.

        Returns:
            One of LOCKED, GAVE_UP, WRONG_ALTITUDE, FC_ENDED, PILOT_TAKEOVER, LINK_LOST.
        """
        cfg = self.config
        stop = self._authority_lost()
        if stop is not None:
            return stop
        telemetry = self._telemetry()
        if telemetry is None:
            logger.error("No fresh position/attitude at handover start; leaving it to the FC")
            return LINK_LOST
        alt_m = telemetry[0]
        if abs(alt_m - cfg.drop_alt_m) > cfg.drop_alt_tolerance_m:
            logger.error("At %.2f m, not within %.1f m of the drop altitude %.2f m; handing "
                         "back without releasing", alt_m, cfg.drop_alt_tolerance_m,
                         cfg.drop_alt_m)
            return self._hand_back(release=False, outcome=WRONG_ALTITUDE)
        logger.info("Centring at %.2f m (drop alt %.2f m); FC releases on servo %d after a "
                    "%.1f s lock", alt_m, cfg.drop_alt_m, cfg.servo_channel, cfg.center_hold_s)
        centered_since = None  # type: Optional[float]
        lost_since = None  # type: Optional[float]

        while True:
            tick = time.monotonic()
            stop = self._authority_lost()
            if stop is not None:
                logger.warning("Stopping guidance: %s", stop)
                return stop

            telemetry = self._telemetry()
            offset = None
            if telemetry is not None:
                alt_m, roll, pitch = telemetry
                offset = self._target_offset(alt_m, roll, pitch)

            if offset is None:
                # Not seen (or no telemetry): hold position and keep looking.
                centered_since = None
                lost_since = tick if lost_since is None else lost_since
                self._send(0.0, 0.0, 0.0)
                if telemetry is not None:
                    self._status(telemetry[0], None)
                if tick - lost_since >= cfg.lost_lock_timeout_s:
                    logger.warning("Target lost for %.0f s; handing back without releasing",
                                   cfg.lost_lock_timeout_s)
                    return self._hand_back(release=False, outcome=GAVE_UP)
                time.sleep(max(0.0, self._period_s - (time.monotonic() - tick)))
                continue

            lost_since = None
            if math.hypot(*offset) <= cfg.center_tolerance_m:
                centered_since = tick if centered_since is None else centered_since
                if tick - centered_since >= cfg.center_hold_s:
                    logger.info("Locked: within %.2f m for %.1f s", cfg.center_tolerance_m,
                                cfg.center_hold_s)
                    return self._hand_back(release=True, outcome=LOCKED)
            else:
                centered_since = None
            vx, vy = centering_velocity(offset[0], offset[1], cfg.horizontal_gain_per_s,
                                        cfg.max_horizontal_speed_m_s)
            self._send(vx, vy, 0.0)
            self._status(alt_m, offset)
            time.sleep(max(0.0, self._period_s - (time.monotonic() - tick)))

    def _hand_back(self, release: bool, outcome: str) -> str:
        """Gives control back to the FC: at the DO_SET_SERVO (release) or past it."""
        self._send(0.0, 0.0, 0.0)
        seq = self.plan.release_seq if release else self.plan.resume_seq
        logger.info("Handing back to the FC at item %d (%s)", seq,
                    "DO_SET_SERVO: FC releases" if release else "release skipped")
        try:
            commands.set_mission_current(self.link, seq, expect_seq=self.plan.resume_seq)
        except CommandError as e:
            # The FC's DO_GUIDED_LIMITS timeout will still move the mission on, and it will
            # then run the DO_SET_SERVO.
            logger.error("Hand-back to item %d not confirmed (%s); the FC limits timeout will "
                         "end the handover", seq, e)
        logger.info("Handover finished: %s", outcome)
        return outcome
