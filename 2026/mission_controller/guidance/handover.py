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

Bench mode (``bench_alt_m`` set): for checking the camera and the data flow
on the ground with the real Cube. Guidance runs exactly as in flight, using
the Cube's real attitude and ``bench_alt_m`` as the height, but nothing is
sent: no velocity setpoints and no hand-back. Mode and mission item are not
checked. Every tick is still traced and logged, including what *would* have
been sent.
"""

import logging
import math
import time
from typing import Callable, Optional, Tuple

from mission_controller.config.flight_params import HandoverConfig
from mission_controller.guidance.camera_geometry import (
    centering_velocity, pixel_to_ground_offset)
from mission_controller.vehicle import commands
from mission_controller.vehicle.commands import CommandError
from mission_controller.vehicle.link import Link
from mission_controller.vehicle.mission import HandoverPlan
from mission_controller.vision.detector import Detection, TargetDetector

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

# Columns of the per-tick trace (see HandoverController's ``trace``). phase is
# ALIGN (moving towards the target), CENTRED (within tolerance, timing the hold),
# SEARCH (target not seen: holding), NO_TELEMETRY (holding), or the final outcome.
TRACE_FIELDS = (
    "t_s", "phase", "alt_m", "roll_deg", "pitch_deg",
    "det_x_px", "det_y_px", "frame_w_px", "frame_h_px", "det_age_s",
    "fwd_m", "right_m", "offset_m", "cmd_fwd_m_s", "cmd_right_m_s",
    "centred_for_s", "lost_for_s", "sent",
)


class HandoverController:
    """Runs one handover. Construct, then call :meth:`wait_for_handover` and :meth:`run`.

    Args:
        link: Connected link (the Link's own heartbeat keeps running throughout).
        detector: Target detector (camera).
        plan: Handover location and FC limits from ``vehicle.mission.check_mission``.
        config: Speeds, tolerances and the drop altitude.
        trace: Called with one dict (keys TRACE_FIELDS) per guidance tick, e.g. a
            CsvLog's ``write``. Errors in it are logged and never stop guidance.
        bench_alt_m: Bench mode: use this height instead of the FC's, and send
            nothing (see the module docstring). None (default) for flight.
    """

    def __init__(self, link: Link, detector: TargetDetector, plan: Optional[HandoverPlan],
                 config: HandoverConfig, trace: Optional[Callable[[dict], None]] = None,
                 bench_alt_m: Optional[float] = None) -> None:
        if plan is None and bench_alt_m is None:
            raise ValueError("a HandoverPlan is required outside bench mode")
        if bench_alt_m is not None and bench_alt_m <= 0:
            raise ValueError("bench_alt_m must be > 0")
        self.link = link
        self.detector = detector
        self.plan = plan
        self.config = config
        self.trace = trace
        self.bench_alt_m = bench_alt_m
        self._period_s = 1.0 / config.command_rate_hz
        self._last_status = float("-inf")
        self._start = time.monotonic()
        self._trace_failed = False

    # --- state ---------------------------------------------------------------

    def _armed(self) -> bool:
        hb = self.link.latest("HEARTBEAT")
        return bool(hb is not None and hb.base_mode & commands.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def _authority_lost(self) -> Optional[str]:
        """Returns an outcome if the Jetson may no longer guide, else None."""
        if not self.link.is_up or self.link.age("HEARTBEAT") > HEARTBEAT_LOSS_S:
            return LINK_LOST
        if self.bench_alt_m is not None:
            return None
        if commands.current_mode(self.link) != "AUTO":
            return PILOT_TAKEOVER
        seq = commands.mission_current_seq(self.link)
        if seq is not None and seq != self.plan.guided_seq:
            return FC_ENDED
        return None

    def _telemetry(self) -> Optional[Tuple[float, float, float]]:
        """(relative alt m, roll rad, pitch rad), or None if stale."""
        max_age = self.config.telemetry_max_age_s
        att = self.link.latest_fresh("ATTITUDE", max_age)
        if self.bench_alt_m is not None:
            return None if att is None else (self.bench_alt_m, att.roll, att.pitch)
        pos = self.link.latest_fresh("GLOBAL_POSITION_INT", max_age)
        if pos is None or att is None:
            return None
        return pos.relative_alt / 1000.0, att.roll, att.pitch

    def _target_offset(self, alt_m: float, roll: float, pitch: float
                       ) -> Tuple[Optional[Detection], Optional[Tuple[float, float]]]:
        """(the detection, its ground offset); the offset is None if not seen or stale."""
        det = self.detector.detect()
        if det is None or time.monotonic() - det.captured_at > self.config.detection_max_age_s:
            return det, None
        return det, pixel_to_ground_offset(det.x_px, det.y_px, det.width_px, det.height_px,
                                           alt_m, roll, pitch, self.config.camera_hfov_deg,
                                           self.config.camera_vfov_deg)

    def _send(self, forward: float, right: float, down: float) -> bool:
        """Sends a setpoint (not in bench mode). Returns True if it was sent."""
        if self.bench_alt_m is not None:
            return False
        commands.send_velocity_body(self.link, forward, right, down)
        return True

    def _trace(self, tick: float, phase: str, telemetry: Optional[Tuple[float, float, float]],
               det: Optional[Detection], offset: Optional[Tuple[float, float]],
               cmd: Optional[Tuple[float, float]], sent: bool,
               centred_for_s: Optional[float] = None,
               lost_for_s: Optional[float] = None) -> None:
        if self.trace is None:
            return
        try:
            self.trace({
                "t_s": round(tick - self._start, 3),
                "phase": phase,
                "alt_m": None if telemetry is None else round(telemetry[0], 2),
                "roll_deg": None if telemetry is None else round(math.degrees(telemetry[1]), 2),
                "pitch_deg": None if telemetry is None else round(math.degrees(telemetry[2]), 2),
                "det_x_px": None if det is None else round(det.x_px, 1),
                "det_y_px": None if det is None else round(det.y_px, 1),
                "frame_w_px": None if det is None else det.width_px,
                "frame_h_px": None if det is None else det.height_px,
                "det_age_s": None if det is None else round(time.monotonic() - det.captured_at, 3),
                "fwd_m": None if offset is None else round(offset[0], 3),
                "right_m": None if offset is None else round(offset[1], 3),
                "offset_m": None if offset is None else round(math.hypot(*offset), 3),
                "cmd_fwd_m_s": None if cmd is None else round(cmd[0], 3),
                "cmd_right_m_s": None if cmd is None else round(cmd[1], 3),
                "centred_for_s": None if centred_for_s is None else round(centred_for_s, 2),
                "lost_for_s": None if lost_for_s is None else round(lost_for_s, 2),
                "sent": int(sent),
            })
        except Exception:
            if not self._trace_failed:
                logger.exception("Handover trace failed; guidance continues without it")
                self._trace_failed = True

    def _status(self, alt_m: float, offset: Optional[Tuple[float, float]],
                cmd: Tuple[float, float] = (0.0, 0.0)) -> None:
        now = time.monotonic()
        if now - self._last_status < 1.0:
            return
        self._last_status = now
        logger.info("[%s] alt %.2f m | target %s | cmd fwd %.2f right %.2f m/s%s",
                    "BENCH" if self.bench_alt_m is not None else "ALIGN", alt_m,
                    "offset %.2f m (fwd %.2f, right %.2f)" % (math.hypot(*offset), offset[0],
                                                             offset[1])
                    if offset else "NOT SEEN", cmd[0], cmd[1],
                    " (not sent)" if self.bench_alt_m is not None else "")

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
        self._start = time.monotonic()
        stop = self._authority_lost()
        if stop is not None:
            return stop
        telemetry = self._telemetry()
        if telemetry is None:
            logger.error("No fresh position/attitude at handover start; leaving it to the FC")
            self._trace(time.monotonic(), LINK_LOST, None, None, None, None, False)
            return LINK_LOST
        alt_m = telemetry[0]
        if abs(alt_m - cfg.drop_alt_m) > cfg.drop_alt_tolerance_m and self.bench_alt_m is None:
            logger.error("At %.2f m, not within %.1f m of the drop altitude %.2f m; handing "
                         "back without releasing", alt_m, cfg.drop_alt_tolerance_m,
                         cfg.drop_alt_m)
            self._trace(time.monotonic(), WRONG_ALTITUDE, telemetry, None, None, None, False)
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
                self._trace(tick, stop, None, None, None, None, False)
                return stop

            telemetry = self._telemetry()
            det, offset = None, None
            if telemetry is not None:
                alt_m, roll, pitch = telemetry
                det, offset = self._target_offset(alt_m, roll, pitch)

            if offset is None:
                # Not seen (or no telemetry): hold position and keep looking.
                centered_since = None
                lost_since = tick if lost_since is None else lost_since
                sent = self._send(0.0, 0.0, 0.0)
                self._trace(tick, "SEARCH" if telemetry is not None else "NO_TELEMETRY",
                            telemetry, det, None, (0.0, 0.0), sent, lost_for_s=tick - lost_since)
                if telemetry is not None:
                    self._status(telemetry[0], None)
                if tick - lost_since >= cfg.lost_lock_timeout_s:
                    logger.warning("Target lost for %.0f s; handing back without releasing",
                                   cfg.lost_lock_timeout_s)
                    self._trace(tick, GAVE_UP, telemetry, det, None, None, False,
                                lost_for_s=tick - lost_since)
                    return self._hand_back(release=False, outcome=GAVE_UP)
                time.sleep(max(0.0, self._period_s - (time.monotonic() - tick)))
                continue

            lost_since = None
            if math.hypot(*offset) <= cfg.center_tolerance_m:
                centered_since = tick if centered_since is None else centered_since
                if tick - centered_since >= cfg.center_hold_s:
                    logger.info("Locked: within %.2f m for %.1f s", cfg.center_tolerance_m,
                                cfg.center_hold_s)
                    self._trace(tick, LOCKED, telemetry, det, offset, None, False,
                                centred_for_s=tick - centered_since)
                    return self._hand_back(release=True, outcome=LOCKED)
            else:
                centered_since = None
            vx, vy = centering_velocity(offset[0], offset[1], cfg.horizontal_gain_per_s,
                                        cfg.max_horizontal_speed_m_s)
            sent = self._send(vx, vy, 0.0)
            self._trace(tick, "ALIGN" if centered_since is None else "CENTRED", telemetry, det,
                        offset, (vx, vy), sent,
                        centred_for_s=None if centered_since is None else tick - centered_since)
            self._status(alt_m, offset, (vx, vy))
            time.sleep(max(0.0, self._period_s - (time.monotonic() - tick)))

    def _hand_back(self, release: bool, outcome: str) -> str:
        """Gives control back to the FC: at the DO_SET_SERVO (release) or past it."""
        if self.bench_alt_m is not None:
            if self.plan is None:
                logger.info("BENCH: %s; would hand back %s (no valid mission loaded)", outcome,
                            "at the release" if release else "past the release")
            else:
                logger.info("BENCH: %s; would hand back at item %d (%s). Nothing sent.", outcome,
                            self.plan.release_seq if release else self.plan.resume_seq,
                            "DO_SET_SERVO: FC releases" if release else "release skipped")
            return outcome
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
