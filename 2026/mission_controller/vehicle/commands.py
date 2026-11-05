"""Vehicle commands for the flight-time mission controller.

Copied from drone_simulator/vehicle/commands.py and trimmed: on a real flight
the pilot / Mission Planner arms, takes off and runs AUTO, so the Jetson never
arms, takes off or changes mode. It only sends COMMAND_LONGs (DO_SET_SERVO,
DO_SET_MISSION_CURRENT), velocity setpoints during a NAV_GUIDED_ENABLE
handover, and reads state. Nothing here touches the pymavlink connection.
"""

import logging
import math
import queue
import time
from typing import List, Optional, Sequence

from pymavlink import mavutil

from mission_controller.vehicle.link import Link, Subscription

logger = logging.getLogger(__name__)

mavlink = mavutil.mavlink

_POLL_INTERVAL_S = 0.05
_STATUSTEXT_DRAIN_S = 0.1  # trailing STATUSTEXTs collected after a rejection

_REJECTED_RESULTS = (
    mavlink.MAV_RESULT_DENIED,
    mavlink.MAV_RESULT_FAILED,
    mavlink.MAV_RESULT_UNSUPPORTED,
    mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
)


def _enum_name(enum: str, value: int) -> str:
    entry = mavlink.enums.get(enum, {}).get(value)
    return entry.name if entry is not None else "%s(%d)" % (enum, value)


class CommandError(Exception):
    """Base class for command failures, also raised when a precondition is not met."""


class CommandDenied(CommandError):
    """The autopilot rejected the command.

    Attributes:
        command: MAV_CMD id.
        result: MAV_RESULT from the COMMAND_ACK.
        statustexts: STATUSTEXT strings received while the command was pending,
            e.g. ``["PreArm: GPS not healthy"]``.
    """

    def __init__(self, command: int, result: int, statustexts: Sequence[str]) -> None:
        self.command = command
        self.result = result
        self.statustexts = list(statustexts)
        message = "%s rejected: %s" % (_enum_name("MAV_CMD", command),
                                       _enum_name("MAV_RESULT", result))
        if self.statustexts:
            message += "; autopilot said: " + " | ".join(self.statustexts)
        super().__init__(message)


class CommandTimeout(CommandError):
    """No matching COMMAND_ACK after every attempt.

    Attributes:
        command: MAV_CMD id.
        attempts: Number of times the command was sent.
    """

    def __init__(self, command: int, attempts: int) -> None:
        self.command = command
        self.attempts = attempts
        super().__init__("No COMMAND_ACK for %s after %d attempt(s)"
                         % (_enum_name("MAV_CMD", command), attempts))


class StateNotReached(CommandError):
    """The command was accepted but the vehicle did not reach the expected state.

    Attributes:
        description: What was being waited for, and why it failed.
        timeout_s: The wait limit that applied, in seconds.
    """

    def __init__(self, description: str, timeout_s: float) -> None:
        self.description = description
        self.timeout_s = timeout_s
        super().__init__("%s (limit %.1f s)" % (description, timeout_s))


def command_long(
    link: Link,
    command: int,
    params: Sequence[float] = (),
    ack_timeout_s: float = 1.5,
    retries: int = 3,
) -> int:
    """Sends a COMMAND_LONG and waits for its COMMAND_ACK.

    Silence is retried (``confirmation`` 0, 1, 2, ...). A rejection is raised
    immediately without retrying. MAV_RESULT_IN_PROGRESS restarts the wait
    for the current attempt instead of counting as a failure.

    Args:
        link: Connected link.
        command: MAV_CMD id, e.g. ``mavlink.MAV_CMD_DO_SET_SERVO``.
        params: Up to 7 params (param1..param7); missing ones are sent as 0.
        ack_timeout_s: Seconds to wait for an ACK per attempt (and after each IN_PROGRESS).
        retries: Total number of attempts, including the first.

    Returns:
        The MAV_RESULT of the final ACK (always MAV_RESULT_ACCEPTED).

    Raises:
        ValueError: More than 7 params, or ``retries`` < 1.
        CommandDenied: ACK with DENIED, FAILED, UNSUPPORTED or TEMPORARILY_REJECTED.
        CommandTimeout: No matching ACK after ``retries`` attempts.
    """
    if len(params) > 7:
        raise ValueError("COMMAND_LONG takes at most 7 params, got %d" % len(params))
    if retries < 1:
        raise ValueError("retries must be >= 1")
    padded = [float(p) for p in params] + [0.0] * (7 - len(params))
    name = _enum_name("MAV_CMD", command)
    statustexts = []  # type: List[str]

    # Subscribe BEFORE sending so a fast ACK cannot be missed.
    with link.subscribe("COMMAND_ACK", "STATUSTEXT") as sub:
        for confirmation in range(retries):
            logger.info("Sending %s params=%s (attempt %d/%d)",
                        name, padded, confirmation + 1, retries)
            link.send("command_long_send", link.target_system, link.target_component,
                      command, confirmation, *padded)
            deadline = time.monotonic() + ack_timeout_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    msg = sub.get(timeout=remaining)
                except queue.Empty:
                    break
                if msg.get_srcSystem() != link.target_system:
                    continue
                if msg.get_type() == "STATUSTEXT":
                    statustexts.append(msg.text)
                    logger.info("Autopilot: %s", msg.text)
                    continue
                if msg.command != command:
                    logger.debug("Ignoring COMMAND_ACK for %s", _enum_name("MAV_CMD", msg.command))
                    continue
                if msg.result == mavlink.MAV_RESULT_ACCEPTED:
                    logger.info("%s accepted", name)
                    return msg.result
                if msg.result == mavlink.MAV_RESULT_IN_PROGRESS:
                    logger.info("%s in progress", name)
                    deadline = time.monotonic() + ack_timeout_s
                    continue
                if msg.result in _REJECTED_RESULTS:
                    statustexts.extend(_drain_statustexts(link, sub))
                    error = CommandDenied(command, msg.result, statustexts)
                    logger.error("%s", error)
                    raise error
                logger.warning("Unexpected %s result for %s; still waiting",
                               _enum_name("MAV_RESULT", msg.result), name)
            logger.warning("No ACK for %s within %.1f s", name, ack_timeout_s)

    error = CommandTimeout(command, retries)
    logger.error("%s", error)
    raise error


def _drain_statustexts(link: Link, sub: Subscription) -> List[str]:
    """Collects STATUSTEXTs that arrive just after a rejection ACK."""
    texts = []
    deadline = time.monotonic() + _STATUSTEXT_DRAIN_S
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return texts
        try:
            msg = sub.get(timeout=remaining)
        except queue.Empty:
            return texts
        if msg.get_type() == "STATUSTEXT" and msg.get_srcSystem() == link.target_system:
            texts.append(msg.text)


SERVO_MIN_PWM_US = 800
SERVO_MAX_PWM_US = 2200
SERVO_STREAM_FRESH_S = 2.0


def current_mode(link: Link) -> Optional[str]:
    """Returns the flight mode name from the latest autopilot HEARTBEAT, e.g. ``"GUIDED"``.

    Args:
        link: Connected link.

    Returns:
        The mode name (``mavutil.mode_string_v10``), or None if no HEARTBEAT yet.
    """
    hb = link.latest("HEARTBEAT")
    return None if hb is None else mavutil.mode_string_v10(hb)


def set_servo(link: Link, channel: int, pwm_us: int, timeout_s: float = 1.5) -> None:
    """Sets a servo output with MAV_CMD_DO_SET_SERVO and checks the output changed.

    If SERVO_OUTPUT_RAW is streaming, waits for a fresh one showing
    ``servo{channel}_raw == pwm_us``. If it is not streaming, only logs a
    warning (the ACK is all there is to go on).

    Args:
        link: Connected link.
        channel: Servo output number, 1 to 16 (e.g. 6 for SERVO6 / "ch6out").
        pwm_us: Pulse width in microseconds, 800 to 2200.
        timeout_s: Seconds to wait for SERVO_OUTPUT_RAW to show the new value.

    Raises:
        ValueError: ``channel`` or ``pwm_us`` out of range (nothing is sent).
        CommandDenied: Rejected (e.g. the output is assigned to another function).
        CommandTimeout: No ACK.
        StateNotReached: SERVO_OUTPUT_RAW is streaming but never showed ``pwm_us``.
    """
    if not 1 <= channel <= 16:
        raise ValueError("servo channel must be 1-16, got %r" % (channel,))
    if not SERVO_MIN_PWM_US <= pwm_us <= SERVO_MAX_PWM_US:
        raise ValueError("pwm_us must be %d-%d, got %r"
                         % (SERVO_MIN_PWM_US, SERVO_MAX_PWM_US, pwm_us))
    logger.info("Setting servo %d to %d us", channel, pwm_us)
    sent_at = time.monotonic()
    command_long(link, mavlink.MAV_CMD_DO_SET_SERVO, [channel, pwm_us])
    if link.age("SERVO_OUTPUT_RAW") > SERVO_STREAM_FRESH_S:
        logger.warning("SERVO_OUTPUT_RAW not streaming; servo %d = %d us accepted but unverified",
                       channel, pwm_us)
        return
    field_name = "servo%d_raw" % channel
    deadline = time.monotonic() + timeout_s
    last = None
    while True:
        msg, rx = link.latest_with_time("SERVO_OUTPUT_RAW")
        if msg is not None and rx > sent_at:
            last = getattr(msg, field_name, None)
            if last is None:  # MAVLink1 SERVO_OUTPUT_RAW has only servo1-8
                logger.warning("SERVO_OUTPUT_RAW has no %s; servo %d unverified",
                               field_name, channel)
                return
            if last == pwm_us:
                logger.info("Servo %d confirmed at %d us", channel, pwm_us)
                return
        if time.monotonic() >= deadline:
            raise StateNotReached("%s is %s us, expected %d us" % (field_name, last, pwm_us),
                                  timeout_s)
        time.sleep(_POLL_INTERVAL_S)


VELOCITY_TYPE_MASK = 0b010111000111  # use vx/vy/vz + yaw_rate; ignore pos, accel, yaw


def send_velocity_body(link: Link, forward_m_s: float, right_m_s: float, down_m_s: float,
                       yaw_rate_rad_s: float = 0.0) -> None:
    """Sends one body-frame velocity setpoint (SET_POSITION_TARGET_LOCAL_NED).

    Accepted by ArduCopter only in GUIDED or while AUTO is running a
    NAV_GUIDED_ENABLE item; otherwise it is silently ignored. ArduPilot rotates
    forward/right by the current yaw only (not roll/pitch). The yaw rate is
    commanded explicitly (default 0) so the heading holds while centering.
    Fire-and-forget: there is no ACK, and if setpoints stop arriving for
    GUID_TIMEOUT (default 3 s) the vehicle stops and holds.

    Args:
        link: Connected link.
        forward_m_s: Velocity towards the nose, in m/s.
        right_m_s: Velocity towards the right side, in m/s.
        down_m_s: Velocity downwards, in m/s (negative climbs).
        yaw_rate_rad_s: Yaw rate in rad/s.

    Raises:
        ValueError: A value is not finite (nothing is sent).
    """
    values = (forward_m_s, right_m_s, down_m_s, yaw_rate_rad_s)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("velocity must be finite, got %r" % (values,))
    link.send("set_position_target_local_ned_send", 0,
              link.target_system, link.target_component,
              mavlink.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_TYPE_MASK,
              0, 0, 0, forward_m_s, right_m_s, down_m_s, 0, 0, 0, 0, yaw_rate_rad_s)


def mission_current_seq(link: Link, max_age_s: float = 2.0) -> Optional[int]:
    """Returns the autopilot's current mission seq, or None if MISSION_CURRENT is stale.

    Args:
        link: Connected link.
        max_age_s: Oldest acceptable MISSION_CURRENT, in seconds.
    """
    msg = link.latest_fresh("MISSION_CURRENT", max_age_s)
    return None if msg is None else msg.seq


def set_mission_current(link: Link, seq: int, expect_seq: Optional[int] = None,
                        timeout_s: float = 3.0) -> None:
    """Makes ``seq`` the current mission item (MAV_CMD_DO_SET_MISSION_CURRENT) and waits for it.

    Used to end a NAV_GUIDED_ENABLE item early and continue the AUTO mission.
    The mode is not changed. If ``seq`` is a DO item, ArduPilot starts it
    immediately and continues at the next NAV item; MISSION_CURRENT reports
    NAV items only, so pass that NAV item as ``expect_seq``.

    Args:
        link: Connected link.
        seq: Mission item to continue from.
        expect_seq: seq MISSION_CURRENT should then show (default ``seq``).
        timeout_s: Seconds to wait, after the ACK, for MISSION_CURRENT to show it.

    Raises:
        CommandDenied: Rejected (e.g. seq out of range).
        CommandTimeout: No ACK.
        StateNotReached: MISSION_CURRENT never showed ``seq``.
    """
    expect_seq = seq if expect_seq is None else expect_seq
    logger.info("Setting current mission item to %d", seq)
    sent_at = time.monotonic()
    command_long(link, mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, [seq])
    deadline = time.monotonic() + timeout_s
    while True:
        msg, rx = link.latest_with_time("MISSION_CURRENT")
        if msg is not None and rx > sent_at and msg.seq == expect_seq:
            logger.info("Mission continuing at item %d", expect_seq)
            return
        if time.monotonic() >= deadline:
            raise StateNotReached("MISSION_CURRENT never showed seq %d" % expect_seq, timeout_s)
        time.sleep(_POLL_INTERVAL_S)
