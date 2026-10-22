"""Vehicle commands built on :class:`~competition.drone_simulator.vehicle.link.Link`.

Every operation sends a COMMAND_LONG, waits for its COMMAND_ACK (retrying on
silence, failing fast on a rejection), then verifies the vehicle actually
reached the requested state using data received AFTER the command was sent.
Nothing here touches the pymavlink connection directly.
"""

import logging
import queue
import time
from typing import Any, Callable, List, Optional, Sequence

from pymavlink import mavutil

from competition.drone_simulator.vehicle.link import Link, Subscription

logger = logging.getLogger(__name__)

mavlink = mavutil.mavlink

ARM_FORCE_MAGIC = 21196
MAX_TAKEOFF_ALT_M = 121.9  # 400 ft
DISARM_MAX_REL_ALT_M = 1.0
TAKEOFF_REACHED_FRACTION = 0.95
TAKEOFF_STALL_WINDOW_S = 5.0
TAKEOFF_STALL_MIN_CLIMB_M = 0.3
PRECONDITION_MAX_AGE_S = 2.0
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
        command: MAV_CMD id, e.g. ``mavlink.MAV_CMD_COMPONENT_ARM_DISARM``.
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


def _is_autopilot_heartbeat(link: Link, msg: Any) -> bool:
    return (msg.get_srcSystem() == link.target_system
            and msg.get_srcComponent() == link.target_component)


def _is_armed(heartbeat: Any) -> bool:
    return bool(heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def _wait_for_heartbeat(
    link: Link,
    hb_sub: Subscription,
    predicate: Callable[[Any], bool],
    timeout_s: float,
    description: str,
) -> Any:
    """Waits for an autopilot HEARTBEAT from ``hb_sub`` that satisfies ``predicate``.

    ``hb_sub`` must have been opened before the command was sent, so every
    heartbeat in it is fresh; the cache is never used as proof.
    """
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise StateNotReached("Timed out waiting for " + description, timeout_s)
        try:
            msg = hb_sub.get(timeout=remaining)
        except queue.Empty:
            raise StateNotReached("Timed out waiting for " + description, timeout_s)
        if _is_autopilot_heartbeat(link, msg) and predicate(msg):
            return msg


def _mode_id(link: Link, mode_name: str) -> int:
    mapping = link.mode_mapping()
    if not mapping:
        raise CommandError("Vehicle mode mapping unknown (no autopilot HEARTBEAT type yet)")
    if mode_name not in mapping:
        raise ValueError("Unknown mode %r; valid modes: %s"
                         % (mode_name, ", ".join(sorted(mapping))))
    return mapping[mode_name]


def set_mode(link: Link, mode_name: str, timeout_s: float = 3.0) -> None:
    """Changes flight mode and waits until the vehicle reports it.

    Args:
        link: Connected link.
        mode_name: ArduPilot mode name, e.g. ``"GUIDED"``.
        timeout_s: Seconds to wait, after the ACK, for a HEARTBEAT showing the new mode.

    Raises:
        ValueError: ``mode_name`` is not a mode of this vehicle.
        CommandDenied: The autopilot rejected the mode change.
        CommandTimeout: No ACK.
        StateNotReached: No fresh HEARTBEAT with the new mode within ``timeout_s``.
    """
    mode_id = _mode_id(link, mode_name)
    logger.info("Setting mode %s (custom_mode=%d)", mode_name, mode_id)
    with link.subscribe("HEARTBEAT", maxsize=20) as hb_sub:
        command_long(link, mavlink.MAV_CMD_DO_SET_MODE,
                     [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id])
        _wait_for_heartbeat(link, hb_sub, lambda hb: hb.custom_mode == mode_id,
                            timeout_s, "HEARTBEAT showing mode %s" % mode_name)
    logger.info("Mode %s confirmed", mode_name)


def arm(link: Link, timeout_s: float = 5.0, force: bool = False,
        allow_force_arm: bool = False) -> None:
    """Arms the motors and waits until the vehicle reports armed.

    ArduPilot disarms automatically a few seconds after arming if there is no
    takeoff (DISARM_DELAY), so call :func:`takeoff` promptly afterwards.

    Args:
        link: Connected link.
        timeout_s: Seconds to wait, after the ACK, for a HEARTBEAT showing armed.
        force: Skip pre-arm checks (param2=21196). SITL only.
        allow_force_arm: Must also be True for ``force`` to be honoured.

    Raises:
        PermissionError: ``force`` requested without ``allow_force_arm``, or on a
            serial (real hardware) link. Nothing is sent.
        CommandDenied: Arming rejected; ``statustexts`` holds the PreArm reasons.
        CommandTimeout: No ACK.
        StateNotReached: No fresh armed HEARTBEAT within ``timeout_s``.
    """
    if force and not allow_force_arm:
        raise PermissionError("force arm requires allow_force_arm=True")
    if force and link.is_serial:
        raise PermissionError("force arm is only allowed on udp/tcp (SITL) links")
    params = [1, ARM_FORCE_MAGIC] if force else [1]
    logger.info("Arming%s", " (FORCED, pre-arm checks skipped)" if force else "")
    with link.subscribe("HEARTBEAT", maxsize=20) as hb_sub:
        command_long(link, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, params)
        _wait_for_heartbeat(link, hb_sub, _is_armed, timeout_s, "armed HEARTBEAT")
    logger.info("Armed")


def _relative_alt_m(link: Link) -> Optional[float]:
    """Latest relative altitude in metres, or None if missing or older than PRECONDITION_MAX_AGE_S."""
    msg = link.latest("GLOBAL_POSITION_INT")
    if msg is None or link.age("GLOBAL_POSITION_INT") > PRECONDITION_MAX_AGE_S:
        return None
    return msg.relative_alt / 1000.0


def disarm(link: Link, timeout_s: float = 5.0) -> None:
    """Disarms the motors and waits until the vehicle reports disarmed.

    Refuses unless a recent GLOBAL_POSITION_INT shows the vehicle within
    1.0 m of home altitude.

    Args:
        link: Connected link.
        timeout_s: Seconds to wait, after the ACK, for a HEARTBEAT showing disarmed.

    Raises:
        CommandError: Altitude above 1.0 m or unknown (nothing is sent).
        CommandDenied: Disarm rejected.
        CommandTimeout: No ACK.
        StateNotReached: No fresh disarmed HEARTBEAT within ``timeout_s``.
    """
    rel_alt_m = _relative_alt_m(link)
    if rel_alt_m is None:
        raise CommandError("Refusing to disarm: no GLOBAL_POSITION_INT in the last %.1f s"
                           % PRECONDITION_MAX_AGE_S)
    if rel_alt_m > DISARM_MAX_REL_ALT_M:
        raise CommandError("Refusing to disarm at %.1f m (limit %.1f m)"
                           % (rel_alt_m, DISARM_MAX_REL_ALT_M))
    logger.info("Disarming at %.2f m", rel_alt_m)
    with link.subscribe("HEARTBEAT", maxsize=20) as hb_sub:
        command_long(link, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [0])
        _wait_for_heartbeat(link, hb_sub, lambda hb: not _is_armed(hb), timeout_s,
                            "disarmed HEARTBEAT")
    logger.info("Disarmed")


def takeoff(link: Link, alt_m: float, climb_rate_m_s: float = 2.5) -> float:
    """Takes off in GUIDED and waits until 95% of the target altitude is reached.

    The vehicle must already be armed in GUIDED. ArduPilot auto-disarms
    shortly after arming if no takeoff follows (DISARM_DELAY), so call this
    promptly after :func:`arm`.

    Fails early if altitude rises by less than 0.3 m in any 5 s window, or if
    the vehicle leaves GUIDED or disarms. Overall limit is
    ``alt_m / climb_rate_m_s * 1.5 + 10`` seconds.

    Args:
        link: Connected link.
        alt_m: Target altitude above home, in metres (0 < alt_m <= 121.9).
        climb_rate_m_s: Expected climb rate in m/s, used only for the timeout.

    Returns:
        The relative altitude reached, in metres.

    Raises:
        ValueError: ``alt_m`` or ``climb_rate_m_s`` out of range (nothing is sent).
        CommandError: Not GUIDED, not armed, or no HEARTBEAT in the last 2 s
            (nothing is sent).
        CommandDenied: Takeoff rejected.
        CommandTimeout: No ACK.
        StateNotReached: Stalled, left GUIDED, disarmed, or timed out.
    """
    if not 0 < alt_m <= MAX_TAKEOFF_ALT_M:
        raise ValueError("alt_m must be in (0, %.1f] m, got %r" % (MAX_TAKEOFF_ALT_M, alt_m))
    if climb_rate_m_s <= 0:
        raise ValueError("climb_rate_m_s must be > 0")
    guided_id = _mode_id(link, "GUIDED")
    hb = link.latest("HEARTBEAT")
    hb_age_s = link.age("HEARTBEAT")
    if hb is None or hb_age_s >= PRECONDITION_MAX_AGE_S:
        raise CommandError("Cannot take off: no HEARTBEAT in the last %.1f s"
                           % PRECONDITION_MAX_AGE_S)
    if hb.custom_mode != guided_id:
        raise CommandError("Cannot take off: mode is custom_mode=%d, not GUIDED" % hb.custom_mode)
    if not _is_armed(hb):
        raise CommandError("Cannot take off: vehicle is not armed")

    timeout_s = alt_m / climb_rate_m_s * 1.5 + 10.0
    target_m = TAKEOFF_REACHED_FRACTION * alt_m
    logger.info("Taking off to %.1f m (timeout %.1f s)", alt_m, timeout_s)
    sent_at = time.monotonic()
    command_long(link, mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, alt_m])

    deadline = sent_at + timeout_s
    window_start = sent_at
    window_alt_m = None  # type: Optional[float]
    last_logged_m = None  # type: Optional[float]
    while True:
        now = time.monotonic()
        hb, hb_rx = link.latest_with_time("HEARTBEAT")
        if hb is not None and hb_rx > sent_at and _is_autopilot_heartbeat(link, hb):
            if hb.custom_mode != guided_id:
                raise StateNotReached("Takeoff aborted: vehicle left GUIDED (custom_mode=%d)"
                                      % hb.custom_mode, timeout_s)
            if not _is_armed(hb):
                raise StateNotReached("Takeoff aborted: vehicle disarmed", timeout_s)

        pos, pos_rx = link.latest_with_time("GLOBAL_POSITION_INT")
        if pos is not None and pos_rx > sent_at:
            rel_alt_m = pos.relative_alt / 1000.0
            if last_logged_m is None or abs(rel_alt_m - last_logged_m) >= 1.0:
                logger.info("Takeoff altitude %.1f / %.1f m", rel_alt_m, alt_m)
                last_logged_m = rel_alt_m
            if rel_alt_m >= target_m:
                logger.info("Takeoff complete at %.1f m", rel_alt_m)
                return rel_alt_m
            if window_alt_m is None:
                window_alt_m = rel_alt_m
            elif rel_alt_m - window_alt_m >= TAKEOFF_STALL_MIN_CLIMB_M:
                window_start, window_alt_m = now, rel_alt_m

        if now - window_start >= TAKEOFF_STALL_WINDOW_S:
            if window_alt_m is None:
                reason = "no fresh GLOBAL_POSITION_INT for %.0f s" % TAKEOFF_STALL_WINDOW_S
            else:
                reason = "climbed < %.1f m in %.0f s (stuck near %.1f m)" % (
                    TAKEOFF_STALL_MIN_CLIMB_M, TAKEOFF_STALL_WINDOW_S, window_alt_m)
            raise StateNotReached("Takeoff stalled: " + reason, timeout_s)
        if now >= deadline:
            raise StateNotReached("Takeoff did not reach %.1f m" % target_m, timeout_s)
        time.sleep(_POLL_INTERVAL_S)
