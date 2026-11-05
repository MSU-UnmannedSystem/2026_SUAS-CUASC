"""Read-only mission access and the pre-flight handover check.

The mission is planned and uploaded from Mission Planner, so the Jetson never
uploads. It downloads the mission (MAVLink mission protocol, copied from
drone_simulator/vehicle/upload.py) and checks that it contains exactly one
handover: DO_GUIDED_LIMITS, NAV_GUIDED_ENABLE (param1 = 1), a guard DO_JUMP,
the release DO_SET_SERVO, and a NAV item to resume at.
"""

import logging
import queue
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Sequence

from pymavlink import mavutil

from mission_controller.vehicle.commands import _enum_name
from mission_controller.vehicle.link import Link, Subscription

logger = logging.getLogger(__name__)

mavlink = mavutil.mavlink

# MAV_CMD ids below this are NAV commands (the mission's "path" items).
_NAV_CMD_LAST = mavlink.MAV_CMD_NAV_LAST


@dataclass
class MissionItem:
    """One mission/fence item, in degrees and metres.

    Attributes:
        command: MAV_CMD id, e.g. ``MAV_CMD_NAV_WAYPOINT``.
        frame: MAV_FRAME id, e.g. ``MAV_FRAME_GLOBAL_RELATIVE_ALT_INT``.
        param1: Command-specific (NAV_WAYPOINT: hold time in s).
        param2: Command-specific.
        param3: Command-specific.
        param4: Command-specific.
        lat_deg: Latitude in degrees.
        lon_deg: Longitude in degrees.
        alt_m: Altitude in metres, relative to ``frame``.
        autocontinue: 1 to continue to the next item automatically.
    """

    command: int
    frame: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_m: float = 0.0
    autocontinue: int = 1


class MissionError(Exception):
    """Base class for mission transfer failures."""


def _mission_type_kwargs(mission_type: int) -> Dict[str, int]:
    # mission_type is a MAVLink2 extension field; omit it for plain missions so
    # they also work on a MAVLink1 link.
    if mission_type == mavlink.MAV_MISSION_TYPE_MISSION:
        return {}
    return {"mission_type": mission_type}


def _for_us(link: Link, msg: Any, mission_type: int) -> bool:
    """True if ``msg`` is from our autopilot, for ``mission_type``, addressed to us."""
    if msg.get_srcSystem() != link.target_system:
        return False
    if getattr(msg, "mission_type", 0) != mission_type:
        return False
    # Mission Planner shares sysid 255; replies to it carry its compid (190).
    target = (getattr(msg, "target_system", 0), getattr(msg, "target_component", 0))
    return (target[0] in (0, link.source_system)
            and target[1] in (0, link.source_component))


def _request_until(link: Link, sub: Subscription, send: Callable[[], None],
                   accept: Callable[[Any], bool], mission_type: int, timeout_s: float,
                   retries: int, what: str) -> Any:
    send()
    resends_left = retries
    while True:
        try:
            msg = sub.get(timeout=timeout_s)
        except queue.Empty:
            if resends_left == 0:
                raise MissionError("No %s after %d re-send(s)" % (what, retries))
            resends_left -= 1
            send()
            continue
        if not _for_us(link, msg, mission_type):
            continue
        if msg.get_type() == "MISSION_ACK":
            raise MissionError("Autopilot aborted the download while waiting for %s: %s"
                               % (what, _enum_name("MAV_MISSION_RESULT", msg.type)))
        if accept(msg):
            return msg


def download_items(
    link: Link,
    mission_type: int = mavlink.MAV_MISSION_TYPE_MISSION,
    timeout_s: float = 1.5,
    retries: int = 3,
) -> List[MissionItem]:
    """Reads back everything the autopilot stores for ``mission_type``.

    Args:
        link: Connected link.
        mission_type: MAV_MISSION_TYPE (MISSION, FENCE or RALLY).
        timeout_s: Seconds to wait for each reply before re-requesting.
        retries: Re-requests allowed per reply.

    Returns:
        Items in seq order (for a mission, seq 0 is the autopilot's home).

    Raises:
        MissionError: No reply after ``retries`` re-requests, or an error MISSION_ACK.
    """
    mt = _mission_type_kwargs(mission_type)
    ts, tc = link.target_system, link.target_component
    with link.subscribe("MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK") as sub:
        count_msg = _request_until(
            link, sub, lambda: link.send("mission_request_list_send", ts, tc, **mt),
            lambda m: m.get_type() == "MISSION_COUNT", mission_type, timeout_s, retries,
            "MISSION_COUNT")
        items = []
        for seq in range(count_msg.count):
            msg = _request_until(
                link, sub, lambda s=seq: link.send("mission_request_int_send", ts, tc, s, **mt),
                lambda m, s=seq: m.get_type() == "MISSION_ITEM_INT" and m.seq == s,
                mission_type, timeout_s, retries, "MISSION_ITEM_INT %d" % seq)
            items.append(MissionItem(msg.command, msg.frame, msg.param1, msg.param2, msg.param3,
                                     msg.param4, msg.x / 1e7, msg.y / 1e7, msg.z,
                                     msg.autocontinue))
        link.send("mission_ack_send", ts, tc, mavlink.MAV_MISSION_ACCEPTED, **mt)
    logger.info("Downloaded %d %s item(s)", len(items),
                _enum_name("MAV_MISSION_TYPE", mission_type))
    return items


class MissionCheckError(MissionError):
    """The loaded mission is not safe or not usable for the handover."""


@dataclass
class HandoverPlan:
    """Where the handover happens in the loaded mission, and the FC's limits for it.

    Attributes:
        target_seq: The NAV waypoint before the handover (the FC descends to the drop
            altitude on its way to / at it).
        target_alt_m: That waypoint's altitude, in metres (informational).
        limits_seq: The DO_GUIDED_LIMITS item.
        guided_seq: The NAV_GUIDED_ENABLE item; the Jetson may guide only while
            MISSION_CURRENT is this seq and the mode is AUTO.
        jump_seq: The guard DO_JUMP right after NAV_GUIDED_ENABLE, to ``resume_seq``.
            When the FC ends the handover itself it reads this first and skips
            the release.
        release_seq: The DO_SET_SERVO after the guard. Handing back here makes the
            FC run it (release) and continue; the guard is never read.
        resume_seq: The first NAV item after the release. Handing back here skips
            the release (lost lock).
        timeout_s: DO_GUIDED_LIMITS param1, in seconds.
        alt_min_m: param2, in metres above the EKF origin (0 = no limit).
        alt_max_m: param3, in metres above the EKF origin (0 = no limit).
        horiz_max_m: param4, maximum horizontal distance from where guided started, m.
    """

    target_seq: int
    target_alt_m: float
    limits_seq: int
    guided_seq: int
    jump_seq: int
    release_seq: int
    resume_seq: int
    timeout_s: float
    alt_min_m: float
    alt_max_m: float
    horiz_max_m: float


def _is_nav(item: MissionItem) -> bool:
    return item.command < _NAV_CMD_LAST


def check_mission(items: Sequence[MissionItem], drop_alt_m: float, servo_channel: int,
                  release_pwm_us: int) -> HandoverPlan:
    """Finds the handover in a downloaded mission and checks it is flyable.

    Expected layout (one target per flight)::

        ... NAV waypoint(s) taking the FC to the target and down to the drop altitude
        DO_GUIDED_LIMITS     timeout, min alt, max alt, max horizontal distance
        NAV_GUIDED_ENABLE    param1 = 1   (the Jetson centres here)
        DO_JUMP              to the resume waypoint, repeat -1   (guard)
        DO_SET_SERVO         servo_channel, release_pwm_us   (the FC releases)
        NAV ...              rest of the mission (the resume waypoint)

    ArduPilot follows a DO_JUMP only when it reads it while advancing. When the
    FC ends the handover itself (limits timeout, limit breach, dead Jetson) it
    advances from the item after NAV_GUIDED_ENABLE, reads the guard and jumps
    past the release. The Jetson hands back by making the DO_SET_SERVO current,
    so the search starts at the release and the guard is never read.

    Rules (all problems are reported together):

    * exactly one NAV_GUIDED_ENABLE with param1 = 1, with a NAV waypoint before it;
    * a DO_GUIDED_LIMITS between that waypoint and the NAV_GUIDED_ENABLE with a
      timeout > 0, a horizontal limit > 0, 0 < alt_min < ``drop_alt_m`` and
      alt_max = 0 or above ``drop_alt_m`` (so the FC doesn't end the handover at
      the drop altitude);
    * right after NAV_GUIDED_ENABLE: DO_JUMP to the resume waypoint with repeat -1,
      then DO_SET_SERVO(servo_channel, release_pwm_us);
    * a NAV item after that to resume at (the jump target).

    The drop altitude itself is not checked here; the Jetson checks the actual
    altitude when the handover starts.

    Args:
        items: The mission as downloaded (seq 0 = home).
        drop_alt_m: Planned drop altitude, in metres above home.
        servo_channel: Release servo output.
        release_pwm_us: Release PWM, in microseconds.

    Returns:
        The handover plan.

    Raises:
        MissionCheckError: Any rule is broken.
    """
    enables = [seq for seq, it in enumerate(items)
               if it.command == mavlink.MAV_CMD_NAV_GUIDED_ENABLE and it.param1 > 0.5]
    if len(enables) != 1:
        raise MissionCheckError(
            "Mission needs exactly one NAV_GUIDED_ENABLE (param1=1), found %d at seq %s"
            % (len(enables), enables))
    guided_seq = enables[0]
    problems = []  # type: List[str]

    target_seq = next((s for s in range(guided_seq - 1, 0, -1) if _is_nav(items[s])), None)
    if target_seq is None:
        raise MissionCheckError("No waypoint before NAV_GUIDED_ENABLE (seq %d)" % guided_seq)
    target = items[target_seq]

    limits_seq = next((s for s in range(target_seq + 1, guided_seq)
                       if items[s].command == mavlink.MAV_CMD_DO_GUIDED_LIMITS), None)
    if limits_seq is None:
        raise MissionCheckError(
            "No DO_GUIDED_LIMITS between waypoint seq %d and NAV_GUIDED_ENABLE seq %d: the FC "
            "would have no timeout or distance limit for the handover" % (target_seq, guided_seq))
    limits = items[limits_seq]
    timeout_s, alt_min_m, alt_max_m, horiz_max_m = (
        limits.param1, limits.param2, limits.param3, limits.param4)
    if timeout_s <= 0:
        problems.append("DO_GUIDED_LIMITS timeout (param1) must be > 0 s")
    if horiz_max_m <= 0:
        problems.append("DO_GUIDED_LIMITS horizontal limit (param4) must be > 0 m")
    if not 0 < alt_min_m < drop_alt_m:
        problems.append("DO_GUIDED_LIMITS min alt (param2) %.2f m must be > 0 and below the "
                        "drop alt %.2f m" % (alt_min_m, drop_alt_m))
    if alt_max_m != 0 and alt_max_m <= drop_alt_m:
        problems.append("DO_GUIDED_LIMITS max alt (param3) %.1f m must be 0 or above the drop "
                        "alt %.2f m" % (alt_max_m, drop_alt_m))

    jump_seq = guided_seq + 1
    release_seq = guided_seq + 2
    release = items[release_seq] if release_seq < len(items) else None
    if (release is None or release.command != mavlink.MAV_CMD_DO_SET_SERVO
            or int(release.param1) != servo_channel or int(release.param2) != release_pwm_us):
        problems.append("item %d must be DO_SET_SERVO servo %d PWM %d, the release (two after "
                        "NAV_GUIDED_ENABLE)" % (release_seq, servo_channel, release_pwm_us))
    resume_seq = next((s for s in range(release_seq + 1, len(items)) if _is_nav(items[s])), None)
    if resume_seq is None:
        problems.append("no NAV item after the release (seq %d) to resume the mission at"
                        % release_seq)
    jump = items[jump_seq] if jump_seq < len(items) else None
    if (jump is None or jump.command != mavlink.MAV_CMD_DO_JUMP or resume_seq is None
            or int(jump.param1) != resume_seq or int(jump.param2) != -1):
        problems.append("item %d (right after NAV_GUIDED_ENABLE) must be DO_JUMP to the resume "
                        "waypoint %s with repeat -1: it makes the FC skip the release when it ends "
                        "the handover itself" % (jump_seq, resume_seq))
    if problems:
        raise MissionCheckError("Mission handover check failed: " + "; ".join(problems))

    plan = HandoverPlan(target_seq, target.alt_m, limits_seq, guided_seq, jump_seq, release_seq,
                        resume_seq, timeout_s, alt_min_m, alt_max_m, horiz_max_m)
    logger.info("Handover: waypoint %d (%.1f m) -> limits %d (timeout %.0f s, alt %.1f-%s m, "
                "horiz %.0f m) -> NAV_GUIDED_ENABLE %d -> guard DO_JUMP %d -> release %d "
                "(servo %d) -> resume %d",
                target_seq, target.alt_m, limits_seq, timeout_s, alt_min_m,
                "%.1f" % alt_max_m if alt_max_m else "no max", horiz_max_m, guided_seq,
                jump_seq, release_seq, servo_channel, resume_seq)
    return plan
