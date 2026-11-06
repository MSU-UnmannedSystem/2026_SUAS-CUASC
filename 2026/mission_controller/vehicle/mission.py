"""Read-only mission access: where the handover is in the loaded mission.

The mission is planned, checked and uploaded from Mission Planner, so the
Jetson never uploads and doesn't validate it. It downloads the mission
(MAVLink mission protocol, copied from drone_simulator/vehicle/upload.py) only
to find the items it hands back at: NAV_GUIDED_ENABLE, the release
DO_SET_SERVO, and the NAV item to resume at.
"""

import logging
import queue
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence

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


@dataclass
class HandoverPlan:
    """Where the handover is in the loaded mission.

    Attributes:
        guided_seq: The NAV_GUIDED_ENABLE item; the Jetson may guide only while
            MISSION_CURRENT is this seq and the mode is AUTO.
        release_seq: The release DO_SET_SERVO after it. Handing back here makes
            the FC run it (release) and continue. None if the mission has none:
            a lock then hands back at ``resume_seq`` and nothing is released.
        resume_seq: The first NAV item after the handover (and the release).
            Handing back here skips the release (lost lock).
    """

    guided_seq: int
    release_seq: Optional[int]
    resume_seq: int


def _is_nav(item: MissionItem) -> bool:
    return item.command < _NAV_CMD_LAST


def find_handover(items: Sequence[MissionItem], servo_channel: int) -> Optional[HandoverPlan]:
    """Finds the handover items in a downloaded mission. Doesn't validate the mission.

    Expected layout (built in Mission Planner)::

        ... NAV waypoint(s) taking the FC to the target and down to the drop altitude
        DO_GUIDED_LIMITS     timeout, min alt, max alt, max horizontal distance
        NAV_GUIDED_ENABLE    param1 = 1   (the Jetson centres here)
        DO_JUMP              to the resume waypoint, repeat -1   (guard)
        DO_SET_SERVO         servo_channel, release PWM   (the FC releases)
        NAV ...              rest of the mission (the resume waypoint)

    Only the first NAV_GUIDED_ENABLE (param1 = 1) is used. The release is the
    first DO_SET_SERVO on ``servo_channel`` between it and the next NAV item.

    Args:
        items: The mission as downloaded (seq 0 = home).
        servo_channel: Release servo output.

    Returns:
        The handover plan, or None if the mission has no NAV_GUIDED_ENABLE or no
        NAV item after it to hand back at (the Jetson then never guides).
    """
    guided_seq = next((seq for seq, it in enumerate(items)
                       if it.command == mavlink.MAV_CMD_NAV_GUIDED_ENABLE and it.param1 > 0.5),
                      None)
    if guided_seq is None:
        logger.warning("No NAV_GUIDED_ENABLE (param1=1) in the mission: the Jetson won't guide "
                       "this flight")
        return None
    resume_seq = next((s for s in range(guided_seq + 1, len(items)) if _is_nav(items[s])), None)
    if resume_seq is None:
        logger.warning("No NAV item after NAV_GUIDED_ENABLE (seq %d) to hand back at: the Jetson "
                       "won't guide this flight", guided_seq)
        return None
    release_seq = next((s for s in range(guided_seq + 1, resume_seq)
                        if items[s].command == mavlink.MAV_CMD_DO_SET_SERVO
                        and int(items[s].param1) == servo_channel), None)
    jump_seq = next((s for s in range(guided_seq + 1, resume_seq)
                     if items[s].command == mavlink.MAV_CMD_DO_JUMP), None)
    if release_seq is None:
        logger.warning("No DO_SET_SERVO on servo %d after NAV_GUIDED_ENABLE (seq %d): a lock "
                       "hands back at item %d and nothing is released", servo_channel,
                       guided_seq, resume_seq)
        release_text = "no release"
    else:
        release_text = "release %d (servo %d, PWM %d)" % (release_seq, servo_channel,
                                                          int(items[release_seq].param2))
    logger.info("Handover: NAV_GUIDED_ENABLE %d -> %s -> %s -> resume %d", guided_seq,
                "no guard DO_JUMP" if jump_seq is None else "guard DO_JUMP %d" % jump_seq,
                release_text, resume_seq)
    return HandoverPlan(guided_seq, release_seq, resume_seq)
