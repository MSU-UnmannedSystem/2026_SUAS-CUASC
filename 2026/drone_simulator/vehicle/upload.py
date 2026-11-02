"""Mission and fence transfer over a :class:`~competition.drone_simulator.vehicle.link.Link`.

Implements the MAVLink mission protocol (MISSION_COUNT -> MISSION_REQUEST(_INT)
-> MISSION_ITEM_INT ... -> MISSION_ACK), answering requests by seq in any order,
and reads missions back to verify what the autopilot stored. Non-default
mission types (fence) need a MAVLink2 link, which ArduPilot ports are.
"""

import logging
import math
import queue
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

from pymavlink import mavutil

from competition.drone_simulator.vehicle.commands import _enum_name
from competition.drone_simulator.vehicle.link import Link, Subscription

logger = logging.getLogger(__name__)

mavlink = mavutil.mavlink

MAX_WAYPOINTS = 50
MAX_WAYPOINT_ALT_M = 121.9  # 400 ft
_LATLON_TOLERANCE_DEG = 1e-7 + 1e-9  # int(deg * 1e7) truncates by < 1e-7
_ALT_TOLERANCE_M = 0.01 + 1e-6


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


class MissionUploadError(MissionError):
    """The autopilot rejected the transfer, or stopped answering."""


class MissionVerifyError(MissionError):
    """The mission read back from the autopilot differs from what was uploaded."""


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


def _send_item(link: Link, seq: int, item: MissionItem, mission_type: int) -> None:
    link.send("mission_item_int_send", link.target_system, link.target_component, seq,
              item.frame, item.command, 0, item.autocontinue,
              item.param1, item.param2, item.param3, item.param4,
              int(item.lat_deg * 1e7), int(item.lon_deg * 1e7), float(item.alt_m),
              **_mission_type_kwargs(mission_type))


def upload_items(
    link: Link,
    items: Sequence[MissionItem],
    mission_type: int = mavlink.MAV_MISSION_TYPE_MISSION,
    timeout_s: float = 1.5,
    retries: int = 3,
) -> None:
    """Uploads ``items`` (replacing what the autopilot has for ``mission_type``).

    Every MISSION_REQUEST or MISSION_REQUEST_INT is answered by its seq with a
    MISSION_ITEM_INT, so out-of-order and repeated requests are fine. If
    nothing arrives for ``timeout_s``, the last message sent (the COUNT or the
    last item) is sent again, up to ``retries`` times in a row.

    Args:
        link: Connected link.
        items: Items in seq order (seq 0 is home for a mission on ArduPilot).
        mission_type: MAV_MISSION_TYPE (MISSION, FENCE or RALLY).
        timeout_s: Seconds of silence before re-sending.
        retries: Re-sends allowed in a row before giving up.

    Raises:
        MissionUploadError: Error MISSION_ACK, an invalid request, or no answer
            after ``retries`` re-sends.
    """
    count = len(items)
    requested = set()  # type: Set[int]
    kind = _enum_name("MAV_MISSION_TYPE", mission_type)
    logger.info("Uploading %d %s item(s)", count, kind)
    # Subscribe BEFORE sending MISSION_COUNT so the first request can't be missed.
    with link.subscribe("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
                        maxsize=max(100, 4 * count)) as sub:

        def send_count() -> None:
            link.send("mission_count_send", link.target_system, link.target_component, count,
                      **_mission_type_kwargs(mission_type))

        resend = send_count  # type: Callable[[], None]
        resend()
        resends_left = retries
        while True:
            try:
                msg = sub.get(timeout=timeout_s)
            except queue.Empty:
                if resends_left == 0:
                    raise MissionUploadError(
                        "%s upload timed out: no reply for %.1f s after %d re-send(s); "
                        "%d of %d item(s) requested" % (kind, timeout_s, retries,
                                                        len(requested), count))
                resends_left -= 1
                logger.warning("No reply for %.1f s during %s upload; re-sending", timeout_s, kind)
                resend()
                continue
            if not _for_us(link, msg, mission_type):
                continue
            resends_left = retries
            if msg.get_type() == "MISSION_ACK":
                if msg.type != mavlink.MAV_MISSION_ACCEPTED:
                    raise MissionUploadError("%s upload rejected: %s"
                                             % (kind, _enum_name("MAV_MISSION_RESULT", msg.type)))
                if len(requested) < count:
                    raise MissionUploadError("%s upload ACCEPTED after only %d of %d item(s)"
                                             % (kind, len(requested), count))
                logger.info("Upload of %d %s item(s) accepted", count, kind)
                return
            seq = msg.seq
            if not 0 <= seq < count:
                raise MissionUploadError("Autopilot requested %s seq %d of %d" % (kind, seq, count))
            _send_item(link, seq, items[seq], mission_type)
            requested.add(seq)
            logger.debug("Sent %s item %d/%d", kind, seq, count - 1)
            resend = (lambda s=seq: _send_item(link, s, items[s], mission_type))


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


def _validate_latlon(lat_deg: float, lon_deg: float, label: str) -> None:
    if not (math.isfinite(lat_deg) and math.isfinite(lon_deg)
            and abs(lat_deg) <= 90 and abs(lon_deg) <= 180):
        raise ValueError("%s: lat/lon (%r, %r) out of range" % (label, lat_deg, lon_deg))


def build_waypoint_mission(
    waypoints: Sequence[Tuple[float, float, float]],
    hold_s: float = 0.0,
    end_action: Optional[str] = "RTL",
) -> List[MissionItem]:
    """Builds a mission: home placeholder, one NAV_WAYPOINT per waypoint, then RTL.

    Args:
        waypoints: ``(lat_deg, lon_deg, alt_m)`` with alt in METRES above home
            (no unit conversion here). 1 to 50 waypoints, 0 < alt_m <= 121.9.
        hold_s: Hold time at each waypoint, in seconds (NAV_WAYPOINT param1).
        end_action: ``"RTL"`` to append NAV_RETURN_TO_LAUNCH, or None for no end item
            (ArduCopter then holds at the last waypoint).

    Returns:
        Items for :func:`upload_items`: seq 0 home, seq 1..N waypoints, then the end item.

    Raises:
        ValueError: Bad waypoint count, lat/lon, altitude, hold time or end_action.
    """
    if not 1 <= len(waypoints) <= MAX_WAYPOINTS:
        raise ValueError("Need 1 to %d waypoints, got %d" % (MAX_WAYPOINTS, len(waypoints)))
    if hold_s < 0:
        raise ValueError("hold_s must be >= 0")
    if end_action not in ("RTL", None):
        raise ValueError("end_action must be 'RTL' or None, got %r" % (end_action,))
    for i, (lat_deg, lon_deg, alt_m) in enumerate(waypoints, start=1):
        _validate_latlon(lat_deg, lon_deg, "waypoint %d" % i)
        if not 0 < alt_m <= MAX_WAYPOINT_ALT_M:
            raise ValueError("waypoint %d: alt %r m outside (0, %.1f] m (400 ft)"
                             % (i, alt_m, MAX_WAYPOINT_ALT_M))
    first_lat, first_lon, _ = waypoints[0]
    # Seq 0 is home on ArduPilot, which manages home itself (it overwrites this
    # item); the placeholder just keeps the waypoints at seq 1..N.
    items = [MissionItem(mavlink.MAV_CMD_NAV_WAYPOINT, mavlink.MAV_FRAME_GLOBAL_INT,
                         lat_deg=first_lat, lon_deg=first_lon, alt_m=0.0)]
    for lat_deg, lon_deg, alt_m in waypoints:
        items.append(MissionItem(mavlink.MAV_CMD_NAV_WAYPOINT,
                                 mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                 param1=hold_s, lat_deg=lat_deg, lon_deg=lon_deg, alt_m=alt_m))
    if end_action == "RTL":
        items.append(MissionItem(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                 mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT))
    return items


def _compare(expected: Sequence[MissionItem], actual: Sequence[MissionItem], first_seq: int,
             check_alt: bool) -> List[str]:
    problems = []
    if len(actual) != len(expected):
        problems.append("autopilot has %d item(s), uploaded %d" % (len(actual), len(expected)))
    for seq in range(first_seq, min(len(expected), len(actual))):
        exp, got = expected[seq], actual[seq]
        if got.command != exp.command:
            problems.append("seq %d: command %d != %d" % (seq, got.command, exp.command))
        if (abs(got.lat_deg - exp.lat_deg) > _LATLON_TOLERANCE_DEG
                or abs(got.lon_deg - exp.lon_deg) > _LATLON_TOLERANCE_DEG):
            problems.append("seq %d: position (%.7f, %.7f) != (%.7f, %.7f)"
                            % (seq, got.lat_deg, got.lon_deg, exp.lat_deg, exp.lon_deg))
        if check_alt and abs(got.alt_m - exp.alt_m) > _ALT_TOLERANCE_M:
            problems.append("seq %d: alt %.2f m != %.2f m" % (seq, got.alt_m, exp.alt_m))
    return problems


def upload_waypoint_mission(
    link: Link,
    waypoints: Sequence[Tuple[float, float, float]],
    hold_s: float = 0.0,
    end_action: Optional[str] = "RTL",
    timeout_s: float = 1.5,
    retries: int = 3,
) -> int:
    """Builds, uploads and read-back-verifies a waypoint mission.

    Args:
        link: Connected link.
        waypoints: ``(lat_deg, lon_deg, alt_m)``, alt in metres above home.
        hold_s: Hold time at each waypoint, in seconds.
        end_action: ``"RTL"`` or None, see :func:`build_waypoint_mission`.
        timeout_s: Per-reply timeout for upload and download, in seconds.
        retries: Re-sends allowed per reply.

    Returns:
        last_nav_seq: seq of the last waypoint (= ``len(waypoints)``). The mission
        is complete when MISSION_ITEM_REACHED reports a seq >= this.

    Raises:
        ValueError: Invalid waypoints (nothing is sent).
        MissionUploadError: Upload failed.
        MissionVerifyError: Read-back differs (seq >= 1: command, lat/lon within
            1e-7 deg, alt within 0.01 m).
    """
    items = build_waypoint_mission(waypoints, hold_s=hold_s, end_action=end_action)
    upload_items(link, items, timeout_s=timeout_s, retries=retries)
    stored = download_items(link, timeout_s=timeout_s, retries=retries)
    problems = _compare(items, stored, first_seq=1, check_alt=True)
    if problems:
        raise MissionVerifyError("Mission read-back mismatch: " + "; ".join(problems))
    logger.info("Mission verified: %d waypoint(s)%s", len(waypoints),
                " + RTL" if end_action == "RTL" else "")
    return len(waypoints)


def upload_fence(
    link: Link,
    polygon: Sequence[Tuple[float, float]],
    timeout_s: float = 1.5,
    retries: int = 3,
) -> int:
    """Uploads an inclusion polygon fence and verifies it by read-back.

    Replaces every fence item on the autopilot. Does NOT enable the fence
    (FENCE_ENABLE / FENCE_TYPE are left alone).

    Args:
        link: Connected (MAVLink2) link.
        polygon: ``(lat_deg, lon_deg)`` vertices in order, at least 3.
        timeout_s: Per-reply timeout, in seconds.
        retries: Re-sends allowed per reply.

    Returns:
        Number of vertices uploaded.

    Raises:
        ValueError: Fewer than 3 vertices or a bad lat/lon (nothing is sent).
        MissionUploadError: Upload failed.
        MissionVerifyError: Read-back differs.
    """
    if len(polygon) < 3:
        raise ValueError("Fence polygon needs at least 3 vertices, got %d" % len(polygon))
    for i, (lat_deg, lon_deg) in enumerate(polygon):
        _validate_latlon(lat_deg, lon_deg, "fence vertex %d" % i)
    items = [MissionItem(mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                         mavlink.MAV_FRAME_GLOBAL_INT, param1=float(len(polygon)),
                         lat_deg=lat_deg, lon_deg=lon_deg)
             for lat_deg, lon_deg in polygon]
    fence = mavlink.MAV_MISSION_TYPE_FENCE
    upload_items(link, items, mission_type=fence, timeout_s=timeout_s, retries=retries)
    stored = download_items(link, mission_type=fence, timeout_s=timeout_s, retries=retries)
    problems = _compare(items, stored, first_seq=0, check_alt=False)
    problems += ["seq %d: vertex count %g != %d" % (seq, got.param1, len(polygon))
                 for seq, got in enumerate(stored) if int(got.param1) != len(polygon)]
    if problems:
        raise MissionVerifyError("Fence read-back mismatch: " + "; ".join(problems))
    logger.info("Fence verified: %d-vertex inclusion polygon (not enabled)", len(polygon))
    return len(polygon)
