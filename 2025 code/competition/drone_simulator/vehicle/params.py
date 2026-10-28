"""Autopilot parameter access over a :class:`~competition.drone_simulator.vehicle.link.Link`.

ArduPilot 4.7 renamed many parameters (e.g. ``SYSID_MYGCS`` -> ``MAV_GCS_SYSID``,
``WPNAV_SPEED_UP`` (cm/s) -> ``WP_SPD_UP``), so reads take several candidate
names and return the first one in order that the autopilot has.
"""

import logging
import queue
import time
from typing import Dict, NamedTuple, Optional

from pymavlink import mavutil

from competition.drone_simulator.vehicle.commands import CommandError
from competition.drone_simulator.vehicle.link import Link

logger = logging.getLogger(__name__)

mavlink = mavutil.mavlink

# After the first PARAM_VALUE answer, how long to keep waiting for a
# higher-priority name (ArduPilot answers reads within a few ms).
_PRIORITY_GRACE_S = 0.5
# WP_SPD_UP's unit is unconfirmed; above this it must be cm/s (2.5 m/s = 250 cm/s).
_CM_S_THRESHOLD = 20.0


class ParamError(CommandError):
    """A parameter set was not confirmed by the autopilot."""


class ParamValue(NamedTuple):
    """A parameter read from the autopilot.

    Attributes:
        name: The parameter name that answered.
        value: Its value (PARAM_VALUE is always float on the wire).
    """

    name: str
    value: float


def read_param(link: Link, *names: str, timeout_s: float = 2.0,
               retries: int = 3) -> Optional[ParamValue]:
    """Reads the first of ``names`` (in order) that the autopilot has.

    All names are requested at once; the result is the earliest name in
    ``names`` that answered. The autopilot does not reply for unknown names, so
    a missing name only costs time.

    Args:
        link: Connected link.
        *names: Candidate parameter names, most preferred first.
        timeout_s: Seconds to wait for answers per attempt.
        retries: Total attempts.

    Returns:
        The name and value that answered, or None if none of ``names`` did.

    Raises:
        ValueError: No names given.
    """
    if not names:
        raise ValueError("read_param needs at least one name")
    found = {}  # type: Dict[str, float]
    # Subscribe BEFORE sending so a fast PARAM_VALUE cannot be missed.
    with link.subscribe("PARAM_VALUE") as sub:
        for _ in range(retries):
            for name in names:
                link.send("param_request_read_send", link.target_system, link.target_component,
                          name.encode("ascii"), -1)
            deadline = time.monotonic() + timeout_s
            while True:
                if names[0] in found:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    msg = sub.get(timeout=remaining)
                except queue.Empty:
                    break
                if msg.get_srcSystem() != link.target_system or msg.param_id not in names:
                    continue
                if not found:
                    deadline = min(deadline, time.monotonic() + _PRIORITY_GRACE_S)
                found[msg.param_id] = msg.param_value
            if found:
                best = next(n for n in names if n in found)
                logger.debug("Param %s = %g", best, found[best])
                return ParamValue(best, found[best])
    logger.info("None of %s answered", ", ".join(names))
    return None


def set_param(link: Link, name: str, value: float, timeout_s: float = 3.0) -> None:
    """Sets a parameter and waits for the autopilot's PARAM_VALUE echo of the new value.

    Args:
        link: Connected link.
        name: Exact parameter name.
        value: New value (sent as float, as on the wire).
        timeout_s: Seconds to wait for the echo.

    Raises:
        ParamError: No PARAM_VALUE with the new value within ``timeout_s``.
    """
    with link.subscribe("PARAM_VALUE") as sub:
        link.send("param_set_send", link.target_system, link.target_component,
                  name.encode("ascii"), float(value), mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                msg = sub.get(timeout=remaining)
            except queue.Empty:
                break
            if (msg.get_srcSystem() == link.target_system and msg.param_id == name
                    and abs(msg.param_value - value) < 1e-3):
                logger.info("Set %s = %g", name, value)
                return
    raise ParamError("No PARAM_VALUE confirming %s = %g within %.1f s" % (name, value, timeout_s))


def climb_speed_m_s(link: Link, default_m_s: float = 2.5) -> float:
    """Returns the autopilot's waypoint climb speed in m/s.

    Reads ``WP_SPD_UP`` (4.7+) then ``WPNAV_SPEED_UP`` (cm/s). ``WP_SPD_UP``'s
    unit is unconfirmed, so a value above 20 is treated as cm/s; the rule used
    is logged.

    Args:
        link: Connected link.
        default_m_s: Returned (with a warning) if neither parameter answers or the
            value is not positive, in m/s.

    Returns:
        Climb speed in m/s.
    """
    param = read_param(link, "WP_SPD_UP", "WPNAV_SPEED_UP", timeout_s=1.0, retries=2)
    if param is None:
        logger.warning("Climb speed unknown (no WP_SPD_UP / WPNAV_SPEED_UP); using %.2f m/s",
                       default_m_s)
        return default_m_s
    if param.name == "WPNAV_SPEED_UP":
        speed_m_s, rule = param.value / 100.0, "WPNAV_SPEED_UP is cm/s"
    elif param.value > _CM_S_THRESHOLD:
        speed_m_s, rule = param.value / 100.0, "WP_SPD_UP > %g treated as cm/s" % _CM_S_THRESHOLD
    else:
        speed_m_s, rule = param.value, "WP_SPD_UP <= %g treated as m/s" % _CM_S_THRESHOLD
    if speed_m_s <= 0:
        logger.warning("%s = %g is not a usable climb speed; using %.2f m/s",
                       param.name, param.value, default_m_s)
        return default_m_s
    logger.info("Climb speed %.2f m/s from %s = %g (%s)", speed_m_s, param.name, param.value, rule)
    return speed_m_s
