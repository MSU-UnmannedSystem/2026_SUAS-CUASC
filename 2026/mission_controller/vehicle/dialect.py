"""The MAVLink dialect module (``mavlink.MAV_*`` constants and message classes).

At runtime this is ``mavutil.mavlink``, the dialect pymavlink loaded. pymavlink
types that as possibly None (it's None until a dialect is loaded), so type
checkers see the ArduPilot dialect module instead, which has the same names.
"""

from typing import TYPE_CHECKING

from pymavlink import mavutil

if TYPE_CHECKING:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
else:
    mavlink = mavutil.mavlink

__all__ = ["mavlink"]
