"""SITL stand-in for the camera: projects a fixed target GPS point into the image.

SITL has no camera, so this detector computes where a target at a known
lat/lon would appear in a straight-down camera (image top = nose), from the
vehicle's own GLOBAL_POSITION_INT and ATTITUDE. It inverts
guidance.camera_geometry.pixel_to_ground_offset, so guidance sees the same
pixels a real camera would. For SITL only: run_flight refuses it on a serial link.
"""

import logging
import math
import time
from typing import Optional

from mission_controller.vehicle.link import Link
from mission_controller.vision.detector import Detection, TargetDetector

logger = logging.getLogger(__name__)

_M_PER_DEG_LAT = 111319.5


class SimTargetDetector(TargetDetector):
    """Reports a simulated target at ``target_lat_deg`` / ``target_lon_deg``.

    Args:
        link: Connected link (reads GLOBAL_POSITION_INT and ATTITUDE).
        target_lat_deg: Target latitude, degrees.
        target_lon_deg: Target longitude, degrees.
        hfov_deg: Camera horizontal field of view, degrees.
        vfov_deg: Camera vertical field of view, degrees.
        width_px: Simulated frame width.
        height_px: Simulated frame height.
        hide_after_s: If set, the target disappears for good this many seconds
            after the first detection (to test lost-lock handling).
    """

    def __init__(self, link: Link, target_lat_deg: float, target_lon_deg: float,
                 hfov_deg: float = 65.0, vfov_deg: float = 39.4, width_px: int = 640,
                 height_px: int = 480, hide_after_s: Optional[float] = None) -> None:
        self.link = link
        self.target_lat_deg = target_lat_deg
        self.target_lon_deg = target_lon_deg
        self.tan_half_h = math.tan(math.radians(hfov_deg) / 2.0)
        self.tan_half_v = math.tan(math.radians(vfov_deg) / 2.0)
        self.width_px = width_px
        self.height_px = height_px
        self.hide_after_s = hide_after_s
        self._first_seen = None  # type: Optional[float]
        self._hidden_logged = False

    def detect(self) -> Optional[Detection]:
        pos, pos_rx = self.link.latest_with_time("GLOBAL_POSITION_INT")
        att = self.link.latest("ATTITUDE")
        if pos is None or att is None:
            return None
        alt_m = pos.relative_alt / 1000.0
        if alt_m <= 0.5:
            return None
        lat = pos.lat / 1e7
        north_m = (self.target_lat_deg - lat) * _M_PER_DEG_LAT
        east_m = ((self.target_lon_deg - pos.lon / 1e7) * _M_PER_DEG_LAT
                  * math.cos(math.radians(lat)))
        yaw = att.yaw
        forward_m = north_m * math.cos(yaw) + east_m * math.sin(yaw)
        right_m = -north_m * math.sin(yaw) + east_m * math.cos(yaw)
        # Inverse of camera_geometry: undo the roll / pitch tilt, then the pinhole.
        right_angle = math.atan2(right_m, alt_m) - att.roll
        forward_angle = math.atan2(forward_m, alt_m) + att.pitch
        nx = math.tan(right_angle) / self.tan_half_h
        ny = -math.tan(forward_angle) / self.tan_half_v
        if abs(nx) > 1.0 or abs(ny) > 1.0:
            return None  # outside the frame
        now = time.monotonic()
        if self._first_seen is None:
            self._first_seen = now
        if self.hide_after_s is not None and now - self._first_seen >= self.hide_after_s:
            if not self._hidden_logged:
                logger.warning("[SIM] Target hidden (simulated lost lock)")
                self._hidden_logged = True
            return None
        return Detection(self.width_px / 2.0 * (1 + nx), self.height_px / 2.0 * (1 + ny),
                         self.width_px, self.height_px, pos_rx)

    def close(self) -> None:
        pass
