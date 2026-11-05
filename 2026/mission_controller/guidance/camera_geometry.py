"""Pixel -> ground offset -> bounded velocity, for a body-fixed downward camera.

Camera mounting (decided): pointing straight down, top of the image towards
the nose, image right towards the drone's right. It is not gimballed, so
roll and pitch tilt the view; the offset corrects for that. Ground is
assumed flat at the home altitude (altitude = GLOBAL_POSITION_INT.relative_alt).
"""

import math
from typing import Optional, Tuple

# Beyond this off-vertical angle the flat-ground projection is meaningless.
_MAX_VIEW_ANGLE_RAD = math.radians(75.0)


def pixel_to_ground_offset(
    x_px: float,
    y_px: float,
    width_px: int,
    height_px: int,
    alt_m: float,
    roll_rad: float,
    pitch_rad: float,
    hfov_deg: float,
    vfov_deg: float,
) -> Optional[Tuple[float, float]]:
    """Returns where the target is relative to the point straight below the drone.

    Pinhole model: a pixel's angle off the optical axis is
    atan(normalised_offset * tan(fov / 2)). Roll (right wing down positive)
    swings the view right; pitch (nose up positive) swings it towards the tail.

    Args:
        x_px: Target column, 0 = left edge.
        y_px: Target row, 0 = top edge (towards the nose).
        width_px: Frame width in pixels.
        height_px: Frame height in pixels.
        alt_m: Height above the ground (home) in metres.
        roll_rad: Roll in radians, right wing down positive (ATTITUDE.roll).
        pitch_rad: Pitch in radians, nose up positive (ATTITUDE.pitch).
        hfov_deg: Horizontal field of view in degrees.
        vfov_deg: Vertical field of view in degrees.

    Returns:
        ``(forward_m, right_m)`` in the drone's yaw frame, or None if the
        altitude is not positive or the view angle is too steep to project.
    """
    if alt_m <= 0 or width_px <= 0 or height_px <= 0:
        return None
    nx = (x_px - width_px / 2.0) / (width_px / 2.0)    # -1 left .. +1 right
    ny = (y_px - height_px / 2.0) / (height_px / 2.0)  # -1 nose .. +1 tail
    right_angle = math.atan(nx * math.tan(math.radians(hfov_deg) / 2.0)) + roll_rad
    forward_angle = -math.atan(ny * math.tan(math.radians(vfov_deg) / 2.0)) - pitch_rad
    if abs(right_angle) > _MAX_VIEW_ANGLE_RAD or abs(forward_angle) > _MAX_VIEW_ANGLE_RAD:
        return None
    return alt_m * math.tan(forward_angle), alt_m * math.tan(right_angle)


def centering_velocity(
    forward_m: float,
    right_m: float,
    gain_per_s: float,
    max_speed_m_s: float,
) -> Tuple[float, float]:
    """Proportional velocity towards the target, clamped to ``max_speed_m_s``.

    The clamp scales both components together, so the direction is kept.

    Args:
        forward_m: Target offset towards the nose, in metres.
        right_m: Target offset to the right, in metres.
        gain_per_s: m/s of velocity per metre of offset.
        max_speed_m_s: Maximum horizontal speed, in m/s.

    Returns:
        ``(forward_m_s, right_m_s)``.
    """
    vx, vy = gain_per_s * forward_m, gain_per_s * right_m
    speed = math.hypot(vx, vy)
    if speed > max_speed_m_s:
        scale = max_speed_m_s / speed
        vx, vy = vx * scale, vy * scale
    return vx, vy
