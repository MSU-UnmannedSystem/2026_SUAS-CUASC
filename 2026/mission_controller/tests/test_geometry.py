import math

import pytest

from mission_controller.guidance.camera_geometry import (
    centering_velocity, pixel_to_ground_offset)

W, H = 640, 480
HFOV, VFOV = 65.0, 39.4


def raw_offset(x, y, alt=10.0, roll=0.0, pitch=0.0):
    return pixel_to_ground_offset(x, y, W, H, alt, roll, pitch, HFOV, VFOV)


def offset(x, y, alt=10.0, roll=0.0, pitch=0.0):
    result = raw_offset(x, y, alt, roll, pitch)
    assert result is not None
    return result


def test_image_centre_is_straight_below():
    assert offset(W / 2, H / 2) == pytest.approx((0.0, 0.0))


def test_image_edges_map_to_half_fov():
    fwd, right = offset(W, H / 2)
    assert fwd == pytest.approx(0.0)
    assert right == pytest.approx(10.0 * math.tan(math.radians(HFOV / 2)))
    fwd, right = offset(W / 2, 0)                     # top of image = nose
    assert fwd == pytest.approx(10.0 * math.tan(math.radians(VFOV / 2)))
    assert right == pytest.approx(0.0)


def test_signs_follow_mounting():
    assert offset(W * 0.75, H / 2)[1] > 0               # right in image -> right of drone
    assert offset(W * 0.25, H / 2)[1] < 0
    assert offset(W / 2, H * 0.25)[0] > 0               # up in image -> ahead
    assert offset(W / 2, H * 0.75)[0] < 0               # down in image -> behind


def test_offset_scales_with_altitude():
    low = offset(W * 0.75, H * 0.25, alt=5.0)
    high = offset(W * 0.75, H * 0.25, alt=10.0)
    assert high[0] == pytest.approx(2 * low[0])
    assert high[1] == pytest.approx(2 * low[1])


def test_roll_and_pitch_tilt_the_view():
    roll = math.radians(5)
    assert offset(W / 2, H / 2, roll=roll)[1] == pytest.approx(10.0 * math.tan(roll))
    pitch = math.radians(5)   # nose up -> camera looks behind
    assert offset(W / 2, H / 2, pitch=pitch)[0] == pytest.approx(-10.0 * math.tan(pitch))


@pytest.mark.parametrize("alt", [0.0, -1.0])
def test_no_offset_without_altitude(alt):
    assert raw_offset(W / 2, H / 2, alt=alt) is None


def test_too_steep_view_is_rejected():
    assert raw_offset(W, H / 2, roll=math.radians(60)) is None


def test_centering_velocity_proportional_then_clamped():
    assert centering_velocity(0.4, -0.2, 0.5, 0.5) == pytest.approx((0.2, -0.1))
    vx, vy = centering_velocity(3.0, 4.0, 0.5, 0.5)
    assert math.hypot(vx, vy) == pytest.approx(0.5)
    assert vx / vy == pytest.approx(3.0 / 4.0)      # direction kept
    assert centering_velocity(0.0, 0.0, 0.5, 0.5) == (0.0, 0.0)
