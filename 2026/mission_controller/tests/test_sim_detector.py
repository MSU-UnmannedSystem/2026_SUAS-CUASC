import math
import time

import pytest

from mission_controller import run_flight
from mission_controller.guidance.camera_geometry import pixel_to_ground_offset
from mission_controller.sitl.sim_detector import SimTargetDetector

from link_fakes import FakeLink, attitude, position

LAT, LON = 35.0610, -118.1570
M_PER_DEG = 111319.5


def at(link, north_m, east_m, alt_m, yaw=0.0, roll=0.0, pitch=0.0):
    """Puts the vehicle north_m / east_m away from the target (negative = south / west)."""
    lat = LAT + north_m / M_PER_DEG
    lon = LON + east_m / (M_PER_DEG * math.cos(math.radians(LAT)))
    link.deliver(position(alt_m, lat_deg=lat, lon_deg=lon), attitude(roll, pitch, yaw))


@pytest.mark.parametrize("yaw_deg, roll_deg, pitch_deg", [
    (0, 0, 0), (90, 0, 0), (-135, 0, 0), (30, 4, -3), (200, -5, 6)])
def test_projection_round_trips_through_guidance_geometry(yaw_deg, roll_deg, pitch_deg):
    link = FakeLink()
    yaw, roll, pitch = (math.radians(a) for a in (yaw_deg, roll_deg, pitch_deg))
    at(link, -4.0, -3.0, 15.0, yaw, roll, pitch)   # target 4 m north, 3 m east of the drone
    det = SimTargetDetector(link, LAT, LON).detect()
    assert det is not None
    fwd, right = pixel_to_ground_offset(det.x_px, det.y_px, det.width_px, det.height_px,
                                        15.0, roll, pitch, 65.0, 39.4)
    north = fwd * math.cos(yaw) - right * math.sin(yaw)
    east = fwd * math.sin(yaw) + right * math.cos(yaw)
    assert north == pytest.approx(4.0, abs=0.05)
    assert east == pytest.approx(3.0, abs=0.05)


def test_target_outside_the_frame_is_not_seen():
    link = FakeLink()
    at(link, -40.0, 0.0, 10.0)
    assert SimTargetDetector(link, LAT, LON).detect() is None


def test_no_telemetry_or_on_the_ground():
    link = FakeLink()
    assert SimTargetDetector(link, LAT, LON).detect() is None
    at(link, 0.0, 0.0, 0.2)
    assert SimTargetDetector(link, LAT, LON).detect() is None


def test_hide_after_simulates_lost_lock():
    link = FakeLink()
    at(link, 1.0, 1.0, 12.0)
    det = SimTargetDetector(link, LAT, LON, hide_after_s=0.2)
    assert det.detect() is not None
    time.sleep(0.25)
    assert det.detect() is None


def test_sim_target_refused_on_serial(monkeypatch, tmp_path):
    link = FakeLink(device="/dev/ttyTHS1")
    link.closed = False
    link.close = lambda: setattr(link, "closed", True)
    monkeypatch.setattr(run_flight.Link, "connect", staticmethod(lambda config: link))
    assert run_flight.main(["--sim-target", "35.061,-118.157", "--log-dir", str(tmp_path)]) == 1
    assert link.closed
    assert link.calls == []


@pytest.mark.parametrize("argv", [["--sim-target", "35.0"], ["--sim-target", "a,b"],
                                  ["--sim-hide-after", "5"]])
def test_sim_argument_validation(argv):
    with pytest.raises(SystemExit):
        run_flight.main(argv)
