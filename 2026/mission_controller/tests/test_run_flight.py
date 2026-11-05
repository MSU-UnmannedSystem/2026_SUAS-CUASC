import re
import time
from pathlib import Path

import pytest
from pymavlink import mavutil

from mission_controller import run_flight
from mission_controller.config.flight_params import HandoverConfig
from mission_controller.vehicle.mission import MissionCheckError

from link_fakes import FakeLink, NavGuidedVehicle
from test_mission_check import good_mission

mavlink = mavutil.mavlink
PACKAGE = Path(__file__).resolve().parents[1]


def load(sim, items):
    sim.missions.stored[0] = [(it.frame, it.command, it.param1, it.param2, it.param3, it.param4,
                               int(it.lat_deg * 1e7), int(it.lon_deg * 1e7), it.alt_m, 1)
                              for it in items]


@pytest.fixture
def vehicle(background):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=5, start_seq=1)
    sim.mode = "STABILIZE"
    sim.armed = False
    background(sim.tick, period_s=0.02)
    time.sleep(0.05)
    return sim


def test_preflight_ok_sends_only_reads(vehicle):
    load(vehicle, good_mission())
    plan = run_flight.preflight(vehicle.link, HandoverConfig())
    assert (plan.guided_seq, plan.jump_seq, plan.release_seq, plan.resume_seq) == (5, 6, 7, 8)
    sent = {c.method for c in vehicle.link.calls}
    assert sent <= {"mission_request_list_send", "mission_request_int_send", "mission_ack_send",
                    "param_request_read_send"}


def test_preflight_rejects_mission_without_handover(vehicle):
    load(vehicle, [it for it in good_mission() if it.command != mavlink.MAV_CMD_NAV_GUIDED_ENABLE])
    with pytest.raises(MissionCheckError):
        run_flight.preflight(vehicle.link, HandoverConfig())


@pytest.mark.parametrize("function, ok", [(0, True), (1, True), (28, True), (51, True),
                                          (33, False), (70, False)])
def test_release_servo_function(vehicle, function, ok):
    vehicle.params["SERVO7_FUNCTION"] = float(function)
    if ok:
        run_flight.check_release_servo(vehicle.link, 7)
    else:
        with pytest.raises(RuntimeError, match="SERVO7_FUNCTION"):
            run_flight.check_release_servo(vehicle.link, 7)


def test_release_servo_unreadable(vehicle):
    del vehicle.params["SERVO7_FUNCTION"]
    with pytest.raises(RuntimeError, match="Could not read"):
        run_flight.check_release_servo(vehicle.link, 7)


def test_confirm_fc_release(vehicle):
    before = run_flight.servo_output_us(vehicle.link, 7)
    assert before == 1100
    vehicle.fire_release()
    assert run_flight.confirm_fc_release(vehicle.link, 7, 1900, before, timeout_s=1.0)


def test_confirm_fc_release_not_seen(vehicle):
    assert not run_flight.confirm_fc_release(vehicle.link, 7, 1900, 1100, timeout_s=0.3)
    assert vehicle.link.calls == []   # only watches


@pytest.mark.parametrize("before", [1900, None])
def test_release_already_at_pwm_cannot_be_confirmed(vehicle, before, caplog):
    # Seen in SITL: servo 7 was still 1900 from an earlier flight.
    vehicle.fire_release()
    with caplog.at_level("WARNING"):
        assert not run_flight.confirm_fc_release(vehicle.link, 7, 1900, before, timeout_s=0.3)
    assert any("can't be confirmed" in r.getMessage() for r in caplog.records)


# --- config ------------------------------------------------------------------------


def test_decided_defaults():
    c = HandoverConfig()
    assert (c.servo_channel, c.servo_release_pwm_us, c.drop_alt_m) == (7, 1900, 7.62)
    assert (c.max_horizontal_speed_m_s, c.center_tolerance_m, c.center_hold_s) == (0.5, 0.5, 2.0)
    assert c.lost_lock_timeout_s == 10.0
    assert (c.camera_hfov_deg, c.camera_vfov_deg) == (65.0, 39.4)
    assert (c.device, c.baud) == ("/dev/ttyTHS1", 921600)


@pytest.mark.parametrize("bad", [dict(drop_alt_m=0), dict(drop_alt_tolerance_m=0),
                                 dict(max_horizontal_speed_m_s=-1),
                                 dict(command_rate_hz=1.0), dict(servo_channel=17),
                                 dict(camera_hfov_deg=0), dict(center_hold_s=-1)])
def test_config_validation(bad):
    with pytest.raises(ValueError):
        HandoverConfig(**bad)


# --- structure ---------------------------------------------------------------------


def test_only_link_touches_the_connection():
    forbidden = ("recv_match", ".mav.", "wait_heartbeat", "motors_armed_wait",
                 "mavlink_connection")
    offenders = []
    for path in sorted(PACKAGE.rglob("*.py")):
        rel = path.relative_to(PACKAGE).as_posix()
        if rel.startswith("tests/") or rel == "vehicle/link.py":
            continue
        text = path.read_text()
        offenders += ["%s: %s" % (rel, w) for w in forbidden if w in text]
    assert offenders == []


def test_independent_of_drone_simulator():
    imports = re.compile(r"^\s*(from|import)\s+\S*(drone_simulator|competition)", re.M)
    offenders = [p.relative_to(PACKAGE).as_posix() for p in PACKAGE.rglob("*.py")
                 if imports.search(p.read_text())]
    assert offenders == []


def test_never_uploads_or_commands_flight():
    text = "\n".join(p.read_text() for p in PACKAGE.rglob("*.py")
                     if not p.relative_to(PACKAGE).as_posix().startswith("tests/"))
    for word in ("mission_count_send", "mission_item_int_send", "MAV_CMD_COMPONENT_ARM_DISARM",
                 "MAV_CMD_NAV_TAKEOFF", "MAV_CMD_DO_SET_MODE"):
        assert word not in text
