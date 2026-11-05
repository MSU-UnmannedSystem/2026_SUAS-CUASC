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


# --- whole runs through main(): log files, FC setup report, camera, bench ----------

import csv  # noqa: E402
import glob  # noqa: E402
import os  # noqa: E402
import threading  # noqa: E402

from mission_controller.vision.detector import Detection  # noqa: E402

GOOD_FC_PARAMS = {"SERIAL2_PROTOCOL": 2, "SERIAL2_BAUD": 921, "BRD_SER2_RTSCTS": 0,
                  "SERIAL2_OPTIONS": 0, "SYSID_MYGCS": 255, "GUID_TIMEOUT": 3,
                  "FS_GCS_ENABLE": 0, "FS_GCS_TIMEOUT": 5}
FAST = dict(max_horizontal_speed_m_s=2.0, horizontal_gain_per_s=1.0, center_hold_s=0.2,
            lost_lock_timeout_s=0.6, command_rate_hz=50.0)


def hardware(vehicle, monkeypatch, **param_overrides):
    """Points main() at the simulated vehicle, with the mission and FC params loaded."""
    load(vehicle, good_mission())
    vehicle.params.update(GOOD_FC_PARAMS, **param_overrides)
    vehicle.link.close = lambda: None
    monkeypatch.setattr(run_flight.Link, "connect", staticmethod(lambda config: vehicle.link))


def run_files(log_dir, name):
    logs = glob.glob(os.path.join(str(log_dir), "*_%s.log" % name))
    assert len(logs) == 1, logs
    stem = logs[0][:-4]
    assert re.match(r"\d{4}-\d\d-\d\d_\d\d-\d\d-\d\d_%s$" % name, os.path.basename(stem))
    return stem, open(logs[0]).read()


def csv_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def test_fc_setup_report_ok(vehicle):
    vehicle.params.update(GOOD_FC_PARAMS)
    assert run_flight.log_fc_setup(vehicle.link, HandoverConfig()) == []


def test_fc_setup_report_warns(vehicle):
    vehicle.params.update(GOOD_FC_PARAMS, SERIAL2_BAUD=57, BRD_SER2_RTSCTS=2, SYSID_MYGCS=254)
    del vehicle.params["SERIAL2_OPTIONS"]       # missing params are only reported
    warnings = run_flight.log_fc_setup(vehicle.link, HandoverConfig())
    assert len(warnings) == 3
    assert any("SERIAL2_BAUD = 57" in w for w in warnings)
    assert any("BRD_SER2_RTSCTS = 2" in w for w in warnings)
    assert any("SYSID_MYGCS = 254" in w for w in warnings)
    sent = {c.method for c in vehicle.link.calls}
    assert sent == {"param_request_read_send"}  # read-only


def test_check_only_writes_its_own_log(vehicle, monkeypatch, tmp_path):
    hardware(vehicle, monkeypatch)
    assert run_flight.main(["--check-only", "--log-dir", str(tmp_path)]) == 0
    stem, text = run_files(tmp_path, "check")
    assert "PRE-FLIGHT OK" in text
    assert "guard DO_JUMP 6 -> release 7" in text
    assert "FC param SYSID_MYGCS = 255" in text
    assert "item 7: MAV_CMD_DO_SET_SERVO p1=7 p2=1900" in text      # whole mission logged
    assert csv_rows(stem + "_telemetry.csv")
    assert not os.path.exists(stem + "_handover.csv")               # no handover in a check
    sent = {c.method for c in vehicle.link.calls}
    assert sent <= {"mission_request_list_send", "mission_request_int_send", "mission_ack_send",
                    "param_request_read_send"}


def test_preflight_failure_is_logged(vehicle, monkeypatch, tmp_path):
    hardware(vehicle, monkeypatch)
    load(vehicle, [it for it in good_mission() if it.command != mavlink.MAV_CMD_DO_JUMP])
    assert run_flight.main(["--check-only", "--name", "bad mission",
                            "--log-dir", str(tmp_path)]) == 1
    _, text = run_files(tmp_path, "bad-mission")
    assert "PRE-FLIGHT FAILED" in text and "must be DO_JUMP" in text


def test_whole_flight_through_main(vehicle, monkeypatch, tmp_path):
    """SITL-style run: wait, handover (simulated target), FC release, mission, disarm."""
    hardware(vehicle, monkeypatch)
    monkeypatch.setattr(run_flight, "HandoverConfig",
                        lambda **kw: HandoverConfig(**dict(FAST, **kw)))
    target_lat = 35.05 + 1.5 / 111319.5           # 1.5 m north of where the vehicle starts

    def pilot_and_fc():
        time.sleep(0.5)
        vehicle.armed, vehicle.mode, vehicle.alt_m = True, "AUTO", 7.62
        vehicle.seq = 5                           # AUTO reaches NAV_GUIDED_ENABLE
        while vehicle.seq != 8:                   # until the Jetson hands back
            time.sleep(0.05)
        time.sleep(1.5)
        vehicle.armed = False                     # rest of the mission, landed

    threading.Thread(target=pilot_and_fc, daemon=True).start()
    assert run_flight.main(["--device", "tcp:127.0.0.1:5762", "--sim-target",
                            "%.7f,-118.15" % target_lat, "--log-dir", str(tmp_path)]) == 0

    stem, text = run_files(tmp_path, "sitl")
    for expected in ("PRE-FLIGHT OK", "Handover started", "Locked: within",
                     "Handing back to the FC at item 7", "Handover outcome: locked",
                     "FC released the payload: servo7_raw 1100 -> 1900 us",
                     "Mode STABILIZE -> AUTO", "Mission item 5 -> 8", "ARMED", "DISARMED",
                     "RUN SUMMARY: handover locked, release confirmed"):
        assert expected in text, expected
    ticks = csv_rows(stem + "_handover.csv")
    assert ticks[0]["phase"] == "ALIGN" and float(ticks[0]["fwd_m"]) > 1.0
    assert ticks[-1]["phase"] == "locked"
    telemetry = csv_rows(stem + "_telemetry.csv")
    assert {"STABILIZE", "AUTO"} <= {r["mode"] for r in telemetry}
    assert telemetry[-1]["release_servo_us"] == "1900"


class FakeCamera:
    """Stands in for VisionPipeline: a target 0.3 m ahead at 7.62 m."""

    def __init__(self, size=(1920, 1080), stop_after=None, **_kwargs):
        self.size = size
        self.stop_after = stop_after
        self.calls = 0
        self.closed = False

    def wait_for_frame(self, timeout_s):
        return self.size

    def detect(self):
        self.calls += 1
        if self.stop_after is not None and self.calls > self.stop_after:
            raise KeyboardInterrupt  # the operator's Ctrl-C
        w, h = self.size
        return Detection(w / 2.0, h / 2.0 - 25, w, h, time.monotonic())

    def close(self):
        self.closed = True


def fake_camera(monkeypatch, camera):
    from mission_controller.vision import camera as camera_module
    monkeypatch.setattr(camera_module, "VisionPipeline", lambda **kw: camera)


def test_bench_mode_through_main(vehicle, monkeypatch, tmp_path):
    hardware(vehicle, monkeypatch)
    camera = FakeCamera(stop_after=30)
    fake_camera(monkeypatch, camera)
    before = len(vehicle.link.calls)
    assert run_flight.main(["--device", "/dev/ttyTHS1", "--bench", "--bench-alt", "1.2",
                            "--log-dir", str(tmp_path)]) == 0
    stem, text = run_files(tmp_path, "bench")
    assert "BENCH MODE at 1.20 m" in text and "Bench run stopped" in text
    rows = csv_rows(stem + "_handover.csv")
    assert len(rows) >= 20 and {r["sent"] for r in rows} == {"0"}
    assert float(rows[0]["fwd_m"]) > 0 and float(rows[0]["alt_m"]) == 1.2   # target ahead
    after = {c.method for c in vehicle.link.calls[before:]}
    assert after <= {"mission_request_list_send", "mission_request_int_send", "mission_ack_send",
                     "param_request_read_send"}                     # nothing that moves it
    assert camera.closed


def test_no_camera_frame_fails_preflight(vehicle, monkeypatch, tmp_path):
    hardware(vehicle, monkeypatch)
    camera = FakeCamera(size=None)
    fake_camera(monkeypatch, camera)
    assert run_flight.main(["--log-dir", str(tmp_path)]) == 1
    _, text = run_files(tmp_path, "flight")
    assert "PRE-FLIGHT FAILED: camera 0 gave no frame" in text
    assert camera.closed
    assert not any(c.method == "set_position_target_local_ned_send" for c in vehicle.link.calls)


def test_wrong_camera_size_warns(caplog):
    with caplog.at_level("WARNING"):
        run_flight.check_camera(FakeCamera(size=(640, 480)), HandoverConfig())
    text = " ".join(r.getMessage() for r in caplog.records)
    assert "not the requested 1920x1080" in text and "not 16:9" in text


@pytest.mark.parametrize("argv", [["--bench", "--check-only"], ["--bench-alt", "2"],
                                  ["--bench", "--bench-alt", "0"],
                                  ["--bench", "--sim-target", "35,-118"]])
def test_bench_argument_validation(argv):
    with pytest.raises(SystemExit):
        run_flight.main(argv)
