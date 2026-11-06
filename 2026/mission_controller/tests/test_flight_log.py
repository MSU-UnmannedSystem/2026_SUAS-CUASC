"""Per-run log files and the whole-run telemetry recorder."""

import csv
import datetime
import logging
import os
import time

from mission_controller import flight_log

from link_fakes import FakeLink, NavGuidedVehicle, statustext

WHEN = datetime.datetime(2026, 11, 5, 14, 32, 10)


def test_file_names_are_date_time_and_name(tmp_path):
    stem = flight_log.run_stem(str(tmp_path), "field test #1", now=WHEN)
    assert os.path.basename(stem) == "2026-11-05_14-32-10_field-test-1"


def test_file_names_never_collide(tmp_path):
    first = flight_log.run_stem(str(tmp_path), "flight", now=WHEN)
    open(first + ".log", "w").close()
    second = flight_log.run_stem(str(tmp_path), "flight", now=WHEN)
    assert second == first + "_2"


def test_safe_name():
    assert flight_log.safe_name("../../etc") == "etc"
    assert flight_log.safe_name("  ") == "run"
    assert flight_log.safe_name("bench_1.5m") == "bench_1-5m"


def test_run_log_writes_everything_to_its_own_file(tmp_path):
    run = flight_log.RunLog.start(str(tmp_path / "new"), "flight", now=WHEN)
    try:
        logging.getLogger("some.module").debug("debug detail")
        logging.getLogger("run_flight").info("READY")
    finally:
        run.close()
    logging.getLogger("run_flight").info("after close")
    text = open(run.log_path).read()
    assert run.log_path.endswith("2026-11-05_14-32-10_flight.log")
    assert "debug detail" in text and "READY" in text
    assert "after close" not in text
    run.close()  # idempotent


def test_csv_log(tmp_path):
    log = flight_log.CsvLog(str(tmp_path / "x.csv"), ("a", "b"))
    log.write({"a": 1, "b": None, "ignored": 3})
    log.close()
    log.write({"a": 2})  # after close: dropped, no error
    rows = list(csv.DictReader(open(str(tmp_path / "x.csv"))))
    assert len(rows) == 1 and log.rows == 1
    assert rows[0]["a"] == "1" and rows[0]["b"] == "" and rows[0]["wall_time"].startswith("20")


def test_telemetry_recorder_records_state_and_events(background, tmp_path, caplog):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=5, start_seq=2)
    background(sim.tick, period_s=0.02)
    time.sleep(0.05)
    path = str(tmp_path / "t.csv")
    recorder = flight_log.TelemetryRecorder(sim.link, path, servo_channel=7, rate_hz=20.0)
    with caplog.at_level(logging.INFO):
        recorder.start()
        time.sleep(0.15)
        sim.seq = 3
        sim.link.deliver(statustext("PreArm: test message"))
        time.sleep(0.15)
        sim.mode = "LOITER"
        sim.fire_release()
        time.sleep(0.15)
        recorder.stop()
    messages = [r.getMessage() for r in caplog.records]
    assert "Mission item 2 -> 3" in messages
    assert "Mode AUTO -> LOITER" in messages
    assert "Release output servo7_raw: 1100 -> 1900 us" in messages
    assert any(m == "FC says: PreArm: test message" for m in messages)

    rows = list(csv.DictReader(open(path)))
    assert len(rows) >= 5
    assert set(flight_log.TELEMETRY_FIELDS) <= set(rows[0])
    assert rows[0]["mode"] == "AUTO" and rows[0]["armed"] == "1"
    assert float(rows[0]["rel_alt_m"]) == 7.62
    assert rows[-1]["mode"] == "LOITER" and rows[-1]["release_servo_us"] == "1900"
    assert sim.link.calls == []  # recording never sends


def test_recorder_survives_errors(tmp_path):
    link = FakeLink()
    link.latest_fresh = None  # broken: every sample raises
    recorder = flight_log.TelemetryRecorder(link, str(tmp_path / "t.csv"), 7, rate_hz=50.0)
    recorder.start()
    time.sleep(0.1)
    assert recorder.is_alive()
    recorder.stop()
    assert not recorder.is_alive()
