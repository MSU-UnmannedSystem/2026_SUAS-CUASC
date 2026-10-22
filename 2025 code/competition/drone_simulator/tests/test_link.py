import collections
import logging
import queue
import threading
import time

import pytest
from pymavlink import mavutil

from competition.drone_simulator.vehicle.link import Link, LinkConfig, LinkTimeoutError

mavlink = mavutil.mavlink
LINK_LOGGER = "competition.drone_simulator.vehicle.link"


def make_heartbeat(sysid, compid, mav_type, autopilot):
    """Encodes and re-parses a HEARTBEAT so its header carries sysid/compid."""
    encoder = mavlink.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    msg = encoder.heartbeat_encode(mav_type, autopilot, 0, 0, mavlink.MAV_STATE_STANDBY)
    buf = msg.pack(encoder)
    parsed = mavlink.MAVLink(None).parse_buffer(bytes(buf))
    assert len(parsed) == 1
    return parsed[0]


def gcs_heartbeat():
    return make_heartbeat(255, 190, mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID)


def copter_heartbeat(sysid=1):
    return make_heartbeat(sysid, 1, mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)


class FakeMav:
    """Stands in for master.mav: records sends and detects overlapping calls."""

    def __init__(self, send_delay=0.0):
        self.send_delay = send_delay
        self.sent = []
        self.overlap_detected = False
        self._active = 0
        self._guard = threading.Lock()

    def _record(self, name, args, kwargs):
        with self._guard:
            self._active += 1
            if self._active > 1:
                self.overlap_detected = True
        try:
            if self.send_delay:
                time.sleep(self.send_delay)
            self.sent.append((name, args, kwargs))
        finally:
            with self._guard:
                self._active -= 1

    def heartbeat_send(self, *args, **kwargs):
        self._record("heartbeat_send", args, kwargs)

    def command_long_send(self, *args, **kwargs):
        self._record("command_long_send", args, kwargs)

    def heartbeat_encode(self, *args, **kwargs):
        raise AssertionError("not a send method; Link must not call it")

    def count(self, name):
        return sum(1 for sent in list(self.sent) if sent[0] == name)


class FakeMaster:
    """Minimal mavfile: serves queued messages, then optionally repeats one forever.

    Tests push more messages with feed() and simulate a dead port with fail().
    """

    def __init__(self, messages=(), repeat=None, repeat_interval=0.02, send_delay=0.0):
        self.queue = collections.deque(messages)
        self.repeat = repeat
        self.repeat_interval = repeat_interval
        self.mav = FakeMav(send_delay=send_delay)
        self.target_system = 0
        self.target_component = 0
        self.close_calls = 0
        self.open_kwargs = None
        self.recv_calls = []
        self.error = None
        self._cv = threading.Condition()

    def feed(self, *msgs):
        with self._cv:
            self.queue.extend(msgs)
            self._cv.notify_all()

    def fail(self, error):
        with self._cv:
            self.error = error
            self._cv.notify_all()

    def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
        self.recv_calls.append((type, blocking, timeout))
        with self._cv:
            if self.repeat is None:
                self._cv.wait_for(lambda: self.queue or self.error, timeout)
            if self.error is not None:
                raise self.error
            if self.queue:
                return self.queue.popleft()
        if self.repeat is not None:
            time.sleep(min(self.repeat_interval, timeout))
            return self.repeat
        return None

    def mode_mapping(self):
        return {"STABILIZE": 0, "AUTO": 3, "GUIDED": 4}

    def close(self):
        self.close_calls += 1


def factory_for(master):
    def factory(device, **kwargs):
        master.open_kwargs = dict(device=device, **kwargs)
        return master
    return factory


def connect(master, **config_kwargs):
    config_kwargs.setdefault("connect_timeout_s", 2.0)
    config = LinkConfig(device="fake", **config_kwargs)
    return Link.connect(config, connection_factory=factory_for(master))


def test_ignores_gcs_heartbeat_and_targets_autopilot(caplog):
    master = FakeMaster([gcs_heartbeat(), copter_heartbeat(sysid=1)])
    with caplog.at_level(logging.INFO, logger=LINK_LOGGER):
        with connect(master) as link:
            assert link.target_system == 1
            assert link.target_component == 1
            assert master.target_system == 1
            assert master.target_component == 1

    ignored = [r.getMessage() for r in caplog.records if "Ignoring heartbeat" in r.getMessage()]
    assert len(ignored) == 1
    assert "sysid=255" in ignored[0]
    assert "compid=190" in ignored[0]
    assert "type=%d" % mavlink.MAV_TYPE_GCS in ignored[0]

    assert master.open_kwargs == dict(device="fake", baud=921600, source_system=255,
                                      source_component=191)
    connect_calls = master.recv_calls[:2]
    assert all(t == "HEARTBEAT" and blocking for t, blocking, _ in connect_calls)


def test_only_gcs_heartbeats_times_out_and_closes():
    master = FakeMaster(repeat=gcs_heartbeat())
    start = time.monotonic()
    with pytest.raises(LinkTimeoutError, match="SERIALx_PROTOCOL"):
        connect(master, connect_timeout_s=0.5)
    elapsed = time.monotonic() - start

    assert 0.5 <= elapsed < 0.8
    assert master.close_calls == 1
    assert all(0 < timeout <= 0.5 for _, _, timeout in master.recv_calls)
    assert master.mav.count("heartbeat_send") == 0


def test_silent_link_times_out():
    master = FakeMaster()
    with pytest.raises(LinkTimeoutError):
        connect(master, connect_timeout_s=0.3)
    assert master.close_calls == 1


def test_expected_sysid_mismatch_is_ignored(caplog):
    master = FakeMaster(repeat=copter_heartbeat(sysid=1))
    with caplog.at_level(logging.INFO, logger=LINK_LOGGER):
        with pytest.raises(LinkTimeoutError):
            connect(master, expected_sysid=2, connect_timeout_s=0.5)
    assert master.close_calls == 1
    assert any("sysid=1" in r.getMessage() and "expected 2" in r.getMessage()
               for r in caplog.records)


def test_expected_sysid_match_is_accepted():
    master = FakeMaster([copter_heartbeat(sysid=1), copter_heartbeat(sysid=2)])
    with connect(master, expected_sysid=2) as link:
        assert link.target_system == 2


@pytest.mark.parametrize("name", ["heartbeat", "heartbeat_encode", "nonexistent_send"])
def test_send_rejects_bad_method_names(name):
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        with pytest.raises(ValueError):
            link.send(name)


def test_send_forwards_args():
    master = FakeMaster([copter_heartbeat()])
    with connect(master, heartbeat_hz=0.1) as link:
        link.send("command_long_send", 1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    assert ("command_long_send", (1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0), {}) in master.mav.sent


def _hammer(target, n_threads=8, calls_each=5):
    barrier = threading.Barrier(n_threads)

    def worker():
        barrier.wait()
        for _ in range(calls_each):
            target()

    threads = [threading.Thread(target=worker) for _ in range(n_threads)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10)


def test_fake_detects_overlap_without_lock():
    # Sanity check that the overlap detector works, so the next test means something.
    mav = FakeMav(send_delay=0.005)
    _hammer(lambda: mav.command_long_send())
    assert mav.overlap_detected


def test_send_serialises_concurrent_calls():
    master = FakeMaster([copter_heartbeat()], send_delay=0.005)
    with connect(master, heartbeat_hz=20) as link:
        _hammer(lambda: link.send("command_long_send"))
    assert master.mav.count("command_long_send") == 40
    assert not master.mav.overlap_detected


def test_heartbeat_thread_rate_and_close():
    master = FakeMaster([copter_heartbeat()])
    link = connect(master, heartbeat_hz=20)
    time.sleep(1.0)
    count = master.mav.count("heartbeat_send")
    assert 17 <= count <= 23

    _, _, kwargs = master.mav.sent[0]
    assert kwargs == dict(type=mavlink.MAV_TYPE_GCS, autopilot=mavlink.MAV_AUTOPILOT_INVALID,
                          base_mode=0, custom_mode=0, system_status=mavlink.MAV_STATE_ACTIVE)

    start = time.monotonic()
    link.close()
    assert time.monotonic() - start < 0.5

    after_close = master.mav.count("heartbeat_send")
    time.sleep(0.2)
    assert master.mav.count("heartbeat_send") == after_close

    link.close()
    assert master.close_calls == 1


def test_heartbeat_thread_survives_send_errors(caplog):
    master = FakeMaster([copter_heartbeat()])
    failures = {"left": 3}
    real_send = master.mav.heartbeat_send

    def flaky_send(*args, **kwargs):
        if failures["left"] > 0:
            failures["left"] -= 1
            raise OSError("port hiccup")
        real_send(*args, **kwargs)

    master.mav.heartbeat_send = flaky_send
    with caplog.at_level(logging.ERROR, logger=LINK_LOGGER):
        with connect(master, heartbeat_hz=50):
            time.sleep(0.3)
    assert master.mav.count("heartbeat_send") > 0
    assert sum("heartbeat send failed" in r.getMessage() for r in caplog.records) == 3


# --- reader thread / cache / subscriptions ---------------------------------


def make_msg(sysid, compid, name, *fields):
    """Encodes and re-parses any message, e.g. make_msg(1, 1, "mission_request", 255, 191, 0)."""
    encoder = mavlink.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    msg = getattr(encoder, name + "_encode")(*fields)
    return mavlink.MAVLink(None).parse_buffer(bytes(msg.pack(encoder)))[0]


def position(lat_deg, sysid=1):
    return make_msg(sysid, 1, "global_position_int",
                    1000, int(lat_deg * 1e7), int(-118.15 * 1e7), 100000, 16000, 0, 0, 0, 9000)


def mission_request(seq):
    return make_msg(1, 1, "mission_request", 255, 191, seq)


def wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def test_connect_heartbeat_is_cached_and_mode_mapping_delegates():
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        assert link.is_up
        assert link.latest("HEARTBEAT").get_srcSystem() == 1
        assert link.mode_mapping()["AUTO"] == 3
    assert not link.is_up


def test_latest_returns_newest_and_age():
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        master.feed(position(35.01), position(35.02), position(35.03))
        assert wait_until(lambda: link.latest("GLOBAL_POSITION_INT") is not None
                          and link.latest("GLOBAL_POSITION_INT").lat == int(35.03 * 1e7))

        assert link.age("GLOBAL_POSITION_INT") < 0.5
        msg, rx_time = link.latest_with_time("GLOBAL_POSITION_INT")
        assert msg.lat == int(35.03 * 1e7)
        assert 0 <= time.monotonic() - rx_time < 0.5

        assert link.age("ATTITUDE") == float("inf")
        assert link.latest("ATTITUDE") is None
        assert link.latest_with_time("ATTITUDE") == (None, None)

        reader_calls = master.recv_calls[1:]
        assert all(t is None and blocking and timeout == 0.2 for t, blocking, timeout in reader_calls)


def test_other_sysid_not_cached_but_delivered_to_subscribers():
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        with link.subscribe("GLOBAL_POSITION_INT", "HEARTBEAT") as sub:
            master.feed(position(35.05, sysid=255), gcs_heartbeat())
            assert sub.get(timeout=1.0).get_srcSystem() == 255
            assert sub.get(timeout=1.0).get_srcSystem() == 255
        assert link.latest("GLOBAL_POSITION_INT") is None
        assert link.latest("HEARTBEAT").get_srcSystem() == 1


def test_burst_delivered_in_order():
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        with link.subscribe("MISSION_REQUEST", "MISSION_REQUEST_INT") as sub:
            master.feed(*[mission_request(seq) for seq in range(7)])
            assert [sub.get(timeout=1.0).seq for _ in range(7)] == list(range(7))
            assert sub.dropped == 0


def test_subscription_only_receives_while_open():
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        with link.subscribe("MISSION_REQUEST") as sub:
            master.feed(mission_request(0))
            assert sub.get(timeout=1.0).seq == 0

        master.feed(mission_request(1))
        assert wait_until(lambda: link.latest("MISSION_REQUEST").seq == 1)
        with pytest.raises(queue.Empty):
            sub.get(timeout=0.05)


def test_subscription_overflow_keeps_newest():
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        with link.subscribe("MISSION_REQUEST", maxsize=3) as sub:
            master.feed(*[mission_request(seq) for seq in range(10)])
            assert wait_until(lambda: sub.dropped == 7)
            assert [sub.get(timeout=1.0).seq for _ in range(3)] == [7, 8, 9]
            with pytest.raises(queue.Empty):
                sub.get(timeout=0.05)


def test_subscribe_validates_arguments():
    master = FakeMaster([copter_heartbeat()])
    with connect(master) as link:
        with pytest.raises(ValueError):
            with link.subscribe():
                pass
        with pytest.raises(ValueError):
            with link.subscribe("HEARTBEAT", maxsize=0):
                pass


def test_handler_exception_does_not_stop_reader(caplog):
    master = FakeMaster([copter_heartbeat()])
    poisoned = position(35.01)

    def explode():
        raise RuntimeError("boom")

    poisoned.get_srcSystem = explode
    with caplog.at_level(logging.ERROR, logger=LINK_LOGGER):
        with connect(master) as link:
            master.feed(poisoned, mavlink.MAVLink_bad_data(b"\x01\x02", "junk"), position(35.02))
            assert wait_until(lambda: link.latest("GLOBAL_POSITION_INT") is not None)
            assert link.latest("GLOBAL_POSITION_INT").lat == int(35.02 * 1e7)
            assert link._reader_thread.is_alive()

    errors = [r for r in caplog.records if "Error handling received message" in r.getMessage()]
    assert len(errors) == 1
    assert errors[0].exc_info is not None


def test_reconnects_with_backoff_after_link_error():
    first = FakeMaster([copter_heartbeat()])
    second = FakeMaster()
    state = {"opens": 0, "allow_reopen": False}
    attempts = []

    def factory(device, **kwargs):
        state["opens"] += 1
        if state["opens"] == 1:
            return first
        attempts.append(time.monotonic())
        if not state["allow_reopen"]:
            raise OSError("No such file or directory: 'fake'")
        return second

    config = LinkConfig(device="fake", connect_timeout_s=2.0)
    with Link.connect(config, connection_factory=factory) as link:
        first.feed(position(35.01))
        assert wait_until(lambda: link.latest("GLOBAL_POSITION_INT") is not None)

        first.fail(OSError("device reports readiness to read but returned no data"))
        assert wait_until(lambda: not link.is_up, timeout=1.0)
        assert first.close_calls >= 1

        cpu_start = time.process_time()
        time.sleep(2.0)
        assert not link.is_up
        assert 1 <= len(attempts) < 10
        assert time.process_time() - cpu_start < 0.3
        assert link.age("GLOBAL_POSITION_INT") > 2.0  # no fake data while down

        state["allow_reopen"] = True
        assert wait_until(lambda: link.is_up, timeout=6.0)
        assert second.target_system == 1

        second.feed(position(35.09))
        assert wait_until(lambda: link.latest("GLOBAL_POSITION_INT").lat == int(35.09 * 1e7))
        link.send("command_long_send", 1)
        assert second.mav.count("command_long_send") == 1


def test_close_stops_both_threads_quickly():
    master = FakeMaster([copter_heartbeat()])
    link = connect(master)
    reader, heartbeat = link._reader_thread, link._heartbeat_thread
    assert reader.is_alive() and heartbeat.is_alive()

    start = time.monotonic()
    link.close()
    assert time.monotonic() - start < 1.0
    assert not reader.is_alive()
    assert not heartbeat.is_alive()
    link.close()
