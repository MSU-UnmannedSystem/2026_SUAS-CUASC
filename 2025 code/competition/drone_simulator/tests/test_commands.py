import collections
import contextlib
import logging
import threading
import time

import pytest
from pymavlink import mavutil

from competition.drone_simulator.vehicle import commands
from competition.drone_simulator.vehicle.commands import (
    CommandDenied, CommandError, CommandTimeout, StateNotReached)
from competition.drone_simulator.vehicle.link import Subscription

mavlink = mavutil.mavlink
MODES = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)

SentCommand = collections.namedtuple("SentCommand", "command confirmation params")


# --- real pymavlink messages ------------------------------------------------


def make_msg(sysid, compid, name, *fields):
    encoder = mavlink.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    msg = getattr(encoder, name + "_encode")(*fields)
    return mavlink.MAVLink(None).parse_buffer(bytes(msg.pack(encoder)))[0]


def heartbeat(mode="GUIDED", armed=False, sysid=1, compid=1):
    base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return make_msg(sysid, compid, "heartbeat", mavlink.MAV_TYPE_QUADROTOR,
                    mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, base_mode, MODES[mode],
                    mavlink.MAV_STATE_ACTIVE)


def ack(command, result, sysid=1):
    return make_msg(sysid, 1, "command_ack", command, result)


def statustext(text):
    return make_msg(1, 1, "statustext", mavlink.MAV_SEVERITY_CRITICAL, text.encode())


def position(rel_alt_m):
    return make_msg(1, 1, "global_position_int", 1000, int(35.05 * 1e7), int(-118.15 * 1e7),
                    int((700 + rel_alt_m) * 1000), int(rel_alt_m * 1000), 0, 0, 0, 0)


# --- FakeLink ---------------------------------------------------------------


class FakeLink:
    """Implements the Link API used by commands.py; tests script the vehicle's replies.

    ``responder(link, sent)`` is called for every COMMAND_LONG sent and may
    ``deliver()`` messages, which go to open subscriptions and the cache
    exactly like the real reader thread does.
    """

    target_system = 1
    target_component = 1

    def __init__(self, device="udp:127.0.0.1:14550", responder=None):
        self.device = device
        self.responder = responder
        self.sent = []
        self._cache = {}
        self._subs = []
        self._lock = threading.Lock()

    @property
    def is_serial(self):
        return not self.device.startswith(("udp", "tcp", "mcast"))

    def send(self, method_name, *args, **kwargs):
        assert method_name == "command_long_send"
        ts, tc, command, confirmation = args[:4]
        assert (ts, tc) == (1, 1)
        sent = SentCommand(command, confirmation, list(args[4:]))
        self.sent.append(sent)
        if self.responder is not None:
            self.responder(self, sent)

    def deliver(self, *msgs):
        with self._lock:
            for msg in msgs:
                if msg.get_srcSystem() == self.target_system:
                    self._cache[msg.get_type()] = (msg, time.monotonic())
                for sub in self._subs:
                    if msg.get_type() in sub.msg_types:
                        sub._put(msg)

    def cache_stale(self, msg, age_s):
        """Puts ``msg`` in the cache as if it was received ``age_s`` seconds ago."""
        with self._lock:
            self._cache[msg.get_type()] = (msg, time.monotonic() - age_s)

    @contextlib.contextmanager
    def subscribe(self, *msg_types, maxsize=100):
        sub = Subscription(frozenset(msg_types), maxsize)
        with self._lock:
            self._subs.append(sub)
        try:
            yield sub
        finally:
            with self._lock:
                self._subs.remove(sub)

    def latest(self, msg_type):
        return self.latest_with_time(msg_type)[0]

    def latest_with_time(self, msg_type):
        with self._lock:
            return self._cache.get(msg_type, (None, None))

    def age(self, msg_type):
        _, rx = self.latest_with_time(msg_type)
        return float("inf") if rx is None else time.monotonic() - rx

    def mode_mapping(self):
        return dict(MODES)


def later(delay_s, fn):
    timer = threading.Timer(delay_s, fn)
    timer.daemon = True
    timer.start()
    return timer


@pytest.fixture
def background():
    """Runs ``fn`` every ``period_s`` in a thread until the test ends."""
    stop = threading.Event()
    threads = []

    def start(fn, period_s=0.1):
        def loop():
            while not stop.is_set():
                fn()
                stop.wait(period_s)
        t = threading.Thread(target=loop, daemon=True)
        threads.append(t)
        t.start()

    yield start
    stop.set()
    for t in threads:
        t.join(timeout=1)


# --- command_long -----------------------------------------------------------


def test_no_ack_retries_with_incrementing_confirmation():
    link = FakeLink()
    with pytest.raises(CommandTimeout) as info:
        commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE, [1, 4], ack_timeout_s=0.1)
    assert [s.confirmation for s in link.sent] == [0, 1, 2]
    assert info.value.attempts == 3
    assert info.value.command == mavlink.MAV_CMD_DO_SET_MODE
    assert link.sent[0].params == [1.0, 4.0, 0, 0, 0, 0, 0]


def test_unrelated_ack_is_ignored():
    def responder(link, sent):
        link.deliver(ack(mavlink.MAV_CMD_NAV_TAKEOFF, mavlink.MAV_RESULT_ACCEPTED))

    link = FakeLink(responder=responder)
    with pytest.raises(CommandTimeout):
        commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE, [1, 4],
                              ack_timeout_s=0.1, retries=1)


def test_unrelated_ack_then_matching_ack_succeeds():
    def responder(link, sent):
        link.deliver(ack(mavlink.MAV_CMD_NAV_TAKEOFF, mavlink.MAV_RESULT_DENIED),
                     ack(sent.command, mavlink.MAV_RESULT_ACCEPTED))

    link = FakeLink(responder=responder)
    result = commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE, [1, 4])
    assert result == mavlink.MAV_RESULT_ACCEPTED
    assert len(link.sent) == 1


def test_ack_from_other_system_is_ignored():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED, sysid=255))

    link = FakeLink(responder=responder)
    with pytest.raises(CommandTimeout):
        commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE, [1, 4],
                              ack_timeout_s=0.1, retries=1)


def test_in_progress_extends_wait_then_accepted():
    cmd = mavlink.MAV_CMD_DO_SET_MODE

    def responder(link, sent):
        link.deliver(ack(cmd, mavlink.MAV_RESULT_IN_PROGRESS))
        later(0.15, lambda: link.deliver(ack(cmd, mavlink.MAV_RESULT_IN_PROGRESS)))
        later(0.30, lambda: link.deliver(ack(cmd, mavlink.MAV_RESULT_ACCEPTED)))

    link = FakeLink(responder=responder)
    # 0.30 s total is longer than one 0.2 s ack window: only works if IN_PROGRESS extends it.
    result = commands.command_long(link, cmd, [1, 4], ack_timeout_s=0.2)
    assert result == mavlink.MAV_RESULT_ACCEPTED
    assert len(link.sent) == 1


@pytest.mark.parametrize("result", [
    mavlink.MAV_RESULT_DENIED, mavlink.MAV_RESULT_FAILED,
    mavlink.MAV_RESULT_UNSUPPORTED, mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
])
def test_rejections_raise_without_retry(result):
    def responder(link, sent):
        link.deliver(ack(sent.command, result))

    link = FakeLink(responder=responder)
    with pytest.raises(CommandDenied) as info:
        commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE, [1, 4])
    assert info.value.result == result
    assert len(link.sent) == 1


def test_command_long_validates_arguments():
    link = FakeLink()
    with pytest.raises(ValueError):
        commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE, [0] * 8)
    with pytest.raises(ValueError):
        commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE, [], retries=0)
    assert link.sent == []


# --- set_mode ---------------------------------------------------------------


def test_set_mode_success():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED))
        later(0.1, lambda: link.deliver(heartbeat("GUIDED")))

    link = FakeLink(responder=responder)
    link.deliver(heartbeat("STABILIZE"))
    commands.set_mode(link, "GUIDED")
    assert len(link.sent) == 1
    sent = link.sent[0]
    assert sent.command == mavlink.MAV_CMD_DO_SET_MODE
    assert sent.params[:2] == [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODES["GUIDED"]]


def test_set_mode_heartbeat_never_changes():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED))

    link = FakeLink(responder=responder)
    with pytest.raises(StateNotReached):
        commands.set_mode(link, "GUIDED", timeout_s=0.3)


def test_set_mode_ignores_stale_cached_heartbeat(background):
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED))

    link = FakeLink(responder=responder)
    link.cache_stale(heartbeat("GUIDED"), age_s=0.5)  # already GUIDED before the send
    background(lambda: link.deliver(heartbeat("STABILIZE")))  # fresh ones show the old mode
    with pytest.raises(StateNotReached):
        commands.set_mode(link, "GUIDED", timeout_s=0.4)


def test_set_mode_ignores_heartbeat_from_other_component():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED),
                     heartbeat("GUIDED", compid=154))  # e.g. a gimbal on sysid 1

    link = FakeLink(responder=responder)
    with pytest.raises(StateNotReached):
        commands.set_mode(link, "GUIDED", timeout_s=0.3)


def test_set_mode_unknown_name_lists_valid_modes():
    link = FakeLink()
    with pytest.raises(ValueError, match="GUIDED"):
        commands.set_mode(link, "WARP_SPEED")
    assert link.sent == []


# --- arm / disarm -----------------------------------------------------------


def test_arm_success():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED), heartbeat(armed=True))

    link = FakeLink(responder=responder)
    commands.arm(link)
    assert link.sent[0].command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert link.sent[0].params[:2] == [1.0, 0.0]


def test_arm_denied_reports_prearm_text_and_does_not_retry():
    def responder(link, sent):
        link.deliver(statustext("PreArm: GPS not healthy"),
                     ack(sent.command, mavlink.MAV_RESULT_DENIED))

    link = FakeLink(responder=responder)
    with pytest.raises(CommandDenied) as info:
        commands.arm(link)
    assert "PreArm: GPS not healthy" in str(info.value)
    assert info.value.statustexts == ["PreArm: GPS not healthy"]
    assert len(link.sent) == 1


def test_arm_denied_collects_statustext_arriving_just_after_ack():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_FAILED))
        later(0.02, lambda: link.deliver(statustext("PreArm: Battery below minimum")))

    link = FakeLink(responder=responder)
    with pytest.raises(CommandDenied, match="Battery below minimum"):
        commands.arm(link)


@pytest.mark.parametrize("device, allow", [
    ("/dev/ttyTHS1", True),
    ("/dev/ttyACM0", True),
    ("udp:127.0.0.1:14550", False),
    ("/dev/ttyTHS1", False),
])
def test_force_arm_refused(device, allow):
    link = FakeLink(device=device)
    with pytest.raises(PermissionError):
        commands.arm(link, force=True, allow_force_arm=allow)
    assert link.sent == []


def test_force_arm_allowed_on_sitl():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED), heartbeat(armed=True))

    link = FakeLink(device="tcp:127.0.0.1:5760", responder=responder)
    commands.arm(link, force=True, allow_force_arm=True)
    assert link.sent[0].params[:2] == [1.0, float(commands.ARM_FORCE_MAGIC)]


def test_disarm_refused_above_one_metre():
    link = FakeLink()
    link.deliver(position(5.0), heartbeat(armed=True))
    with pytest.raises(CommandError, match="5.0 m"):
        commands.disarm(link)
    assert link.sent == []


def test_disarm_refused_without_fresh_position():
    link = FakeLink()
    link.cache_stale(position(0.0), age_s=5.0)
    with pytest.raises(CommandError):
        commands.disarm(link)
    assert link.sent == []


def test_disarm_on_ground():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED), heartbeat(armed=False))

    link = FakeLink(responder=responder)
    link.deliver(position(0.2), heartbeat(armed=True))
    commands.disarm(link)
    assert link.sent[0].params[0] == 0.0


# --- takeoff ----------------------------------------------------------------


@pytest.mark.parametrize("setup, alt_m, error", [
    (lambda l: l.deliver(heartbeat("LOITER", armed=True)), 10, CommandError),
    (lambda l: l.deliver(heartbeat("GUIDED", armed=False)), 10, CommandError),
    (lambda l: l.cache_stale(heartbeat("GUIDED", armed=True), age_s=3.0), 10, CommandError),
    (lambda l: None, 10, CommandError),
    (lambda l: l.deliver(heartbeat("GUIDED", armed=True)), 122.0, ValueError),
    (lambda l: l.deliver(heartbeat("GUIDED", armed=True)), 0, ValueError),
    (lambda l: l.deliver(heartbeat("GUIDED", armed=True)), -5, ValueError),
])
def test_takeoff_preconditions_fail_before_sending(setup, alt_m, error):
    link = FakeLink()
    setup(link)
    with pytest.raises(error):
        commands.takeoff(link, alt_m)
    assert link.sent == []


class ClimbingVehicle:
    """Background vehicle model: climbs at ``rate_m_s`` until ``ceiling_m`` after takeoff."""

    def __init__(self, link, rate_m_s, ceiling_m, mode="GUIDED"):
        self.link = link
        self.rate_m_s = rate_m_s
        self.ceiling_m = ceiling_m
        self.mode = mode
        self.started_at = None

    def respond(self, link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED))
        self.started_at = time.monotonic()

    def tick(self):
        alt_m = 0.0
        if self.started_at is not None:
            alt_m = min(self.ceiling_m, (time.monotonic() - self.started_at) * self.rate_m_s)
        self.link.deliver(heartbeat(self.mode, armed=True), position(alt_m))


def test_takeoff_reaches_altitude(background):
    link = FakeLink()
    vehicle = ClimbingVehicle(link, rate_m_s=20.0, ceiling_m=10.0)
    link.responder = vehicle.respond
    link.deliver(heartbeat("GUIDED", armed=True))
    background(vehicle.tick, period_s=0.05)

    reached_m = commands.takeoff(link, 10.0)
    assert reached_m >= 9.5
    assert link.sent[0].command == mavlink.MAV_CMD_NAV_TAKEOFF
    assert link.sent[0].params[6] == 10.0


def test_takeoff_stall_fails_early(background):
    link = FakeLink()
    vehicle = ClimbingVehicle(link, rate_m_s=2.0, ceiling_m=1.0)
    link.responder = vehicle.respond
    link.deliver(heartbeat("GUIDED", armed=True))
    background(vehicle.tick, period_s=0.1)

    full_timeout_s = 10.0 / 2.5 * 1.5 + 10.0
    start = time.monotonic()
    with pytest.raises(StateNotReached, match="stalled"):
        commands.takeoff(link, 10.0)
    elapsed = time.monotonic() - start
    assert commands.TAKEOFF_STALL_WINDOW_S <= elapsed < full_timeout_s
    assert elapsed < commands.TAKEOFF_STALL_WINDOW_S + 2.0


def test_takeoff_fails_if_mode_leaves_guided(background):
    link = FakeLink()
    vehicle = ClimbingVehicle(link, rate_m_s=1.0, ceiling_m=10.0)

    def respond(link_, sent):
        vehicle.respond(link_, sent)
        vehicle.mode = "LOITER"  # e.g. the safety pilot flips the mode switch

    link.responder = respond
    link.deliver(heartbeat("GUIDED", armed=True))
    background(vehicle.tick, period_s=0.05)

    start = time.monotonic()
    with pytest.raises(StateNotReached, match="left GUIDED"):
        commands.takeoff(link, 10.0)
    assert time.monotonic() - start < 1.0


def test_steps_are_logged(caplog):
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED), heartbeat(armed=True))

    link = FakeLink(responder=responder)
    with caplog.at_level(logging.INFO, logger=commands.__name__):
        commands.arm(link)
    messages = [r.getMessage() for r in caplog.records]
    assert any("Sending MAV_CMD_COMPONENT_ARM_DISARM" in m for m in messages)
    assert any("accepted" in m for m in messages)
    assert "Armed" in messages
