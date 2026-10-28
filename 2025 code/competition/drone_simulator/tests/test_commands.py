import logging
import time

import pytest
from pymavlink import mavutil

from competition.drone_simulator.vehicle import commands
from competition.drone_simulator.vehicle.commands import (
    CommandDenied, CommandError, CommandTimeout, StateNotReached)
from link_fakes import (
    MODES, FakeLink, ack, heartbeat, later, position, servo_output, statustext)

mavlink = mavutil.mavlink


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


# --- set_servo / send_position_target / current_mode ------------------------------


class ServoBench:
    """Vehicle whose servo outputs follow DO_SET_SERVO (unless ``stuck``)."""

    def __init__(self, link, stuck=False, result=mavlink.MAV_RESULT_ACCEPTED):
        self.link = link
        self.outputs = {}
        self.stuck = stuck
        self.result = result
        link.responder = self.respond

    def respond(self, link, sent):
        if sent.command == mavlink.MAV_CMD_DO_SET_SERVO:
            if self.result == mavlink.MAV_RESULT_ACCEPTED and not self.stuck:
                self.outputs[int(sent.params[0])] = int(sent.params[1])
            link.deliver(ack(sent.command, self.result))

    def tick(self):
        self.link.deliver(servo_output(**{"servo%d_raw" % ch: pwm
                                          for ch, pwm in self.outputs.items()}))


def test_set_servo_verified_from_servo_output_raw(background):
    link = FakeLink()
    bench = ServoBench(link)
    background(bench.tick, period_s=0.05)
    time.sleep(0.1)
    commands.set_servo(link, 6, 1900)
    assert link.sent[0].command == mavlink.MAV_CMD_DO_SET_SERVO
    assert link.sent[0].params[:2] == [6.0, 1900.0]


def test_set_servo_not_streaming_only_warns(caplog):
    link = FakeLink()
    ServoBench(link)
    with caplog.at_level(logging.WARNING, logger=commands.__name__):
        commands.set_servo(link, 7, 1900)
    assert any("unverified" in r.getMessage() for r in caplog.records)


def test_set_servo_mismatch_raises(background):
    link = FakeLink()
    bench = ServoBench(link, stuck=True)
    background(bench.tick, period_s=0.05)
    time.sleep(0.1)
    with pytest.raises(StateNotReached, match="servo6_raw is 1100"):
        commands.set_servo(link, 6, 1900, timeout_s=0.3)


def test_set_servo_stale_reading_from_before_send_is_not_proof():
    link = FakeLink()
    ServoBench(link, stuck=True)
    link.deliver(servo_output(servo6_raw=1900))  # fresh (age < 2 s) but from before the send
    with pytest.raises(StateNotReached):
        commands.set_servo(link, 6, 1900, timeout_s=0.2)


def test_set_servo_rejected():
    link = FakeLink()
    ServoBench(link, result=mavlink.MAV_RESULT_FAILED)
    with pytest.raises(CommandDenied):
        commands.set_servo(link, 6, 1900)


@pytest.mark.parametrize("channel, pwm", [(0, 1500), (17, 1500), (6, 799), (6, 2201)])
def test_set_servo_validation(channel, pwm):
    link = FakeLink()
    with pytest.raises(ValueError):
        commands.set_servo(link, channel, pwm)
    assert link.calls == []


def test_send_position_target_arguments():
    link = FakeLink()
    commands.send_position_target(link, 350495780 / 1e7, -1181513480 / 1e7, 16.0)
    (call,) = link.calls
    assert call.method == "set_position_target_global_int_send"
    assert call.args == (0, 1, 1, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b110111111000,
                         350495780, -1181513480, 16.0, 0, 0, 0, 0, 0, 0, 0, 0)
    assert link.sent == []  # not a COMMAND_LONG: no ACK expected


def test_send_position_target_validation():
    link = FakeLink()
    with pytest.raises(ValueError):
        commands.send_position_target(link, 95.0, 0.0, 10.0)
    assert link.calls == []


def test_current_mode():
    link = FakeLink()
    assert commands.current_mode(link) is None
    link.deliver(heartbeat("LOITER", armed=True))
    assert commands.current_mode(link) == "LOITER"
