import logging
import time

import pytest
from pymavlink import mavutil

from mission_controller.vehicle import commands
from mission_controller.vehicle.commands import (
    CommandDenied, CommandTimeout, StateNotReached)
from link_fakes import (
    FakeLink, ack, heartbeat, later, mission_current, servo_output, statustext)

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


def test_current_mode():
    link = FakeLink()
    assert commands.current_mode(link) is None
    link.deliver(heartbeat("LOITER", armed=True))
    assert commands.current_mode(link) == "LOITER"


def test_rejection_reports_statustext():
    def responder(link, sent):
        link.deliver(statustext("PreArm: GPS not healthy"),
                     ack(sent.command, mavlink.MAV_RESULT_FAILED))

    link = FakeLink(responder=responder)
    with pytest.raises(CommandDenied, match="GPS not healthy"):
        commands.command_long(link, mavlink.MAV_CMD_DO_SET_SERVO, [7, 1900])
    assert len(link.sent) == 1


# --- velocity setpoints / mission current ----------------------------------------


def test_send_velocity_body_arguments():
    link = FakeLink()
    commands.send_velocity_body(link, 0.3, -0.2, 0.5)
    (call,) = link.calls
    assert call.method == "set_position_target_local_ned_send"
    assert call.args == (0, 1, 1, mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b010111000111,
                         0, 0, 0, 0.3, -0.2, 0.5, 0, 0, 0, 0, 0.0)
    mask = call.args[4]
    ignored = {name for bit, name in [(1, "x"), (2, "y"), (4, "z"), (8, "vx"), (16, "vy"),
                                      (32, "vz"), (64, "afx"), (128, "afy"), (256, "afz"),
                                      (1024, "yaw"), (2048, "yaw_rate")] if mask & bit}
    assert ignored == {"x", "y", "z", "afx", "afy", "afz", "yaw"}   # velocity + yaw rate used
    assert link.sent == []  # not a COMMAND_LONG: no ACK


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_send_velocity_body_rejects_non_finite(bad):
    link = FakeLink()
    with pytest.raises(ValueError):
        commands.send_velocity_body(link, bad, 0.0, 0.0)
    assert link.calls == []


def test_set_mission_current_waits_for_mission_current():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED),
                     mission_current(int(sent.params[0])))

    link = FakeLink(responder=responder)
    link.deliver(mission_current(4))
    commands.set_mission_current(link, 5)
    assert link.sent[0].command == mavlink.MAV_CMD_DO_SET_MISSION_CURRENT
    assert link.sent[0].params[0] == 5.0


def test_set_mission_current_to_a_do_item_expects_the_next_nav():
    # ArduPilot starts a DO item at once and reports the next NAV item in MISSION_CURRENT.
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED), mission_current(7))

    link = FakeLink(responder=responder)
    commands.set_mission_current(link, 6, expect_seq=7)
    assert link.sent[0].params[0] == 6.0
    with pytest.raises(StateNotReached, match="seq 6"):
        commands.set_mission_current(link, 6, timeout_s=0.2)


def test_set_mission_current_not_reached():
    def responder(link, sent):
        link.deliver(ack(sent.command, mavlink.MAV_RESULT_ACCEPTED), mission_current(4))

    link = FakeLink(responder=responder)
    with pytest.raises(StateNotReached, match="seq 5"):
        commands.set_mission_current(link, 5, timeout_s=0.2)


def test_mission_current_seq_needs_fresh_message():
    link = FakeLink()
    assert commands.mission_current_seq(link) is None
    link.cache_stale(mission_current(3), age_s=5.0)
    assert commands.mission_current_seq(link) is None
    link.deliver(mission_current(3))
    assert commands.mission_current_seq(link) == 3


def test_no_mode_or_flight_commands_exposed():
    # The Jetson must never arm, take off or change mode in this design.
    for name in ("set_mode", "arm", "disarm", "takeoff", "send_position_target"):
        assert not hasattr(commands, name)
