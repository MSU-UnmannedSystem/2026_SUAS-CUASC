"""Closed-loop tests: HandoverController against a simulated NAV_GUIDED_ENABLE vehicle."""

import math
import threading
import time

import pytest

from mission_controller.config.flight_params import HandoverConfig
from mission_controller.guidance import handover
from mission_controller.guidance.handover import HandoverController
from mission_controller.vehicle.mission import HandoverPlan

from link_fakes import FakeLink, NavGuidedVehicle, SimDetector

PLAN = HandoverPlan(target_seq=3, target_alt_m=7.62, limits_seq=4, guided_seq=5, jump_seq=6,
                    release_seq=7, resume_seq=8, timeout_s=120.0, alt_min_m=5.0,
                    alt_max_m=12.0, horiz_max_m=25.0)
SET_MISSION_CURRENT = 224

# Same logic as the flight defaults, compressed in time so the suite stays fast.
FAST = dict(max_horizontal_speed_m_s=2.0, horizontal_gain_per_s=1.0, center_hold_s=0.2,
            lost_lock_timeout_s=0.6, command_rate_hz=50.0)


# At the 7.62 m drop altitude the 65 x 39.4 deg camera sees only about +/-2.7 m
# nose-to-tail and +/-4.9 m side-to-side, so the target starts within that.
TARGET = dict(target_north_m=2.0, target_east_m=-1.5)


@pytest.fixture
def vehicle(background):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, **TARGET)
    background(sim.tick, period_s=0.01)
    time.sleep(0.03)
    return sim


def make(vehicle, **overrides):
    config = HandoverConfig(**dict(FAST, **overrides))
    detector = SimDetector(vehicle, config.camera_hfov_deg, config.camera_vfov_deg)
    return HandoverController(vehicle.link, detector, PLAN, config), detector


def handbacks(vehicle):
    return [int(s.params[0]) for s in vehicle.link.sent if s.command == SET_MISSION_CURRENT]


def test_lock_hands_back_at_release_item_and_fc_releases(vehicle):
    controller, _ = make(vehicle)
    assert controller.run() == handover.LOCKED

    assert handbacks(vehicle) == [PLAN.release_seq]       # FC runs the DO_SET_SERVO
    assert vehicle.servos == {7: 1900}                    # ...which released
    assert vehicle.seq == PLAN.resume_seq                 # ...and continues the mission
    fwd, right = vehicle.offset_to_target()
    assert math.hypot(fwd, right) <= 0.5 + 0.05           # over the target
    assert not any(s.command == 183 for s in vehicle.link.sent)   # Jetson never fires it


def test_horizontal_only(vehicle):
    controller, _ = make(vehicle)
    controller.run()
    assert vehicle.setpoints
    assert all(d == 0 for _, _, _, d in vehicle.setpoints)        # vertical velocity always 0
    assert vehicle.alt_m == pytest.approx(7.62)
    speeds = [math.hypot(f, r) for _, f, r, _ in vehicle.setpoints]
    assert max(speeds) <= FAST["max_horizontal_speed_m_s"] + 1e-9


def test_default_config_clamps_to_half_metre_per_second(vehicle):
    config = HandoverConfig(command_rate_hz=50.0)                # flight defaults
    controller = HandoverController(vehicle.link, SimDetector(vehicle), PLAN, config)
    threading.Timer(1.0, lambda: setattr(vehicle, "mode", "LOITER")).start()
    assert controller.run() == handover.PILOT_TAKEOVER
    speeds = [math.hypot(f, r) for _, f, r, _ in vehicle.setpoints]
    assert speeds and max(speeds) <= 0.5 + 1e-9


def test_lost_lock_hands_back_past_the_release(vehicle):
    controller, detector = make(vehicle)
    threading.Timer(0.15, lambda: setattr(detector, "visible", False)).start()
    start = time.monotonic()
    assert controller.run() == handover.GAVE_UP

    assert handbacks(vehicle) == [PLAN.resume_seq]        # DO_SET_SERVO skipped
    assert vehicle.servos == {}
    assert vehicle.seq == PLAN.resume_seq
    assert time.monotonic() - start >= FAST["lost_lock_timeout_s"]


def test_lost_lock_holds_position_while_searching(vehicle):
    controller, detector = make(vehicle, lost_lock_timeout_s=0.5)
    detector.visible = False
    assert controller.run() == handover.GAVE_UP
    assert all(f == 0 and r == 0 and d == 0 for _, f, r, d in vehicle.setpoints)


def test_lock_regained_within_timeout_continues(vehicle):
    controller, detector = make(vehicle, lost_lock_timeout_s=1.0)
    detector.visible = False
    threading.Timer(0.5, lambda: setattr(detector, "visible", True)).start()
    assert controller.run() == handover.LOCKED
    assert vehicle.servos == {7: 1900}


def test_stale_detection_counts_as_lost(vehicle):
    controller, detector = make(vehicle, lost_lock_timeout_s=0.3)
    detector.stale_by_s = 2.0        # camera frozen: last frame is 2 s old
    assert controller.run() == handover.GAVE_UP
    assert vehicle.servos == {}


@pytest.mark.parametrize("alt_m, expected", [(15.0, handover.WRONG_ALTITUDE),
                                             (5.5, handover.WRONG_ALTITUDE),
                                             (8.4, handover.LOCKED)])
def test_drop_altitude_checked_at_handover(background, alt_m, expected):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, start_alt_m=alt_m, **TARGET)
    background(sim.tick, period_s=0.01)
    time.sleep(0.03)
    controller, _ = make(sim)
    assert controller.run() == expected
    if expected == handover.WRONG_ALTITUDE:
        assert handbacks(sim) == [PLAN.resume_seq]        # release skipped
        assert sim.servos == {}
        assert all(f == 0 and r == 0 for _, f, r, _ in sim.setpoints)


def test_target_outside_the_drop_altitude_view_is_never_acquired(background):
    # 3 m ahead at 7.62 m is beyond the +/-2.7 m the camera sees nose-to-tail.
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, target_north_m=3.0,
                           target_east_m=0.0)
    background(sim.tick, period_s=0.01)
    time.sleep(0.03)
    controller, _ = make(sim)
    assert controller.run() == handover.GAVE_UP
    assert sim.servos == {}


def test_pilot_takeover_stops_sending_immediately(vehicle):
    controller, _ = make(vehicle, max_horizontal_speed_m_s=0.5)
    marks = {}

    def pilot():
        marks["sent"] = len(vehicle.link.calls)
        vehicle.mode = "LOITER"

    threading.Timer(0.4, pilot).start()
    assert controller.run() == handover.PILOT_TAKEOVER
    assert len(vehicle.link.calls[marks["sent"]:]) <= 1   # at most one in-flight setpoint
    assert handbacks(vehicle) == []
    assert vehicle.servos == {}


def test_fc_limit_ends_handover_without_release(vehicle):
    # DO_GUIDED_LIMITS timeout/limit: the FC advances via the guard DO_JUMP, so it
    # skips the DO_SET_SERVO even though the target was never centred.
    controller, _ = make(vehicle, max_horizontal_speed_m_s=0.2)
    threading.Timer(0.3, vehicle.fc_ends_handover).start()
    assert controller.run() == handover.FC_ENDED
    assert handbacks(vehicle) == []                       # the Jetson didn't hand back
    assert vehicle.servos == {}                           # and nothing was released
    assert vehicle.seq == PLAN.resume_seq


def test_link_loss_stops_guidance(vehicle):
    controller, _ = make(vehicle, max_horizontal_speed_m_s=0.1, lost_lock_timeout_s=10.0)
    threading.Timer(0.3, lambda: setattr(vehicle, "publishing", False)).start()
    start = time.monotonic()
    assert controller.run() == handover.LINK_LOST
    assert time.monotonic() - start >= handover.HEARTBEAT_LOSS_S
    assert vehicle.setpoints[-1][1:] == (0.0, 0.0, 0.0)   # telemetry went stale: holding


# --- waiting for the handover ---------------------------------------------------------


def test_wait_sends_nothing_until_the_guided_item(background):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, start_seq=2)
    background(sim.tick, period_s=0.01)
    controller = HandoverController(sim.link, SimDetector(sim), PLAN, HandoverConfig())
    threading.Timer(0.5, lambda: setattr(sim, "seq", PLAN.guided_seq)).start()
    start = time.monotonic()
    assert controller.wait_for_handover(poll_s=0.02, timeout_s=3.0)
    assert time.monotonic() - start >= 0.45
    assert sim.link.calls == []                          # silent until the handover


def test_wait_requires_auto_mode(background):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq)
    sim.mode = "LOITER"
    background(sim.tick, period_s=0.01)
    controller = HandoverController(sim.link, SimDetector(sim), PLAN, HandoverConfig())
    assert not controller.wait_for_handover(poll_s=0.02, timeout_s=0.3)


def test_wait_returns_false_when_vehicle_lands_and_disarms(background):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, start_seq=2)
    background(sim.tick, period_s=0.01)
    controller = HandoverController(sim.link, SimDetector(sim), PLAN, HandoverConfig())
    threading.Timer(0.3, lambda: setattr(sim, "armed", False)).start()
    assert not controller.wait_for_handover(poll_s=0.02, timeout_s=3.0)


def test_wait_survives_disarm_on_the_ground(background):
    # Seen in SITL: armed, auto-disarmed after DISARM_DELAY, re-armed, then flew.
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, start_seq=0, start_alt_m=0.0)
    background(sim.tick, period_s=0.01)
    controller = HandoverController(sim.link, SimDetector(sim), PLAN, HandoverConfig())

    def flight():
        sim.armed = False             # DISARM_DELAY on the ground
        time.sleep(0.3)
        sim.armed = True              # pilot re-arms and takes off
        sim.alt_m = 15.0
        time.sleep(0.3)
        sim.seq = PLAN.guided_seq     # AUTO reaches the handover

    threading.Timer(0.2, flight).start()
    assert controller.wait_for_handover(poll_s=0.02, timeout_s=3.0)


def test_wait_warns_when_mission_is_past_the_handover(background, caplog):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, start_seq=PLAN.resume_seq)
    background(sim.tick, period_s=0.01)
    controller = HandoverController(sim.link, SimDetector(sim), PLAN, HandoverConfig())
    with caplog.at_level("WARNING"):
        assert not controller.wait_for_handover(poll_s=0.02, timeout_s=0.3)
    assert any("past NAV_GUIDED_ENABLE" in r.getMessage() for r in caplog.records)


# --- per-tick trace --------------------------------------------------------------------


def test_trace_records_every_tick_through_to_the_lock(vehicle):
    rows = []
    config = HandoverConfig(**FAST)
    controller = HandoverController(vehicle.link, SimDetector(vehicle), PLAN, config,
                                    trace=rows.append)
    assert controller.run() == handover.LOCKED
    assert all(set(r) == set(handover.TRACE_FIELDS) for r in rows)
    phases = [r["phase"] for r in rows]
    assert phases[0] == "ALIGN" and phases[-1] == handover.LOCKED
    assert "CENTRED" in phases
    align = rows[0]
    assert align["sent"] == 1 and align["frame_w_px"] == 640
    assert align["offset_m"] > 0.5 and align["cmd_fwd_m_s"] > 0    # target is ahead
    assert align["alt_m"] == pytest.approx(7.62)
    sent_rows = [r for r in rows if r["sent"]]
    # one row per guidance setpoint, plus the zero "stop" the hand-back sends
    assert len(sent_rows) + 1 == len(vehicle.setpoints)


def test_trace_records_search_while_target_is_lost(vehicle):
    rows = []
    controller = HandoverController(vehicle.link, SimDetector(vehicle),
                                    PLAN, HandoverConfig(**dict(FAST, lost_lock_timeout_s=0.3)),
                                    trace=rows.append)
    controller.detector.visible = False
    assert controller.run() == handover.GAVE_UP
    assert {r["phase"] for r in rows[:-1]} == {"SEARCH"}
    assert rows[-1]["phase"] == handover.GAVE_UP and rows[-1]["lost_for_s"] >= 0.3


def test_broken_trace_never_stops_guidance(vehicle, caplog):
    def broken(row):
        raise IOError("disk full")
    controller = HandoverController(vehicle.link, SimDetector(vehicle), PLAN,
                                    HandoverConfig(**FAST), trace=broken)
    with caplog.at_level("ERROR"):
        assert controller.run() == handover.LOCKED
    assert vehicle.servos == {7: 1900}
    assert sum("trace failed" in r.getMessage() for r in caplog.records) == 1


# --- bench mode --------------------------------------------------------------------------


def bench_vehicle(background, **kw):
    sim = NavGuidedVehicle(FakeLink(), guided_seq=PLAN.guided_seq, start_seq=0, **kw)
    sim.mode = "STABILIZE"        # on the bench: not AUTO, not on the handover item
    sim.armed = False
    background(sim.tick, period_s=0.01)
    time.sleep(0.03)
    return sim


@pytest.mark.parametrize("plan", [PLAN, None])
def test_bench_runs_guidance_but_sends_nothing(background, plan):
    sim = bench_vehicle(background, target_north_m=0.2, target_east_m=0.1)
    rows = []
    controller = HandoverController(sim.link, SimDetector(sim), plan, HandoverConfig(**FAST),
                                    trace=rows.append, bench_alt_m=7.62)
    assert controller.run() == handover.LOCKED      # would lock: target is within 0.5 m
    assert sim.link.calls == []                      # no setpoints, no hand-back, nothing
    assert sim.servos == {} and sim.seq == 0
    assert rows and all(r["sent"] == 0 for r in rows)


def test_bench_uses_the_bench_height(background):
    sim = bench_vehicle(background, target_north_m=2.0, target_east_m=0.0)
    rows = []
    controller = HandoverController(sim.link, SimDetector(sim), PLAN,
                                    HandoverConfig(**dict(FAST, lost_lock_timeout_s=0.2)),
                                    trace=rows.append, bench_alt_m=1.5)
    threading.Timer(0.2, lambda: setattr(controller.detector, "visible", False)).start()
    assert controller.run() == handover.GAVE_UP
    assert rows[0]["alt_m"] == 1.5
    # SimDetector draws the target as seen from 7.62 m, so at 1.5 m the same pixel is
    # 1.5 / 7.62 as far away: 2.0 m * 0.197 = 0.39 m ahead.
    assert rows[0]["fwd_m"] == pytest.approx(2.0 * 1.5 / 7.62, abs=0.01)
    assert sim.link.calls == []


def test_bench_needs_a_positive_height_and_flight_needs_a_plan(vehicle):
    with pytest.raises(ValueError):
        HandoverController(vehicle.link, SimDetector(vehicle), PLAN, HandoverConfig(),
                           bench_alt_m=0.0)
    with pytest.raises(ValueError):
        HandoverController(vehicle.link, SimDetector(vehicle), None, HandoverConfig())
