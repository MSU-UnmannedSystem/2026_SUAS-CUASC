import logging
import threading
import time

import pytest
from pymavlink import mavutil

from competition.drone_simulator.missions.mission_base import MissionBase, PilotTakeover

from link_fakes import (
    FakeLink, MODES, SimVehicle, ack, heartbeat, mission_item_reached, position)

mavlink = mavutil.mavlink

CONFIG = {
    "takeoff_alt": 10.0,
    "max_alt": 60.0,
    "min_alt": 5.0,
    "waypoints": [(35.0610, -118.1570, 15.0), (35.0620, -118.1555, 20.0),
                  (35.0605, -118.1545, 15.0)],
    "time_limit": 600,
    "end_mode": "RTL",
    "geofence": (35.0, 35.1, -118.2, -118.1),
}


def commands_sent(link):
    """Readable list of the COMMAND_LONGs sent, e.g. ['MODE GUIDED', 'ARM', 'TAKEOFF 10']."""
    names = {v: k for k, v in MODES.items()}
    out = []
    for s in link.sent:
        if s.command == mavlink.MAV_CMD_DO_SET_MODE:
            out.append("MODE " + names[int(s.params[1])])
        elif s.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            out.append("ARM" if s.params[0] == 1 else "DISARM")
        elif s.command == mavlink.MAV_CMD_NAV_TAKEOFF:
            out.append("TAKEOFF %g" % s.params[6])
        else:
            out.append(str(s.command))
    return out


@pytest.fixture
def vehicle(background):
    link = FakeLink()
    sim = SimVehicle(link)
    background(sim.tick, period_s=0.02)
    time.sleep(0.05)
    return sim


def test_full_run(vehicle, caplog):
    link = vehicle.link
    mission = MissionBase(link, dict(CONFIG))
    with caplog.at_level(logging.INFO):
        mission.run()

    assert commands_sent(link) == ["MODE GUIDED", "ARM", "TAKEOFF 10", "MODE AUTO", "MODE RTL"]
    assert vehicle.mode_log == ["GUIDED", "AUTO", "RTL"]
    # uploaded: home + 3 waypoints + RTL, verified by read-back
    assert len(vehicle.missions.stored[0]) == 5
    assert link.calls_to("mission_request_list_send")
    assert mission.last_nav_seq == 3
    assert mission.last_waypoint >= 3  # the RTL item (seq 4) may be drained in the same pass
    assert mission.start_time is not None
    assert not mission.active
    assert mission._reached is None  # subscription closed
    messages = [r.getMessage() for r in caplog.records]
    assert "All waypoints reached — mission complete" in messages
    assert not any("Mission failed" in m for m in messages)


def test_clock_starts_before_arming(vehicle):
    link = vehicle.link
    mission = MissionBase(link, dict(CONFIG))
    seen = {}
    original = link.responder

    def spy(link_, sent):
        if sent.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            seen["start_time_at_arm"] = mission.start_time
        original(link_, sent)

    link.responder = spy
    mission.run()
    assert seen["start_time_at_arm"] is not None


def test_pilot_takeover_sends_nothing_after(vehicle, caplog):
    link = vehicle.link
    vehicle.emit_reached = False
    marks = {}

    def pilot_flips_switch(sim):
        def flip():
            marks["calls_at_takeover"] = len(link.calls)
            sim.set_mode("LOITER")
        threading.Timer(0.3, flip).start()

    vehicle.on_mode["AUTO"] = pilot_flips_switch
    mission = MissionBase(link, dict(CONFIG))
    with caplog.at_level(logging.CRITICAL):
        mission.run()  # returns instead of sys.exit

    assert marks, "takeover never happened"
    assert len(link.calls) == marks["calls_at_takeover"]
    assert commands_sent(link)[-1] == "MODE AUTO"  # no RTL / LAND after the takeover
    assert any("pilot has control" in r.getMessage() for r in caplog.records)
    assert mission._reached is None


def test_failure_aborts_to_end_mode(vehicle):
    link = vehicle.link
    vehicle.params = {}                       # climb speed falls back to default
    original = link.responder

    def reject_takeoff(link_, sent):
        if sent.command == mavlink.MAV_CMD_NAV_TAKEOFF:
            link_.deliver(ack(sent.command, mavlink.MAV_RESULT_FAILED))
            return
        original(link_, sent)

    link.responder = reject_takeoff
    MissionBase(link, dict(CONFIG)).run()
    assert commands_sent(link)[-1] == "MODE RTL"


def test_invalid_waypoints_abort_before_arming(vehicle):
    link = vehicle.link
    config = dict(CONFIG, waypoints=[(35.05, -118.15, 150)])  # 150 m > 121.9 m
    MissionBase(link, config).run()
    assert "ARM" not in commands_sent(link)
    assert not link.calls_to("mission_count_send")


# --- check_safety ---------------------------------------------------------------


def make_mission(expected_mode="AUTO", **config):
    link = FakeLink()
    mission = MissionBase(link, dict(CONFIG, **config))
    mission.set_expected_mode(expected_mode)
    return link, mission


def test_stale_position_skips_position_checks():
    link, mission = make_mission()
    link.deliver(heartbeat("AUTO", armed=True))
    link.cache_stale(position(1.0), age_s=3.0)       # below min_alt, but 3 s old
    mission.check_safety()
    link.deliver(position(1.0))                       # now fresh
    with pytest.raises(RuntimeError, match="below minimum"):
        mission.check_safety()


def test_safety_checks():
    link, mission = make_mission()
    link.deliver(heartbeat("AUTO", armed=True), position(70.0))
    with pytest.raises(RuntimeError, match="above maximum"):
        mission.check_safety()
    link.deliver(position(20.0, lat_deg=35.2))
    with pytest.raises(RuntimeError, match="Geofence"):
        mission.check_safety()
    link.deliver(heartbeat("RTL", armed=True))
    with pytest.raises(RuntimeError, match="Unexpected mode"):
        mission.check_safety()
    link.deliver(heartbeat("LOITER", armed=True))
    with pytest.raises(PilotTakeover):
        mission.check_safety()


def test_landing_skips_min_alt():
    link, mission = make_mission()
    mission._landing = True
    link.deliver(heartbeat("AUTO", armed=True), position(1.0))
    mission.check_safety()


def test_heartbeat_loss_is_link_failure():
    link, mission = make_mission()
    link.cache_stale(heartbeat("AUTO", armed=True), age_s=6.0)
    with pytest.raises(RuntimeError, match="link failure"):
        mission.check_safety()


def test_monitor_ignores_other_systems_and_completes_at_last_seq():
    link, mission = make_mission()
    mission.last_nav_seq = 3
    with link.subscribe("MISSION_ITEM_REACHED") as sub:
        mission._reached = sub
        mission.active = True
        link.deliver(mission_item_reached(3, sysid=2), mission_item_reached(1))
        mission.monitor()
        assert mission.active and mission.last_waypoint == 1
        link.deliver(mission_item_reached(2), mission_item_reached(3))
        mission.monitor()
        assert not mission.active and mission.last_waypoint == 3
