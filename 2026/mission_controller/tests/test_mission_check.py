import pytest
from pymavlink import mavutil

from mission_controller.vehicle.mission import (
    MissionItem, download_items, find_handover)

from link_fakes import FakeLink, MissionSim

mavlink = mavutil.mavlink
REL = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
DROP_ALT = 7.62
SERVO, PWM = 7, 1900


def wp(lat, lon, alt):
    return MissionItem(mavlink.MAV_CMD_NAV_WAYPOINT, REL, lat_deg=lat, lon_deg=lon, alt_m=alt)


def limits(timeout=120.0, alt_min=5.0, alt_max=12.0, horiz=25.0):
    return MissionItem(mavlink.MAV_CMD_DO_GUIDED_LIMITS, 0, timeout, alt_min, alt_max, horiz)


def enable(on=1.0):
    return MissionItem(mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0, on)


def jump(target=8, repeat=-1.0):
    return MissionItem(mavlink.MAV_CMD_DO_JUMP, 0, float(target), repeat)


def release(channel=SERVO, pwm=PWM):
    return MissionItem(mavlink.MAV_CMD_DO_SET_SERVO, 0, float(channel), float(pwm))


def good_mission():
    return [
        wp(35.0605, -118.156, 0.0),          # 0 home
        MissionItem(mavlink.MAV_CMD_NAV_TAKEOFF, REL, alt_m=15.0),
        wp(35.0610, -118.1570, 15.0),        # 2 transit, above the target
        wp(35.0610, -118.1570, DROP_ALT),    # 3 FC descends to the drop altitude
        limits(),                             # 4
        enable(),                             # 5 Jetson centres
        jump(),                               # 6 guard: FC fallback jumps to 8
        release(),                            # 7 FC releases (only via the Jetson's hand-back)
        wp(35.0620, -118.1555, 20.0),        # 8 resume
        MissionItem(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, REL),
    ]


def find(items):
    return find_handover(items, SERVO)


def test_good_mission():
    plan = find(good_mission())
    assert (plan.guided_seq, plan.release_seq, plan.resume_seq) == (5, 7, 8)


def test_disable_items_are_not_handovers():
    items = good_mission()
    items.insert(5, enable(on=0.0))
    assert find(items).guided_seq == 6


@pytest.mark.parametrize("mutate", [
    lambda m: m.pop(4),                                   # no DO_GUIDED_LIMITS
    lambda m: m.__setitem__(4, limits(timeout=0, alt_min=0, alt_max=7.0, horiz=0)),
    lambda m: m.pop(6),                                   # no guard
    lambda m: m.__setitem__(6, jump(target=9, repeat=1.0)),
    lambda m: m.__setitem__(7, release(pwm=1100)),
    lambda m: m.__setitem__(3, wp(35.0610, -118.1570, 15.0)),
])
def test_layout_is_not_validated(mutate):
    """Mission Planner checks the mission; the Jetson only needs to find the handover."""
    items = good_mission()
    mutate(items)
    plan = find(items)
    enable_seq = next(i for i, it in enumerate(items)
                      if it.command == mavlink.MAV_CMD_NAV_GUIDED_ENABLE)
    assert plan is not None and plan.guided_seq == enable_seq and plan.release_seq is not None


def test_release_and_guard_in_either_order():
    items = good_mission()
    items[6], items[7] = items[7], items[6]
    assert (find(items).release_seq, find(items).resume_seq) == (6, 8)


@pytest.mark.parametrize("mutate", [lambda m: m.pop(7),
                                    lambda m: m.__setitem__(7, release(channel=6))])
def test_no_release_item(mutate):
    items = good_mission()
    mutate(items)
    plan = find(items)
    assert plan.release_seq is None and plan.resume_seq == (7 if len(items) == 9 else 8)


def test_first_handover_is_used():
    items = good_mission()
    items.insert(9, enable())
    assert find(items).guided_seq == 5


@pytest.mark.parametrize("mutate", [lambda m: m.pop(5),
                                    lambda m: m.__delitem__(slice(8, None))])
def test_no_handover(mutate):
    items = good_mission()
    mutate(items)
    assert find(items) is None


def stored(items):
    return [(it.frame, it.command, it.param1, it.param2, it.param3, it.param4,
             int(it.lat_deg * 1e7), int(it.lon_deg * 1e7), it.alt_m, 1) for it in items]


def test_download_then_find():
    link = FakeLink()
    sim = MissionSim(link)
    sim.stored[0] = stored(good_mission())
    items = download_items(link)
    assert len(items) == 10
    assert find(items).release_seq == 7
    assert not link.calls_to("mission_count_send")       # read-only: nothing uploaded
    assert not link.calls_to("mission_item_int_send")
