import pytest
from pymavlink import mavutil

from mission_controller.vehicle.mission import (
    MissionCheckError, MissionItem, check_mission, download_items)

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


def check(items):
    return check_mission(items, DROP_ALT, SERVO, PWM)


def test_good_mission():
    plan = check(good_mission())
    assert (plan.target_seq, plan.limits_seq, plan.guided_seq, plan.jump_seq, plan.release_seq,
            plan.resume_seq) == (3, 4, 5, 6, 7, 8)
    assert plan.target_alt_m == DROP_ALT
    assert (plan.timeout_s, plan.alt_min_m, plan.alt_max_m, plan.horiz_max_m) == (
        120.0, 5.0, 12.0, 25.0)


def test_drop_altitude_is_not_checked_before_flight():
    items = good_mission()
    items[3] = wp(35.0610, -118.1570, 15.0)   # no descent waypoint: checked at handover time
    assert check(items).target_alt_m == 15.0


def test_disable_items_are_not_handovers():
    items = good_mission()
    items.insert(9, enable(on=0.0))
    assert check(items).guided_seq == 5


@pytest.mark.parametrize("mutate, message", [
    (lambda m: m.pop(5), "exactly one NAV_GUIDED_ENABLE"),
    (lambda m: m.insert(8, enable()), "exactly one NAV_GUIDED_ENABLE"),
    (lambda m: m.pop(4), "No DO_GUIDED_LIMITS"),
    (lambda m: m.__setitem__(4, limits(timeout=0)), "timeout"),
    (lambda m: m.__setitem__(4, limits(horiz=0)), "horizontal limit"),
    (lambda m: m.__setitem__(4, limits(alt_min=0)), "min alt"),
    (lambda m: m.__setitem__(4, limits(alt_min=8.0)), "min alt"),
    (lambda m: m.__setitem__(4, limits(alt_max=7.0)), "max alt"),
    (lambda m: m.pop(7), "must be DO_SET_SERVO"),
    (lambda m: m.__setitem__(7, release(channel=6)), "must be DO_SET_SERVO servo 7"),
    (lambda m: m.__setitem__(7, release(pwm=1100)), "PWM 1900"),
    (lambda m: m.pop(6), "must be DO_JUMP"),                       # no guard
    (lambda m: m.__setitem__(6, jump(target=9)), "must be DO_JUMP"),  # wrong target
    (lambda m: m.__setitem__(6, jump(repeat=1.0)), "repeat -1"),
    (lambda m: m.__setitem__(6, MissionItem(mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, 5)),
     "must be DO_JUMP"),
    (lambda m: (m.__setitem__(6, release()), m.__setitem__(7, jump())), "must be DO_JUMP"),
    (lambda m: m.__delitem__(slice(8, None)), "no NAV item after the release"),
])
def test_bad_missions(mutate, message):
    items = good_mission()
    mutate(items)
    with pytest.raises(MissionCheckError, match=message):
        check(items)


def test_limits_before_the_target_waypoint_do_not_count():
    items = good_mission()
    items.insert(3, items.pop(4))   # limits now before the descent waypoint
    with pytest.raises(MissionCheckError, match="No DO_GUIDED_LIMITS"):
        check(items)


def test_unlimited_max_alt_is_allowed():
    items = good_mission()
    items[4] = limits(alt_max=0.0)
    assert check(items).alt_max_m == 0.0


def stored(items):
    return [(it.frame, it.command, it.param1, it.param2, it.param3, it.param4,
             int(it.lat_deg * 1e7), int(it.lon_deg * 1e7), it.alt_m, 1) for it in items]


def test_download_then_check():
    link = FakeLink()
    sim = MissionSim(link)
    sim.stored[0] = stored(good_mission())
    items = download_items(link)
    assert len(items) == 10
    assert check(items).release_seq == 7
    assert not link.calls_to("mission_count_send")       # read-only: nothing uploaded
    assert not link.calls_to("mission_item_int_send")
