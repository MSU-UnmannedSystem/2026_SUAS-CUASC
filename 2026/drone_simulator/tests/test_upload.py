import time

import pytest
from pymavlink import mavutil

from competition.drone_simulator.vehicle import upload
from competition.drone_simulator.vehicle.upload import (
    MissionItem, MissionUploadError, MissionVerifyError)

from link_fakes import (
    FakeLink, MissionSim, mission_ack, mission_item_int, mission_request)

mavlink = mavutil.mavlink
FENCE = mavlink.MAV_MISSION_TYPE_FENCE
WAYPOINTS = [(35.0610, -118.1570, 15.0), (35.0620, -118.1555, 20.0), (35.0605, -118.1545, 15.0)]


def items(n):
    return [MissionItem(mavlink.MAV_CMD_NAV_WAYPOINT, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        lat_deg=35.0 + i / 1000, lon_deg=-118.0 - i / 1000, alt_m=10.0 + i)
            for i in range(n)]


def sent_item_seqs(link):
    return [c.args[2] for c in link.calls_to("mission_item_int_send")]


# --- upload_items ------------------------------------------------------------


def test_upload_in_order():
    link = FakeLink()
    sim = MissionSim(link)
    upload.upload_items(link, items(4))
    assert sent_item_seqs(link) == [0, 1, 2, 3]
    assert len(sim.stored[0]) == 4
    first = link.calls_to("mission_item_int_send")[1].args
    assert first[11:14] == (int(35.001 * 1e7), int(-118.001 * 1e7), 11.0)  # x/y = int(deg*1e7)
    assert "mission_type" not in link.calls_to("mission_count_send")[0].kwargs


def test_upload_answers_out_of_order_and_duplicate_requests():
    link = FakeLink()
    sim = MissionSim(link, order=[2, 0, 0, 3, 1, 3])
    upload.upload_items(link, items(4))
    assert sent_item_seqs(link) == [2, 0, 0, 3, 1, 3]
    assert [row[6] for row in sim.stored[0]] == [int((35.0 + i / 1000) * 1e7) for i in range(4)]


def test_upload_answers_legacy_mission_request_with_item_int():
    link = FakeLink()
    MissionSim(link, use_int=False)
    upload.upload_items(link, items(3))
    assert sent_item_seqs(link) == [0, 1, 2]
    assert not link.calls_to("mission_item_send")


def test_upload_resends_count_after_silence():
    link = FakeLink()
    sim = MissionSim(link, ignore_counts=2)
    upload.upload_items(link, items(2), timeout_s=0.05, retries=3)
    assert sim.counts_seen == 3
    assert sent_item_seqs(link) == [0, 1]


def test_upload_resends_last_item_after_silence():
    link = FakeLink()
    sim = MissionSim(link)
    dropped = {"done": False}
    original = link.handlers["mission_item_int_send"]

    def drop_first_item_1(link_, call):
        if call.args[2] == 1 and not dropped["done"]:
            dropped["done"] = True  # autopilot never saw it
            return
        original(link_, call)

    link.handlers["mission_item_int_send"] = drop_first_item_1
    upload.upload_items(link, items(3), timeout_s=0.05)
    assert sent_item_seqs(link) == [0, 1, 1, 2]
    assert len(sim.stored[0]) == 3


def test_upload_gives_up_after_retries():
    link = FakeLink()
    MissionSim(link, silent=True)
    start = time.monotonic()
    with pytest.raises(MissionUploadError, match="timed out"):
        upload.upload_items(link, items(2), timeout_s=0.05, retries=3)
    assert len(link.calls_to("mission_count_send")) == 4  # first + 3 re-sends
    assert time.monotonic() - start < 1.0


def test_upload_error_ack_raises_with_result_name():
    link = FakeLink()
    MissionSim(link, reject_with=mavlink.MAV_MISSION_NO_SPACE)
    with pytest.raises(MissionUploadError, match="MAV_MISSION_NO_SPACE"):
        upload.upload_items(link, items(2))


def test_upload_ignores_other_systems_and_other_gcs():
    noise = [
        mission_request(5, sysid=2),                           # another vehicle
        mission_request(5, target=(255, 190)),                 # reply to Mission Planner
        mission_request(5, mission_type=FENCE),                # another mission type
        mission_ack(mavlink.MAV_MISSION_ERROR, target=(255, 190)),
    ]
    link = FakeLink()
    MissionSim(link, noise=noise)
    upload.upload_items(link, items(3))
    assert sent_item_seqs(link) == [0, 1, 2]


def test_upload_rejects_out_of_range_request():
    link = FakeLink()
    MissionSim(link, order=[0, 7])
    with pytest.raises(MissionUploadError, match="seq 7"):
        upload.upload_items(link, items(2))


# --- download ------------------------------------------------------------------


def test_download_round_trip_and_final_ack():
    link = FakeLink()
    sim = MissionSim(link)
    sent = items(3)
    upload.upload_items(link, sent)
    got = upload.download_items(link)
    assert [i.command for i in got] == [i.command for i in sent]
    assert got[2].lat_deg == pytest.approx(sent[2].lat_deg, abs=1e-7)
    assert got[2].alt_m == pytest.approx(12.0)
    assert sim.acks_received == [(mavlink.MAV_MISSION_ACCEPTED, 0)]


# --- build_waypoint_mission --------------------------------------------------------


def test_build_waypoint_mission_layout():
    built = upload.build_waypoint_mission(WAYPOINTS, hold_s=2.0)
    assert len(built) == len(WAYPOINTS) + 2
    home, wps, rtl = built[0], built[1:-1], built[-1]
    assert (home.command, home.frame, home.alt_m) == (
        mavlink.MAV_CMD_NAV_WAYPOINT, mavlink.MAV_FRAME_GLOBAL_INT, 0.0)
    assert (home.lat_deg, home.lon_deg) == WAYPOINTS[0][:2]
    for item, (lat, lon, alt) in zip(wps, WAYPOINTS):
        assert item.command == mavlink.MAV_CMD_NAV_WAYPOINT
        assert item.frame == mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        assert (item.lat_deg, item.lon_deg, item.alt_m, item.param1) == (lat, lon, alt, 2.0)
    assert rtl.command == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
    assert upload.build_waypoint_mission(WAYPOINTS, end_action=None)[-1].command == \
        mavlink.MAV_CMD_NAV_WAYPOINT


@pytest.mark.parametrize("waypoints", [
    [],
    [(35.0, -118.0, 10.0)] * 51,
    [(90.5, -118.0, 10.0)],
    [(35.0, -180.5, 10.0)],
    [(35.0, -118.0, 0.0)],
    [(35.0, -118.0, -3.0)],
    [(35.0, -118.0, 122.0)],
    [(35.0, -118.0, 150.0)],          # e.g. feet passed as metres
    [(float("nan"), -118.0, 10.0)],
])
def test_build_waypoint_mission_validation(waypoints):
    with pytest.raises(ValueError):
        upload.build_waypoint_mission(waypoints)


def test_build_waypoint_mission_bad_options():
    with pytest.raises(ValueError):
        upload.build_waypoint_mission(WAYPOINTS, end_action="LAND")
    with pytest.raises(ValueError):
        upload.build_waypoint_mission(WAYPOINTS, hold_s=-1)


# --- upload_waypoint_mission / fence ------------------------------------------------


def test_upload_waypoint_mission_verifies_and_returns_last_nav_seq():
    link = FakeLink()
    sim = MissionSim(link)
    assert upload.upload_waypoint_mission(link, WAYPOINTS) == 3
    assert len(sim.stored[0]) == 5


def test_invalid_waypoints_send_nothing():
    link = FakeLink()
    MissionSim(link)
    with pytest.raises(ValueError):
        upload.upload_waypoint_mission(link, [(35.0, -118.0, 150.0)])
    assert link.calls == []


@pytest.mark.parametrize("field, value, text", [
    ("x", 5, "position"),          # 5e-7 deg (about 5 cm)
    ("z", 99.0, "alt"),
    ("command", mavlink.MAV_CMD_NAV_LOITER_UNLIM, "command"),
])
def test_readback_mismatch_raises(field, value, text):
    def corrupt(mt, seq, msg):
        if seq != 2:
            return msg
        f = dict(frame=msg.frame, command=msg.command, x=msg.x, y=msg.y, z=msg.z)
        f[field] = msg.x + value if field == "x" else value
        return mission_item_int(seq, f["frame"], f["command"], msg.param1, msg.param2,
                                msg.param3, msg.param4, f["x"], f["y"], f["z"], mt)

    link = FakeLink()
    MissionSim(link, corrupt=corrupt)
    with pytest.raises(MissionVerifyError, match="seq 2: " + text):
        upload.upload_waypoint_mission(link, WAYPOINTS)


def test_readback_accepts_one_unit_rounding():
    def nudge(mt, seq, msg):  # autopilot stores 1e-7 deg off (within tolerance)
        return mission_item_int(seq, msg.frame, msg.command, msg.param1, msg.param2, msg.param3,
                                msg.param4, msg.x + 1, msg.y - 1, msg.z, mt)

    link = FakeLink()
    MissionSim(link, corrupt=nudge)
    assert upload.upload_waypoint_mission(link, WAYPOINTS) == 3


def test_readback_ignores_home_seq0():
    def move_home(mt, seq, msg):  # ArduPilot replaces seq 0 with its real home
        if seq != 0:
            return msg
        return mission_item_int(0, msg.frame, msg.command, 0, 0, 0, 0,
                                int(35.0605 * 1e7), int(-118.156 * 1e7), 850.0, mt)

    link = FakeLink()
    MissionSim(link, corrupt=move_home)
    assert upload.upload_waypoint_mission(link, WAYPOINTS) == 3


POLYGON = [(35.05932, -118.149), (35.06496, -118.156), (35.06062, -118.163), (35.05932, -118.163)]


def test_upload_fence_items_and_verify():
    link = FakeLink()
    sim = MissionSim(link)
    assert upload.upload_fence(link, POLYGON) == 4
    count_call = link.calls_to("mission_count_send")[0]
    assert count_call.kwargs == {"mission_type": FENCE}
    assert all(c.kwargs == {"mission_type": FENCE}
               for c in link.calls_to("mission_item_int_send"))
    rows = sim.stored[FENCE]
    assert len(rows) == 4
    for row, (lat, lon) in zip(rows, POLYGON):
        frame, command, p1 = row[0], row[1], row[2]
        assert command == mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        assert frame == mavlink.MAV_FRAME_GLOBAL_INT
        assert p1 == 4.0
        assert (row[6], row[7]) == (int(lat * 1e7), int(lon * 1e7))
    assert 0 not in sim.stored  # the mission itself is untouched


def test_fence_validation():
    link = FakeLink()
    with pytest.raises(ValueError):
        upload.upload_fence(link, POLYGON[:2])
    with pytest.raises(ValueError):
        upload.upload_fence(link, [(35.0, -118.0), (35.1, -118.0), (95.0, -118.1)])
    assert link.calls == []
