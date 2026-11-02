"""Smoke tests for the flight scripts migrated onto the Link, plus a guard against regressions."""

import importlib
import sys
import time
import types
from pathlib import Path

import pytest
from pymavlink import mavutil

from link_fakes import FakeLink, MODES, SimVehicle

mavlink = mavutil.mavlink

DRONE_SIM = Path(__file__).resolve().parents[1]

# Only vehicle/link.py may touch a pymavlink connection. waypoint_nav.py and
# circuit_trial.py read .BIN logs, not a live link.
GUARD_EXEMPT = {"vehicle/link.py", "missions/waypoint_nav.py", "missions/circuit_trial.py"}
FORBIDDEN = ("recv_match", ".mav.", "wait_heartbeat", "motors_armed_wait", "mavlink_connection")


def test_only_link_touches_the_connection():
    offenders = []
    for path in sorted(DRONE_SIM.rglob("*.py")):
        rel = path.relative_to(DRONE_SIM).as_posix()
        if rel.startswith("tests/") or rel in GUARD_EXEMPT:
            continue
        text = path.read_text()
        offenders += ["%s: %s" % (rel, word) for word in FORBIDDEN if word in text]
    assert offenders == []


def test_old_helpers_are_gone():
    assert not (DRONE_SIM / "vehicle" / "connection.py").exists()
    assert not (DRONE_SIM / "vehicle" / "modes.py").exists()


# --- import smoke tests with the vision stack stubbed out ------------------------


class _Stub:
    """Accepts any constructor/method call; get_target_pixel sees nothing."""

    def __init__(self, *args, **kwargs):
        self.released = False

    def get_target_pixel(self):
        return None, None

    def release(self):
        self.released = True

    def log_data(self, *args):
        pass

    def calculate_target_coordinates(self, *args):
        return 0.0, 0.0, None, None


VISION_MODULES = {
    "competition.jetson_rescue": {},
    "competition.jetson_rescue.geolocation": {"TargetGeolocator": _Stub},
    "competition.jetson_rescue.camera": {"VisionPipeline": _Stub},
    "listener": {},
    "listener.Logger": {"TelemetryLogger": _Stub},
    "inference": {},
    "inference.camera": {"VisionPipeline": _Stub},
}

SCRIPTS = [
    "competition.drone_simulator.main",
    "competition.drone_simulator.missions.package_delivery",
    "competition.drone_simulator.missions.payload_drop",
    "competition.drone_simulator.missions.object_localization",
    "competition.drone_simulator.table_test",
    "competition.drone_simulator.barebones_test",
    "competition.drone_simulator.connect_drone",
]


@pytest.fixture
def scripts(monkeypatch, tmp_path):
    """Imports a migrated script with stub vision modules; cleans sys.modules after."""
    for name, attrs in VISION_MODULES.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.chdir(tmp_path)  # object_localization writes a CSV in the cwd
    imported = []

    def load(name):
        sys.modules.pop(name, None)
        imported.append(name)
        return importlib.import_module(name)

    yield load
    for name in imported:
        sys.modules.pop(name, None)


@pytest.mark.parametrize("name", SCRIPTS)
def test_script_imports(scripts, name):
    module = scripts(name)
    assert not hasattr(module, "connect_vehicle")


VALID_CONFIG = {
    "takeoff_alt": 10.0, "max_alt": 60.0, "min_alt": 5.0,
    "waypoints": [(35.0610, -118.1570, 15.0), (35.0620, -118.1555, 20.0)],
    "time_limit": 600, "end_mode": "RTL", "geofence": None,
}


@pytest.fixture
def vehicle(background):
    link = FakeLink()
    sim = SimVehicle(link)
    background(sim.tick, period_s=0.02)
    time.sleep(0.05)
    return sim


def modes_commanded(link):
    names = {v: k for k, v in MODES.items()}
    return [names[int(s.params[1])] for s in link.sent if s.command == mavlink.MAV_CMD_DO_SET_MODE]


def test_master_mission_runs_on_link(scripts, vehicle):
    main = scripts("competition.drone_simulator.main")
    mission = main.MasterFlightMission(vehicle.link, dict(VALID_CONFIG))
    mission.run()
    # MissionBase sets active=True only after execute(), so both drop phases'
    # `while self.active` loops are skipped (unchanged behaviour, see PR notes).
    assert modes_commanded(vehicle.link) == ["GUIDED", "GUIDED", "GUIDED", "AUTO", "RTL"]
    assert mission.vision.released


@pytest.mark.parametrize("module_name, cls_name, expected_mode", [
    ("competition.drone_simulator.missions.package_delivery", "PackageDeliveryMission", "GUIDED"),
    ("competition.drone_simulator.missions.payload_drop", "PayloadDropMission", "GUIDED"),
    ("competition.drone_simulator.missions.object_localization", "ObjectLocalizationMission",
     "AUTO"),
])
def test_desk_test_paths(scripts, vehicle, module_name, cls_name, expected_mode):
    cls = getattr(scripts(module_name), cls_name)
    mission = cls(vehicle.link, dict(VALID_CONFIG), desk_test=True)
    mission.arm_and_takeoff()
    assert vehicle.armed
    assert mission.expected_mode == expected_mode
    assert not any(s.command == mavlink.MAV_CMD_NAV_TAKEOFF for s in vehicle.link.sent)


def test_payload_mission_execute_opens_and_closes_statustext_subscription(scripts, vehicle):
    module = scripts("competition.drone_simulator.missions.payload_drop")
    mission = module.PayloadDropMission(vehicle.link, dict(VALID_CONFIG))
    mission.execute()
    assert modes_commanded(vehicle.link) == ["GUIDED"]
    assert mission.vision.released
    assert vehicle.link._subs == []
