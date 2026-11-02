import logging
import time

import pytest

from competition.drone_simulator.vehicle import params
from competition.drone_simulator.vehicle.params import ParamError, ParamValue

from link_fakes import FakeLink, param_value


def autopilot_with(values, sysid=1):
    """FakeLink whose autopilot answers PARAM_REQUEST_READ for names in ``values``."""
    link = FakeLink()

    def on_read(link_, call):
        name = call.args[2].decode()
        if name in values:
            link_.deliver(param_value(name, values[name], sysid=sysid))

    def on_set(link_, call):
        name, value = call.args[2].decode(), call.args[3]
        values[name] = value
        link_.deliver(param_value(name, value, sysid=sysid))

    link.handlers["param_request_read_send"] = on_read
    link.handlers["param_set_send"] = on_set
    return link


def test_read_single_param():
    link = autopilot_with({"FS_GCS_ENABLE": 1.0})
    assert params.read_param(link, "FS_GCS_ENABLE") == ParamValue("FS_GCS_ENABLE", 1.0)


def test_read_prefers_earlier_name_when_both_exist():
    link = autopilot_with({"MAV_GCS_SYSID": 255.0, "SYSID_MYGCS": 254.0})
    assert params.read_param(link, "MAV_GCS_SYSID", "SYSID_MYGCS").name == "MAV_GCS_SYSID"
    assert params.read_param(link, "SYSID_MYGCS", "MAV_GCS_SYSID").name == "SYSID_MYGCS"


def test_read_falls_back_to_later_name():
    link = autopilot_with({"SYSID_MYGCS": 255.0})
    start = time.monotonic()
    found = params.read_param(link, "MAV_GCS_SYSID", "SYSID_MYGCS", timeout_s=2.0)
    assert found == ParamValue("SYSID_MYGCS", 255.0)
    assert time.monotonic() - start < 1.0  # short grace wait, not the full timeout
    sent = [c.args[2] for c in link.calls_to("param_request_read_send")]
    assert sent == [b"MAV_GCS_SYSID", b"SYSID_MYGCS"]  # one attempt, both names


def test_read_missing_returns_none_after_retries():
    link = autopilot_with({})
    assert params.read_param(link, "NOPE", timeout_s=0.05, retries=3) is None
    assert len(link.calls_to("param_request_read_send")) == 3


def test_read_ignores_other_systems():
    link = autopilot_with({"FS_GCS_ENABLE": 1.0}, sysid=255)
    assert params.read_param(link, "FS_GCS_ENABLE", timeout_s=0.05, retries=1) is None


def test_set_param_verified_by_echo():
    values = {"FS_GCS_ENABLE": 0.0}
    link = autopilot_with(values)
    params.set_param(link, "FS_GCS_ENABLE", 1)
    assert values["FS_GCS_ENABLE"] == 1.0


def test_set_param_without_matching_echo_raises():
    link = FakeLink()
    link.handlers["param_set_send"] = lambda l, call: l.deliver(param_value("FS_GCS_ENABLE", 0.0))
    with pytest.raises(ParamError, match="FS_GCS_ENABLE"):
        params.set_param(link, "FS_GCS_ENABLE", 1, timeout_s=0.1)


@pytest.mark.parametrize("values, expected, rule", [
    ({"WP_SPD_UP": 2.5}, 2.5, "treated as m/s"),
    ({"WP_SPD_UP": 250.0}, 2.5, "treated as cm/s"),
    ({"WPNAV_SPEED_UP": 300.0}, 3.0, "WPNAV_SPEED_UP is cm/s"),
    ({"WP_SPD_UP": 1.5, "WPNAV_SPEED_UP": 300.0}, 1.5, "treated as m/s"),
])
def test_climb_speed_conversion(values, expected, rule, caplog):
    link = autopilot_with(values)
    with caplog.at_level(logging.INFO, logger=params.__name__):
        assert params.climb_speed_m_s(link) == pytest.approx(expected)
    assert any(rule in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize("values", [{}, {"WP_SPD_UP": 0.0}])
def test_climb_speed_falls_back_to_default(values, caplog):
    link = autopilot_with(values)
    with caplog.at_level(logging.WARNING, logger=params.__name__):
        assert params.climb_speed_m_s(link, default_m_s=1.8) == 1.8
    assert any("1.80 m/s" in r.getMessage() for r in caplog.records)
