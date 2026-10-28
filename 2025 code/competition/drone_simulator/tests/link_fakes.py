"""Test doubles for code that uses the Link API (commands, params, upload, missions).

Messages are real pymavlink objects encoded with the MAVLink2 dialect (so
extension fields such as ``mission_type`` exist), with sysid/compid set in
the header like a real autopilot's.
"""

import collections
import contextlib
import threading
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2

from competition.drone_simulator.vehicle.link import Subscription

mavlink = mavutil.mavlink
MODES = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)

SentCommand = collections.namedtuple("SentCommand", "command confirmation params")
Call = collections.namedtuple("Call", "method args kwargs")


# --- real pymavlink messages ------------------------------------------------


def make_msg(sysid, compid, name, *fields, **kwargs):
    encoder = mav2.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    msg = getattr(encoder, name + "_encode")(*fields, **kwargs)
    return mav2.MAVLink(None).parse_buffer(bytes(msg.pack(encoder)))[0]


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


def position(rel_alt_m, lat_deg=35.05, lon_deg=-118.15, sysid=1):
    return make_msg(sysid, 1, "global_position_int", 1000, int(lat_deg * 1e7),
                    int(lon_deg * 1e7), int((700 + rel_alt_m) * 1000), int(rel_alt_m * 1000),
                    0, 0, 0, 0)


def attitude(roll=0.0, pitch=0.0, yaw=0.0):
    return make_msg(1, 1, "attitude", 1000, roll, pitch, yaw, 0, 0, 0)


def servo_output(**raw):
    """SERVO_OUTPUT_RAW with servoN_raw from kwargs (e.g. servo6_raw=1900), others 1100."""
    values = [raw.get("servo%d_raw" % n, 1100) for n in range(1, 17)]
    return make_msg(1, 1, "servo_output_raw", 0, 0, *values)


def param_value(name, value, sysid=1):
    return make_msg(sysid, 1, "param_value", name.encode(), float(value),
                    mavlink.MAV_PARAM_TYPE_REAL32, 1000, 0)


def mission_request(seq, mission_type=0, use_int=True, sysid=1, target=(255, 191)):
    name = "mission_request_int" if use_int else "mission_request"
    return make_msg(sysid, 1, name, target[0], target[1], seq, mission_type)


def mission_ack(result, mission_type=0, sysid=1, target=(255, 191)):
    return make_msg(sysid, 1, "mission_ack", target[0], target[1], result, mission_type)


def mission_count(count, mission_type=0, sysid=1, target=(255, 191)):
    return make_msg(sysid, 1, "mission_count", target[0], target[1], count, mission_type)


def mission_item_int(seq, frame, command, p1, p2, p3, p4, x, y, z, mission_type=0,
                     autocontinue=1, target=(255, 191)):
    return make_msg(1, 1, "mission_item_int", target[0], target[1], seq, frame, command, 0,
                    autocontinue, p1, p2, p3, p4, x, y, z, mission_type)


def mission_item_reached(seq, sysid=1):
    return make_msg(sysid, 1, "mission_item_reached", seq)


# --- FakeLink ---------------------------------------------------------------


class FakeLink:
    """Implements the Link API; tests script the vehicle's replies.

    Every send is recorded in ``calls``; COMMAND_LONGs are also recorded in
    ``sent`` as :data:`SentCommand`. ``responder(link, sent)`` is called for
    every COMMAND_LONG, and ``handlers[method_name](link, call)`` for any send.
    Both may ``deliver()`` messages, which go to open subscriptions and the
    cache exactly like the real reader thread does.
    """

    target_system = 1
    target_component = 1
    source_system = 255
    source_component = 191

    def __init__(self, device="udp:127.0.0.1:14550", responder=None):
        self.device = device
        self.responder = responder
        self.handlers = {}
        self.sent = []
        self.calls = []
        self._cache = {}
        self._subs = []
        self._lock = threading.RLock()

    @property
    def is_serial(self):
        return not self.device.startswith(("udp", "tcp", "mcast"))

    def send(self, method_name, *args, **kwargs):
        if not method_name.endswith("_send"):
            raise ValueError(method_name)
        call = Call(method_name, args, kwargs)
        self.calls.append(call)
        if method_name == "command_long_send":
            ts, tc, command, confirmation = args[:4]
            assert (ts, tc) == (1, 1)
            sent = SentCommand(command, confirmation, list(args[4:]))
            self.sent.append(sent)
            if self.responder is not None:
                self.responder(self, sent)
        handler = self.handlers.get(method_name)
        if handler is not None:
            handler(self, call)

    def calls_to(self, method_name):
        return [c for c in list(self.calls) if c.method == method_name]

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

    def latest_fresh(self, msg_type, max_age_s):
        msg, rx = self.latest_with_time(msg_type)
        if msg is None or time.monotonic() - rx > max_age_s:
            return None
        return msg

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
