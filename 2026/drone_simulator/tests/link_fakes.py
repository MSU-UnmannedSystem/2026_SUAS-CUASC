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


# --- mission protocol -------------------------------------------------------


class MissionSim:
    """ArduPilot-like mission protocol on a FakeLink: stores items per mission_type.

    Args:
        link: The FakeLink to attach to.
        order: seqs to request, in order (repeats allowed); default 0..count-1.
        use_int: Request with MISSION_REQUEST_INT (True) or legacy MISSION_REQUEST.
        ignore_counts: How many MISSION_COUNTs to ignore (simulates lost packets).
        reject_with: MAV_MISSION_RESULT to ACK the COUNT with, instead of requesting.
        silent: Never answer uploads.
        noise: Messages delivered before each real reply (e.g. from other systems).
        corrupt: ``fn(mission_type, seq, item_msg) -> item_msg`` applied on download.
    """

    def __init__(self, link, order=None, use_int=True, ignore_counts=0, reject_with=None,
                 silent=False, noise=(), corrupt=None):
        self.link = link
        self.order = order
        self.use_int = use_int
        self.ignore_counts = ignore_counts
        self.reject_with = reject_with
        self.silent = silent
        self.noise = list(noise)
        self.corrupt = corrupt
        self.stored = {}          # mission_type -> list of (frame, command, p1..p4, x, y, z)
        self.counts_seen = 0
        self.acks_received = []
        self._pending = {}        # mission_type -> {"count": n, "plan": [...], "items": {}}
        link.handlers.update({
            "mission_count_send": self._on_count,
            "mission_item_int_send": self._on_item,
            "mission_request_list_send": self._on_request_list,
            "mission_request_int_send": self._on_request_int,
            "mission_ack_send": self._on_ack,
        })

    @staticmethod
    def _mission_type(call, positional_index):
        if "mission_type" in call.kwargs:
            return call.kwargs["mission_type"]
        return call.args[positional_index] if len(call.args) > positional_index else 0

    def _reply(self, msg):
        self.link.deliver(*self.noise)
        self.link.deliver(msg)

    def _request(self, mt, state):
        if state["plan"]:
            seq = state["plan"].pop(0)
            self._reply(mission_request(seq, mt, use_int=self.use_int))
        else:
            self.stored[mt] = [state["items"][s] for s in range(state["count"])]
            self._reply(mission_ack(mavlink.MAV_MISSION_ACCEPTED, mt))

    def _on_count(self, link, call):
        self.counts_seen += 1
        if self.silent or self.counts_seen <= self.ignore_counts:
            return
        count, mt = call.args[2], self._mission_type(call, 3)
        if self.reject_with is not None:
            self._reply(mission_ack(self.reject_with, mt))
            return
        plan = list(self.order) if self.order is not None else list(range(count))
        self._pending[mt] = {"count": count, "plan": plan, "items": {}}
        self._request(mt, self._pending[mt])

    def _on_item(self, link, call):
        a = call.args
        mt = self._mission_type(call, 14)
        state = self._pending.get(mt)
        if state is None:
            return
        seq = a[2]
        state["items"][seq] = (a[3], a[4], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[6])
        self._request(mt, state)

    def _on_request_list(self, link, call):
        mt = self._mission_type(call, 2)
        self._reply(mission_count(len(self.stored.get(mt, [])), mt))

    def _on_request_int(self, link, call):
        seq, mt = call.args[2], self._mission_type(call, 3)
        frame, command, p1, p2, p3, p4, x, y, z, autocontinue = self.stored[mt][seq]
        msg = mission_item_int(seq, frame, command, p1, p2, p3, p4, x, y, z, mt,
                               autocontinue=autocontinue)
        if self.corrupt is not None:
            msg = self.corrupt(mt, seq, msg)
        self._reply(msg)

    def _on_ack(self, link, call):
        self.acks_received.append((call.args[2], self._mission_type(call, 3)))


# --- simulated vehicle ------------------------------------------------------


class SimVehicle:
    """ArduCopter-like vehicle on a FakeLink, for whole-mission tests.

    Handles DO_SET_MODE, arm/disarm, NAV_TAKEOFF, DO_SET_SERVO and
    SET_MESSAGE_INTERVAL; answers parameter reads from ``params``; stores
    missions via :class:`MissionSim`. Call :meth:`tick` periodically (e.g. with
    the ``background`` fixture) to stream HEARTBEAT/GLOBAL_POSITION_INT, climb,
    and emit MISSION_ITEM_REACHED in AUTO.
    """

    def __init__(self, link, climb_rate_m_s=40.0, reached_every_ticks=4):
        self.link = link
        self.mode = "STABILIZE"
        self.armed = False
        self.alt_m = 0.0
        self.target_alt_m = None
        self.climb_rate_m_s = climb_rate_m_s
        self.reached_every_ticks = reached_every_ticks
        self.params = {"WP_SPD_UP": 2.5}
        self.missions = MissionSim(link)
        self.mode_log = []
        self.on_mode = {}          # mode name -> fn(vehicle) when entered
        self.emit_reached = True
        self._next_reached = None
        self._ticks = 0
        self._last_tick = time.monotonic()
        self._lock = threading.Lock()
        link.responder = self._on_command
        link.handlers["param_request_read_send"] = self._on_param_read

    def set_mode(self, name):
        with self._lock:
            self.mode = name
            self.mode_log.append(name)
        if name == "AUTO":
            self._next_reached = 1
        callback = self.on_mode.get(name)
        if callback is not None:
            callback(self)

    def _on_param_read(self, link, call):
        name = call.args[2].decode()
        if name in self.params:
            link.deliver(param_value(name, self.params[name]))

    def _on_command(self, link, sent):
        cmd, p = sent.command, sent.params
        result = mavlink.MAV_RESULT_ACCEPTED
        if cmd == mavlink.MAV_CMD_DO_SET_MODE:
            names = {v: k for k, v in MODES.items()}
            if int(p[1]) in names:
                self.set_mode(names[int(p[1])])
            else:
                result = mavlink.MAV_RESULT_FAILED
        elif cmd == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = p[0] == 1
        elif cmd == mavlink.MAV_CMD_NAV_TAKEOFF:
            if self.armed and self.mode == "GUIDED" and self.alt_m < 1.0:
                self.target_alt_m = p[6]
            else:
                result = mavlink.MAV_RESULT_FAILED
        link.deliver(ack(cmd, result))
        link.deliver(heartbeat(self.mode, self.armed))

    def tick(self):
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        self._ticks += 1
        if self.target_alt_m is not None and self.alt_m < self.target_alt_m:
            self.alt_m = min(self.target_alt_m, self.alt_m + self.climb_rate_m_s * dt)
        self.link.deliver(heartbeat(self.mode, self.armed), position(self.alt_m))
        if (self.mode == "AUTO" and self.emit_reached and self._next_reached is not None
                and self._ticks % self.reached_every_ticks == 0):
            stored = self.missions.stored.get(0, [])
            if self._next_reached < len(stored):
                self.link.deliver(mission_item_reached(self._next_reached))
                self._next_reached += 1
