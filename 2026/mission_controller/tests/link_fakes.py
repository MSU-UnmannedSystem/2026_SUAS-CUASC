# FakeLink stands in for Link (see below): its plain attributes replace Link's
# read-only properties, which type checkers would otherwise report.
# pyright: reportIncompatibleMethodOverride=false, reportAssignmentType=false

"""Test doubles for code that uses the Link API (commands, params, mission, guidance).

Messages are real pymavlink objects encoded with the MAVLink2 dialect (so
extension fields such as ``mission_type`` exist), with sysid/compid set in
the header like a real autopilot's.
"""

import collections
import contextlib
import math
import threading
import time
from typing import TYPE_CHECKING

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2

from mission_controller.vehicle.link import Link, Subscription
from mission_controller.vision.detector import Detection, TargetDetector
from mission_controller.vehicle.dialect import mavlink

MODES = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR) or {}

SentCommand = collections.namedtuple("SentCommand", "command confirmation params")
Call = collections.namedtuple("Call", "method args kwargs")


# --- real pymavlink messages ------------------------------------------------


def make_msg(sysid, compid, name, *fields, **kwargs):
    encoder = mav2.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    msg = getattr(encoder, name + "_encode")(*fields, **kwargs)
    parsed = mav2.MAVLink(None).parse_buffer(bytes(msg.pack(encoder)))
    assert parsed
    return parsed[0]


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


def mission_current(seq, sysid=1):
    return make_msg(sysid, 1, "mission_current", seq)


# --- FakeLink ---------------------------------------------------------------

if TYPE_CHECKING:
    # Type checkers treat FakeLink as a Link, so it can be passed wherever the
    # code expects one. At runtime it's a plain class: nothing is inherited.
    _LinkBase = Link
else:
    _LinkBase = object


class FakeLink(_LinkBase):
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
    is_up = True

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
        if msg is None or rx is None or time.monotonic() - rx > max_age_s:
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


# --- NAV_GUIDED_ENABLE vehicle ------------------------------------------------


class NavGuidedVehicle:
    """ArduCopter-like vehicle during a mission's NAV_GUIDED_ENABLE item.

    Starts armed in AUTO at ``start_alt_m`` with the mission on ``start_seq``.
    Velocity setpoints (SET_POSITION_TARGET_LOCAL_NED, body frame, yaw 0 so
    forward = north) are accepted only while the mode is AUTO and the mission
    is on ``guided_seq``; after ``guid_timeout_s`` without one it stops. Handles
    DO_SET_MISSION_CURRENT like ArduPilot with the mission layout
    GUIDED_ENABLE, guard DO_JUMP(resume), DO_SET_SERVO, resume waypoint:
    setting the release item (``release_seq``) fires ``release`` and
    continues at ``resume_seq``; setting the guard follows the jump (no
    release). MISSION_CURRENT shows NAV items only. Also handles DO_SET_SERVO commands
    (outputs in ``servo_rejects`` are refused) and parameter reads. Call
    :meth:`tick` periodically.

    The target sits at ``target_north_m`` / ``target_east_m`` from the start.
    """

    def __init__(self, link, guided_seq=3, start_seq=None, start_alt_m=7.62,
                 target_north_m=3.0, target_east_m=-2.0, guid_timeout_s=3.0,
                 release=(7, 1900)):
        self.link = link
        self.guided_seq = guided_seq
        self.jump_seq = guided_seq + 1
        self.release_seq = guided_seq + 2
        self.resume_seq = guided_seq + 3
        self.release = release
        self.seq = guided_seq if start_seq is None else start_seq
        self.mode = "AUTO"
        self.armed = True
        self.north_m = 0.0
        self.east_m = 0.0
        self.alt_m = start_alt_m
        self.velocity = (0.0, 0.0, 0.0)
        self.target = (target_north_m, target_east_m)
        self.guid_timeout_s = guid_timeout_s
        self.last_setpoint = float("-inf")
        self.setpoints = []        # (time, forward, right, down) of every accepted setpoint
        self.ignored_setpoints = 0
        self.servos = {}
        self.servo_rejects = set()
        self.publishing = True     # False = the link has gone silent
        self.params = {"SERVO7_FUNCTION": 0.0}
        self.missions = MissionSim(link)
        self._last_tick = time.monotonic()
        self._lock = threading.Lock()
        link.responder = self._on_command
        link.handlers["set_position_target_local_ned_send"] = self._on_setpoint
        link.handlers["param_request_read_send"] = self._on_param_read

    def offset_to_target(self):
        """(forward_m, right_m) from the vehicle to the target (yaw 0)."""
        with self._lock:
            return self.target[0] - self.north_m, self.target[1] - self.east_m

    def _on_setpoint(self, link, call):
        a = call.args
        assert a[3] == mavlink.MAV_FRAME_BODY_OFFSET_NED
        if self.mode != "AUTO" or self.seq != self.guided_seq:
            self.ignored_setpoints += 1  # ArduPilot ignores these outside guided
            return
        with self._lock:
            self.velocity = (a[8], a[9], a[10])
            self.last_setpoint = time.monotonic()
            self.setpoints.append((self.last_setpoint,) + self.velocity)

    def _on_param_read(self, link, call):
        name = call.args[2].decode()
        if name in self.params:
            link.deliver(param_value(name, self.params[name]))

    def _on_command(self, link, sent):
        cmd, p = sent.command, sent.params
        result = mavlink.MAV_RESULT_ACCEPTED
        if cmd == mavlink.MAV_CMD_DO_SET_SERVO:
            if int(p[0]) in self.servo_rejects:
                result = mavlink.MAV_RESULT_FAILED
            else:
                self.servos[int(p[0])] = int(p[1])
        elif cmd == mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
            if int(p[0]) == self.release_seq:
                self.fire_release()
                self.seq = self.resume_seq
            else:
                self.seq = int(p[0])
        link.deliver(ack(cmd, result))
        self.publish()

    def fire_release(self):
        """The FC running the mission's DO_SET_SERVO item."""
        self.servos[self.release[0]] = self.release[1]

    def fc_ends_handover(self):
        """DO_GUIDED_LIMITS timeout/limit: the FC advances, reads the guard DO_JUMP first
        and jumps to the resume waypoint, skipping the release."""
        self.seq = self.resume_seq

    def publish(self):
        if not self.publishing:
            return
        lat = 35.05 + self.north_m / 111319.0
        self.link.deliver(heartbeat(self.mode, self.armed),
                          position(self.alt_m, lat_deg=lat),
                          attitude(), mission_current(self.seq),
                          servo_output(**{"servo%d_raw" % ch: pwm
                                          for ch, pwm in self.servos.items()}))

    def tick(self):
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        with self._lock:
            guided = self.mode == "AUTO" and self.seq == self.guided_seq
            if not guided or now - self.last_setpoint > self.guid_timeout_s:
                self.velocity = (0.0, 0.0, 0.0)
            fwd, right, down = self.velocity
            self.north_m += fwd * dt
            self.east_m += right * dt
            self.alt_m -= down * dt
        self.publish()


class SimDetector(TargetDetector):
    """Projects the NavGuidedVehicle's target into a 640x480 down-looking image.

    Uses the same pinhole model as guidance (level attitude). Returns None when
    ``visible`` is False or the target is outside the frame.
    """

    def __init__(self, vehicle, hfov_deg=65.0, vfov_deg=39.4, width=640, height=480):
        self.vehicle = vehicle
        self.hfov = math.radians(hfov_deg)
        self.vfov = math.radians(vfov_deg)
        self.width = width
        self.height = height
        self.visible = True
        self.stale_by_s = 0.0
        self.closed = False

    def detect(self):
        if not self.visible:
            return None
        m = math
        fwd, right = self.vehicle.offset_to_target()
        alt = self.vehicle.alt_m
        nx = m.tan(m.atan(right / alt)) / m.tan(self.hfov / 2)
        ny = -m.tan(m.atan(fwd / alt)) / m.tan(self.vfov / 2)
        if abs(nx) > 1 or abs(ny) > 1:
            return None
        x = self.width / 2 * (1 + nx)
        y = self.height / 2 * (1 + ny)
        return Detection(x, y, self.width, self.height, time.monotonic() - self.stale_by_s)

    def close(self):
        self.closed = True
