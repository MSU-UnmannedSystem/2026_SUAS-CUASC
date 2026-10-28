"""SITL verification checks for Link and vehicle commands.

Run from the "2025 code" folder, e.g.::

    python3 competition/drone_simulator/tools/sitl_link_check.py lock --device tcp:127.0.0.1:5762

Subcommands (see docs/runbooks/sitl_link_checks.md for the full procedure):
    lock       Link targets the autopilot, not Mission Planner's forwarded heartbeat.
    failsafe   ArduPilot's GCS failsafe counts our heartbeats (SYSID_MYGCS question).
    acks       Record the COMMAND_ACKs ArduPilot sends for DO_SET_MODE / TAKEOFF / ARM.
    reconnect  Watch is_up/age()/CPU while you kill and restart the link.
    params     Find the waypoint climb-rate param (WPNAV_SPEED_UP or its renamed
               successor) and compare it with config climb_rate_m_s.

Only ``params`` (read-only) is allowed on a serial device; everything else
flies the vehicle and refuses to run unless the link is udp/tcp or
``--i-know-this-is-sitl`` is given (needed for the pty bridge).
"""

import argparse
import logging
import queue
import re
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from pymavlink import mavutil

if not __package__:
    # Run as `python3 .../tools/sitl_link_check.py`: put "2025 code" on sys.path, as `-m` would.
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from competition.drone_simulator.config.mission_params import parameters
from competition.drone_simulator.vehicle import commands, params
from competition.drone_simulator.vehicle.commands import CommandDenied, CommandError
from competition.drone_simulator.vehicle.link import Link, LinkConfig, LinkTimeoutError

logger = logging.getLogger("sitl")
mavlink = mavutil.mavlink

# --- helpers -----------------------------------------------------------------


def _mode_name(link: Link, custom_mode: int) -> str:
    for name, number in (link.mode_mapping() or {}).items():
        if number == custom_mode:
            return name
    return "mode%d" % custom_mode


def _is_autopilot(link: Link, msg) -> bool:
    return (msg.get_srcSystem() == link.target_system
            and msg.get_srcComponent() == link.target_component)


class Monitor:
    """Background log of ACKs, STATUSTEXTs, heartbeat sources and mode/arm changes.

    Times are seconds since the monitor started. While active it wraps
    ``link.send`` to timestamp every COMMAND_LONG, so each ACK's latency is
    measured from the latest send of that command BEFORE the ACK arrived
    (pymavlink stamps receive time; the monitor may process the ACK later).
    """

    def __init__(self, link: Link) -> None:
        self.link = link
        self.t0 = time.monotonic()
        self.acks = []  # type: List[Tuple[float, int, int, Optional[float]]]
        self.statustexts = []  # type: List[Tuple[float, str]]
        self.heartbeat_sources = {}  # type: Dict[Tuple[int, int, int], int]
        self.gcs_last_seen = {}  # type: Dict[Tuple[int, int], float]
        self.mode_changes = []  # type: List[Tuple[float, str, bool]]
        self._last_state = None  # type: Optional[Tuple[int, bool]]
        self._sent_at = {}  # type: Dict[int, List[float]]  # wall time, like msg._timestamp
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sitl-monitor", daemon=True)

    def __enter__(self) -> "Monitor":
        original_send = self.link.send

        def recording_send(method_name, *args, **kwargs):
            if method_name == "command_long_send" and len(args) >= 3:
                self._sent_at.setdefault(args[2], []).append(time.time())
            return original_send(method_name, *args, **kwargs)

        self.link.send = recording_send  # diagnostic only: shadows Link.send on this instance
        self._thread.start()
        self._ready.wait(2.0)
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        self._thread.join(2.0)
        del self.link.send

    def now(self) -> float:
        return time.monotonic() - self.t0

    def _latency_ms(self, msg) -> Optional[float]:
        rx = getattr(msg, "_timestamp", None)
        sends = [s for s in self._sent_at.get(msg.command, []) if rx is not None and s <= rx]
        return (rx - sends[-1]) * 1000.0 if sends else None

    def _run(self) -> None:
        with self.link.subscribe("COMMAND_ACK", "STATUSTEXT", "HEARTBEAT", maxsize=500) as sub:
            self._ready.set()
            while not self._stop.is_set():
                try:
                    msg = sub.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    self._handle(msg)
                except Exception:
                    logger.exception("monitor failed on %s", msg.get_type())

    def _handle(self, msg) -> None:
        t = self.now()
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            key = (msg.get_srcSystem(), msg.get_srcComponent(), msg.type)
            if key not in self.heartbeat_sources:
                logger.info("[%6.1fs] heartbeat source seen: sysid=%d compid=%d type=%s",
                            t, key[0], key[1], commands._enum_name("MAV_TYPE", key[2]))
            self.heartbeat_sources[key] = self.heartbeat_sources.get(key, 0) + 1
            if msg.type == mavlink.MAV_TYPE_GCS:
                self.gcs_last_seen[key[:2]] = t
            if _is_autopilot(self.link, msg):
                state = (msg.custom_mode, bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
                if state != self._last_state:
                    self._last_state = state
                    name = _mode_name(self.link, state[0])
                    self.mode_changes.append((t, name, state[1]))
                    logger.info("[%6.1fs] vehicle now %s, %s", t, name,
                                "ARMED" if state[1] else "disarmed")
        elif kind == "STATUSTEXT" and msg.get_srcSystem() == self.link.target_system:
            self.statustexts.append((t, msg.text))
            logger.info("[%6.1fs] STATUSTEXT: %s", t, msg.text)
        elif kind == "COMMAND_ACK" and msg.get_srcSystem() == self.link.target_system:
            latency_ms = self._latency_ms(msg)
            self.acks.append((t, msg.command, msg.result, latency_ms))
            logger.info("[%6.1fs] COMMAND_ACK %s -> %s%s", t,
                        commands._enum_name("MAV_CMD", msg.command),
                        commands._enum_name("MAV_RESULT", msg.result),
                        "" if latency_ms is None else " (%.0f ms after send)" % latency_ms)

    def gcs_heartbeat_seen(self) -> bool:
        return any(t == mavlink.MAV_TYPE_GCS for (_, _, t) in self.heartbeat_sources)

    def gcs_sources_since(self, t_start: float) -> List[str]:
        """Other GCSs (not us; the autopilot doesn't echo our own heartbeats) heard after t_start."""
        return ["sysid=%d compid=%d%s" % (s, c, " (MAVProxy)" if c == 230 else
                                          " (Mission Planner)" if c == 190 else "")
                for (s, c), t in sorted(self.gcs_last_seen.items()) if t >= t_start]

    def failsafe_seen_since(self, t_start: float, start_mode: str) -> Optional[str]:
        """Returns a description if the vehicle left ``start_mode`` or said 'failsafe' after t_start."""
        for t, text in self.statustexts:
            if t >= t_start and "failsafe" in text.lower():
                return "STATUSTEXT '%s' at %.1fs" % (text, t)
        for t, name, _ in self.mode_changes:
            if t >= t_start and name != start_mode:
                return "mode changed to %s at %.1fs" % (name, t)
        return None


def open_link(args: argparse.Namespace) -> Link:
    config = LinkConfig(device=args.device, baud=args.baud, source_system=args.source_system,
                        expected_sysid=args.expected_sysid, connect_timeout_s=args.timeout)
    logger.info("Connecting to %s as sysid=%d compid=%d", args.device,
                config.source_system, config.source_component)
    return Link.connect(config)


def require_sitl(link: Link, args: argparse.Namespace) -> None:
    if link.is_serial and not args.i_know_this_is_sitl:
        raise SystemExit("Refusing to fly: %s looks like real hardware. Use a udp/tcp SITL "
                         "device, or --i-know-this-is-sitl for the pty bridge." % args.device)


def arm_when_ready(link: Link, timeout_s: float = 120.0) -> None:
    """Retries arm() while SITL's EKF/GPS settle (pre-arm failures are expected for ~30 s)."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            commands.arm(link)
            return
        except CommandDenied as exc:
            if time.monotonic() > deadline:
                raise
            logger.info("Not ready to arm yet (%s); retrying in 5 s", exc)
            time.sleep(5.0)


def climb_rate_m_s() -> float:
    return float(parameters().get("climb_rate_m_s", 2.5))


def take_off(link: Link, alt_m: float) -> None:
    climb_m_s = params.climb_speed_m_s(link, default_m_s=climb_rate_m_s())  # before arming
    commands.set_mode(link, "GUIDED")
    arm_when_ready(link)
    commands.takeoff(link, alt_m, climb_rate_m_s=climb_m_s)


def land_and_wait(link: Link, timeout_s: float = 90.0) -> None:
    commands.set_mode(link, "LAND")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        hb = link.latest("HEARTBEAT")
        if hb is not None and not hb.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            logger.info("Landed and disarmed")
            return
        time.sleep(0.5)
    logger.warning("Still armed %.0f s after LAND", timeout_s)


def result(ok: Optional[bool], message: str) -> int:
    label = {True: "PASS", False: "FAIL", None: "INCONCLUSIVE"}[ok]
    logger.info("RESULT: %s - %s", label, message)
    return 0 if ok else 1


def prompt(message: str) -> None:
    logger.info(">>> %s", message)
    while True:
        typed = input(">>> press Enter (here) to continue ").strip()
        if not typed:
            return
        logger.warning("'%s' was typed into THIS script, not MAVProxy. Type MAVProxy commands "
                       "at the GUIDED> prompt in the sim_vehicle.py terminal, then press Enter "
                       "here.", typed)


# --- checks ------------------------------------------------------------------


def check_lock(args: argparse.Namespace) -> int:
    ignored = []

    class _Capture(logging.Handler):
        def emit(self, record):
            if "Ignoring heartbeat" in record.getMessage():
                ignored.append(record.getMessage())

    capture = _Capture()
    logging.getLogger("competition.drone_simulator.vehicle.link").addHandler(capture)
    try:
        link = open_link(args)
    finally:
        logging.getLogger("competition.drone_simulator.vehicle.link").removeHandler(capture)

    with link, Monitor(link) as monitor:
        hb = link.latest("HEARTBEAT")
        logger.info("Locked onto sysid=%d compid=%d type=%s autopilot=%s",
                    link.target_system, link.target_component,
                    commands._enum_name("MAV_TYPE", hb.type),
                    commands._enum_name("MAV_AUTOPILOT", hb.autopilot))
        logger.info("Listening %.0f s for every heartbeat source on this port...", args.listen)
        time.sleep(args.listen)

    is_autopilot = (hb.type != mavlink.MAV_TYPE_GCS
                    and hb.autopilot != mavlink.MAV_AUTOPILOT_INVALID
                    and link.target_system == args.vehicle_sysid)
    logger.info("Heartbeats rejected during connect: %d", len(ignored))
    if not is_autopilot:
        return result(False, "locked onto sysid=%d, expected autopilot sysid=%d"
                      % (link.target_system, args.vehicle_sysid))
    if not (ignored or monitor.gcs_heartbeat_seen()):
        return result(None, "locked onto the autopilot, but no GCS heartbeat was seen on this "
                      "port, so the Mission Planner case was not exercised. Is MP connected "
                      "to a different SITL port?")
    return result(True, "locked onto autopilot sysid=%d while GCS heartbeats were present"
                  % link.target_system)


def wait_for_gcs_silence(monitor: Monitor, timeout_s: float, quiet_s: float = 3.0) -> List[str]:
    """Waits until no other GCS heartbeat for ``quiet_s``; returns the ones still heard on timeout."""
    deadline = time.monotonic() + timeout_s
    next_report = 0.0
    while True:
        still = monitor.gcs_sources_since(monitor.now() - quiet_s)
        if not still:
            logger.info("No other GCS heard for %.0f s", quiet_s)
            return []
        if time.monotonic() >= deadline:
            return still
        if time.monotonic() >= next_report:
            logger.info("Still hearing: %s", ", ".join(still))
            next_report = time.monotonic() + 5.0
        time.sleep(0.5)


def check_failsafe(args: argparse.Namespace) -> int:
    with open_link(args) as link, Monitor(link) as monitor:
        require_sitl(link, args)
        my_gcs = None
        gcs_param = params.read_param(link, "MAV_GCS_SYSID", "SYSID_MYGCS")  # newer name first
        if gcs_param is not None:
            my_gcs = (gcs_param.name, int(gcs_param.value))
        fs_enable_param = params.read_param(link, "FS_GCS_ENABLE")
        fs_timeout_param = params.read_param(link, "FS_GCS_TIMEOUT")
        fs_enable = fs_enable_param.value if fs_enable_param else None
        fs_timeout = fs_timeout_param.value if fs_timeout_param else None
        logger.info("Autopilot GCS id param: %s | FS_GCS_ENABLE=%s | FS_GCS_TIMEOUT=%s s | "
                    "our source_system=%d", "%s=%d" % my_gcs if my_gcs else "NOT FOUND",
                    fs_enable, fs_timeout, args.source_system)
        if not fs_enable:
            if not args.enable_fs:
                return result(None, "FS_GCS_ENABLE is 0, so the failsafe can't trigger. "
                              "Re-run with --enable-fs.")
            params.set_param(link, "FS_GCS_ENABLE", 1)
        wait_s = (fs_timeout or 5.0) + 10.0
        expect_counted = my_gcs is not None and my_gcs[1] == args.source_system

        take_off(link, args.alt)

        logger.info("Other GCSs heartbeating right now: %s",
                    ", ".join(monitor.gcs_sources_since(monitor.now() - 3.0)) or "none")
        prompt("Vehicle is hovering in GUIDED. Stop EVERY other GCS now: press Disconnect in "
               "Mission Planner, and if MAVProxy is in the chain type `set heartbeat 0` at the "
               "GUIDED> prompt in the terminal where you ran sim_vehicle.py (not the 'ArduCopter' "
               "xterm, not here), or start SITL with -m \"--heartbeat-rate 0\". Then come back")
        still = wait_for_gcs_silence(monitor, timeout_s=30.0)
        if still:
            logger.info("Vehicle left hovering in GUIDED; land it from Mission Planner.")
            return result(None, "still hearing %s 30 s after Enter; phase 1 not started. In "
                          "MAVProxy (sim_vehicle.py terminal, check with `set heartbeat`): `set "
                          "heartbeat 0`, or restart SITL with -m \"--heartbeat-rate 0\"; in "
                          "Mission Planner: Disconnect. Then re-run." % ", ".join(still))
        logger.info("Phase 1: other GCSs gone, our heartbeats still running. Watching %.0f s...",
                    wait_s)
        t1 = monitor.now()
        time.sleep(wait_s)
        phase1 = monitor.failsafe_seen_since(t1, "GUIDED")
        others = monitor.gcs_sources_since(t1 + 3.0)  # grace for the disconnect to land
        logger.info("Phase 1 result: %s", phase1 or "no failsafe")
        if others and phase1 is None:
            logger.info("Vehicle left hovering in GUIDED; land it from Mission Planner.")
            return result(None, "other GCS heartbeats were still arriving during phase 1 (%s), "
                          "so they, not ours, may have kept the failsafe away. Stop them and "
                          "re-run (or start SITL with --no-mavproxy)." % ", ".join(others))

        phase2 = None
        if phase1 is None:
            logger.info("Phase 2: stopping OUR heartbeats (simulates the Jetson dying). "
                        "Watching %.0f s...", wait_s)
            link._heartbeat_thread.stop()  # diagnostic only: Link has no public API for this
            t2 = monitor.now()
            time.sleep(wait_s)
            phase2 = monitor.failsafe_seen_since(t2, "GUIDED")
            logger.info("Phase 2 result: %s", phase2 or "no failsafe")
        logger.info("Vehicle is left in its failsafe mode; reconnect Mission Planner to land/disarm.")

    if phase1 is None and phase2 is not None:
        return result(expect_counted or my_gcs is None,
                      "our heartbeats (sysid %d) kept the GCS failsafe away; it fired once they "
                      "stopped (%s)" % (args.source_system, phase2))
    if phase1 is not None:
        return result(not expect_counted,
                      "failsafe fired while our heartbeats (sysid %d) were running: they are NOT "
                      "counted (%s)" % (args.source_system, phase1))
    return result(None, "no failsafe in either phase; check FS_GCS_ENABLE and that the vehicle "
                  "was armed and flying")


def check_acks(args: argparse.Namespace) -> int:
    with open_link(args) as link, Monitor(link) as monitor:
        require_sitl(link, args)

        logger.info("--- DO_SET_MODE GUIDED (valid)")
        commands.set_mode(link, "GUIDED")

        logger.info("--- DO_SET_MODE with an invalid mode number (99)")
        try:
            commands.command_long(link, mavlink.MAV_CMD_DO_SET_MODE,
                                  [mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 99])
            logger.warning("invalid mode was ACCEPTED")
        except CommandError as exc:
            logger.info("invalid mode: %s", exc)

        logger.info("--- NAV_TAKEOFF while DISARMED (bypasses takeoff() preconditions)")
        try:
            commands.command_long(link, mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, args.alt])
            logger.warning("takeoff while disarmed was ACCEPTED")
        except CommandError as exc:
            logger.info("takeoff while disarmed: %s", exc)

        logger.info("--- ARM, then NAV_TAKEOFF to %.0f m", args.alt)
        take_off(link, args.alt)

        logger.info("--- LAND")
        land_and_wait(link)

    logger.info("ACK summary (t, command, result, latency):")
    for t, cmd, res, latency_ms in monitor.acks:
        logger.info("  %6.1fs  %-32s %-32s %s", t, commands._enum_name("MAV_CMD", cmd),
                    commands._enum_name("MAV_RESULT", res),
                    "-" if latency_ms is None else "%.0f ms" % latency_ms)
    in_progress = [a for a in monitor.acks if a[2] == mavlink.MAV_RESULT_IN_PROGRESS]
    logger.info("IN_PROGRESS ACKs seen: %d", len(in_progress))
    return result(True, "ACK behaviour recorded above")


def check_reconnect(args: argparse.Namespace) -> int:
    with open_link(args) as link:
        require_sitl(link, args)
        logger.info("Watching for %.0f s. Kill the link (Ctrl-C the pty bridge), wait ~10 s, "
                    "restart it.", args.duration)
        downs, ups = [], []
        max_age_down = 0.0
        max_cpu_down = 0.0
        was_up = True
        last_cpu, last_wall = time.process_time(), time.monotonic()
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            time.sleep(1.0)
            cpu, wall = time.process_time(), time.monotonic()
            cpu_pct = 100.0 * (cpu - last_cpu) / (wall - last_wall)
            last_cpu, last_wall = cpu, wall
            up = link.is_up
            hb_age = link.age("HEARTBEAT")
            pos_age = link.age("GLOBAL_POSITION_INT")
            logger.info("is_up=%-5s age(HEARTBEAT)=%6.1fs age(GLOBAL_POSITION_INT)=%6.1fs cpu=%4.1f%%",
                        up, hb_age, pos_age, cpu_pct)
            if was_up and not up:
                downs.append(wall)
            if not was_up and up:
                ups.append(wall)
            if not up:
                max_age_down = max(max_age_down, hb_age)
                max_cpu_down = max(max_cpu_down, cpu_pct)
            was_up = up
        recovered_fresh = link.is_up and link.age("HEARTBEAT") < 2.0
        position_resumed = link.age("GLOBAL_POSITION_INT") < 2.0

    if not downs:
        return result(None, "the link never went down; did you kill the bridge?")
    if not position_resumed:
        # Not a reconnect failure: Link does not re-request streams after reconnecting.
        logger.warning("GLOBAL_POSITION_INT did not resume after reconnect: the stream request "
                       "was lost and must be re-sent after a reconnect")
    outage_s = (ups[0] - downs[0]) if ups else None
    ok = bool(ups) and recovered_fresh and max_cpu_down < 20.0 and max_age_down >= 3.0
    return result(ok, "down %d time(s); first outage %s; max age(HEARTBEAT) while down %.1f s; "
                  "max CPU while down %.1f%%; fresh heartbeat after recovery: %s; position "
                  "stream resumed: %s"
                  % (len(downs), "%.1f s" % outage_s if outage_s else "never recovered",
                     max_age_down, max_cpu_down, recovered_fresh, position_resumed))


def fetch_all_params(link: Link, idle_timeout_s: float = 5.0,
                     max_s: float = 180.0) -> Tuple[Dict[str, float], Optional[int]]:
    """Downloads the full parameter list. Returns (name -> value, count the autopilot reported)."""
    values = {}  # type: Dict[str, float]
    expected = None  # type: Optional[int]
    with link.subscribe("PARAM_VALUE", maxsize=5000) as sub:
        link.send("param_request_list_send", link.target_system, link.target_component)
        deadline = time.monotonic() + max_s
        last_progress = time.monotonic()
        while time.monotonic() < deadline:
            if expected is not None and len(values) >= expected:
                break
            if time.monotonic() - last_progress > idle_timeout_s:
                break
            try:
                msg = sub.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg.get_srcSystem() != link.target_system:
                continue
            expected = msg.param_count
            if msg.param_id not in values:
                last_progress = time.monotonic()
                if len(values) % 200 == 0:
                    logger.info("Downloading parameters... %d/%d", len(values), expected)
            values[msg.param_id] = msg.param_value
    logger.info("Got %d of %s parameters", len(values), expected)
    return values, expected


# Waypoint-navigation climb rate: WPNAV_SPEED_UP (cm/s) on older firmware; newer
# firmware renamed many params, so also accept any WP*_SPEED_UP / WP*_SPD_UP.
_CLIMB_PARAM = re.compile(r"^WP\w*_(SPEED|SPD)_UP$")
_INTERESTING = re.compile(r"(SPEED|SPD)_(UP|DN)$|TKOFF|^(SYSID_MYGCS|MAV_GCS_SYSID)$")


def check_params(args: argparse.Namespace) -> int:
    with open_link(args) as link:
        values, expected = fetch_all_params(link)
    if not values:
        return result(False, "no PARAM_VALUE received")
    pattern = re.compile(args.grep, re.IGNORECASE) if args.grep else _INTERESTING
    for name in sorted(n for n in values if pattern.search(n)):
        logger.info("  %-20s = %g", name, values[name])

    candidates = sorted(n for n in values if _CLIMB_PARAM.match(n))
    configured = climb_rate_m_s()
    if len(candidates) != 1:
        return result(None, "expected one waypoint climb-rate param, found %s; pick it from the "
                      "list above" % (candidates or "none"))
    name = candidates[0]
    raw = values[name]
    # Old params are cm/s (default 250); renamed SI ones are m/s (default 2.5).
    actual = raw / 100.0 if raw > 20 else raw
    logger.info("%s = %g -> %.2f m/s (%s) | config climb_rate_m_s = %.2f m/s",
                name, raw, actual, "read as cm/s" if raw > 20 else "read as m/s", configured)
    if expected is not None and len(values) < expected:
        logger.warning("Parameter download incomplete; re-run if a name you need is missing")
    if abs(actual - configured) < 0.01:
        return result(True, "config matches the vehicle (%s)" % name)
    return result(False, 'set "climb_rate_m_s": %.2f in config/mission_params.py (from %s)'
                  % (actual, name))


# --- CLI ---------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--device", default="tcp:127.0.0.1:5762",
                        help="pymavlink device (default: SITL SERIAL1, tcp:127.0.0.1:5762)")
    common.add_argument("--baud", type=int, default=921600)
    common.add_argument("--source-system", type=int, default=255,
                        help="MAVLink sysid we send as (default 255)")
    common.add_argument("--expected-sysid", type=int, default=None)
    common.add_argument("--timeout", type=float, default=30.0, help="connect timeout, s")
    common.add_argument("--i-know-this-is-sitl", action="store_true",
                        help="allow flying checks on a serial-looking device (pty bridge)")
    common.add_argument("-v", "--verbose", action="store_true")

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="check", required=True)

    p = sub.add_parser("lock", parents=[common], help=check_lock.__doc__)
    p.add_argument("--listen", type=float, default=10.0, help="seconds to list heartbeat sources")
    p.add_argument("--vehicle-sysid", type=int, default=1)
    p.set_defaults(func=check_lock)

    p = sub.add_parser("failsafe", parents=[common])
    p.add_argument("--alt", type=float, default=10.0, help="hover altitude, m")
    p.add_argument("--enable-fs", action="store_true", help="set FS_GCS_ENABLE=1 if it is 0")
    p.set_defaults(func=check_failsafe)

    p = sub.add_parser("acks", parents=[common])
    p.add_argument("--alt", type=float, default=5.0, help="takeoff altitude, m")
    p.set_defaults(func=check_acks)

    p = sub.add_parser("reconnect", parents=[common])
    p.add_argument("--duration", type=float, default=60.0, help="seconds to watch")
    p.set_defaults(func=check_reconnect)

    p = sub.add_parser("params", parents=[common])
    p.add_argument("--grep", default=None,
                   help="regex of parameter names to print (default: climb/takeoff/GCS-id params)")
    p.set_defaults(func=check_params)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    try:
        return args.func(args)
    except LinkTimeoutError as exc:
        return result(False, str(exc))
    except CommandError as exc:
        return result(False, "%s: %s" % (type(exc).__name__, exc))
    except OSError as exc:  # includes serial.SerialException
        return result(False, "could not open %s: %s" % (args.device, exc))
    except KeyboardInterrupt:
        logger.info("Interrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
