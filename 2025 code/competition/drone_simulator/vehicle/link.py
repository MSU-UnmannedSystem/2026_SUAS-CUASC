"""Single owner of the MAVLink connection to the autopilot.

``Link`` is the only object that should touch the pymavlink connection. All
writes go through :meth:`Link.send`, which serialises them with a lock so the
GCS heartbeat thread and mission code can share the port safely. A single
reader thread is the only code that reads from the connection; it keeps a
latest-value cache (:meth:`Link.latest`, :meth:`Link.age`) and feeds
per-caller queues (:meth:`Link.subscribe`).

ArduPilot only sends HEARTBEAT on a port until someone asks for streams, so
:meth:`Link.connect` requests ``LinkConfig.stream_rates_hz`` with
SET_MESSAGE_INTERVAL, and a keeper thread re-requests any stream that goes
stale while the autopilot is still heartbeating (e.g. after a Cube reboot).
"""

import contextlib
import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, FrozenSet, Iterator, List, Optional, Tuple

from pymavlink import mavutil

try:
    from serial import SerialException
except ImportError:  # pyserial is optional (UDP/TCP-only installs)
    SerialException = OSError

logger = logging.getLogger(__name__)

# pyserial's SerialException subclasses OSError in 3.x; listed for older versions.
_LINK_ERRORS = (OSError, SerialException)

# pymavlink device prefixes that are network links rather than a serial port.
_NETWORK_DEVICE_PREFIXES = ("udp", "tcp", "mcast")

# Telemetry requested on connect, in Hz.
DEFAULT_STREAM_RATES_HZ = {
    "GLOBAL_POSITION_INT": 10.0,
    "ATTITUDE": 10.0,
    "VFR_HUD": 4.0,
    "SYS_STATUS": 2.0,
    "GPS_RAW_INT": 2.0,
    "MISSION_CURRENT": 2.0,
    "SERVO_OUTPUT_RAW": 2.0,
}


class LinkTimeoutError(Exception):
    """Raised when no acceptable autopilot heartbeat arrives before the connect timeout."""


class LinkStreamError(Exception):
    """Raised when a required telemetry stream never arrives after it was requested."""


def _raise_on_tcp_close(master: Any) -> None:
    """Makes a closed TCP peer raise instead of being reported as "no data".

    With autoreconnect=False, pymavlink's mavtcp.recv() prints "EOF on TCP
    socket" and returns b"" forever when the peer closes, so the reader spun
    at 100% CPU and never reconnected. Raising ConnectionResetError (an
    OSError) sends it through the normal reconnect path instead.
    """
    def closed(*_args: Any, **_kwargs: Any) -> None:
        raise ConnectionResetError("TCP connection to %s closed by peer"
                                   % (getattr(master, "address", "?"),))

    master.handle_eof = closed
    master.handle_disconnect = closed


def _message_id(msg_type: str) -> int:
    msg_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_" + msg_type, None)
    if msg_id is None:
        raise ValueError("Unknown MAVLink message type %r" % msg_type)
    return msg_id


@dataclass
class LinkConfig:
    """Connection settings for :class:`Link`.

    Attributes:
        device: pymavlink device string, e.g. ``"/dev/ttyTHS1"``, ``"/dev/ttyACM0"``
            or ``"udpin:0.0.0.0:14550"``.
        baud: Serial baud rate in bits/s. Ignored for UDP/TCP devices.
        source_system: MAVLink system ID we transmit as.
        source_component: MAVLink component ID we transmit as
            (191 = MAV_COMP_ID_ONBOARD_COMPUTER).
        expected_sysid: If set, only accept autopilot heartbeats from this system ID.
        connect_timeout_s: Seconds to wait for an autopilot heartbeat in :meth:`Link.connect`.
        heartbeat_hz: Rate of our outgoing GCS heartbeats, in Hz.
        stream_rates_hz: Telemetry to request on connect, message name -> rate in Hz.
        required_streams: Streams that must arrive within 3 s of being requested, or
            :meth:`Link.connect` raises :class:`LinkStreamError`. Each must be in
            ``stream_rates_hz``.
        silence_timeout_s: Seconds without any message from the autopilot before the
            link is treated as down and reopened. A pulled wire on a 3-wire UART is
            silent rather than an error, so this is the only way to notice it.
    """

    device: str
    baud: int = 921600

    # ArduPilot's GCS failsafe only counts heartbeats from the system ID in
    # MAV_GCS_SYSID (SYSID_MYGCS on older firmware; default 255), so
    # source_system must match it or the failsafe will not see us. Verified in
    # SITL 2026-10-28 (`tools/sitl_link_check.py failsafe`): with MAV_GCS_SYSID=255 our
    # heartbeats alone held off the failsafe; it fired 5 s after they stopped.
    # TODO: check which name/value the Cube's firmware uses (`tools/sitl_link_check.py params`).
    source_system: int = 255
    source_component: int = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    expected_sysid: Optional[int] = None
    connect_timeout_s: float = 30.0
    heartbeat_hz: float = 2.0
    stream_rates_hz: Dict[str, float] = field(
        default_factory=lambda: dict(DEFAULT_STREAM_RATES_HZ))
    required_streams: Tuple[str, ...] = ("GLOBAL_POSITION_INT",)
    silence_timeout_s: float = 3.0

    def __post_init__(self) -> None:
        if self.silence_timeout_s <= 0:
            raise ValueError("silence_timeout_s must be > 0")
        if self.connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be > 0")
        if self.heartbeat_hz <= 0:
            raise ValueError("heartbeat_hz must be > 0")
        for name, rate_hz in self.stream_rates_hz.items():
            _message_id(name)
            if rate_hz <= 0:
                raise ValueError("stream rate for %s must be > 0 Hz" % name)
        missing = [n for n in self.required_streams if n not in self.stream_rates_hz]
        if missing:
            raise ValueError("required_streams not in stream_rates_hz: %s" % ", ".join(missing))


class _RateLimiter:
    """Allows one event per interval and counts the ones it suppressed."""

    def __init__(self, interval_s: float) -> None:
        self._interval_s = interval_s
        self._last = float("-inf")
        self._suppressed = 0
        self._lock = threading.Lock()

    def allow(self) -> Optional[int]:
        """Returns the number suppressed since the last allowed event, or None if suppressed."""
        with self._lock:
            now = time.monotonic()
            if now - self._last < self._interval_s:
                self._suppressed += 1
                return None
            self._last = now
            suppressed, self._suppressed = self._suppressed, 0
            return suppressed


class _HeartbeatThread(threading.Thread):
    """Sends GCS heartbeats through ``Link.send`` until stopped.

    Must stay alive for the whole flight: if it stops, the autopilot's GCS
    failsafe will trigger. Sends are skipped while the link is down.
    """

    def __init__(self, link: "Link", hz: float) -> None:
        super().__init__(name="link-heartbeat", daemon=True)
        self._link = link
        self._period = 1.0 / hz
        self._stop_event = threading.Event()
        self._error_log = _RateLimiter(1.0)

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self._link.is_up:
                try:
                    self._link.send(
                        "heartbeat_send",
                        type=mavutil.mavlink.MAV_TYPE_GCS,
                        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        base_mode=0,
                        custom_mode=0,
                        system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                except Exception:
                    suppressed = self._error_log.allow()
                    if suppressed is not None:
                        logger.exception("GCS heartbeat send failed; will retry "
                                         "(%d similar suppressed)", suppressed)
            self._stop_event.wait(self._period)

    def stop(self) -> None:
        self._stop_event.set()


class Subscription:
    """Queue of incoming messages of selected types, filled by the reader thread.

    Created by :meth:`Link.subscribe`; do not construct directly.

    Attributes:
        msg_types: The MAVLink message names this subscription receives.
        dropped: Number of messages discarded because the queue was full.
    """

    def __init__(self, msg_types: FrozenSet[str], maxsize: int) -> None:
        self.msg_types = msg_types
        self.dropped = 0
        self._queue = queue.Queue(maxsize)  # type: queue.Queue
        self._overflow_log = _RateLimiter(1.0)

    def get(self, timeout: float) -> Any:
        """Returns the next queued message, oldest first.

        Args:
            timeout: Maximum time to wait, in seconds.

        Returns:
            The next pymavlink message.

        Raises:
            queue.Empty: Nothing arrived within ``timeout`` seconds.
        """
        return self._queue.get(timeout=timeout)

    def _put(self, msg: Any) -> None:
        # Only the reader thread puts, so dropping the oldest and retrying terminates.
        while True:
            try:
                self._queue.put_nowait(msg)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    continue
                self.dropped += 1
                suppressed = self._overflow_log.allow()
                if suppressed is not None:
                    logger.warning("Subscription %s full; dropped oldest (%d dropped in total)",
                                   sorted(self.msg_types), self.dropped)


class Link:
    """Owns the MAVLink connection to one autopilot.

    Create with :meth:`Link.connect`. Use as a context manager, or call
    :meth:`close` when done.
    """

    _CLOSE_TIMEOUT_S = 0.9
    _READ_TIMEOUT_S = 0.2
    _RECONNECT_BACKOFF_S = (0.5, 1.0, 2.0, 4.0, 5.0)
    _STREAM_ACK_TIMEOUT_S = 1.5
    _REQUIRED_STREAM_WAIT_S = 3.0
    _STREAM_CHECK_PERIOD_S = 1.0
    _STREAM_STALE_MIN_S = 3.0        # a stream is stale after max(3 / rate, this)
    _STREAM_REREQUEST_MIN_S = 5.0    # per stream
    _HEARTBEAT_FRESH_S = 3.0

    def __init__(
        self,
        master: Any,
        config: LinkConfig,
        connection_factory: Callable[..., Any] = mavutil.mavlink_connection,
    ) -> None:
        """Wraps an already-open connection. Prefer :meth:`Link.connect`.

        Args:
            master: An open pymavlink connection (``mavfile``).
            config: The settings the connection was opened with.
            connection_factory: Used to reopen the connection after a link error.
        """
        self._master = master
        self._config = config
        self._connection_factory = connection_factory
        self._send_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._closed = False
        self._is_up = False
        self._last_rx = time.monotonic()  # last message from target_system
        self._target_system = 0
        self._target_component = 0
        self._stop_event = threading.Event()
        self._heartbeat_thread = None  # type: Optional[_HeartbeatThread]
        self._reader_thread = None  # type: Optional[threading.Thread]
        self._stream_thread = None  # type: Optional[threading.Thread]
        self._stream_requested_at = {}  # type: Dict[str, float]
        self._stream_log = {}  # type: Dict[str, _RateLimiter]
        self._cache = {}  # type: Dict[str, Tuple[Any, float]]
        self._cache_lock = threading.Lock()
        self._subscriptions = []  # type: List[Subscription]
        self._subs_lock = threading.Lock()
        self._handler_error_log = _RateLimiter(1.0)
        self._loop_error_log = _RateLimiter(1.0)

    @classmethod
    def connect(
        cls,
        config: LinkConfig,
        connection_factory: Callable[..., Any] = mavutil.mavlink_connection,
    ) -> "Link":
        """Opens the connection and locks onto the autopilot.

        Waits for a HEARTBEAT that comes from an autopilot (not a GCS such as
        Mission Planner, whose heartbeats ArduPilot forwards between ports),
        targets its system/component ID, then starts the reader and GCS
        heartbeat threads, requests ``config.stream_rates_hz`` and waits for
        ``config.required_streams``, then starts the stream keeper.

        Args:
            config: Connection settings.
            connection_factory: Callable with the signature of
                ``mavutil.mavlink_connection``. Also used to reopen the
                connection after a link error. Injectable for tests.

        Returns:
            A connected ``Link`` with its reader, heartbeat and stream threads running.

        Raises:
            LinkTimeoutError: No acceptable heartbeat within ``config.connect_timeout_s``
                seconds. The connection is closed before raising.
            LinkStreamError: A required stream did not arrive within 3 s of being
                requested. The connection is closed before raising.
        """
        master = cls._open(config, connection_factory)
        link = cls(master, config, connection_factory)
        try:
            hb = link._wait_for_autopilot()
            link._set_target(hb.get_srcSystem(), hb.get_srcComponent())
            link._cache_message(hb)
            link._last_rx = time.monotonic()
            link._is_up = True
            link._start_reader()
            link._start_heartbeat()
            link._request_streams()
            link._wait_for_required_streams()
            link._start_stream_keeper()
        except BaseException:
            link.close()
            raise
        return link

    @staticmethod
    def _open(config: LinkConfig, connection_factory: Callable[..., Any]) -> Any:
        master = connection_factory(
            config.device,
            baud=config.baud,
            source_system=config.source_system,
            source_component=config.source_component,
        )
        if isinstance(master, mavutil.mavtcp):
            _raise_on_tcp_close(master)
        return master

    def _wait_for_autopilot(self) -> Any:
        cfg = self._config
        logger.info("Waiting for autopilot heartbeat on %s (timeout %.1f s)",
                    cfg.device, cfg.connect_timeout_s)
        deadline = time.monotonic() + cfg.connect_timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            msg = self._master.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
            if msg is None:
                continue
            sysid, compid = msg.get_srcSystem(), msg.get_srcComponent()
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                reason = "not an autopilot"
            elif msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                reason = "GCS"
            elif cfg.expected_sysid is not None and sysid != cfg.expected_sysid:
                reason = "sysid != expected %d" % cfg.expected_sysid
            else:
                logger.info("Accepted autopilot heartbeat sysid=%d compid=%d type=%d",
                            sysid, compid, msg.type)
                return msg
            logger.info("Ignoring heartbeat sysid=%d compid=%d type=%d (%s)",
                        sysid, compid, msg.type, reason)

        raise LinkTimeoutError(
            "No autopilot heartbeat on %s within %.1f s. Check the cable/port, "
            "that baud (%d) matches the autopilot's SERIALx_BAUD, and that "
            "SERIALx_PROTOCOL is MAVLink2 (2) on that port%s."
            % (cfg.device, cfg.connect_timeout_s, cfg.baud,
               "" if cfg.expected_sysid is None
               else "; also check SYSID_THISMAV == %d" % cfg.expected_sysid)
        )

    def _set_target(self, system: int, component: int) -> None:
        self._target_system = system
        self._target_component = component
        self._master.target_system = system
        self._master.target_component = component

    def _start_heartbeat(self) -> None:
        self._heartbeat_thread = _HeartbeatThread(self, self._config.heartbeat_hz)
        self._heartbeat_thread.start()
        logger.info("GCS heartbeat thread started @ %.1f Hz", self._config.heartbeat_hz)

    def _start_reader(self) -> None:
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="link-reader", daemon=True)
        self._reader_thread.start()

    # --- streams ---------------------------------------------------------------

    def _send_message_interval(self, msg_type: str, rate_hz: float) -> None:
        interval_us = 1e6 / rate_hz
        self.send("command_long_send", self._target_system, self._target_component,
                  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                  float(_message_id(msg_type)), interval_us, 0, 0, 0, 0, 0)
        self._stream_requested_at[msg_type] = time.monotonic()

    def _request_stream_acked(self, msg_type: str, rate_hz: float) -> Optional[int]:
        """Requests one stream and waits for its COMMAND_ACK. Never call from the reader."""
        with self.subscribe("COMMAND_ACK") as sub:
            self._send_message_interval(msg_type, rate_hz)
            deadline = time.monotonic() + self._STREAM_ACK_TIMEOUT_S
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    ack = sub.get(timeout=remaining)
                except queue.Empty:
                    break
                if (ack.get_srcSystem() == self._target_system
                        and ack.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL):
                    return ack.result
        return None

    def _request_streams(self) -> None:
        for msg_type, rate_hz in self._config.stream_rates_hz.items():
            result = self._request_stream_acked(msg_type, rate_hz)
            if result is None:
                logger.warning("No ACK for SET_MESSAGE_INTERVAL %s @ %g Hz", msg_type, rate_hz)
            elif result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.warning("SET_MESSAGE_INTERVAL %s @ %g Hz rejected: result %d",
                               msg_type, rate_hz, result)
            else:
                logger.debug("Streaming %s @ %g Hz", msg_type, rate_hz)

    def _wait_for_required_streams(self) -> None:
        required = self._config.required_streams
        deadline = time.monotonic() + self._REQUIRED_STREAM_WAIT_S
        while True:
            missing = [n for n in required if self.latest(n) is None]
            if not missing:
                if required:
                    logger.info("Required streams arriving: %s", ", ".join(required))
                return
            if time.monotonic() >= deadline:
                raise LinkStreamError(
                    "Required stream(s) not received within %.1f s of SET_MESSAGE_INTERVAL: %s"
                    % (self._REQUIRED_STREAM_WAIT_S, ", ".join(missing)))
            time.sleep(0.05)

    def _start_stream_keeper(self) -> None:
        if not self._config.stream_rates_hz:
            return
        self._stream_thread = threading.Thread(
            target=self._stream_keeper_loop, name="link-streams", daemon=True)
        self._stream_thread.start()

    def _stream_keeper_loop(self) -> None:
        # Fire-and-forget only: ACKs are delivered by the reader thread, so this
        # loop (and the reader) must never block waiting for one.
        while not self._stop_event.wait(self._STREAM_CHECK_PERIOD_S):
            try:
                self._check_streams()
            except Exception:
                suppressed = self._loop_error_log.allow()
                if suppressed is not None:
                    logger.exception("Stream keeper error (%d similar suppressed)", suppressed)

    def _check_streams(self) -> None:
        if not self._is_up or self.age("HEARTBEAT") >= self._HEARTBEAT_FRESH_S:
            return  # link down or autopilot silent: re-requesting can't help
        now = time.monotonic()
        for msg_type, rate_hz in self._config.stream_rates_hz.items():
            stale_after_s = max(3.0 / rate_hz, self._STREAM_STALE_MIN_S)
            age_s = self.age(msg_type)
            requested_ago_s = now - self._stream_requested_at.get(msg_type, float("-inf"))
            if age_s <= stale_after_s or requested_ago_s < max(stale_after_s,
                                                              self._STREAM_REREQUEST_MIN_S):
                continue
            log = self._stream_log.setdefault(msg_type, _RateLimiter(60.0))
            suppressed = log.allow()
            if suppressed is not None:
                logger.warning("%s stale (age %.1f s) while autopilot is heartbeating; "
                               "re-requesting @ %g Hz (%d similar suppressed)",
                               msg_type, age_s, rate_hz, suppressed)
            self._send_message_interval(msg_type, rate_hz)

    # --- reader thread -------------------------------------------------------

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._read_one()
            except Exception:
                if self._stop_event.is_set():
                    break
                suppressed = self._loop_error_log.allow()
                if suppressed is not None:
                    logger.exception("Unexpected error in reader loop (%d similar suppressed)",
                                     suppressed)
                self._stop_event.wait(0.1)
        logger.debug("Reader thread exiting")

    def _read_one(self) -> None:
        start = time.monotonic()
        try:
            msg = self._master.recv_match(blocking=True, timeout=self._READ_TIMEOUT_S)
        except _LINK_ERRORS as exc:
            if not self._stop_event.is_set():
                self._reconnect("%s: %s" % (type(exc).__name__, exc))
            return
        if msg is None:
            # Guard against a connection that returns immediately instead of blocking.
            if time.monotonic() - start < 0.01:
                self._stop_event.wait(0.01)
        elif msg.get_type() != "BAD_DATA":
            try:
                self._dispatch(msg)
            except Exception:
                suppressed = self._handler_error_log.allow()
                if suppressed is not None:
                    logger.exception("Error handling received message (%d similar suppressed)",
                                     suppressed)
        silent_s = time.monotonic() - self._last_rx
        if (self._is_up and silent_s > self._config.silence_timeout_s
                and not self._stop_event.is_set()):
            self._reconnect("silence: nothing from sysid %d for %.1f s"
                            % (self._target_system, silent_s))

    def _dispatch(self, msg: Any) -> None:
        if msg.get_srcSystem() == self._target_system:
            self._last_rx = time.monotonic()
            self._cache_message(msg)
        msg_type = msg.get_type()
        # Put while holding the lock so nothing is queued after unsubscribe returns.
        with self._subs_lock:
            for sub in self._subscriptions:
                if msg_type in sub.msg_types:
                    sub._put(msg)

    def _cache_message(self, msg: Any) -> None:
        with self._cache_lock:
            self._cache[msg.get_type()] = (msg, time.monotonic())

    def _reconnect(self, reason: str) -> None:
        """Marks the link down and reopens it with backoff until it succeeds or close()."""
        self._is_up = False
        logger.error("Link to %s down (%s); reconnecting", self._config.device, reason)
        with self._send_lock:
            try:
                self._master.close()
            except Exception:
                logger.debug("Error closing dead connection", exc_info=True)

        attempt = 0
        while True:
            delay = self._RECONNECT_BACKOFF_S[min(attempt, len(self._RECONNECT_BACKOFF_S) - 1)]
            if self._stop_event.wait(delay):
                return
            attempt += 1
            try:
                master = self._open(self._config, self._connection_factory)
            except Exception as open_exc:
                logger.debug("Reconnect attempt %d failed: %s", attempt, open_exc)
                continue
            if self._stop_event.is_set():
                master.close()
                return
            with self._send_lock:
                self._master = master
                self._set_target(self._target_system, self._target_component)
            self._last_rx = time.monotonic()  # fresh grace period for the new connection
            self._is_up = True
            logger.info("Link to %s recovered after %d attempt(s)", self._config.device, attempt)
            return

    # --- public API ----------------------------------------------------------

    @property
    def target_system(self) -> int:
        """int: System ID of the autopilot this link is bound to."""
        return self._target_system

    @property
    def target_component(self) -> int:
        """int: Component ID of the autopilot this link is bound to."""
        return self._target_component

    @property
    def is_up(self) -> bool:
        """bool: False after an I/O error OR silence_timeout_s of silence, until the reopen succeeds.

        True means the port is open and the autopilot has sent something within
        ``silence_timeout_s``; use ``age(msg_type)`` for how fresh a given
        message is.
        """
        return self._is_up

    @property
    def is_serial(self) -> bool:
        """bool: False only for network devices (udp*/tcp*/mcast), i.e. SITL-style links.

        Anything that is not clearly a network device counts as serial, so
        safety checks keyed on this err towards "real hardware".
        """
        return not self._config.device.lower().startswith(_NETWORK_DEVICE_PREFIXES)

    def send(self, method_name: str, *args: Any, **kwargs: Any) -> Any:
        """Sends one MAVLink message. The only way to write to the vehicle.

        Calls ``master.mav.<method_name>(*args, **kwargs)`` while holding the
        send lock, so writes from different threads never interleave.

        Args:
            method_name: A pymavlink ``*_send`` method name, e.g. ``"command_long_send"``.
            *args: Positional arguments for that method.
            **kwargs: Keyword arguments for that method.

        Returns:
            Whatever the pymavlink method returns.

        Raises:
            ValueError: ``method_name`` does not end in ``_send`` or does not exist.
        """
        if not method_name.endswith("_send"):
            raise ValueError("send() only accepts *_send methods, got %r" % method_name)
        with self._send_lock:
            method = getattr(self._master.mav, method_name, None)
            if not callable(method):
                raise ValueError("Unknown MAVLink send method %r" % method_name)
            return method(*args, **kwargs)

    def latest(self, msg_type: str) -> Optional[Any]:
        """Returns the newest message of a type received from the target autopilot.

        Args:
            msg_type: MAVLink message name, e.g. ``"GLOBAL_POSITION_INT"``.

        Returns:
            The most recent message of that type, or None if none has been seen.
        """
        return self.latest_with_time(msg_type)[0]

    def latest_with_time(self, msg_type: str) -> Tuple[Optional[Any], Optional[float]]:
        """Returns the newest message of a type and when it was received.

        Args:
            msg_type: MAVLink message name, e.g. ``"GLOBAL_POSITION_INT"``.

        Returns:
            ``(msg, rx_time)`` where ``rx_time`` is a ``time.monotonic()`` value
            in seconds, or ``(None, None)`` if none has been seen.
        """
        with self._cache_lock:
            entry = self._cache.get(msg_type)
        if entry is None:
            return None, None
        return entry

    def age(self, msg_type: str) -> float:
        """Returns how long ago a message type was last received from the target autopilot.

        Keeps growing while the link is down.

        Args:
            msg_type: MAVLink message name, e.g. ``"HEARTBEAT"``.

        Returns:
            Seconds since the last message of that type, or ``float("inf")`` if never seen.
        """
        _, rx_time = self.latest_with_time(msg_type)
        if rx_time is None:
            return float("inf")
        return time.monotonic() - rx_time

    @contextlib.contextmanager
    def subscribe(self, *msg_types: str, maxsize: int = 100) -> Iterator[Subscription]:
        """Opens a queue of incoming messages of the given types.

        Every matching message the reader receives, from any system (including
        GCS traffic), is queued in arrival order until the ``with`` block exits.

        Open the subscription BEFORE sending the command you wait on, otherwise
        a fast reply can arrive before the queue exists and be missed::

            with link.subscribe("COMMAND_ACK") as sub:
                link.send("command_long_send", ...)
                ack = sub.get(timeout=3.0)

        Args:
            *msg_types: MAVLink message names to receive.
            maxsize: Maximum queued messages; when full the oldest is dropped
                and ``Subscription.dropped`` is incremented.

        Yields:
            A :class:`Subscription`; ``get(timeout)`` (seconds) returns the next
            message or raises ``queue.Empty``.

        Raises:
            ValueError: No message types given, or ``maxsize`` < 1.
        """
        if not msg_types:
            raise ValueError("subscribe() needs at least one message type")
        if maxsize < 1:
            raise ValueError("maxsize must be >= 1")
        sub = Subscription(frozenset(msg_types), maxsize)
        with self._subs_lock:
            self._subscriptions.append(sub)
        try:
            yield sub
        finally:
            with self._subs_lock:
                self._subscriptions.remove(sub)

    def mode_mapping(self) -> Optional[Dict[str, int]]:
        """Returns the flight-mode name to custom_mode number map for the vehicle.

        Returns:
            e.g. ``{"STABILIZE": 0, "AUTO": 3, ...}``, or None if pymavlink does
            not yet know the vehicle type.
        """
        return self._master.mode_mapping()

    def close(self) -> None:
        """Stops the reader, heartbeat and stream threads and closes the connection.

        Returns within about 1 s. Safe to call more than once.
        """
        with self._close_lock:
            if self._closed:
                return
            self._closed = True

        self._stop_event.set()
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.stop()
        deadline = time.monotonic() + self._CLOSE_TIMEOUT_S
        for thread in (self._reader_thread, self._heartbeat_thread, self._stream_thread):
            if thread is None:
                continue
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
            if thread.is_alive():
                logger.warning("%s thread did not stop within %.1f s",
                               thread.name, self._CLOSE_TIMEOUT_S)
        self._is_up = False
        with self._send_lock:
            try:
                self._master.close()
            except Exception:
                logger.exception("Error closing MAVLink connection")
        logger.info("Link closed")

    def __enter__(self) -> "Link":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()
