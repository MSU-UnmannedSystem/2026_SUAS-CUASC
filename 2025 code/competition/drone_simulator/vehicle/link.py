"""Single owner of the MAVLink connection to the autopilot.

``Link`` is the only object that should touch the pymavlink connection. All
writes go through :meth:`Link.send`, which serialises them with a lock so the
GCS heartbeat thread and mission code can share the port safely. A single
reader thread is the only code that reads from the connection; it keeps a
latest-value cache (:meth:`Link.latest`, :meth:`Link.age`) and feeds
per-caller queues (:meth:`Link.subscribe`).
"""

import contextlib
import logging
import queue
import threading
import time
from dataclasses import dataclass
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


class LinkTimeoutError(Exception):
    """Raised when no acceptable autopilot heartbeat arrives before the connect timeout."""


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

    def __post_init__(self) -> None:
        if self.connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be > 0")
        if self.heartbeat_hz <= 0:
            raise ValueError("heartbeat_hz must be > 0")


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
                    logger.exception("GCS heartbeat send failed; will retry")
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
        self._target_system = 0
        self._target_component = 0
        self._stop_event = threading.Event()
        self._heartbeat_thread = None  # type: Optional[_HeartbeatThread]
        self._reader_thread = None  # type: Optional[threading.Thread]
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
        heartbeat threads.

        Args:
            config: Connection settings.
            connection_factory: Callable with the signature of
                ``mavutil.mavlink_connection``. Also used to reopen the
                connection after a link error. Injectable for tests.

        Returns:
            A connected ``Link`` with its reader and heartbeat threads running.

        Raises:
            LinkTimeoutError: No acceptable heartbeat within ``config.connect_timeout_s``
                seconds. The connection is closed before raising.
        """
        master = cls._open(config, connection_factory)
        link = cls(master, config, connection_factory)
        try:
            hb = link._wait_for_autopilot()
            link._set_target(hb.get_srcSystem(), hb.get_srcComponent())
            link._cache_message(hb)
            link._is_up = True
            link._start_reader()
            link._start_heartbeat()
        except BaseException:
            link.close()
            raise
        return link

    @staticmethod
    def _open(config: LinkConfig, connection_factory: Callable[..., Any]) -> Any:
        return connection_factory(
            config.device,
            baud=config.baud,
            source_system=config.source_system,
            source_component=config.source_component,
        )

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
                self._reconnect(exc)
            return
        if msg is None:
            # Guard against a connection that returns immediately instead of blocking.
            if time.monotonic() - start < 0.01:
                self._stop_event.wait(0.01)
            return
        if msg.get_type() == "BAD_DATA":
            return
        try:
            self._dispatch(msg)
        except Exception:
            suppressed = self._handler_error_log.allow()
            if suppressed is not None:
                logger.exception("Error handling received message (%d similar suppressed)",
                                 suppressed)

    def _dispatch(self, msg: Any) -> None:
        if msg.get_srcSystem() == self._target_system:
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

    def _reconnect(self, exc: BaseException) -> None:
        """Marks the link down and reopens it with backoff until it succeeds or close()."""
        self._is_up = False
        logger.error("Link to %s down (%s: %s); reconnecting",
                     self._config.device, type(exc).__name__, exc)
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
        """bool: True while the connection is open and readable.

        This means the port is open, not that the autopilot is talking; use
        ``age("HEARTBEAT")`` for that.
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
        """Stops the reader and heartbeat threads and closes the connection.

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
        for thread in (self._reader_thread, self._heartbeat_thread):
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
