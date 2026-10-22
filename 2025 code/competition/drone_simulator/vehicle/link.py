"""Single owner of the MAVLink connection to the autopilot.

``Link`` is the only object that should touch the pymavlink connection. All
writes go through :meth:`Link.send`, which serialises them with a lock so the
GCS heartbeat thread and mission code can share the port safely. Reads
(:meth:`Link.latest`, :meth:`Link.age`, :meth:`Link.subscribe`) are stubbed
here and will be backed by a reader thread + state cache in step 3.
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, ContextManager, Optional

from pymavlink import mavutil

logger = logging.getLogger(__name__)


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
    # SYSID_MYGCS (default 255), so source_system must match that param or the
    # failsafe will not see us. TODO: verify in SITL (and check whether the
    # param is named MAV_GCS_SYSID on the firmware we fly).
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


class _HeartbeatThread(threading.Thread):
    """Sends GCS heartbeats through ``Link.send`` until stopped.

    Must stay alive for the whole flight: if it stops, the autopilot's GCS
    failsafe will trigger.
    """

    def __init__(self, link: "Link", hz: float) -> None:
        super().__init__(name="link-heartbeat", daemon=True)
        self._link = link
        self._period = 1.0 / hz
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
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


class Link:
    """Owns the MAVLink connection to one autopilot.

    Create with :meth:`Link.connect`. Use as a context manager, or call
    :meth:`close` when done.
    """

    _CLOSE_JOIN_TIMEOUT_S = 2.0

    def __init__(self, master: Any, config: LinkConfig) -> None:
        """Wraps an already-open connection. Prefer :meth:`Link.connect`.

        Args:
            master: An open pymavlink connection (``mavfile``).
            config: The settings the connection was opened with.
        """
        self._master = master
        self._config = config
        self._send_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._closed = False
        self._target_system = 0
        self._target_component = 0
        self._heartbeat_thread = None  # type: Optional[_HeartbeatThread]

    @classmethod
    def connect(
        cls,
        config: LinkConfig,
        connection_factory: Callable[..., Any] = mavutil.mavlink_connection,
    ) -> "Link":
        """Opens the connection and locks onto the autopilot.

        Waits for a HEARTBEAT that comes from an autopilot (not a GCS such as
        Mission Planner, whose heartbeats ArduPilot forwards between ports),
        targets its system/component ID, then starts the GCS heartbeat thread.

        Args:
            config: Connection settings.
            connection_factory: Callable with the signature of
                ``mavutil.mavlink_connection``. Injectable for tests.

        Returns:
            A connected ``Link`` with its heartbeat thread running.

        Raises:
            LinkTimeoutError: No acceptable heartbeat within ``config.connect_timeout_s``
                seconds. The connection is closed before raising.
        """
        master = connection_factory(
            config.device,
            baud=config.baud,
            source_system=config.source_system,
            source_component=config.source_component,
        )
        link = cls(master, config)
        try:
            hb = link._wait_for_autopilot()
            link._set_target(hb.get_srcSystem(), hb.get_srcComponent())
            link._start_heartbeat()
        except BaseException:
            link.close()
            raise
        return link

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

    @property
    def target_system(self) -> int:
        """int: System ID of the autopilot this link is bound to."""
        return self._target_system

    @property
    def target_component(self) -> int:
        """int: Component ID of the autopilot this link is bound to."""
        return self._target_component

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
        method = getattr(self._master.mav, method_name, None)
        if not callable(method):
            raise ValueError("Unknown MAVLink send method %r" % method_name)
        with self._send_lock:
            return method(*args, **kwargs)

    def latest(self, msg_type: str) -> Optional[Any]:
        """Returns the newest received message of a type.

        Args:
            msg_type: MAVLink message name, e.g. ``"GLOBAL_POSITION_INT"``.

        Returns:
            The most recent message of that type, or None if none has been seen.
        """
        raise NotImplementedError("step 3")

    def age(self, msg_type: str) -> float:
        """Returns how long ago a message type was last received.

        Args:
            msg_type: MAVLink message name, e.g. ``"HEARTBEAT"``.

        Returns:
            Seconds since the last message of that type, or ``float("inf")`` if never seen.
        """
        raise NotImplementedError("step 3")

    def subscribe(self, *msg_types: str, maxsize: int = 100) -> ContextManager[Any]:
        """Opens a queue of incoming messages of the given types.

        Open the subscription BEFORE sending the command whose reply it waits
        on, otherwise a fast reply can be missed::

            with link.subscribe("COMMAND_ACK") as sub:
                link.send("command_long_send", ...)
                ack = sub.get(timeout=3.0)

        Args:
            *msg_types: MAVLink message names to receive.
            maxsize: Maximum queued messages; oldest are dropped when full.

        Returns:
            A context manager yielding an object with ``get(timeout: float)``
            (timeout in seconds) that returns the next message or None on timeout.
        """
        raise NotImplementedError("step 3")

    def close(self) -> None:
        """Stops the heartbeat thread and closes the connection.

        Safe to call more than once.
        """
        with self._close_lock:
            if self._closed:
                return
            self._closed = True

        thread = self._heartbeat_thread
        if thread is not None:
            thread.stop()
            thread.join(timeout=self._CLOSE_JOIN_TIMEOUT_S)
            if thread.is_alive():
                logger.warning("Heartbeat thread did not stop within %.1f s",
                               self._CLOSE_JOIN_TIMEOUT_S)
        try:
            self._master.close()
        except Exception:
            logger.exception("Error closing MAVLink connection")
        logger.info("Link closed")

    def __enter__(self) -> "Link":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()
