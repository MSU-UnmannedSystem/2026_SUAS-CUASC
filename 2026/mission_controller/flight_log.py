"""Per-run log files: every run of run_flight.py gets its own files in ``logs/``.

All files for one run share a stem, ``<YYYY-MM-DD>_<HH-MM-SS>_<name>``, e.g.
``2026-11-05_14-32-10_flight``:

* ``<stem>.log``: everything the Jetson logs (DEBUG and up), the FC's
  STATUSTEXT messages, and every mode / arm / mission-item / link change.
* ``<stem>_telemetry.csv``: the FC's state, sampled for the whole run
  (position, attitude, mode, mission item, GPS, battery, release servo).
* ``<stem>_handover.csv``: one row per guidance tick during the handover:
  what the camera saw, the ground offset it became, and the velocity sent.

Every line is flushed as it is written, so a crash or power cut loses at
most the line being written.

The date comes from the Jetson's clock. Without network time or a charged
RTC battery it can be wrong, but the files still sort and never collide.
"""

import csv
import datetime
import logging
import os
import queue
import re
import threading
import time
from typing import Any, Optional, Sequence

from mission_controller.vehicle import commands
from mission_controller.vehicle.link import Link

logger = logging.getLogger(__name__)
fc_logger = logging.getLogger("fc")

DEFAULT_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

_FILE_FORMAT = "%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s"
_FILE_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"
_CONSOLE_FORMAT = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
_CONSOLE_DATE_FORMAT = "%H:%M:%S"


def safe_name(name: str) -> str:
    """Turns a run label into something safe for a file name ("Field test 1" -> "Field-test-1")."""
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "-", name).strip("-")
    return cleaned or "run"


def run_stem(log_dir: str, name: str, now: Optional[datetime.datetime] = None) -> str:
    """Returns ``<log_dir>/<date>_<time>_<name>``, with ``_2``, ``_3``... if that's taken."""
    now = datetime.datetime.now() if now is None else now
    base = os.path.join(log_dir, "%s_%s" % (now.strftime("%Y-%m-%d_%H-%M-%S"), safe_name(name)))
    stem, n = base, 1
    while os.path.exists(stem + ".log"):
        n += 1
        stem = "%s_%d" % (base, n)
    return stem


class RunLog:
    """The log files of one run. Create with :meth:`start`, finish with :meth:`close`.

    Attributes:
        stem: Path of the files without extension.
        log_path: The text log.
        telemetry_path: The whole-run telemetry CSV.
        handover_path: The per-tick handover CSV.
    """

    def __init__(self, stem: str) -> None:
        self.stem = stem
        self.log_path = stem + ".log"
        self.telemetry_path = stem + "_telemetry.csv"
        self.handover_path = stem + "_handover.csv"
        self._handlers = []  # type: list

    @classmethod
    def start(cls, log_dir: str, name: str, console_level: int = logging.INFO,
              now: Optional[datetime.datetime] = None) -> "RunLog":
        """Creates ``log_dir`` if needed and sends all logging to the console and a new file.

        Args:
            log_dir: Folder for the run's files.
            name: Label for the file names (e.g. "flight", "bench", "field-test-1").
            console_level: Lowest level shown on the console; the file gets DEBUG and up.
            now: Time for the file name (default: now).
        """
        os.makedirs(log_dir, exist_ok=True)
        run = cls(run_stem(log_dir, name, now))
        file_handler = logging.FileHandler(run.log_path, encoding="utf-8")
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(logging.Formatter(_FILE_FORMAT, _FILE_DATE_FORMAT))
        console = logging.StreamHandler()
        console.setLevel(console_level)
        console.setFormatter(logging.Formatter(_CONSOLE_FORMAT, _CONSOLE_DATE_FORMAT))
        root = logging.getLogger()
        root.setLevel(logging.DEBUG)
        for handler in (file_handler, console):
            root.addHandler(handler)
            run._handlers.append(handler)
        return run

    def close(self) -> None:
        """Detaches and closes this run's handlers. Safe to call more than once."""
        root = logging.getLogger()
        for handler in self._handlers:
            root.removeHandler(handler)
            handler.close()
        self._handlers = []


class CsvLog:
    """A CSV file written a row at a time, flushed after every row. Thread-safe.

    A ``wall_time`` column (local time, ms) is added in front of ``fields``.
    Values that are None are written as empty cells; unknown keys are ignored.
    """

    def __init__(self, path: str, fields: Sequence[str]) -> None:
        self.path = path
        self.fields = ("wall_time",) + tuple(fields)
        self._lock = threading.Lock()
        self._file = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, self.fields, extrasaction="ignore")
        self._writer.writeheader()
        self._file.flush()
        self.rows = 0

    def write(self, row: dict) -> None:
        with self._lock:
            if self._file.closed:
                return
            out = {k: ("" if v is None else v) for k, v in row.items()}
            out["wall_time"] = datetime.datetime.now().isoformat(sep=" ", timespec="milliseconds")
            self._writer.writerow(out)
            self._file.flush()
            self.rows += 1

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.close()


def _r(value: Optional[float], digits: int) -> Optional[float]:
    return None if value is None else round(value, digits)


TELEMETRY_FIELDS = (
    "t_s", "link_up", "heartbeat_age_s", "mode", "armed", "mission_seq",
    "lat_deg", "lon_deg", "rel_alt_m", "roll_deg", "pitch_deg", "yaw_deg",
    "groundspeed_m_s", "climb_m_s", "gps_fix", "satellites", "battery_v", "release_servo_us",
)

_STATUSTEXT_WARNING = 4  # MAV_SEVERITY_WARNING and more severe are logged as warnings


class TelemetryRecorder(threading.Thread):
    """Records the FC's state for the whole run.

    Every ``1 / rate_hz`` s it writes a telemetry CSV row, logs the FC's
    STATUSTEXT messages, and logs changes of link state, mode, arming and
    mission item. It only reads the Link's cache and a STATUSTEXT
    subscription, so it never sends anything and can't affect guidance.

    Args:
        link: Connected link.
        csv_path: Telemetry CSV to create.
        servo_channel: Release servo output to record.
        rate_hz: Sample rate, in Hz.
    """

    def __init__(self, link: Link, csv_path: str, servo_channel: int,
                 rate_hz: float = 2.0) -> None:
        super().__init__(name="telemetry-recorder", daemon=True)
        self.link = link
        self.csv = CsvLog(csv_path, TELEMETRY_FIELDS)
        self.servo_field = "servo%d_raw" % servo_channel
        self.period_s = 1.0 / rate_hz
        self._stop_event = threading.Event()
        self._start = time.monotonic()
        self._last = {}  # type: dict
        self._last_error_log = float("-inf")

    def stop(self) -> None:
        """Stops the thread and closes the CSV."""
        self._stop_event.set()
        if self.is_alive():
            self.join(timeout=2.0)
        self.csv.close()

    def run(self) -> None:
        with self.link.subscribe("STATUSTEXT") as sub:
            while True:
                try:
                    self._drain_statustext(sub)
                    self.sample()
                except Exception:  # recording must never take the run down
                    now = time.monotonic()
                    if now - self._last_error_log > 10.0:
                        logger.exception("Telemetry recording error")
                        self._last_error_log = now
                if self._stop_event.wait(self.period_s):
                    return

    def _drain_statustext(self, sub: Any) -> None:
        while True:
            try:
                msg = sub.get(timeout=0)
            except queue.Empty:
                return
            if msg.get_srcSystem() != self.link.target_system:
                continue
            level = logging.WARNING if msg.severity <= _STATUSTEXT_WARNING else logging.INFO
            fc_logger.log(level, "FC says: %s", msg.text)

    def _changed(self, key: str, value: Any, message: str, *args: Any) -> None:
        if key in self._last and self._last[key] != value:
            fc_logger.info(message, *args)
        self._last[key] = value

    def sample(self) -> dict:
        """Writes one CSV row and logs any state change. Returns the row."""
        link = self.link
        hb = link.latest("HEARTBEAT")
        mode = commands.current_mode(link)
        armed = None if hb is None else bool(
            hb.base_mode & commands.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        seq = commands.mission_current_seq(link)
        pos = link.latest_fresh("GLOBAL_POSITION_INT", 2.0)
        att = link.latest_fresh("ATTITUDE", 2.0)
        hud = link.latest_fresh("VFR_HUD", 2.0)
        gps = link.latest_fresh("GPS_RAW_INT", 3.0)
        sys_status = link.latest_fresh("SYS_STATUS", 3.0)
        servo = link.latest_fresh("SERVO_OUTPUT_RAW", 3.0)
        up = link.is_up

        self._changed("link_up", up, "Link %s", "UP again" if up else "DOWN")
        self._changed("mode", mode, "Mode %s -> %s", self._last.get("mode"), mode)
        self._changed("armed", armed, "%s", "ARMED" if armed else "DISARMED")
        if seq is not None:
            self._changed("seq", seq, "Mission item %s -> %s", self._last.get("seq"), seq)
        release_us = None if servo is None else getattr(servo, self.servo_field, None)
        if release_us is not None:
            self._changed("servo", release_us, "Release output %s: %s -> %s us", self.servo_field,
                          self._last.get("servo"), release_us)

        row = {
            "t_s": round(time.monotonic() - self._start, 2),
            "link_up": int(up),
            "heartbeat_age_s": _r(min(link.age("HEARTBEAT"), 9999.0), 2),
            "mode": mode,
            "armed": None if armed is None else int(armed),
            "mission_seq": seq,
            "lat_deg": None if pos is None else round(pos.lat / 1e7, 7),
            "lon_deg": None if pos is None else round(pos.lon / 1e7, 7),
            "rel_alt_m": None if pos is None else round(pos.relative_alt / 1000.0, 2),
            "roll_deg": None if att is None else round(_deg(att.roll), 1),
            "pitch_deg": None if att is None else round(_deg(att.pitch), 1),
            "yaw_deg": None if att is None else round(_deg(att.yaw), 1),
            "groundspeed_m_s": None if hud is None else round(hud.groundspeed, 2),
            "climb_m_s": None if hud is None else round(hud.climb, 2),
            "gps_fix": None if gps is None else gps.fix_type,
            "satellites": None if gps is None else gps.satellites_visible,
            "battery_v": None if sys_status is None else round(sys_status.voltage_battery / 1000.0, 2),
            "release_servo_us": release_us,
        }
        self.csv.write(row)
        return row


def _deg(rad: float) -> float:
    return rad * 57.29577951308232
