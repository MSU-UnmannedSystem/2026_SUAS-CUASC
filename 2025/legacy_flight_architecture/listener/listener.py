"""
listener.py

Safe testing-only MAVLink listener.

Adds:
1. Real camera module interface with structured result
2. Stable altitude gating with consecutive in-range readings
3. Explicit state machine
6. Camera throttling

Still does NOT:
- actuate any servo
- send payload release commands
"""

from __future__ import annotations

import argparse
import logging
import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from pymavlink import mavutil
from camera import CameraDetector, DetectionResult


class State(Enum):
    WAITING_FOR_AUTO = auto()
    WAITING_FOR_ALTITUDE = auto()
    CAMERA_CHECK = auto()
    TARGET_CONFIRMED = auto()
    ACTION_SIMULATED = auto()
    DONE = auto()


@dataclass
class FlightState:
    mode_name: Optional[str] = None
    rel_alt_ft: Optional[float] = None
    last_mode_update: float = 0.0
    last_alt_update: float = 0.0
    stable_in_range_count: int = 0
    last_camera_check_time: float = 0.0
    state: State = State.WAITING_FOR_AUTO
    simulated_action_done: bool = False


def meters_to_feet(meters: float) -> float:
    return meters * 3.28084


def within_window(value: float, lower: float, upper: float) -> bool:
    return lower <= value <= upper


def get_mode_name(master: mavutil.mavfile, heartbeat_msg) -> str:
    try:
        return mavutil.mode_string_v10(heartbeat_msg)
    except Exception:
        try:
            if master.flightmode:
                return str(master.flightmode)
        except Exception:
            pass
    return "UNKNOWN"


def update_stable_altitude_count(
    state: FlightState,
    alt_lower_ft: float,
    alt_upper_ft: float,
) -> None:
    if state.rel_alt_ft is None:
        state.stable_in_range_count = 0
        return

    if within_window(state.rel_alt_ft, alt_lower_ft, alt_upper_ft):
        state.stable_in_range_count += 1
    else:
        state.stable_in_range_count = 0


def can_run_camera(
    state: FlightState,
    required_stable_reads: int,
    camera_interval_s: float,
    now: float,
) -> bool:
    if state.mode_name != "AUTO":
        return False

    if state.stable_in_range_count < required_stable_reads:
        return False

    if (now - state.last_camera_check_time) < camera_interval_s:
        return False

    return True


def transition_state(state: FlightState, new_state: State) -> None:
    if state.state != new_state:
        logging.info("STATE: %s -> %s", state.state.name, new_state.name)
        state.state = new_state


def run_listener(
    port: str,
    baud: int,
    alt_lower_ft: float,
    alt_upper_ft: float,
    required_stable_reads: int,
    heartbeat_timeout: float,
    camera_interval_s: float,
    min_confidence: float,
    camera_index: int,
) -> None:
    logging.info("Connecting to MAVLink on %s @ %d baud...", port, baud)
    master = mavutil.mavlink_connection(port, baud=baud)

    logging.info("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=heartbeat_timeout)
    logging.info(
        "Heartbeat received from system=%s component=%s",
        master.target_system,
        master.target_component,
    )

    detector = CameraDetector(camera_index=camera_index)
    state = FlightState()

    while True:
        msg = master.recv_match(blocking=True, timeout=1.0)
        now = time.time()

        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == "BAD_DATA":
            continue

        if msg_type == "HEARTBEAT":
            state.mode_name = get_mode_name(master, msg)
            state.last_mode_update = now
            logging.info("Mode update: %s", state.mode_name)

            if state.mode_name != "AUTO" and not state.simulated_action_done:
                state.stable_in_range_count = 0
                transition_state(state, State.WAITING_FOR_AUTO)
            elif state.mode_name == "AUTO" and state.state == State.WAITING_FOR_AUTO:
                transition_state(state, State.WAITING_FOR_ALTITUDE)

        elif msg_type == "GLOBAL_POSITION_INT":
            rel_alt_m = msg.relative_alt / 1000.0
            state.rel_alt_ft = meters_to_feet(rel_alt_m)
            state.last_alt_update = now
            logging.info("Relative altitude: %.2f ft", state.rel_alt_ft)

            update_stable_altitude_count(state, alt_lower_ft, alt_upper_ft)
            logging.info(
                "Stable in-range count: %d/%d",
                state.stable_in_range_count,
                required_stable_reads,
            )

            if state.mode_name == "AUTO" and not state.simulated_action_done:
                if state.stable_in_range_count < required_stable_reads:
                    transition_state(state, State.WAITING_FOR_ALTITUDE)

        if state.simulated_action_done:
            transition_state(state, State.DONE)
            time.sleep(0.01)
            continue

        if can_run_camera(
            state=state,
            required_stable_reads=required_stable_reads,
            camera_interval_s=camera_interval_s,
            now=now,
        ):
            transition_state(state, State.CAMERA_CHECK)
            state.last_camera_check_time = now

            try:
                result: DetectionResult = detector.detect_target()
            except Exception as exc:
                logging.exception("Camera detector failed: %s", exc)
                transition_state(state, State.WAITING_FOR_ALTITUDE)
                time.sleep(0.01)
                continue

            logging.info(
                "Camera result | detected=%s confidence=%.3f timestamp=%.3f info=%s",
                result.detected,
                result.confidence,
                result.timestamp,
                result.info,
            )

            if result.detected and result.confidence >= min_confidence:
                transition_state(state, State.TARGET_CONFIRMED)
                logging.warning(
                    "SAFE TEST MODE: target confirmed. "
                    "Would trigger payload action here, but no actuator command is sent."
                )
                transition_state(state, State.ACTION_SIMULATED)
                state.simulated_action_done = True
            else:
                transition_state(state, State.WAITING_FOR_ALTITUDE)

        time.sleep(0.01)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safe testing-only MAVLink listener")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=57600, help="Serial baud rate")
    parser.add_argument(
        "--alt-lower-ft",
        type=float,
        default=50.0,
        help="Minimum allowed relative altitude in feet",
    )
    parser.add_argument(
        "--alt-upper-ft",
        type=float,
        default=52.0,
        help="Maximum allowed relative altitude in feet",
    )
    parser.add_argument(
        "--required-stable-reads",
        type=int,
        default=5,
        help="Consecutive in-range altitude readings required before camera check",
    )
    parser.add_argument(
        "--camera-interval-s",
        type=float,
        default=1.0,
        help="Minimum seconds between camera checks",
    )
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=0.7,
        help="Minimum confidence needed to count as a confirmed target",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="OpenCV camera index",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for first heartbeat",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

    run_listener(
        port=args.port,
        baud=args.baud,
        alt_lower_ft=args.alt_lower_ft,
        alt_upper_ft=args.alt_upper_ft,
        required_stable_reads=args.required_stable_reads,
        heartbeat_timeout=args.heartbeat_timeout,
        camera_interval_s=args.camera_interval_s,
        min_confidence=args.min_confidence,
        camera_index=args.camera_index,
    )


if __name__ == "__main__":
    main()