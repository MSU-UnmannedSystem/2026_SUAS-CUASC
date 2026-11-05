"""Jetson entry point for a real flight.

The pilot / Mission Planner uploads the mission, arms, takes off and starts
AUTO; the FC flies to the target and down to the drop altitude. This script
connects to the Cube, checks the loaded mission, the release servo and the
camera, then sends nothing until AUTO reaches the NAV_GUIDED_ENABLE item. It
then centres over the target (see guidance/handover.py) and hands back; the
FC fires the release (the mission's DO_SET_SERVO) and flies the rest. It stays
connected, recording, until the vehicle disarms.

Every run writes its own files to ``mission_controller/logs/`` (see
flight_log.py), named ``<date>_<time>_<name>``::

    python3 mission_controller/run_flight.py                    # flight
    python3 mission_controller/run_flight.py --name field-test-1
    python3 mission_controller/run_flight.py --check-only       # pre-flight checks, then exit
    python3 mission_controller/run_flight.py --bench --bench-alt 1.5   # camera + data flow on the ground

``--bench`` runs guidance against the real Cube and camera on the ground, using
``--bench-alt`` as the height, and sends nothing (no setpoints, no hand-back).

SITL only (no camera): ``--device tcp:127.0.0.1:5762 --sim-target LAT,LON`` replaces
the camera with a simulated target at LAT,LON (see sitl/). Refused on a serial device.
"""

import argparse
import logging
import os
import platform
import sys
import time
from typing import List, Optional

# Run as a plain script: put the "2026" folder on sys.path for the package imports.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from mission_controller import flight_log  # noqa: E402
from mission_controller.config.flight_params import HandoverConfig  # noqa: E402
from mission_controller.guidance.handover import (  # noqa: E402
    LINK_LOST, LOCKED, TRACE_FIELDS, HandoverController)
from mission_controller.vehicle import commands, params  # noqa: E402
from mission_controller.vehicle.link import (  # noqa: E402
    Link, LinkConfig, LinkStreamError, LinkTimeoutError)
from mission_controller.vehicle.mission import (  # noqa: E402
    HandoverPlan, MissionError, check_mission, download_items)

logger = logging.getLogger("run_flight")

# SERVOn_FUNCTION values for which ArduPilot honours MAV_CMD_DO_SET_SERVO
# (AP_ServoRelayEvents::do_set_servo): disabled, RCPassThru, sprayer pump and
# spinner, gripper, RCIN1-16 and RCIN1-16 mapped.
DO_SET_SERVO_FUNCTIONS = {0, 1, 22, 23, 28} | set(range(51, 67)) | set(range(140, 156))


def check_release_servo(link: Link, channel: int) -> None:
    """Fails if MAV_CMD_DO_SET_SERVO can't move the release output.

    Raises:
        RuntimeError: SERVOn_FUNCTION is unreadable or assigned to another function.
    """
    name = "SERVO%d_FUNCTION" % channel
    param = params.read_param(link, name)
    if param is None:
        raise RuntimeError("Could not read %s" % name)
    function = int(param.value)
    if function not in DO_SET_SERVO_FUNCTIONS:
        raise RuntimeError("%s = %d: ArduPilot will reject DO_SET_SERVO on this output; set it to "
                           "0 (Disabled)" % (name, function))
    logger.info("%s = %d: DO_SET_SERVO allowed", name, function)


def preflight(link: Link, config: HandoverConfig) -> HandoverPlan:
    """Downloads and checks the mission, and checks the release servo."""
    items = download_items(link)
    logger.info("Mission loaded on the Cube: %d items", len(items))
    for seq, item in enumerate(items):
        logger.debug("  item %d: %s p1=%g p2=%g p3=%g p4=%g lat=%.7f lon=%.7f alt=%.2f", seq,
                     commands._enum_name("MAV_CMD", item.command), item.param1, item.param2,
                     item.param3, item.param4, item.lat_deg, item.lon_deg, item.alt_m)
    plan = check_mission(items, config.drop_alt_m, config.servo_channel,
                         config.servo_release_pwm_us)
    check_release_servo(link, config.servo_channel)
    logger.info("Mode %s, %s", commands.current_mode(link),
                "armed" if link.latest("HEARTBEAT").base_mode
                & commands.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "disarmed")
    return plan


def log_fc_setup(link: Link, config: HandoverConfig) -> List[str]:
    """Reads and logs the Cube settings this setup depends on; warns about suspicious ones.

    Read-only, and never fails the run: the link is already working, so these
    are reported for the log and the operator rather than enforced.

    Returns:
        The warnings logged.
    """
    port = config.fc_serial_port
    baud_code = config.baud // 1000  # SERIALn_BAUD is in kbaud (921600 -> 921)
    checks = [
        (("SERIAL%d_PROTOCOL" % port,), lambda v: v == 2,
         "should be 2 (MAVLink2) on the Jetson's port (SERIAL%d)" % port),
        (("SERIAL%d_BAUD" % port,), lambda v: v == baud_code,
         "should be %d to match the Jetson's %d baud" % (baud_code, config.baud)),
        (("BRD_SER%d_RTSCTS" % port,), lambda v: v == 0,
         "should be 0 (no flow control) for a 3-wire TX/RX/GND UART"),
        (("SERIAL%d_OPTIONS" % port,), lambda v: v == 0,
         "is not 0; 0 is the value the team's 3-wire UART is known to work with"),
        (("MAV_GCS_SYSID", "SYSID_MYGCS"), lambda v: v == link.source_system,
         "should be %d, or the FC won't count the Jetson's heartbeat as a GCS"
         % link.source_system),
        (("GUID_TIMEOUT",), None, None),
        (("FS_GCS_ENABLE",), None, None),
        (("FS_GCS_TIMEOUT",), None, None),
    ]
    warnings = []
    for names, ok, problem in checks:
        value = params.read_param(link, *names, timeout_s=1.0, retries=1)
        if value is None:
            logger.info("FC param %s: not on this firmware/board", " / ".join(names))
            continue
        if ok is not None and not ok(value.value):
            message = "FC param %s = %g %s" % (value.name, value.value, problem)
            logger.warning("%s", message)
            warnings.append(message)
        else:
            logger.info("FC param %s = %g", value.name, value.value)
    return warnings


def servo_output_us(link: Link, channel: int) -> Optional[int]:
    """Current PWM of a servo output from a fresh SERVO_OUTPUT_RAW, or None."""
    msg = link.latest_fresh("SERVO_OUTPUT_RAW", 1.0)
    return None if msg is None else getattr(msg, "servo%d_raw" % channel, None)


def confirm_fc_release(link: Link, channel: int, pwm_us: int, before_us: Optional[int],
                       timeout_s: float = 5.0) -> bool:
    """Watches SERVO_OUTPUT_RAW for the FC's DO_SET_SERVO release; logs the result.

    The FC fires the release, so this is only a report: nothing is sent. A
    release only counts if the output *changes* to ``pwm_us``: if it was
    already there before the handover, the release can't be confirmed.

    Args:
        link: Connected link.
        channel: Release servo output.
        pwm_us: Release PWM, in microseconds.
        before_us: The output's PWM when the handover started (None = unknown).
        timeout_s: How long to watch, in seconds.

    Returns:
        True if servo ``channel`` changed to ``pwm_us`` within ``timeout_s`` seconds.
    """
    field = "servo%d_raw" % channel
    if before_us is None or before_us == pwm_us:
        logger.warning("Release can't be confirmed: %s was %s before the handover", field,
                       "unknown" if before_us is None else "%d us already" % before_us)
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = link.latest_fresh("SERVO_OUTPUT_RAW", 1.0)
        if msg is not None and getattr(msg, field, None) == pwm_us:
            logger.info("FC released the payload: %s %d -> %d us", field, before_us, pwm_us)
            return True
        time.sleep(0.1)
    logger.warning("Release not seen: %s did not show %d us within %.0f s (SERVO_OUTPUT_RAW "
                   "may not be streaming)", field, pwm_us, timeout_s)
    return False


def wait_until_disarmed(link: Link) -> None:
    """Stays connected (heartbeats keep running) and reports progress until the vehicle disarms."""
    last_report = 0.0
    while True:
        hb = link.latest("HEARTBEAT")
        if hb is not None and not hb.base_mode & commands.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            logger.info("Vehicle disarmed; flight over")
            return
        if time.monotonic() - last_report > 5.0:
            logger.info("Mode %s, mission item %s", commands.current_mode(link),
                        commands.mission_current_seq(link))
            last_report = time.monotonic()
        time.sleep(0.5)


def check_camera(detector, config: HandoverConfig) -> None:
    """Fails unless the camera delivers a frame; warns if its size doesn't match the FOV.

    Raises:
        RuntimeError: No frame within ``config.camera_startup_timeout_s``.
    """
    size = detector.wait_for_frame(config.camera_startup_timeout_s)
    if size is None:
        raise RuntimeError("camera %r gave no frame within %.0f s (plugged in? right index? "
                           "check `v4l2-ctl --list-devices`)"
                           % (config.camera_index, config.camera_startup_timeout_s))
    width, height = size
    logger.info("Camera OK: frames are %dx%d", width, height)
    if (width, height) != (config.camera_width_px, config.camera_height_px):
        logger.warning("Camera gives %dx%d, not the requested %dx%d", width, height,
                       config.camera_width_px, config.camera_height_px)
    if abs(width / float(height) - 16.0 / 9.0) > 0.02:
        logger.warning("Camera frames are %dx%d, not 16:9. The %.1f x %.1f deg FOV in the config "
                       "is for 1920x1080, so ground offsets will be wrong until the FOV is "
                       "measured at this size", width, height, config.camera_hfov_deg,
                       config.camera_vfov_deg)


def run_bench(link: Link, detector, plan: Optional[HandoverPlan], config: HandoverConfig,
              bench_alt_m: float, trace) -> None:
    """Runs guidance on the ground against the real Cube and camera, sending nothing. Ctrl-C ends it."""
    logger.warning("BENCH MODE at %.2f m: guidance runs on the real camera and the Cube's real "
                   "attitude, but NOTHING is sent (no setpoints, no hand-back). Ctrl-C to stop.",
                   bench_alt_m)
    logger.info("Sign check: move the target towards the nose -> fwd > 0; to the right -> "
                "right > 0. The commanded velocity should point the same way.")
    controller = HandoverController(link, detector, plan, config, trace=trace,
                                    bench_alt_m=bench_alt_m)
    while True:
        outcome = controller.run()
        logger.info("BENCH: handover would have ended with: %s; starting again", outcome)
        if outcome == LINK_LOST:
            time.sleep(1.0)


def _camera_arg(value: str):
    return int(value) if value.isdigit() else value


def main(argv: Optional[List[str]] = None) -> int:
    defaults = HandoverConfig()
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default=defaults.device)
    parser.add_argument("--baud", type=int, default=defaults.baud)
    parser.add_argument("--camera", type=_camera_arg, default=defaults.camera_index,
                        help="camera index or device path (default %(default)s = /dev/video0)")
    parser.add_argument("--check-only", action="store_true",
                        help="run the pre-flight checks and exit")
    parser.add_argument("--bench", action="store_true",
                        help="on the ground: run guidance on the real camera and Cube, send nothing")
    parser.add_argument("--bench-alt", type=float, default=None, metavar="M",
                        help="--bench only: camera height above the target, in m "
                             "(default: the drop altitude)")
    parser.add_argument("--name", default=None,
                        help="label for this run's log files (default: flight, check, bench "
                             "or sitl)")
    parser.add_argument("--log-dir", default=flight_log.DEFAULT_LOG_DIR,
                        help="folder for the log files (default: mission_controller/logs)")
    parser.add_argument("--sim-target", metavar="LAT,LON", default=None,
                        help="SITL only: simulated target position instead of the camera")
    parser.add_argument("--sim-hide-after", type=float, default=None, metavar="S",
                        help="SITL only: hide the simulated target S s after first seeing it")
    args = parser.parse_args(argv)
    sim_target = None
    if args.sim_target is not None:
        try:
            sim_target = tuple(float(v) for v in args.sim_target.split(","))
        except ValueError:
            sim_target = ()
        if len(sim_target) != 2:
            parser.error("--sim-target must be LAT,LON")
    elif args.sim_hide_after is not None:
        parser.error("--sim-hide-after needs --sim-target")
    if args.bench and (args.check_only or sim_target is not None):
        parser.error("--bench can't be combined with --check-only or --sim-target")
    if args.bench_alt is not None and (not args.bench or args.bench_alt <= 0):
        parser.error("--bench-alt needs --bench and must be > 0")

    name = args.name or ("check" if args.check_only else "bench" if args.bench
                         else "sitl" if sim_target is not None else "flight")
    run_log = flight_log.RunLog.start(args.log_dir, name)
    try:
        return _run(args, sim_target, run_log)
    finally:
        written = [p for p in (run_log.log_path, run_log.telemetry_path, run_log.handover_path)
                   if os.path.exists(p)]
        logger.info("Log files: %s", ", ".join(written))
        run_log.close()


def _run(args: argparse.Namespace, sim_target, run_log: flight_log.RunLog) -> int:
    config = HandoverConfig(device=args.device, baud=args.baud, camera_index=args.camera)
    logger.info("Run log: %s", run_log.log_path)
    logger.info("Arguments: %s", vars(args))
    logger.info("Config: %s", config)
    logger.info("Host %s, Python %s", platform.node(), platform.python_version())
    logger.info("Drop alt %.2f m (FC releases servo %d -> %d us) | max %.1f m/s | "
                "lost-lock %.0f s",
                config.drop_alt_m, config.servo_channel, config.servo_release_pwm_us,
                config.max_horizontal_speed_m_s, config.lost_lock_timeout_s)
    try:
        link = Link.connect(LinkConfig(config.device, baud=config.baud))
    except (LinkTimeoutError, LinkStreamError, OSError) as e:  # OSError covers SerialException
        logger.error("Could not connect to the Cube on %s: %s", config.device, e)
        return 1
    if sim_target is not None and link.is_serial:
        logger.error("--sim-target is for SITL only; refusing on serial device %s", config.device)
        link.close()
        return 1
    detector = None
    recorder = None
    trace_csv = None
    try:
        recorder = flight_log.TelemetryRecorder(link, run_log.telemetry_path,
                                                config.servo_channel)
        recorder.start()
        plan = None  # type: Optional[HandoverPlan]
        try:
            plan = preflight(link, config)
        except (MissionError, RuntimeError) as e:
            if not args.bench:
                logger.error("PRE-FLIGHT FAILED: %s", e)
                return 1
            logger.warning("Pre-flight check failed (%s); bench mode continues anyway", e)
        log_fc_setup(link, config)

        if sim_target is not None:
            from mission_controller.sitl.sim_detector import SimTargetDetector
            detector = SimTargetDetector(link, sim_target[0], sim_target[1],
                                         config.camera_hfov_deg, config.camera_vfov_deg,
                                         hide_after_s=args.sim_hide_after)
            logger.warning("SITL: simulated target at %.7f, %.7f%s", sim_target[0],
                           sim_target[1], "" if args.sim_hide_after is None
                           else " (hidden after %.0f s)" % args.sim_hide_after)
        elif not args.check_only:
            # Imported here so --check-only works without OpenCV.
            from mission_controller.vision.camera import VisionPipeline
            detector = VisionPipeline(camera_index=config.camera_index,
                                      target_type=config.target_type,
                                      width=config.camera_width_px,
                                      height=config.camera_height_px,
                                      fourcc=config.camera_fourcc)
            try:
                check_camera(detector, config)
            except RuntimeError as e:
                logger.error("PRE-FLIGHT FAILED: %s", e)
                return 1
        logger.info("PRE-FLIGHT OK")
        if args.check_only:
            return 0

        trace_csv = flight_log.CsvLog(run_log.handover_path, TRACE_FIELDS)
        if args.bench:
            run_bench(link, detector, plan, config,
                      args.bench_alt if args.bench_alt is not None else config.drop_alt_m,
                      trace_csv.write)
            return 0  # not reached: bench runs until Ctrl-C

        controller = HandoverController(link, detector, plan, config, trace=trace_csv.write)
        outcome, released = None, None
        if controller.wait_for_handover():
            servo_before = servo_output_us(link, config.servo_channel)
            outcome = controller.run()
            logger.info("Handover outcome: %s", outcome)
            if outcome == LOCKED:
                released = confirm_fc_release(link, config.servo_channel,
                                              config.servo_release_pwm_us, servo_before)
        wait_until_disarmed(link)
        logger.info("RUN SUMMARY: handover %s, release %s, %d handover ticks recorded",
                    outcome or "never started",
                    "confirmed" if released else "not confirmed" if released is False
                    else "not attempted", trace_csv.rows)
        return 0
    except KeyboardInterrupt:
        if args.bench:
            logger.info("Bench run stopped")
            return 0
        logger.warning("Interrupted; the FC keeps flying its mission")
        return 130
    except Exception:
        logger.exception("Unexpected error; the Jetson stops here and the FC keeps flying its "
                         "mission (it skips the release if this was mid-handover)")
        return 1
    finally:
        if detector is not None:
            detector.close()
        if recorder is not None:
            recorder.stop()
        if trace_csv is not None:
            trace_csv.close()
        link.close()


if __name__ == "__main__":
    sys.exit(main())
