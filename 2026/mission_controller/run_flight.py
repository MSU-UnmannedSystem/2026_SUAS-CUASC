"""Jetson entry point for a real flight.

The pilot / Mission Planner uploads the mission, arms, takes off and starts
AUTO; the FC flies to the target and down to the drop altitude. This script
connects to the Cube, checks the loaded mission and the release servo, then
sends nothing until AUTO reaches the NAV_GUIDED_ENABLE item. It then centres
over the target (see guidance/handover.py) and hands back; the FC fires the
release (the mission's DO_SET_SERVO) and flies the rest. It stays connected
until the vehicle disarms.

Run from the ``2026`` folder (or anywhere; the path is set below)::

    python3 mission_controller/run_flight.py
    python3 mission_controller/run_flight.py --check-only   # pre-flight checks, then exit

SITL only (no camera): ``--device tcp:127.0.0.1:5762 --sim-target LAT,LON`` replaces
the camera with a simulated target at LAT,LON (see sitl/). Refused on a serial device.
"""

import argparse
import logging
import os
import sys
import time
from typing import List, Optional

# Run as a plain script: put the "2026" folder on sys.path for the package imports.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from mission_controller.config.flight_params import HandoverConfig  # noqa: E402
from mission_controller.guidance.handover import LOCKED, HandoverController  # noqa: E402
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
    plan = check_mission(items, config.drop_alt_m, config.servo_channel,
                         config.servo_release_pwm_us)
    check_release_servo(link, config.servo_channel)
    logger.info("Mode %s, %s", commands.current_mode(link),
                "armed" if link.latest("HEARTBEAT").base_mode
                & commands.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "disarmed")
    return plan


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


def main(argv: Optional[List[str]] = None) -> int:
    defaults = HandoverConfig()
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default=defaults.device)
    parser.add_argument("--baud", type=int, default=defaults.baud)
    parser.add_argument("--check-only", action="store_true",
                        help="run the pre-flight checks and exit")
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
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    config = HandoverConfig(device=args.device, baud=args.baud)

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
    try:
        try:
            plan = preflight(link, config)
        except (MissionError, RuntimeError) as e:
            logger.error("PRE-FLIGHT FAILED: %s", e)
            return 1
        logger.info("PRE-FLIGHT OK")
        if args.check_only:
            return 0

        if sim_target is not None:
            from mission_controller.sitl.sim_detector import SimTargetDetector
            detector = SimTargetDetector(link, sim_target[0], sim_target[1],
                                         config.camera_hfov_deg, config.camera_vfov_deg,
                                         hide_after_s=args.sim_hide_after)
            logger.warning("SITL: simulated target at %.7f, %.7f%s", sim_target[0],
                           sim_target[1], "" if args.sim_hide_after is None
                           else " (hidden after %.0f s)" % args.sim_hide_after)
        else:
            # Imported here so --check-only works without OpenCV.
            from mission_controller.vision.camera import VisionPipeline
            detector = VisionPipeline(camera_index=config.camera_index,
                                      target_type=config.target_type)
        controller = HandoverController(link, detector, plan, config)
        if controller.wait_for_handover():
            servo_before = servo_output_us(link, config.servo_channel)
            outcome = controller.run()
            logger.info("Handover outcome: %s", outcome)
            if outcome == LOCKED:
                confirm_fc_release(link, config.servo_channel, config.servo_release_pwm_us,
                                   servo_before)
        wait_until_disarmed(link)
        return 0
    except KeyboardInterrupt:
        logger.warning("Interrupted; the FC keeps flying its mission")
        return 130
    finally:
        if detector is not None:
            detector.close()
        link.close()


if __name__ == "__main__":
    sys.exit(main())
