"""Flight-time settings for the Jetson handover (one target / one payload per flight)."""

from dataclasses import dataclass


@dataclass
class HandoverConfig:
    """Everything the handover controller needs, in SI units.

    Values marked DECIDED were chosen by the team; values marked TUNE are
    starting points that have not been flight-tested.

    Attributes:
        device: Serial device to the Cube (Jetson UART to TELEM2).
        baud: Baud rate in bits/s.
        drop_alt_m: Altitude above home the FC descends to before the handover, in
            metres. The Jetson only centres if it is within ``drop_alt_tolerance_m``
            of this when the handover starts. DECIDED: 7.62 (25 ft).
        drop_alt_tolerance_m: Allowed difference from ``drop_alt_m`` at the start of
            the handover, in metres. TUNE.
        servo_channel: Release servo. The FC fires it from the mission's DO_SET_SERVO
            item; pre-flight checks that item uses this channel. DECIDED: 7 (ball).
        servo_release_pwm_us: PWM the mission's DO_SET_SERVO must set, in microseconds.
            DECIDED: 1900.
        max_horizontal_speed_m_s: Clamp on centring velocity, in m/s. DECIDED: 0.5.
        horizontal_gain_per_s: Centring velocity per metre of ground offset
            (m/s per m). TUNE.
        center_tolerance_m: Ground offset that counts as centred, in metres. DECIDED: 0.5.
        center_hold_s: How long it must stay centred before handing back to the FC
            (which then releases), in seconds. DECIDED: 2.0.
        lost_lock_timeout_s: Hold and search this long after losing the target, then
            hand back skipping the release, in seconds. DECIDED: 10.0.
        command_rate_hz: Velocity setpoint rate, in Hz. Must be well above
            1 / GUID_TIMEOUT (3 s default). TUNE.
        detection_max_age_s: A detection older than this counts as no detection, in
            seconds. TUNE.
        telemetry_max_age_s: Position/attitude older than this stops guidance (zero
            velocity), in seconds. TUNE.
        camera_hfov_deg: Horizontal field of view in degrees. From main.py: 65.0.
        camera_vfov_deg: Vertical field of view in degrees. From main.py: 39.4.
        camera_index: OpenCV camera index.
        target_type: Detector mode in the copied VisionPipeline ('red_bullseye').
    """

    device: str = "/dev/ttyTHS1"
    baud: int = 921600

    drop_alt_m: float = 7.62
    drop_alt_tolerance_m: float = 1.0
    servo_channel: int = 7
    servo_release_pwm_us: int = 1900

    max_horizontal_speed_m_s: float = 0.5
    horizontal_gain_per_s: float = 0.5
    center_tolerance_m: float = 0.5
    center_hold_s: float = 2.0
    lost_lock_timeout_s: float = 10.0

    command_rate_hz: float = 10.0
    detection_max_age_s: float = 0.5
    telemetry_max_age_s: float = 1.0

    camera_hfov_deg: float = 65.0
    camera_vfov_deg: float = 39.4
    camera_index: int = 0
    target_type: str = "red_bullseye"

    def __post_init__(self) -> None:
        positive = ("drop_alt_m", "drop_alt_tolerance_m", "max_horizontal_speed_m_s",
                    "horizontal_gain_per_s", "center_tolerance_m", "lost_lock_timeout_s",
                    "command_rate_hz", "detection_max_age_s", "telemetry_max_age_s")
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError("%s must be > 0" % name)
        if self.center_hold_s < 0:
            raise ValueError("center_hold_s must be >= 0")
        if self.command_rate_hz < 2.0:
            raise ValueError("command_rate_hz must be >= 2 Hz (GUID_TIMEOUT stops the vehicle)")
        if not 0 < self.camera_hfov_deg < 180 or not 0 < self.camera_vfov_deg < 180:
            raise ValueError("camera FOV must be between 0 and 180 degrees")
        if not 1 <= self.servo_channel <= 16:
            raise ValueError("servo_channel must be 1-16")
