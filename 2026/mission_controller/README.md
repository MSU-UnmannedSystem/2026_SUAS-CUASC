# mission_controller: Jetson code for real flights

`drone_simulator/` is for local testing. `mission_controller/` is what runs on the Jetson during a
flight. It's standalone: the parts it reuses from `drone_simulator` (Link, commands, mission
read-back, params) were copied in, so it doesn't import `drone_simulator` at all.

## Who does what

| | Flight controller (ArduPilot, Cube) | Jetson (this code) |
|---|---|---|
| Mission | Flies the mission uploaded from **Mission Planner** in AUTO | Downloads it read-only and checks it; never uploads |
| Arm / takeoff / modes | Pilot / Mission Planner | Never arms, takes off or changes mode |
| Stabilisation, failsafes | Always | Sends a GCS heartbeat (sysid 255) the whole flight |
| Handover limits | Enforces `DO_GUIDED_LIMITS` (timeout, min/max alt, distance) | Stays inside them |
| Descent to drop altitude | Flies it as mission waypoints | Checks it's within 1 m of 7.62 m before centring |
| Final guidance | Accepts velocity setpoints only during `NAV_GUIDED_ENABLE` | Horizontal corrections only, until locked |
| Release | Runs the mission's `DO_SET_SERVO` when the Jetson hands back | Never fires the servo; only hands back at the right item |

## Building the mission in Mission Planner (one target per flight)

```
...  WAYPOINT          rough position over the target (any altitude)
     WAYPOINT          same position at the drop altitude, 7.62 m (the FC descends)
     DO_GUIDED_LIMITS  p1 timeout s (e.g. 120) | p2 min alt m (e.g. 5, must be < 7.62)
                       p3 max alt m (0 = none, else above 7.62) | p4 max horizontal m (e.g. 25)
     GUIDED_ENABLE     p1 = 1   (MAV_CMD_NAV_GUIDED_ENABLE): the Jetson centres here
     DO_JUMP           p1 = the resume WAYPOINT's number, p2 (repeat) = -1: the guard
     DO_SET_SERVO      servo 7, PWM 1900: the FC releases
     WAYPOINT ...      resume: the rest of the mission
```

`run_flight.py` refuses to start unless the loaded mission has:
- exactly one `GUIDED_ENABLE` (p1 = 1);
- a `DO_GUIDED_LIMITS` between the waypoint before it and it, with sane values;
- right after it, the guard `DO_JUMP` to the resume waypoint with repeat −1;
- then the release `DO_SET_SERVO` (servo 7, 1900);
- then the resume waypoint (the jump's target).

The drop altitude isn't checked before flight. The Jetson checks the real altitude when the
handover starts.

**The rough waypoint has to be close.** At 7.62 m, the camera (65° × 39.4°, straight down) sees only
about **5.5 m nose-to-tail × 9.7 m side-to-side** of ground: ±2.7 m forward/back and ±4.9 m
sideways. For comparison, it's 10.7 × 19.1 m at 15 m. If the waypoint leaves the target outside
that, the Jetson never acquires it and hands back after 10 s without releasing.

The `DO_GUIDED_LIMITS` altitudes are measured from ArduPilot's EKF origin, which is normally
close to home but not guaranteed to be the same. Leave margin.

**Cube parameters (ArduCopter 4.6.3 on the Cube Orange Plus, Jetson on TELEM2 = SERIAL2):**

| Parameter | Value | Why |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 | MAVLink2 |
| `SERIAL2_BAUD` | 921 | 921600, matches `--baud` |
| `BRD_SER2_RTSCTS` | 0 | No flow control: the Jetson link is 3-wire TX/RX/GND |
| `SERIAL2_OPTIONS` | 0 | The value the team's 3-wire link is known to work with |
| `SYSID_MYGCS` | 255 | The Jetson's heartbeat counts as a GCS (4.7+ renames it `MAV_GCS_SYSID`) |
| `SERVO7_FUNCTION` | 0 | Disabled. Otherwise ArduPilot rejects DO_SET_SERVO (pre-flight **fails** on this) |
| `GUID_TIMEOUT` | 3 (default) | How long without a Jetson setpoint before the drone stops and holds |
| `FS_GCS_ENABLE` | team decision | The Jetson's heartbeat keeps this failsafe from firing even if Mission Planner drops |

Every run reads all of these from the Cube and logs them. Anything that looks wrong is logged as
a warning; only `SERVO7_FUNCTION` stops the run.

## The handover (`guidance/handover.py`)

The Jetson stays silent until AUTO reaches `GUIDED_ENABLE` while armed. ArduCopter stays in AUTO
but accepts the Jetson's body-frame velocity setpoints. Then the Jetson:

1. **Checks altitude.** If it's not within **1.0 m** of **7.62 m**, it hands straight back,
   skipping the release.
2. **ALIGN:** bounded horizontal velocity from the pixel error (gain × ground offset, clamped to
   **0.5 m/s**). Vertical velocity is always 0, and the heading is held.
3. **LOCKED:** once the target has been **≤ 0.5 m for 2 s**, it hands back by making the
   `DO_SET_SERVO` item current. ArduPilot starts that DO item immediately, bypassing the guard
   before it: **the FC releases**, then flies the rest of the mission. The Jetson then watches `SERVO_OUTPUT_RAW` to confirm the
   release and logs it.

| If… | Then |
|---|---|
| Target not seen (or frame > 0.5 s old) | Hold position and keep looking. After **10 s** without it: hand back at the NAV item after the `DO_SET_SERVO`, so **the release is skipped** |
| Not near the drop altitude at the start | Hand back at once, release skipped |
| Mode is no longer AUTO (pilot / GCS took over) | Stop sending immediately; the pilot has it |
| Mission moved past `GUIDED_ENABLE` (FC limit breached) | Stop sending immediately. The FC continues via the guard, **no release** |
| Link lost (no heartbeat 3 s) | Stop. The FC holds after `GUID_TIMEOUT`, then continues at the limits timeout via the guard, **no release** |
| Jetson crashes / stops | Same: FC holds, then continues on its own, **no release** |

**Why the guard works.** When ArduPilot advances a mission, it follows a `DO_JUMP` only if it
actually reads it (`AP_Mission::get_next_cmd`). When the FC ends the handover itself, it advances
from the item after `GUIDED_ENABLE`, reads the `DO_JUMP` first, and jumps to the resume waypoint,
so the release is never read. The Jetson's hand-back sets the `DO_SET_SERVO` as current, so the
search starts at the release and never sees the jump. Repeat −1 means "always", so the guard can't
run out on a re-flown mission. **Only the Jetson's lock can trigger the release.**

## Jetson setup (Orin Nano, JetPack 6.2.1 / L4T R36.4.7, Ubuntu 22.04, Python 3.10)

```bash
cd 2026
python3 -m pip install -r mission_controller/requirements.txt
python3 -c "import cv2; print(cv2.__version__)"     # OpenCV, see below
sudo usermod -aG dialout,video $USER                # serial port + camera; log out and back in
ls -l /dev/ttyTHS* /dev/video*                      # find the UART and the camera
python3 -m pytest mission_controller/tests -v       # all pass, camera tests included
```

- **OpenCV:** use JetPack's (`sudo apt install python3-opencv` if `import cv2` fails). If you
  `pip install opencv-python` instead, keep numpy below 2 if anything else on the Jetson uses
  JetPack's OpenCV. Don't install both.
- **UART:** the default is `/dev/ttyTHS1`. On some Orin carrier boards / JetPack versions, the
  40-pin header's pins 8/10 are `/dev/ttyTHS0`. Use `--device` if yours is. The Orin's serial
  console is `ttyTCU0`, so the `ttyTHS` ports are normally free.
- **Camera:** the ELP USB12MP02AF is a USB camera (its red/white/green/black lead is USB power,
  D-, D+ and ground), so it shows up as `/dev/video0`. Use `--camera 1` or `--camera /dev/video2`
  if it's elsewhere. `v4l2-ctl --list-devices` (package `v4l-utils`) lists the cameras.

## Running it

From the `2026` folder (or by full path). The defaults are `/dev/ttyTHS1` @ 921600 and camera 0.

```bash
python3 mission_controller/run_flight.py --check-only               # read-only checks, then exit
python3 mission_controller/run_flight.py --bench --bench-alt 1.5    # on the ground, sends nothing
python3 mission_controller/run_flight.py --name field-test-1        # flight
```

- **`--check-only`:** connects, downloads and checks the mission, reads the Cube's settings,
  then exits. It doesn't open the camera.
- **Flight (no flag):** the same checks, then the camera must deliver a frame within 5 s, or it
  refuses to start (`PRE-FLIGHT FAILED`). It then waits silently for AUTO to reach
  `GUIDED_ENABLE`, runs the handover, confirms the FC's release, and keeps recording until the
  vehicle disarms.
- **`--bench`:** for checking the camera and the Jetson <-> Cube data flow without flying (props
  off). It connects to the real Cube and camera and runs the real guidance loop continuously. It
  uses the Cube's live attitude, and `--bench-alt` (camera height above the target, in m) in place
  of altitude. **It sends nothing that moves anything**: no setpoints, no hand-back. Each second
  it logs the target offset and the velocity it *would* command. When it would lock, or give up,
  it logs which mission item it would hand back at, then starts again. Ctrl-C stops it. A failed
  mission check is only a warning here.

## Logs: one set of files per run

Every run, including failed connections and `--check-only`, writes its own files to
`mission_controller/logs/` (or `--log-dir`). They're named with the date, time and `--name`
(default `flight`, `check`, `bench` or `sitl`):

```
logs/2026-11-05_14-32-10_field-test-1.log             everything: checks, FC params, the loaded
                                                      mission, FC messages (STATUSTEXT), every
                                                      mode / arm / mission item / link change,
                                                      the handover, the release, the summary
logs/2026-11-05_14-32-10_field-test-1_telemetry.csv   the FC's state at 2 Hz for the whole run:
                                                      mode, armed, mission item, lat/lon/alt,
                                                      attitude, speed, GPS, battery, servo 7
logs/2026-11-05_14-32-10_field-test-1_handover.csv    one row per guidance tick (10 Hz): camera
                                                      pixel and frame size, detection age,
                                                      ground offset, velocity commanded, whether
                                                      it was sent, centred/lost timers
```

Lines are flushed as they're written, so a crash or power cut loses at most one line. The
console shows INFO and up; the `.log` file also has DEBUG. The logs folder is git-ignored, so
copy the runs you want to keep. Keep the Cube's dataflash log of the same flight too. The date
comes from the Jetson's clock: without network time or an RTC battery it can be wrong, but the
files never collide.

## Settings (`config/flight_params.py`)

**Decided by the team:**
- drop altitude 7.62 m, with the FC descending;
- FC releases ball on servo 7 @ 1900 µs (mission item), guarded by a `DO_JUMP` so only the
  Jetson's lock reaches it;
- 0.5 m/s max horizontal speed;
- locked = ≤ 0.5 m for 2 s;
- 10 s lost-lock search, then hand back without release;
- camera straight down, image top = nose, 65° × 39.4° FOV, captured at 1920x1080 MJPG.

**TUNE (starting points, not flight-tested):**
- `drop_alt_tolerance_m = 1.0`;
- `horizontal_gain_per_s = 0.5` (m/s per m of offset);
- `command_rate_hz = 10`;
- `detection_max_age_s = 0.5`;
- `telemetry_max_age_s = 1.0`.

## Vision

`vision/camera.py` is the working red-bullseye `VisionPipeline` from
`2025/competition/jetson_rescue/camera.py`, with its detection code unchanged. It implements
`vision/detector.py`'s `TargetDetector`, and it records frame times so a frozen camera reads as "no
target". It asks the camera for **1920x1080 MJPG**, the size the 65° × 39.4° FOV was measured at.
It logs what the camera actually gives, and warns if that isn't 1920x1080 or isn't 16:9: the FOV,
and so every ground offset, would be wrong. If reads start failing mid-flight (cable), it logs a
warning every 5 s and guidance sees "no target". A YOLO detector replaces it by implementing
`detect()` / `close()`.

## Testing in SITL (no camera)

`sitl/handover_test.waypoints`:
1. Takeoff to 15 m.
2. A waypoint 2.0 m N / 1.5 m E of a simulated target at 35.0610, -118.1570, at 15 m.
3. The same point at 7.62 m.
4. `DO_GUIDED_LIMITS` (120 s, 5-12 m, 25 m).
5. `GUIDED_ENABLE`.
6. `DO_JUMP` to item 8, repeat −1 (the guard).
7. `DO_SET_SERVO` 7 1900.
8. A waypoint, then RTL.

`sitl/handover_fallback_test.waypoints` is the same mission with a **15 s** limits timeout. It's
for checking the FC's own fallback: stop the Jetson code mid-centring (Ctrl-C), then expect the FC
to hold, end the handover at 15 s, jump past the release, and finish the mission with servo 7
unchanged.

`--sim-target` replaces the camera with `sitl/sim_detector.py`, which projects that target into
the image from the drone's own position and attitude. It's refused on a serial device.

```bash
# terminal 1: SITL with MAVProxy
cd ~/projects/Drone_SIM/ArduPilot && Tools/autotest/sim_vehicle.py -v ArduCopter -w --custom-location=35.0605,-118.156,850,0 --console --map
# at the MAVProxy prompt
wp load <repo>/2026/mission_controller/sitl/handover_test.waypoints
# terminal 2 (from 2026/): the Jetson code against SERIAL1
python3 mission_controller/run_flight.py --device tcp:127.0.0.1:5762 --sim-target 35.0610,-118.1570
#   add --sim-hide-after 3 to make the target vanish while centring (lost-lock test)
# back at the MAVProxy prompt
mode guided / arm throttle / takeoff 15 / (at ~15 m) mode auto
# before flying it again after landing: wp set 1
```

Reset the release output before each run (`servo set 7 1100` at the MAVProxy prompt). The Jetson
only counts a release as confirmed if the output *changes* to 1900, so a value left over from an
earlier flight can't fake one.

Results on 2026-11-03 (ArduPilot master after 4.6.0-beta1), current flow:

- **Lock → FC release:** the FC descended to 7.62 m. The Jetson centred from 2.44 m away, horizontal
  only (altitude stayed 7.62–7.64 m), and locked after 2 s, about 7 s in. It handed back at item 6,
  and the FC fired the release: `servo7_raw 1100 -> 1900 us`. The FC continued at item 7, then
  RTL, and landed.
- **Lost lock (`--sim-hide-after 3`):** the target vanished 1.8 m off. The drone held at
  7.63–7.64 m for 10 s, then the Jetson handed back at item 7, skipping the release. Servo 7 was
  still 1100 µs after landing.

Results with the guard `DO_JUMP` (2026-11-03 and 2026-11-05):

- **Lock → release through the guard:** the Jetson centred from 2.45 m to 0.12 m in 6 s at
  7.64 m and handed back at item 7 (the `DO_SET_SERVO`). The FC released (`servo7_raw 0 -> 1900`),
  continued at item 8 without taking the jump, then RTL'd and landed.
- **Jetson crash → no release (`handover_fallback_test.waypoints`):** the Jetson code was killed with
  SIGKILL 2 s into the handover. The FC held at 7.63 m on item 5 and, at the 15 s limits timeout,
  went straight to "Mission: 8 WP", following the guard past the release. It RTL'd and landed.
  Servo 7 stayed at 0 in all 144 one-second samples.

**Real camera against SITL** (sign check without flying): run the Jetson code without
`--sim-target`, pointed at SITL on the laptop (`--device tcp:<laptop-ip>:5762`; SITL must accept
connections from the Jetson's network). Fly SITL to the handover and move the real target. The
logged `fwd`/`right` and commanded directions should match. SITL won't respond to the target, so
expect lost-lock or limit fallbacks; this run is only about directions.

Earlier SITL results (2026-11-02) were for the previous flow, where the Jetson descended and
released itself.

**Found and fixed during that testing:** the Jetson code used to quit when ArduPilot auto-disarmed
on the ground before takeoff (`DISARM_DELAY`). It now only stops waiting on a disarm after the
vehicle has flown.

## Hardware test order

Don't move on until each stage passes. Keep every stage's log files.

1. **Jetson:** the setup above; all tests pass, camera tests included.
2. **Cube (Mission Planner):** set the parameters above; upload a mission with the required layout.
   If Mission Planner won't take `GUIDED_ENABLE` or a `DO_JUMP` with repeat -1, stop: everything
   depends on them.
3. **Link (props off):** `--check-only`. Pass: `PRE-FLIGHT OK`, the handover line shows
   `NAV_GUIDED_ENABLE 5 -> guard DO_JUMP 6 -> release 7 -> resume 8`, no `FC param` warnings.
   Then run the flight command and leave it waiting 10 min. Pass: no setpoints or mission changes
   (it only sends heartbeats, stream requests, and the start-up mission/param reads). Pull the
   UART wire for 10 s and reconnect. Pass: `Link ... down (silence ...)`, then `recovered`, then
   `Link UP again`, with no crash.
4. **Release (props off):** fire servo 7 to 1900 from Mission Planner's Servo/Relay tab, check the
   mechanism, reset it to 1100. The telemetry CSV's `release_servo_us` column must follow it
   (`--check-only` or `--bench` records it); the Jetson's release confirmation depends on it.
5. **Camera (props off):** `--bench --bench-alt <measured camera height>` with the target under
   the camera. Pass: `Camera OK: frames are 1920x1080`, no 16:9 warning. Moving the target towards
   the nose gives fwd > 0; moving it right gives right > 0. The commanded velocity points the same
   way. Tilt the drone: the Cube's roll/pitch show in the handover CSV, and the offset corrects
   for them. Measure where the image edges fall on the ground at that height, against 65° × 39.4°.
6. **Real camera against SITL** (above), then **flights**, in order: no target and no payload
   (expect lost lock, hand-back past the release, servo 7 unchanged); pilot takeover to LOITER
   (expect `pilot_takeover`, Jetson stops at once); target without payload (expect lock, FC
   release, mission continues); then with payload.

After a pilot takeover, switching back to AUTO puts the FC back on `GUIDED_ENABLE` with nobody
steering it. It holds there until the `DO_GUIDED_LIMITS` timeout, then the guard skips the
release. That's safe, but the safety pilot should expect the hover.

## Tests

```bash
python3 -m pytest 2026/mission_controller/tests -v
```

The tests run without hardware or SITL. They include closed-loop handover runs against a
simulated `NAV_GUIDED_ENABLE` vehicle that handles hand-back like ArduPilot:
- lock → FC release;
- lost lock → release skipped;
- regained lock;
- a frozen camera;
- wrong altitude;
- a target outside the drop-altitude view;
- pilot takeover;
- the FC ending the handover (and releasing);
- link loss;
- bench mode, which sends nothing.

They also run whole sessions through `run_flight.main()`: a check-only run, a full simulated
flight, bench mode, and a missing camera. Each checks the log files it writes. They also cover the
mission check, the Cube settings report, the camera maths and capture start-up, the per-run log
files and telemetry recorder, and the copied Link, command and param tests. Tests never write
into `logs/`. `test_camera.py` is skipped where OpenCV isn't installed.
