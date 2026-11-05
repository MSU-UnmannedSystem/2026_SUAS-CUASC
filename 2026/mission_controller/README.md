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

**Cube parameters to check:**
- `SERVO7_FUNCTION = 0` (Disabled). Otherwise ArduPilot rejects DO_SET_SERVO; the pre-flight
  check reads this.
- `GUID_TIMEOUT` (default 3 s): how long without a Jetson setpoint before the drone stops and
  holds.
- The TELEM2 port the Jetson uses must speak MAVLink2 (`SERIALx_PROTOCOL = 2`).

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

## Running it on the Jetson

```bash
python3 mission_controller/run_flight.py --check-only   # connect, check mission + servo, exit
python3 mission_controller/run_flight.py                # wait for the handover, run it, stay until disarmed
```

Run these from the `2026` folder (or by full path). The defaults are `/dev/ttyTHS1` @ 921600; use
`--device` / `--baud` to change them.

## Settings (`config/flight_params.py`)

**Decided by the team:**
- drop altitude 7.62 m, with the FC descending;
- FC releases ball on servo 7 @ 1900 µs (mission item), guarded by a `DO_JUMP` so only the
  Jetson's lock reaches it;
- 0.5 m/s max horizontal speed;
- locked = ≤ 0.5 m for 2 s;
- 10 s lost-lock search, then hand back without release;
- camera straight down, image top = nose, 65° × 39.4° FOV.

**TUNE (starting points, not flight-tested):**
- `drop_alt_tolerance_m = 1.0`;
- `horizontal_gain_per_s = 0.5` (m/s per m of offset);
- `command_rate_hz = 10`;
- `detection_max_age_s = 0.5`;
- `telemetry_max_age_s = 1.0`.

## Vision

`vision/camera.py` is the working red-bullseye `VisionPipeline` from
`2025/competition/jetson_rescue/camera.py`, with its detection code unchanged. It now implements
`vision/detector.py`'s `TargetDetector`, and it records frame times so a frozen camera reads as "no
target". The capture resolution isn't set in that code, so guidance uses the frame size the camera
actually returns. A YOLO detector replaces it by implementing `detect()` / `close()`.

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

Earlier SITL results (2026-11-02) were for the previous flow, where the Jetson descended and
released itself.

**Found and fixed during that testing:** the Jetson code used to quit when ArduPilot auto-disarmed
on the ground before takeoff (`DISARM_DELAY`). It now only stops waiting on a disarm after the
vehicle has flown.

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
- link loss. They also cover the
mission check, the camera maths, and the copied Link, command and param tests. `test_camera.py` is
skipped where OpenCV isn't installed.
