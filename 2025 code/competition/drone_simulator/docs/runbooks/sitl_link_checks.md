# SITL checks for `Link` and `commands`

These scripts check the parts of `vehicle/link.py` and `vehicle/commands.py` that unit tests can't
reach. Every run ends with a `RESULT: PASS | FAIL | INCONCLUSIVE` line. Run them from the
`2025 code` folder:

```bash
python3 competition/drone_simulator/tools/sitl_link_check.py <check> [--device ...]
```

| Check       | Question it answers                                                    | Flies? |
|-------------|------------------------------------------------------------------------|--------|
| `lock`      | Does Link lock onto the autopilot, not Mission Planner's forwarded heartbeat? | no |
| `failsafe`  | Does ArduPilot's GCS failsafe count our heartbeats (`SYSID_MYGCS`)?    | yes |
| `acks`      | Which COMMAND_ACKs does ArduPilot send for DO_SET_MODE, TAKEOFF and ARM? | yes |
| `reconnect` | After the link dies, does `is_up` drop, does `age()` grow, does it reconnect, and is CPU low? | no |
| `params`    | What is the real `WPNAV_SPEED_UP`, and does config match it?           | no (read-only) |

The checks that fly refuse to run on a serial device unless you pass `--i-know-this-is-sitl`.
`params` is read-only, so it is safe to run against the real Cube.

## SITL setup (all checks)

Run ArduPilot SITL **without MAVProxy**. Then each SITL serial port is its own TCP port, and
ArduPilot itself forwards between them, exactly as the Cube forwards between USB and TELEM2:

| SITL port | ArduPilot serial | Who connects                     |
|-----------|------------------|----------------------------------|
| 5760      | SERIAL0          | Mission Planner (the "GCS")      |
| 5762      | SERIAL1          | Link / these checks (the "Jetson") |
| 5763      | SERIAL2          | spare                            |

**Team setup: SITL with MAVProxy** (works for every check, and keeps the console and map):

```bash
Tools/autotest/sim_vehicle.py --custom-location=35.05932,-118.149,0,0 -v ArduCopter --console --map --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14553
```

MAVProxy takes 5760 (SERIAL0), and Mission Planner connects to MAVProxy's UDP 14550. ArduPilot
still serves SERIAL1 on TCP 5762, so the checks connect there by default, directly to ArduPilot
like TELEM2. Don't point the checks at 14553: that path goes through MAVProxy, and a UDP link can't
run `reconnect`.

MAVProxy is a second GCS (compid 230), and its heartbeats keep the GCS failsafe from firing. For
the `failsafe` check, start SITL with MAVProxy's heartbeat off. Add `-m "--heartbeat-rate 0"`:

```bash
Tools/autotest/sim_vehicle.py --custom-location=35.05932,-118.149,840,0 -v ArduCopter --console --map --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14553 -m "--heartbeat-rate 0"
```

Or type `set heartbeat 0` into MAVProxy itself, at the `STABILIZE>`/`GUIDED>` prompt in the
terminal where you ran `sim_vehicle.py`. Don't type it into the separate **"ArduCopter" xterm**
(that's the SITL program, not MAVProxy), the `--console` window, or the check's own prompt. Type
`set heartbeat` on its own to print the current value and confirm it's 0. Mission Planner's
heartbeats still keep the failsafe away during normal use.

**Option A: SITL in WSL without MAVProxy** (needs an ArduPilot checkout built for SITL):

```bash
cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy
```

Then connect Mission Planner on Windows over **TCP** to `127.0.0.1` port `5760`. WSL forwards
localhost. SITL doesn't start until something connects to 5760, so connect Mission Planner first.
If Mission Planner can't reach WSL, use MAVProxy as the GCS stand-in:
`mavproxy.py --master tcp:127.0.0.1:5760`. It also sends GCS heartbeats as sysid 255.

**Option B: Mission Planner's built-in simulator** (Simulation tab > Multirotor). Mission Planner
connects itself to 5760. From WSL, the checks must use the Windows host IP, for example
`--device tcp:$(ip route | awk '/default/{print $3}'):5762`. Windows Firewall may need to allow
`ArduCopter.exe`. (Not tried yet.)

Wait until Mission Planner shows GPS 3D fix / EKF OK (about 30 s after SITL starts) before running
the checks that fly. `arm` retries while pre-arm checks are still failing.

> **Position stream:** on SITL's SERIAL1, ArduPilot doesn't send `GLOBAL_POSITION_INT` unless that
> port asks for it, and Mission Planner's requests only apply to its own port. The checks send
> `SET_MESSAGE_INTERVAL` themselves (5 Hz). On the Cube, TELEM2 behaves the same unless the
> `SR2_*` parameters are set. `takeoff()` and `disarm()` need this stream.

---

## 1. `lock`: lock onto the Cube, not the GCS

```bash
python3 competition/drone_simulator/tools/sitl_link_check.py lock --device tcp:127.0.0.1:5762
```

Mission Planner must be connected to 5760 first, so ArduPilot is forwarding its heartbeat to 5762.

- **PASS:** `Ignoring heartbeat sysid=255 compid=190 type=6 ...` lines (or a `heartbeat source seen:
  sysid=255 ... MAV_TYPE_GCS` line), then `Locked onto sysid=1 compid=1 type=MAV_TYPE_QUADROTOR`.
- **INCONCLUSIVE:** no GCS heartbeat ever showed up on 5762, so the bug case wasn't exercised. Check
  that Mission Planner is connected to 5760, not 5762.
- Run it a few times. The old `wait_heartbeat()` code got this wrong about half the time, because it
  locked onto whichever heartbeat arrived first.

## 2. `failsafe`: are our heartbeats counted? (the `SYSID_MYGCS` / `MAV_GCS_SYSID` question)

```bash
python3 competition/drone_simulator/tools/sitl_link_check.py failsafe --enable-fs
```

1. It prints whichever of `SYSID_MYGCS` / `MAV_GCS_SYSID` this firmware has, plus `FS_GCS_ENABLE`
   and `FS_GCS_TIMEOUT`. `--enable-fs` sets `FS_GCS_ENABLE=1` (SITL only).
2. It takes off to 10 m in GUIDED, lists the other GCSs it can hear, then asks you to **stop every
   other GCS**. Press Disconnect in Mission Planner. If SITL was started with MAVProxy (a
   `compid=230` heartbeat source), also type `set heartbeat 0` at the `GUIDED>` prompt in the `sim_vehicle.py` terminal (not in the check's prompt, and not the `--console` window). Or start
   SITL with `--no-mavproxy` so this step isn't needed.
   After you press Enter, it waits up to 30 s for the other GCS heartbeats to actually stop. It
   prints `Still hearing: ...` with the ones that are still running.
3. **Phase 1:** with only our heartbeats (sysid 255) running, it watches for
   `FS_GCS_TIMEOUT` + 10 s. Expected: **no failsafe**. If any other GCS heartbeat is still
   arriving, the result is INCONCLUSIVE, because that GCS, not ours, could be keeping the failsafe
   away.
4. **Phase 2:** it stops our heartbeat thread (simulating the Jetson dying) and watches again.
   Expected: the failsafe fires, the mode switches to RTL/LAND, and a `GCS Failsafe` STATUSTEXT
   appears. This proves the failsafe was actually armed during phase 1.
5. Reconnect Mission Planner and let the vehicle land. Record the GCS id parameter name it printed
   in the `link.py` TODO.

Control run: `--source-system 254`. Now the failsafe should fire in phase 1, even though we keep
sending heartbeats. This confirms the sysid must match.

## 3. `acks`: what ArduPilot actually ACKs

```bash
python3 competition/drone_simulator/tools/sitl_link_check.py acks
```

It sends, in order:
1. DO_SET_MODE GUIDED
2. DO_SET_MODE with an invalid mode number 99
3. NAV_TAKEOFF while disarmed
4. arm + `takeoff()` to 5 m
5. LAND

At the end it prints an **ACK summary** of every ACK with its result and latency, plus a count of
IN_PROGRESS ACKs. Record:
- Is DO_SET_MODE answered with ACCEPTED, and does a bad mode come back as FAILED or DENIED?
- Is NAV_TAKEOFF answered with ACCEPTED straight away, or IN_PROGRESS first?
- Does disarmed takeoff come back as FAILED, DENIED or TEMPORARILY_REJECTED?

`commands.py` handles all of these. This check is to confirm the assumptions written into it.

## 4. `reconnect`: kill the link mid-run

A plain TCP link can't test this. When the other end closes, pymavlink reports "no data" instead of
raising, so `Link` never sees an error. (That's a known gap, see below.) Use the pty bridge instead.
It turns SITL's TCP port into a pseudo-serial device, and killing it behaves like unplugging a USB or
UART cable.

Terminal 1, the "cable":
```bash
python3 competition/drone_simulator/tools/pty_bridge.py --tcp 127.0.0.1:5762 --link /tmp/sitl-serial
```

Terminal 2:
```bash
python3 competition/drone_simulator/tools/sitl_link_check.py reconnect --device /tmp/sitl-serial --i-know-this-is-sitl --duration 60
```

About 10 s in, **Ctrl-C terminal 1**. Wait about 10 s, then start the bridge again. The check prints
`is_up`, `age(HEARTBEAT)`, `age(GLOBAL_POSITION_INT)` and the process CPU % once per second.

- **PASS:** one `Link ... down (SerialException ...)` error, ages climbing while down, CPU in single
  digits, then `recovered after N attempt(s)` and `age(HEARTBEAT)` back under 1 s.
- Also note `position stream resumed:` in the result. If it says False, the vehicle forgot the
  `SET_MESSAGE_INTERVAL` request across the reconnect, and Link will have to re-request streams
  after reconnecting.

Known gap (optional to see): run `reconnect` straight on `tcp:127.0.0.1:5762` and kill SITL.
`is_up` stays True, pymavlink prints `EOF on TCP socket` over and over, and CPU goes up. The real
UART/USB path raises errors properly; TCP/UDP links need separate handling.

## 5. `params`: put the real climb rate in config

```bash
# against the real Cube (read-only, safe): the value that matters for flight
python3 competition/drone_simulator/tools/sitl_link_check.py params --device /dev/ttyTHS1
# or against SITL, to see the default
python3 competition/drone_simulator/tools/sitl_link_check.py params
```

It downloads the full parameter list, prints every climb, takeoff and GCS-id parameter, then picks
the waypoint climb-rate parameter. That's `WPNAV_SPEED_UP` (cm/s) on older firmware. Our SITL build
doesn't have that name (the same rename wave turned `SYSID_MYGCS` into `MAV_GCS_SYSID`), so any
`WP*_SPEED_UP` / `WP*_SPD_UP` is accepted. Values above 20 are read as cm/s, anything lower as m/s.
Check that guess against the parameter's units in Mission Planner. It compares the result with
`climb_rate_m_s` in `config/mission_params.py` and, on a mismatch, prints the exact line to use.
`--grep REGEX` prints any other parameters you want.

Use the Cube's value, not SITL's (SITL just has the default). The download takes a few seconds over
TCP and longer over the UART. Run it on the Cube to also see which GCS-id name its firmware uses.
