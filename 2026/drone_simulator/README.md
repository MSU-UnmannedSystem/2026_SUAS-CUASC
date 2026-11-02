# /drone_simulator/ : Phase 2 Active Edge-Computing Payload

**Architecture Status:** Archived (Pre-Pivot Flight Stack)
**Target Hardware:** NVIDIA Jetson Orin Nano $\rightarrow$ Cube Orange (ArduPilot)
**Communication Protocol:** MAVLink 2.0 via `pymavlink`

## 📖 System Overview
This directory houses the complete Phase 2 monolithic active flight architecture. Originally designed to run live on the Jetson Orin Nano, this codebase represents the transition from modular desk-testing scripts into a fully automated edge-computing payload. It was engineered to maintain a direct 3-wire serial UART bridge (`/dev/ttyTHS1` to `TELEM2` at `921600` baud) to dynamically command ArduPilot through the entire C-UASC Mojave mission sequence.

---

## 🗂️ Directory Breakdown

### `/vehicle/`
This is the core execution and hardware bridging layer. It is responsible for establishing the physical MAVLink connection, negotiating the handshake protocols with the Cube Orange, and orchestrating the overarching flight sequence. During the "Naked Cube" testing phase, this directory was utilized heavily to bypass ArduPilot's pre-arm safety firewalls (`ARMING_CHECK = 0`, `BRD_SAFETY_DEFLT = 0`) to verify logic on the bench.

* **`main.py`:** The monolithic master orchestrator. This script combined all of the isolated mission routes into a single continuous execution chain. When triggered via SSH over the field router, it fully automated the entire competition run: Handshake $\rightarrow$ Arm $\rightarrow$ Execute Cube Drop (Pin 6) $\rightarrow$ Execute Ball Drop (Pin 7) $\rightarrow$ Waypoint Navigation Circuit $\rightarrow$ RTL.
* **`connect_drone.py`:** The foundational `pymavlink` and `dronekit` wrapper. It handles the initial serial binding, establishes the telemetry pipeline, and runs the critical `wait_heartbeat()` loop seeking the `MAV_TYPE_QUADROTOR` confirmation before allowing script execution.
* **`barebones_test.py`:** A stripped-down execution script used for isolated desk testing, allowing the team to force mode changes without invoking the heavy logic of `main.py`.
* **`table_test.py`:** A hardware verification tool used to fire MAVLink commands and verify PWM signal output (e.g., `ch6out` jumping to 1900) via Mission Planner’s digital oscilloscope without spinning the physical motors.

### `/missions/`
This directory contains the highly specific operational logic blocks for every C-UASC task. These scripts handle the active `GUIDED` and `AUTO` mode transitions, strict positional telemetry polling (ensuring the drone does not violate the 6-foot penalty boundaries), and the execution of specific payload actuation commands.

* **`mission_base.py`:** The parent class architecture. It establishes the standard logging, state tracking, and MAVLink initialization protocols that every other specific mission module inherits.
* **`package_delivery.py`:** Executes the active flight logic for the primary Box drop. It commands a descent below the 6-foot threshold and fires `MAV_CMD_DO_SET_SERVO` to Pin 6 to release the mechanism.
* **`payload_drop.py`:** Executes the secondary Ball drop logic. It manages the climb to a safe transit altitude, approaches the secondary target, and fires `MAV_CMD_DO_SET_SERVO` to Pin 7.
* **`waypoint_nav.py`:** The routing engine for the circuit. It clears the existing mission memory (`MAV_CMD_MISSION_CLEAR_ALL`) and uploads the competition coordinates as an array of `MISSION_ITEM` messages utilizing `MAV_FRAME_GLOBAL_RELATIVE_ALT`.
* **`circuit_trial.py`:** The specific logic handling the timed circuit run configuration.
* **`object_localization.py`:** The mathematical bridge script. It is designed to take a 2D pixel bounding box from the YOLO vision model and project it onto the 3D ground plane using the drone's active telemetry (altitude and heading) to generate a physical target GPS coordinate.

### `/config/`
This is the centralized configuration matrix. It isolates all hardcoded variables from the active logic scripts, allowing for rapid parameter tuning on the flight line without risking syntax errors deep within the operational code.

* **`mission_params.py`:** The master configuration dictionary. It stores the static Mojave target coordinates, safe transit altitude ceilings, minimum descent thresholds, and the crucial PWM actuation values required to trigger the servos.