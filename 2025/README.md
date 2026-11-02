# C-UASC 2026: MSU Unmanned Systems
**Campaign:** Mojave Flight Line
**Architecture Status:** Post-Flight Forensics & Scoring Suite
**Official Performance Records:** [C-UASC 2026 Official Scoring Spreadsheet](https://docs.google.com/spreadsheets/d/1VuGpZPVMiQJwbJ27cN-Mkd64FEsq-SwYX0pbtUBadOQ/edit?usp=sharing)

---

## 📖 Project Overview
This repository contains the complete software architecture for the Michigan State University Unmanned Systems 2026 C-UASC campaign. 

Due to hardware-in-the-loop firewalls and intense flight line constraints in the Mojave, the architecture underwent a massive operational pivot during the competition. The original active edge-computing scripts—designed to run autonomously on an onboard Jetson Orin Nano via live MAVLink injection—were shelved. The final operational software is a highly optimized, automated post-flight forensics suite designed to parse ArduPilot DataFlash (`.bin`) logs and dynamically calculate official judge scoring matrices.

---

## 📊 Results & Standings
With a highly consistent performance across all operational categories, the team secured an **Average Categorical Finish of 4.75**. The breakdown of individual mission outcomes is detailed below:

| Mission Event | Placement | Operational Highlights |
| :--- | :---: | :--- |
| **Package Delivery (Box)** | **3rd Place** | Flawless mechanism actuation and delivery accuracy. |
| **Circuit Time Trial** | **3rd Place** | Completed the course with a true flight duration of 1m 40s. |
| **Waypoint Navigation** | **4th Place** | Max error of 0.55m across the 10-point scoring pool. |
| **Package Drop (Ball)** | **9th Place** | Scored a direct 11-foot proximity target hit on the field. |

### Flight Documentation & Leaderboards
Visual proof of flight metrics, telemetry configurations, and official standings tables are archived directly within the repository root:

![Package Delivery](results_images/package_delivery.png)
![Time Trial](results_images/time_trial_results.png)
![Waypoint Navigation](results_images/waypoint_nav_results.png)
![Payload Drop](results_images/payload_drop.png)

---

## 🗂️ Repository Structure
The repository is structured to preserve the historical engineering phases of the project while maintaining the current baseline data:

* `/drone_simulator/`: Contains the Phase 2 monolithic active flight codebase. This was the master payload designed to sit on the Jetson Orin Nano.
* `/inference/`: Computer vision pipelines and trained YOLOv11 models utilized for target ID and localization.
* `/legacy_flight_architecture/`: (Archived) Contains the offline `/train` datasets, the Phase 1 CLI modular test scripts (`logger.py`, `package_delivery.py`), and hardware rescue tools (`jetson_rescue/`).
* `master_scorer.py`: The final, unified forensic scoring engine.

---

## 🛠️ The Technology Stack
Regardless of the architectural pivots, the foundational communication and computation layers remained consistent:
* **Flight Controller:** Cube Orange running ArduPilot.
* **Companion Hardware (Pre-Pivot):** NVIDIA Jetson Orin Nano.
* **Core Language:** Python 3.
* **Translation Engine:** `pymavlink` (wrapping Python logic into standardized MAVLink 2 message packets).
* **Data Bridge:** Direct 3-wire serial connection (TX/RX/GND) mapping `/dev/ttyTHS1` to `TELEM2` at a strict `921600` Baud Rate.

---

## 🏛️ Architectural Evolution (The Lore)

### Phase 1: The Distributed Pre-Comp Codebase
Before hardware consolidation, the stack was a modular system designed for laptop-tethered testing. It utilized a Command Line Interface (CLI) prompt in `main.py` to manually trigger individual mission phases (`package_delivery.py`, `ball_drop.py`, `waypoint_nav.py`) while a parallel `logger.py` script passively polled MAVLink telemetry to ensure safe groundspeed and GPS locks.

### Phase 2: Monolithic Active Edge-Computing
The modular scripts were fused into a single `main.py` master script housed on the Jetson Orin Nano. The network topology evolved from a hardwired Ethernet debugging tether to a field-mounted wireless router, allowing the ground station to SSH into the Jetson and trigger the fully autonomous sequence: Handshake $\rightarrow$ Arm $\rightarrow$ Box Drop (Pin 6 PWM) $\rightarrow$ Ball Drop (Pin 7 PWM) $\rightarrow$ Waypoint Circuit $\rightarrow$ RTL.

### The "Naked Cube" Hardware Firewalls
Bench-testing the Jetson against a stationary Cube Orange triggered violent resistance from ArduPilot’s pre-flight safety logic. The team executed parameter surgery to force the MAVLink bridge open:
* `BRD_SAFETY_DEFLT = 0` (Bypassed missing physical Here3 switch)
* `ARMING_CHECK = 0` (Blinded pre-arm sensor arrays)
* `DISARM_DELAY = 0` (Killed the 3-second zero-throttle timeout)
* `SERIAL2_OPTIONS = 0` & `BRD_SER2_RTSCTS = 0` (Shattered the "One-Way Glass" protocol, forcing ArduPilot to accept 3-wire UART by ignoring missing RTS/CTS handshakes).

### Phase 3: Post-Flight Forensics (The Pivot)
On the flight line, the Jetson was removed from the airframe. The team pivoted into a rapid-response DataFlash parsing unit. The final codebase, `master_scorer.py`, was engineered to ingest raw `.bin` logs and automatically output competition-ready artifacts.

---

## ⚙️ The Math Engine
The final forensics suite does not rely on simple Cartesian mapping. It utilizes a custom Haversine-lite spherical distance algorithm to process ArduPilot's `Lat`, `Lng`, and `RelHomeAlt` telemetry fields:

$$Distance = \sqrt{\Delta X^2 + (\Delta Y \cdot \cos(lat))^2 + \Delta Z^2}$$

This equation actively accounts for the curvature of the Earth as longitude lines compress, mapping the drone's true 3D spatial deviation from the Mojave scoring waypoints in meters.

### Flight Line Algorithm Patches
* **Waypoint Navigation:** Overhauled from a 70-point static ceiling to the official **10-point total pool** algorithm, applying a strict 1-point-per-meter deduction over a 1m radius boundary. 
* **Time Trial "Early Cross" Patch:** Patched a critical early-termination bug where the drone clipped the 4-meter boundary of Waypoint 5 mid-maneuver. Adjusted the array index from `valid_ends[0]` to `valid_ends[-1]` to capture the final exit microsecond, revealing the true 1 minute 40-second runtime.