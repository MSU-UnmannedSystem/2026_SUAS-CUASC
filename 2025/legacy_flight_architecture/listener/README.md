# /listener/ : Telemetry Logging Tools

This folder contains legacy scripts utilized during the development phase to monitor drone telemetry without interfering with active flight control.

* **`listener.py` & `Logger.py`:** Active background listeners designed to poll MAVLink `GLOBAL_POSITION_INT` and `VFR_HUD` messages.
* **`test_logger.py`:** A verification script to ensure the telemetry buffer was parsing correctly before deployment to the Jetson Orin Nano.