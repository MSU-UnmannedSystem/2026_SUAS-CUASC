# /jetson_rescue/ : Flight Line Hardware Debugging

This is the "emergency" directory. It contains raw, isolated hardware-interrogation scripts used on the Mojave flight line to bypass the main software stack when the drone was unresponsive or when hardware firewalls blocked MAVLink commands[cite: 1].

* **`camera.py` & `inference.py`:** Stripped-down versions of the primary vision pipeline, used purely for sterile environment testing on the equipment stack to confirm GPU status without the drone's logic attached[cite: 1].
* **`debug_vision.py` & `test_vision.py`:** Rapid-response scripts for verifying camera stream validity and confirming the TensorRT engine was loading successfully during flight line downtime[cite: 1].
* **`gpio.py` & `gpio_version.py`:** Low-level hardware overrides. Used to directly toggle the Jetson's physical header pins, verifying that the payload-drop servos were receiving power and responding to physical signals[cite: 1].
* **`geolocation.py`:** A standalone utility for validating the math that maps pixel-space coordinates to geographic waypoints, tested in isolation from the drone's flight controller[cite: 1].
* **`requirements_backup.txt.txt`:** A last-ditch environment manifest generated on-site, capturing the `pip freeze` state to allow for full software re-installation if the Jetson's environment were to corrupt in the desert[cite: 1].