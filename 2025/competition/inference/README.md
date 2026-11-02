# /inference/ : Computer Vision Pipeline

This directory contains the optimized, edge-compute-ready vision stack designed to run in real-time on the Jetson Orin Nano’s GPU[cite: 1].

* **`camera.py`:** The primary ingestion interface, handling hardware-level camera stream initialization, frame buffer management, and resolution scaling[cite: 1].
* **`inference.py`:** The active AI orchestration loop. It streams frames from the camera, passes them through the YOLOv11 model, and extracts bounding box coordinates for ground target identification[cite: 1].
* **`yolo11m.engine`:** The high-performance TensorRT-compiled model. This is the production-ready binary optimized specifically for the Jetson Orin Nano’s architecture to maximize inference frames per second[cite: 1].
* **`yolo11m.onnx` & `yolo11m.pt`:** The intermediate Open Neural Network Exchange file and original PyTorch weights, preserved for model retraining and pipeline validation[cite: 1].
* **`yoloExportTensor.py`:** A utility script used to bridge the gap between standard training weights and production inference; it handles the architectural conversion to TensorRT format[cite: 1].
* **`HoughCircleMain.py` & `HoughCircleTuning.py`:** The legacy, non-ML fallback vision system. These utilize classical OpenCV edge detection to mathematically trace circular bullseye patterns, ensuring target detection could continue if the primary ML model failed under unique lighting conditions[cite: 1].