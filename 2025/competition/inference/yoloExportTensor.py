from ultralytics import YOLO

# Load a YOLO PyTorch model
model = YOLO("yolo11m.pt")

# Export the model to TensorRT
model.export(format="engine")

# Load the exported TensorRT model
trt_model = YOLO("yolo11m.engine")

# Run inference
results = trt_model("https://ultralytics.com/images/bus.jpg")

