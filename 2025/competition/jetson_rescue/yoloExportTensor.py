from ultralytics import YOLO

# Load a YOLO PyTorch model
model = YOLO("last.pt")

# Export the model to TensorRT
model.export(format="engine")

# Load the exported TensorRT model
# trt_model = YOLO("bullseye_v1.engine")

# Run inference
# results = trt_model("https://ultralytics.com/images/bus.jpg")

