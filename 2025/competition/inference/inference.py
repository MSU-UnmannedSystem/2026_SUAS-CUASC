import time
import cv2
from ultralytics import YOLO

# Camera object that will init later
camera = None

# Constant
CONFIDENCE_THRESHOLD = 0.5
FRAME_WIDTH, FRAME_HEIGHT = 640, 360
FPS_CALC_INTERVAL_SEC = 1
SCREENSHOT_INTERVAL_SEC = 5
SHOW_INFERENCE_FRAME = True
PRINT_INFERENCE_TERMINAL = False
INIT_CAMERA_ATTEMPT = 10
IS_CENTER_TOLERANCE = 0.50

def at_center(bbox: list):
    x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
    center_x = (x1 + abs(x2 - x1)) / FRAME_WIDTH
    center_y = (y1 - abs(y2 - y1)) / FRAME_HEIGHT
    x_true = abs(center_x - 0.5) <= IS_CENTER_TOLERANCE
    y_true = abs(center_y - 0.5) <= IS_CENTER_TOLERANCE
    return x_true and y_true

def main():	
    # Load model using pure CPU
    # model = YOLO("model/yolov9t.pt")
    
    # Load model using Coral Accelerator
    model = YOLO("yolo11m.engine", task="detect")
       
    print("\nStatus:\tModel Loaded")

    # Setup camera with opencv
    global camera
    for attempt in range(INIT_CAMERA_ATTEMPT):
        print("Status:\tStarting Camera Attempt {} / {}".format(attempt, INIT_CAMERA_ATTEMPT))
        
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        
        try:
            _, frame = camera.read()
            break
        except:
            camera.release()
            camera = None
            time.sleep(2)

    print("Status:\tCamera On")
    print("Frame:\t{}x{}\n".format(FRAME_WIDTH, FRAME_HEIGHT))
    print("Ctrl + C to Quit\n")

    loop_counter = current_time = fps = class_id = confidence = 0
    label_prev = class_label = ""
    annotated_frame = None
    bbox = bboex_format = []
    start_time1 = start_time2 = time.time()
    
    while True:
        _, frame = camera.read()
        # frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        results = model(frame, verbose = False, conf = CONFIDENCE_THRESHOLD)
        
        if cv2.waitKey(1) == ord('q'):
            break        

        if len(results[0].boxes) == 0:
            class_id = -1
            class_label = "Nothing"
            confidence = -1
            bboex_format = [-1, -1, -1, -1]

        # Extract detection result from 1st object
        for result in results[0].boxes:
            class_id = int(result.cls)
            class_label = results[0].names[class_id]
            confidence = int(result.conf * 100)
            bbox = result.xyxy[0].tolist()
            bboex_format = [int(bbox[0]), int(bbox[2]), int(bbox[2]), int(bbox[3])]
            break
            
        # Show annotated frame with fps
        annotated_frame = results[0].plot()
        cv2.putText(img         = annotated_frame, 
                    text        = "FPS: " + str(fps),
                    org         = (25, 25),
                    fontFace    = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale   = 0.5,
                    thickness   = 1,
                    color       = (255, 0, 0))
		
        if SHOW_INFERENCE_FRAME:
            cv2.imshow("Inference result", annotated_frame)

        # Calculate fps
        current_time = time.time()
        if (current_time - start_time1) >= FPS_CALC_INTERVAL_SEC:
            fps = loop_counter
            start_time1 = current_time
            loop_counter = 0

        loop_counter += 1

    print("\nStatus:\tCamera Off")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        camera.release()
        cv2.destroyAllWindows()
        print("\nStatus:\tCamera Off")

