import cv2
import numpy as np
import math
import time

def main():
    # Make sure this index matches the one that worked! (0, 1, or 2)
    cap = cv2.VideoCapture(0) 
    
    if not cap.isOpened():
        print("[VISION] Error: Could not open webcam.")
        return

    print("Visual Debug Mode Active. Press 'q' on your keyboard to quit.")
    
    # Used to limit terminal spam to 2 prints per second
    last_print_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        red_mask = mask1 + mask2

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_locked = False
        cX, cY = None, None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * math.pi * (area / (perimeter * perimeter))
                    if 0.7 < circularity < 1.2:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            target_locked = True
                            
                            # Draw the red bounding box
                            x, y, w, h = cv2.boundingRect(contour)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                            
                            # Draw the center dot and coordinates
                            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                            cv2.putText(frame, f"({cX}, {cY})", (cX - 25, cY - 25), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            
                            # Break after finding the first valid target so we don't draw duplicates
                            break 

        # Print to terminal twice a second
        current_time = time.time()
        if current_time - last_print_time >= 0.5:
            if target_locked:
                print(f"[LOCKED] Target Acquired at Pixel: ({cX}, {cY})")
            else:
                print("[SEARCHING] No target detected...")
            last_print_time = current_time

        # Show the video feed
        cv2.imshow('Vision Debug', frame)
        
        # Press 'q' to safely close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
