"""Red-bullseye / square detector, copied from 2025/competition/jetson_rescue/camera.py.

Detection code (_find_red_bullseye, _find_bw_square) is unchanged. Changes for
flight use: it implements TargetDetector, records when each frame was captured
(so a frozen camera reads as "no target" instead of a stale target), reports
the real frame size (the capture resolution is not set, so it is whatever the
camera defaults to), logs instead of printing, and close() is idempotent.
"""

import logging
import cv2
import numpy as np
import math
import threading
import time
from typing import Optional

from mission_controller.vision.detector import Detection, TargetDetector

logger = logging.getLogger(__name__)

class VisionPipeline(TargetDetector):
    def __init__(self, camera_index=0, target_type='red_bullseye'):
        self.cap = cv2.VideoCapture(camera_index)
        self.target_type = target_type
        
        if not self.cap.isOpened():
            logger.error("[VISION] Error: Could not open webcam.")
            
        self.ret = False
        self.frame = None
        self.frame_time = None  # time.monotonic() when self.frame was captured
        self.running = True
        self.lock = threading.Lock()
        
        # Start the background daemon thread
        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()
        logger.info("[VISION] Daemon thread active. Mode: %s", self.target_type.upper())

    def _update_frame(self):
        """Continuously flushes the hardware buffer to grab the freshest frame."""
        while self.running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                with self.lock:
                    self.ret = ret
                    self.frame = frame
                    if ret:
                        self.frame_time = time.monotonic()
            # FIX: Prevent thread starvation / CPU pegging on the Jetson
            time.sleep(0.01)

    def get_target_pixel(self):
        """Safely pulls the newest frame and routes to the correct math engine."""
        with self.lock:
            if not self.ret or self.frame is None:
                return None, None
            frame = self.frame.copy()

        # The Router
        if self.target_type == 'red_bullseye':
            return self._find_red_bullseye(frame)
        elif self.target_type == 'bw_square':
            return self._find_bw_square(frame)
            
        return None, None

    def detect(self) -> Optional[Detection]:
        """TargetDetector API: the target in the newest frame, with frame size and capture time."""
        with self.lock:
            if not self.ret or self.frame is None:
                return None
            frame = self.frame.copy()
            frame_time = self.frame_time
        if self.target_type == 'red_bullseye':
            x, y = self._find_red_bullseye(frame)
        elif self.target_type == 'bw_square':
            x, y = self._find_bw_square(frame)
        else:
            return None
        if x is None or y is None:
            return None
        height, width = frame.shape[:2]
        return Detection(x, y, width, height, frame_time)

    def _find_red_bullseye(self, frame):
        """Mission: Package Drop & Package Delivery"""
        # FIX: Downsample immediately to eliminate compute bottleneck
        orig_h, orig_w = frame.shape[:2]
        proc_w, proc_h = 640, 480
        resized = cv2.resize(frame, (proc_w, proc_h))

        # FIX: Implement CLAHE to neutralize Mojave specular sun glare
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v_clahe = clahe.apply(v)
        hsv_clahe = cv2.merge((h, s, v_clahe))

        # Red hue wraps around the HSV spectrum
        mask1 = cv2.inRange(hsv_clahe, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv_clahe, np.array([170, 120, 70]), np.array([180, 255, 255]))
        red_mask = mask1 + mask2

        # Cleanup noise
        kernel = np.ones((5, 5), np.uint8)
        cleaned = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(cleaned, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100: continue # Lowered threshold due to 640x480 scaling
            
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0: continue
            
            circularity = 4 * math.pi * (area / (perimeter * perimeter))
            if 0.7 < circularity < 1.2:
                M = cv2.moments(contour)
                if M["m00"] != 0: 
                    px_x = int(M["m10"] / M["m00"])
                    px_y = int(M["m01"] / M["m00"])
                    # Scale coordinates back to native 1080p resolution for geolocation
                    return int(px_x * (orig_w / proc_w)), int(px_y * (orig_h / proc_h))
        return None, None

    def _find_bw_square(self, frame):
        """Mission: Target Localization (GCPs)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((5, 5), np.uint8)
        cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(cleaned, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                
                if len(approx) == 4:
                    M = cv2.moments(contour)
                    if M["m00"] != 0: 
                        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        return None, None

    def release(self):
        """Safely kills the thread and releases the hardware."""
        if not self.running:
            return
        self.running = False
        self.thread.join()
        self.cap.release()
        logger.info("[VISION] Hardware released cleanly.")

    def close(self):
        """TargetDetector API."""
        self.release()