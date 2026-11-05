"""The copied VisionPipeline, without a real camera (skipped if OpenCV isn't installed)."""

import threading
import time

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from mission_controller.vision.camera import VisionPipeline  # noqa: E402
from mission_controller.vision.detector import TargetDetector  # noqa: E402


def pipeline_with_frame(frame):
    """A VisionPipeline that never opened a camera, holding one captured frame."""
    vp = object.__new__(VisionPipeline)
    vp.target_type = "red_bullseye"
    vp.lock = threading.Lock()
    vp.ret = True
    vp.frame = frame
    vp.frame_time = time.monotonic()
    vp.running = False
    return vp


def test_detects_red_disc_in_full_resolution_coordinates():
    frame = np.zeros((1080, 1920, 3), np.uint8)
    cv2.circle(frame, (1400, 300), 80, (0, 0, 255), -1)       # BGR red
    det = pipeline_with_frame(frame).detect()
    assert isinstance(pipeline_with_frame(frame), TargetDetector)
    assert det is not None
    assert (det.width_px, det.height_px) == (1920, 1080)
    assert abs(det.x_px - 1400) < 10 and abs(det.y_px - 300) < 10


def test_no_target_and_no_frame():
    assert pipeline_with_frame(np.zeros((480, 640, 3), np.uint8)).detect() is None
    vp = pipeline_with_frame(None)
    vp.ret = False
    assert vp.detect() is None
