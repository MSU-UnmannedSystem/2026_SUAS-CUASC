"""The interface guidance uses to ask "where is the target in the image?".

Today's implementation is the copied red-bullseye VisionPipeline
(vision/camera.py). A YOLO detector can replace it by implementing the same
two methods; nothing in guidance needs to change.
"""

from typing import NamedTuple, Optional


class Detection(NamedTuple):
    """One target detection.

    Attributes:
        x_px: Target column in the full-resolution frame, 0 = left edge.
        y_px: Target row in the full-resolution frame, 0 = top edge (the nose).
        width_px: Width of the frame the detection came from.
        height_px: Height of the frame the detection came from.
        captured_at: ``time.monotonic()`` when the frame was captured, in seconds.
    """

    x_px: float
    y_px: float
    width_px: int
    height_px: int
    captured_at: float


class TargetDetector:
    """Base class for target detectors."""

    def detect(self) -> Optional[Detection]:
        """Returns the target in the newest frame, or None if it is not seen."""
        raise NotImplementedError

    def close(self) -> None:
        """Releases the camera / model. Safe to call more than once."""
