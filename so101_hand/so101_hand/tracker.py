"""MediaPipe Hand Landmarker on video frames: the only module of so101_hand that imports MediaPipe (ADR 0002)."""

import os
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

DEFAULT_MODEL = Path(os.environ.get("DATA_DIR", "~/data")).expanduser() / "models" / "hand_landmarker.task"


class HandTracker:
    """Image landmarks of one hand per frame, in pixels of the raw (not mirrored) frame.

    VIDEO mode: the hand is tracked from frame to frame and the slower palm detector runs only when the track is lost.
    Handedness is ignored: in the side view it flickers between "Left" and "Right" (ROB-19).
    """

    def __init__(self, model_path: Path = DEFAULT_MODEL):
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"Hand Landmarker model not found: {model_path} (see the README)")
        vision = mp.tasks.vision
        options = vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._last_ms = -1

    def detect(self, frame_bgr: np.ndarray, t_s: float) -> np.ndarray | None:
        """21 landmarks with x and y in pixels, shape (21, 2), or None without a hand. `t_s`: frame time in seconds."""
        t_ms = max(int(t_s * 1000), self._last_ms + 1)  # VIDEO mode needs strictly increasing timestamps
        self._last_ms = t_ms
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        result = self._landmarker.detect_for_video(image, t_ms)
        if not result.hand_landmarks:
            return None
        height, width = frame_bgr.shape[:2]
        return to_pixels(result.hand_landmarks[0], width, height)

    def close(self) -> None:
        self._landmarker.close()

    def __enter__(self) -> "HandTracker":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def to_pixels(landmarks, width: int, height: int) -> np.ndarray:
    """MediaPipe's normalised landmarks (x, y as fractions of the image width and height) to pixels."""
    return np.array([(p.x * width, p.y * height) for p in landmarks])
