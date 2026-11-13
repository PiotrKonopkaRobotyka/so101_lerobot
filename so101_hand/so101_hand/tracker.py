"""MediaPipe Gesture Recognizer on video frames: the only module of so101_hand that imports MediaPipe (ADR 0002).

The recognizer runs MediaPipe's Hand Landmarker and classifies the whole hand into canned gestures (ADR 0008).
"""

import os
from pathlib import Path
from typing import NamedTuple

import cv2
import mediapipe as mp
import numpy as np

DEFAULT_MODEL = Path(os.environ.get("DATA_DIR", "~/data")).expanduser() / "models" / "gesture_recognizer.task"


class Hand(NamedTuple):
    image: np.ndarray  # (21, 2): x and y in pixels of the raw frame
    world: np.ndarray  # (21, 3): metres, origin near the hand's centre (MediaPipe "world landmarks")
    gesture: str = "None"  # the whole hand: "Closed_Fist", "Open_Palm", "Thumb_Up", ... or "None"
    score: float = 0.0  # the gesture's confidence, 0..1


class HandTracker:
    """One hand per frame: its landmarks in pixels of the raw (not mirrored) frame and in metres, and its gesture.

    VIDEO mode: the hand is tracked from frame to frame and the slower palm detector runs only when the track is lost.
    Handedness is ignored: in the side view it flickers between "Left" and "Right" (ROB-19).
    """

    def __init__(self, model_path: Path = DEFAULT_MODEL):
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"Gesture Recognizer model not found: {model_path} (see the README)")
        vision = mp.tasks.vision
        options = vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
        )
        self._recognizer = vision.GestureRecognizer.create_from_options(options)
        self._last_ms = -1

    def detect(self, frame_bgr: np.ndarray, t_s: float) -> Hand | None:
        """The hand's 21 landmarks and its gesture, or None without a hand. `t_s`: frame time in seconds."""
        t_ms = max(int(t_s * 1000), self._last_ms + 1)  # VIDEO mode needs strictly increasing timestamps
        self._last_ms = t_ms
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        result = self._recognizer.recognize_for_video(image, t_ms)
        if not result.hand_landmarks:
            return None
        height, width = frame_bgr.shape[:2]
        top = result.gestures[0][0] if result.gestures and result.gestures[0] else None
        return Hand(
            to_pixels(result.hand_landmarks[0], width, height),
            to_world(result.hand_world_landmarks[0]),
            top.category_name if top else "None",
            float(top.score) if top else 0.0,
        )

    def close(self) -> None:
        self._recognizer.close()

    def __enter__(self) -> "HandTracker":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def to_pixels(landmarks, width: int, height: int) -> np.ndarray:
    """MediaPipe's normalised landmarks (x, y as fractions of the image width and height) to pixels."""
    return np.array([(p.x * width, p.y * height) for p in landmarks])


def to_world(landmarks) -> np.ndarray:
    """MediaPipe's world landmarks (already in metres) as an array."""
    return np.array([(p.x, p.y, p.z) for p in landmarks])
