import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("mediapipe")  # optional dependency: pip install -e ".[tracker]"

import cv2  # noqa: E402

from so101_hand.tracker import DEFAULT_MODEL, HandTracker, to_pixels, to_world  # noqa: E402

needs_model = pytest.mark.skipif(not DEFAULT_MODEL.is_file(), reason=f"model not found: {DEFAULT_MODEL}")
BLACK = np.zeros((480, 640, 3), np.uint8)
# ROB-34's recording from the front: the hand closed for 2 s, then opened, in turns (local data, not in git)
GRIP = Path(os.environ.get("DATA_DIR", "~/data")).expanduser() / "hand_depth_spike" / "grip.avi"


def test_normalised_landmarks_are_scaled_to_pixels():
    # MediaPipe gives x and y as fractions of the image width and height
    landmarks = [SimpleNamespace(x=0.5, y=0.25, z=0.1)] * 21
    pixels = to_pixels(landmarks, width=640, height=480)
    assert pixels.shape == (21, 2)
    assert pixels[0] == pytest.approx([320.0, 120.0])


def test_world_landmarks_keep_metres_and_depth():
    landmarks = [SimpleNamespace(x=0.01, y=-0.02, z=0.03)] * 21
    world = to_world(landmarks)
    assert world.shape == (21, 3)
    assert world[0] == pytest.approx([0.01, -0.02, 0.03])


def test_missing_model_gives_a_clear_error(tmp_path):
    with pytest.raises(FileNotFoundError, match="Gesture Recognizer"):
        HandTracker(tmp_path / "gesture_recognizer.task")


@needs_model
def test_frame_without_a_hand_gives_none():
    with HandTracker(DEFAULT_MODEL) as tracker:
        assert tracker.detect(BLACK, t_s=0.0) is None


@needs_model
def test_timestamps_that_do_not_increase_are_accepted():
    # MediaPipe's VIDEO mode raises on a timestamp that is not larger than the previous one
    with HandTracker(DEFAULT_MODEL) as tracker:
        for t_s in (1.0, 1.0, 0.5):
            assert tracker.detect(BLACK, t_s) is None


@needs_model
@pytest.mark.skipif(not GRIP.is_file(), reason=f"recording not found: {GRIP}")
def test_the_closed_hand_of_a_recording_is_a_fist():
    cap, gestures = cv2.VideoCapture(str(GRIP)), []
    with HandTracker(DEFAULT_MODEL) as tracker:
        for i in range(20):  # the first second: the hand closed
            ok, frame = cap.read()
            hand = tracker.detect(frame, i / 20) if ok else None
            gestures.append(hand.gesture if hand else "no hand")
    assert gestures.count("Closed_Fist") >= 15
