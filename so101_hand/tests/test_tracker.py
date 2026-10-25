from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("mediapipe")  # optional dependency: pip install -e ".[tracker]"

from so101_hand.tracker import DEFAULT_MODEL, HandTracker, to_pixels  # noqa: E402

needs_model = pytest.mark.skipif(not DEFAULT_MODEL.is_file(), reason=f"model not found: {DEFAULT_MODEL}")
BLACK = np.zeros((480, 640, 3), np.uint8)


def test_normalised_landmarks_are_scaled_to_pixels():
    # MediaPipe gives x and y as fractions of the image width and height
    landmarks = [SimpleNamespace(x=0.5, y=0.25, z=0.1)] * 21
    pixels = to_pixels(landmarks, width=640, height=480)
    assert pixels.shape == (21, 2)
    assert pixels[0] == pytest.approx([320.0, 120.0])


def test_missing_model_gives_a_clear_error(tmp_path):
    with pytest.raises(FileNotFoundError, match="hand_landmarker"):
        HandTracker(tmp_path / "hand_landmarker.task")


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
