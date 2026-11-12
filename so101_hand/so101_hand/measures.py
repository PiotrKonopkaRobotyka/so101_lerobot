"""Hand measures from MediaPipe landmarks: the openness measure chosen in the ROB-19 spike and the palm's centre."""

import numpy as np

WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP = 0, 4, 8, 9
PALM = (WRIST, 5, MIDDLE_MCP, 13, 17)  # the wrist and the four knuckles (MCP joints): the part that does not curl


def aperture(landmarks: np.ndarray) -> float:
    """Thumb tip to index tip distance divided by palm length (wrist to middle-finger MCP).

    `landmarks` has shape (21, 2) or (21, 3) in pixels; only x and y are used. Pixels, not MediaPipe's normalised
    coordinates: those scale x by the image width and y by the height, which distorts distances.
    Dividing by the palm length makes the value independent of the hand's distance to the camera.
    """
    xy = _image_xy(landmarks)
    return float(np.linalg.norm(xy[THUMB_TIP] - xy[INDEX_TIP]) / _palm_px(xy))


def palm_centre(landmarks: np.ndarray) -> np.ndarray:
    """Mean of the wrist and the knuckles, (x, y) in pixels: where the hand is (ADR 0006).

    The knuckles do not curl, so closing the hand hardly moves this point."""
    return _image_xy(landmarks)[list(PALM)].mean(axis=0)


def _image_xy(landmarks) -> np.ndarray:
    points = np.asarray(landmarks, dtype=float)
    if points.ndim != 2 or points.shape[0] != 21 or points.shape[1] < 2:
        raise ValueError(f"expected 21 landmarks with x and y, got shape {points.shape}")
    return points[:, :2]


def _palm_px(xy: np.ndarray) -> float:
    palm = float(np.linalg.norm(xy[WRIST] - xy[MIDDLE_MCP]))
    if palm < 1e-6:
        raise ValueError("palm length is zero")
    return palm
