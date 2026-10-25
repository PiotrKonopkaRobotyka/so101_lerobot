"""Hand openness measure from MediaPipe image landmarks (chosen in the ROB-19 spike)."""

import numpy as np

WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP = 0, 4, 8, 9


def aperture(landmarks: np.ndarray) -> float:
    """Thumb tip to index tip distance divided by palm length (wrist to middle-finger MCP).

    `landmarks` has shape (21, 2) or (21, 3) in pixels; only x and y are used. Pixels, not MediaPipe's normalised
    coordinates: those scale x by the image width and y by the height, which distorts distances.
    Dividing by the palm length makes the value independent of the hand's distance to the camera.
    """
    points = np.asarray(landmarks, dtype=float)
    if points.ndim != 2 or points.shape[0] != 21 or points.shape[1] < 2:
        raise ValueError(f"expected 21 landmarks with x and y, got shape {points.shape}")
    xy = points[:, :2]
    palm = np.linalg.norm(xy[WRIST] - xy[MIDDLE_MCP])
    if palm < 1e-6:
        raise ValueError("palm length is zero")
    return float(np.linalg.norm(xy[THUMB_TIP] - xy[INDEX_TIP]) / palm)
