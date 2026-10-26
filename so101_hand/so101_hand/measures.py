"""Hand measures from MediaPipe landmarks: the openness measure chosen in the ROB-19 spike, the hand's pitch and the
image scale."""

import math

import numpy as np

WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP = 0, 4, 8, 9


def aperture(landmarks: np.ndarray) -> float:
    """Thumb tip to index tip distance divided by palm length (wrist to middle-finger MCP).

    `landmarks` has shape (21, 2) or (21, 3) in pixels; only x and y are used. Pixels, not MediaPipe's normalised
    coordinates: those scale x by the image width and y by the height, which distorts distances.
    Dividing by the palm length makes the value independent of the hand's distance to the camera.
    """
    xy = _image_xy(landmarks)
    return float(np.linalg.norm(xy[THUMB_TIP] - xy[INDEX_TIP]) / _palm_px(xy))


def hand_pitch(landmarks: np.ndarray) -> float:
    """Angle of the palm line (wrist → middle-finger MCP) above the forward direction, in radians.

    Side view of ADR 0003 on the raw frame: forward = image right (+u), up = image up (-v). The same convention as
    the jaws' pitch (kinematics.Pose): 0 = pointing forward, -pi/2 = pointing straight down.
    """
    xy = _image_xy(landmarks)
    _palm_px(xy)  # rejects a zero palm, whose direction is undefined
    du, dv = xy[MIDDLE_MCP] - xy[WRIST]
    return math.atan2(-dv, du)


def metres_per_pixel(image_landmarks: np.ndarray, world_landmarks: np.ndarray) -> float:
    """Image scale at the hand's depth: palm length in MediaPipe's world landmarks (metres) / in the image (pixels).

    Turns a hand displacement in pixels into metres without a calibration template (ADR 0003).
    """
    world = np.asarray(world_landmarks, dtype=float)
    return float(np.linalg.norm(world[WRIST] - world[MIDDLE_MCP]) / _palm_px(_image_xy(image_landmarks)))


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
