"""The SO-101 gripper: its state from the hand (ADR 0008) and its two goals for joint "6" in radians."""

from so101_hand.calibration import Calibration

# Limits of joint "6" in the URDF of lerobot_ws (checked by a test). A command beyond a limit makes the simulated
# jaw stick at the limit and ignore later commands, so commands stay a safety margin inside the range.
JOINT_LOWER_RAD = -0.174533
JOINT_UPPER_RAD = 1.74533
SAFETY_MARGIN_RAD = 0.02

CLOSED_RAD = 0.0  # jaws touching (lerobot-ros: closed position 0.0)
OPEN_RAD = JOINT_UPPER_RAD - SAFETY_MARGIN_RAD

# MediaPipe's canned gestures that decide at once (ADR 0008): a fist, also with the thumb out, closes the gripper and
# an open palm opens it. Other gestures ("None", "Pointing_Up", "Victory", "ILoveYou") and weaker scores leave the
# decision to the aperture.
CLOSING_GESTURES = frozenset({"Closed_Fist", "Thumb_Up", "Thumb_Down"})
OPENING_GESTURES = frozenset({"Open_Palm"})
MIN_GESTURE_SCORE = 0.5


def next_state(was_open: bool, gesture: str, score: float, aperture: float, calibration: Calibration):
    """The gripper's state for a frame with a hand (True = open) and what decided it: the gesture or "aperture".

    A fist or an open palm decides at once, with no smoothing. Any other hand is left to the smoothed aperture and
    its hysteresis (Calibration.is_open): it catches the closing "beak" (the thumb on the fingertips), which the model
    calls "None", and an open hand that it does not call "Open_Palm" (near the face, tilted).
    """
    if score >= MIN_GESTURE_SCORE:
        if gesture in CLOSING_GESTURES:
            return False, gesture
        if gesture in OPENING_GESTURES:
            return True, gesture
    return calibration.is_open(aperture, was_open), "aperture"
