"""Hand openness (0 = closed, 1 = open) to the SO-101 gripper joint "6" in radians."""

import math

# Limits of joint "6" in the URDF of lerobot_ws (checked by a test). A command beyond a limit makes the simulated
# jaw stick at the limit and ignore later commands, so commands stay a safety margin inside the range.
JOINT_LOWER_RAD = -0.174533
JOINT_UPPER_RAD = 1.74533
SAFETY_MARGIN_RAD = 0.02

CLOSED_RAD = 0.0  # jaws touching (lerobot-ros: closed position 0.0)
OPEN_RAD = JOINT_UPPER_RAD - SAFETY_MARGIN_RAD


def openness_to_joint(openness: float) -> float:
    """Map hand openness to the joint angle, clamped to [CLOSED_RAD, OPEN_RAD]."""
    if math.isnan(openness):
        raise ValueError("openness is NaN")
    openness = min(max(openness, 0.0), 1.0)
    return CLOSED_RAD + openness * (OPEN_RAD - CLOSED_RAD)
