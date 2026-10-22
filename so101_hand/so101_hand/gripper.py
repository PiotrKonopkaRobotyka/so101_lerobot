"""Hand openness (0 = closed, 1 = open) to the SO-101 gripper joint "6" in radians."""

import math

CLOSED_RAD = 0.0
# Upper joint limit in the URDF of lerobot_ws (lower limit: -0.174533). A command beyond a limit makes the
# simulated jaw stick at the limit and ignore later commands, so the range is enforced here.
OPEN_RAD = 1.74533


def openness_to_joint(openness: float) -> float:
    """Map hand openness to the joint angle, clamped to [CLOSED_RAD, OPEN_RAD]."""
    if math.isnan(openness):
        raise ValueError("openness is NaN")
    openness = min(max(openness, 0.0), 1.0)
    return CLOSED_RAD + openness * (OPEN_RAD - CLOSED_RAD)
