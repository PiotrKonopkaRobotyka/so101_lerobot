"""The SO-101 gripper joint "6": its two goals in radians, closed and open (ADR 0008)."""

# Limits of joint "6" in the URDF of lerobot_ws (checked by a test). A command beyond a limit makes the simulated
# jaw stick at the limit and ignore later commands, so commands stay a safety margin inside the range.
JOINT_LOWER_RAD = -0.174533
JOINT_UPPER_RAD = 1.74533
SAFETY_MARGIN_RAD = 0.02

CLOSED_RAD = 0.0  # jaws touching (lerobot-ros: closed position 0.0)
OPEN_RAD = JOINT_UPPER_RAD - SAFETY_MARGIN_RAD
