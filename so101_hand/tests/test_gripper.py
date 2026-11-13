import re
from pathlib import Path

import pytest

from so101_hand.calibration import Calibration
from so101_hand.gripper import (
    CLOSED_RAD,
    CLOSING_GESTURES,
    JOINT_LOWER_RAD,
    JOINT_UPPER_RAD,
    MIN_GESTURE_SCORE,
    OPEN_RAD,
    next_state,
)

# so101_ws/src/lerobot_ws: third-party URDF next to this repository in the workspace
URDF = Path(__file__).resolve().parents[3] / "lerobot_ws/src/lerobot_description/urdf/so101_base.xacro"
# My hand from the front, as in config/hand_teleop.yaml
FRONT = Calibration(closed=0.36, open=0.70)


def test_commands_keep_a_safety_margin_inside_the_joint_limits():
    # Beyond the URDF limit (1.74533 rad) the simulated jaw sticks at the limit and ignores later commands.
    assert JOINT_LOWER_RAD < CLOSED_RAD < OPEN_RAD < JOINT_UPPER_RAD


@pytest.mark.skipif(not URDF.is_file(), reason="third-party URDF not in the workspace")
def test_joint_limits_match_the_urdf():
    joint6 = re.search(r'<joint name="6".*?</joint>', URDF.read_text(), re.S).group(0)
    limit = re.search(r'lower="([-\d.]+)"\s+upper="([-\d.]+)"', joint6)
    assert (JOINT_LOWER_RAD, JOINT_UPPER_RAD) == (float(limit.group(1)), float(limit.group(2)))


@pytest.mark.parametrize("gesture", sorted(CLOSING_GESTURES))
def test_a_fist_closes_whatever_the_aperture(gesture):
    # the fist of the 2026-11-13 test with the thumb along the index finger: aperture 0.41-0.52, over the threshold
    assert next_state(True, gesture, 0.9, 0.52, FRONT) == (False, gesture)


def test_an_open_palm_opens_whatever_the_aperture():
    assert next_state(False, "Open_Palm", 0.9, 0.45, FRONT) == (True, "Open_Palm")


@pytest.mark.parametrize("gesture", ["None", "Pointing_Up", "Victory", "ILoveYou", ""])
def test_other_gestures_leave_it_to_the_aperture(gesture):
    # the closing "beak" (the thumb on the fingertips) is "None" for the model; its aperture 0.15 closes
    assert next_state(True, gesture, 0.9, 0.15, FRONT) == (False, "aperture")
    assert next_state(False, gesture, 0.9, 0.85, FRONT) == (True, "aperture")
    assert next_state(False, gesture, 0.9, 0.50, FRONT) == (False, "aperture")  # between the thresholds: kept


def test_an_unsure_gesture_leaves_it_to_the_aperture():
    assert next_state(True, "Closed_Fist", MIN_GESTURE_SCORE - 0.01, 0.50, FRONT) == (True, "aperture")
