import math
import re
from pathlib import Path

import pytest

from so101_hand.gripper import CLOSED_RAD, JOINT_LOWER_RAD, JOINT_UPPER_RAD, OPEN_RAD, openness_to_joint

# so101_ws/src/lerobot_ws: third-party URDF next to this repository in the workspace
URDF = Path(__file__).resolve().parents[3] / "lerobot_ws/src/lerobot_description/urdf/so101_base.xacro"


def test_commands_keep_a_safety_margin_inside_the_joint_limits():
    assert JOINT_LOWER_RAD < CLOSED_RAD < OPEN_RAD < JOINT_UPPER_RAD


@pytest.mark.skipif(not URDF.is_file(), reason="third-party URDF not in the workspace")
def test_joint_limits_match_the_urdf():
    joint6 = re.search(r'<joint name="6".*?</joint>', URDF.read_text(), re.S).group(0)
    limit = re.search(r'lower="([-\d.]+)"\s+upper="([-\d.]+)"', joint6)
    assert (JOINT_LOWER_RAD, JOINT_UPPER_RAD) == (float(limit.group(1)), float(limit.group(2)))


def test_closed_hand_gives_closed_gripper():
    assert openness_to_joint(0.0) == CLOSED_RAD


def test_open_hand_gives_open_gripper():
    assert openness_to_joint(1.0) == OPEN_RAD


def test_halfway_is_linear():
    assert openness_to_joint(0.5) == pytest.approx((CLOSED_RAD + OPEN_RAD) / 2)


@pytest.mark.parametrize("openness", [-5.0, -0.1, 1.1, 2.0, 1e9, math.inf, -math.inf])
def test_command_never_leaves_the_joint_range(openness):
    # Beyond the URDF limit (1.74533 rad) the simulated jaw sticks at the limit and ignores later commands.
    assert CLOSED_RAD <= openness_to_joint(openness) <= OPEN_RAD


def test_nan_is_rejected_instead_of_sent_to_the_robot():
    with pytest.raises(ValueError):
        openness_to_joint(math.nan)
