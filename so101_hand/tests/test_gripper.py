import re
from pathlib import Path

import pytest

from so101_hand.gripper import CLOSED_RAD, JOINT_LOWER_RAD, JOINT_UPPER_RAD, OPEN_RAD

# so101_ws/src/lerobot_ws: third-party URDF next to this repository in the workspace
URDF = Path(__file__).resolve().parents[3] / "lerobot_ws/src/lerobot_description/urdf/so101_base.xacro"


def test_commands_keep_a_safety_margin_inside_the_joint_limits():
    # Beyond the URDF limit (1.74533 rad) the simulated jaw sticks at the limit and ignores later commands.
    assert JOINT_LOWER_RAD < CLOSED_RAD < OPEN_RAD < JOINT_UPPER_RAD


@pytest.mark.skipif(not URDF.is_file(), reason="third-party URDF not in the workspace")
def test_joint_limits_match_the_urdf():
    joint6 = re.search(r'<joint name="6".*?</joint>', URDF.read_text(), re.S).group(0)
    limit = re.search(r'lower="([-\d.]+)"\s+upper="([-\d.]+)"', joint6)
    assert (JOINT_LOWER_RAD, JOINT_UPPER_RAD) == (float(limit.group(1)), float(limit.group(2)))
