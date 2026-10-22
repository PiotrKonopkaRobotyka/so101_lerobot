import math

import pytest

from so101_hand.gripper import CLOSED_RAD, OPEN_RAD, openness_to_joint


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
