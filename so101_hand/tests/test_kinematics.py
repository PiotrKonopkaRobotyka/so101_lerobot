import math

import numpy as np
import pytest

from so101_hand.kinematics import (
    JOINT_LIMITS, SHOULDER, TCP_0, TCP_IN_GRIPPER, WRIST_0, Pose, forward, inverse, solutions,
)
from urdf_reference import URDF, UrdfArm

needs_urdf = pytest.mark.skipif(not URDF.is_file(), reason="third-party URDF not in the workspace")


def random_joints(seed: int, n: int) -> np.ndarray:
    """n random (q2, q3, q4) inside the joint limits."""
    lower, upper = zip(*(JOINT_LIMITS[j] for j in ("2", "3", "4")))
    return np.random.default_rng(seed).uniform(lower, upper, size=(n, 3))


def angle_gap(a: float, b: float) -> float:
    """Distance between two angles, so that -pi and pi count as equal."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


@needs_urdf
def test_joint_limits_match_the_urdf():
    arm = UrdfArm()
    assert JOINT_LIMITS == {j: arm.limits(j) for j in ("2", "3", "4")}


@needs_urdf
def test_forward_matches_the_urdf():
    # the same TCP computed by the full 3D chain of the URDF, with joints "1" and "5" at 0
    arm = UrdfArm()
    for q2, q3, q4 in random_joints(seed=0, n=200):
        gripper = arm.link_pose("gripper", {"2": q2, "3": q3, "4": q4})
        tcp = gripper[:3, :3] @ TCP_IN_GRIPPER + gripper[:3, 3]
        approach = gripper[:3, :3] @ [0.0, 0.0, -1.0]  # the jaws point along -Z of the gripper link
        pose = forward(q2, q3, q4)
        assert (pose.reach, pose.height) == pytest.approx((-tcp[1], tcp[2]), abs=1e-4)  # reach = -Y, height = +Z
        assert angle_gap(pose.pitch, math.atan2(approach[2], -approach[1])) < 1e-4


def test_zero_joints_stretch_the_arm_forward():
    assert forward(0.0, 0.0, 0.0) == pytest.approx(Pose(reach=TCP_0[0], height=TCP_0[1], pitch=0.0))


def test_inverse_undoes_forward():
    for q in random_joints(seed=1, n=200):
        assert inverse(forward(*q), current=q) == pytest.approx(tuple(q), abs=1e-6)


def test_every_solution_reaches_the_target_inside_the_limits():
    for reach in np.linspace(-0.1, 0.5, 13):
        for height in np.linspace(-0.1, 0.5, 13):
            for pitch in (-math.pi / 2, -math.pi / 4, 0.0, math.pi / 4):
                target = Pose(reach, height, pitch)
                for q in solutions(target):
                    assert all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS.values()))
                    reached = forward(*q)
                    assert (reached.reach, reached.height) == pytest.approx((reach, height), abs=1e-9)
                    assert angle_gap(reached.pitch, pitch) < 1e-9


def test_target_out_of_reach_gives_none():
    assert inverse(Pose(reach=1.0, height=0.2, pitch=0.0)) is None


def test_wrist_at_the_shoulder_gives_none():
    # the forearm is longer than the upper arm, so the wrist cannot come closer to the shoulder than their difference
    tcp = SHOULDER + (TCP_0 - WRIST_0)  # pitch 0: the gripper sticks straight forward from the wrist
    assert inverse(Pose(reach=tcp[0], height=tcp[1], pitch=0.0)) is None


def test_the_solution_closest_to_the_current_joints_is_chosen():
    # an almost straight arm: elbow "up" and elbow "down" are both inside the limits
    elbow_up, elbow_down = (0.5, -1.0, 0.5), (0.8105, -1.577, 0.7665)
    target = forward(*elbow_up)
    assert len(solutions(target)) == 2
    assert inverse(target, current=elbow_up) == pytest.approx(elbow_up, abs=1e-6)
    assert inverse(target, current=elbow_down) == pytest.approx(elbow_down, abs=1e-3)
