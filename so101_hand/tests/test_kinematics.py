import math

import numpy as np
import pytest

from so101_hand.kinematics import (
    ELBOW_STRAIGHT, JOINT_LIMITS, MIN_ELBOW_BEND, PAN_AXIS, PAN_LIMITS, SHOULDER, TCP_0, TCP_IN_GRIPPER, WRIST_0, Point,
    Pose, forward, forward3d, inverse, inverse3d, inverse_steepest, pan_angle, solutions,
)
from urdf_reference import URDF, UrdfArm

needs_urdf = pytest.mark.skipif(not URDF.is_file(), reason="third-party URDF not in the workspace")


def random_joints(seed: int, n: int) -> np.ndarray:
    """n random (q2, q3, q4) inside the joint limits."""
    lower, upper = zip(*(JOINT_LIMITS[j] for j in ("2", "3", "4")))
    return np.random.default_rng(seed).uniform(lower, upper, size=(n, 3))


def random_joints4(seed: int, n: int) -> np.ndarray:
    """n random (q1, q2, q3, q4) inside the joint limits."""
    lower, upper = zip(PAN_LIMITS, *(JOINT_LIMITS[j] for j in ("2", "3", "4")))
    return np.random.default_rng(seed).uniform(lower, upper, size=(n, 4))


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


# --- over the table: the pan turns the plane (ADR 0006) ---


@needs_urdf
def test_pan_limits_match_the_urdf():
    assert PAN_LIMITS == UrdfArm().limits("1")


@needs_urdf
def test_forward3d_matches_the_urdf():
    arm = UrdfArm()
    for q1, q2, q3, q4 in random_joints4(seed=2, n=200):
        gripper = arm.link_pose("gripper", {"1": q1, "2": q2, "3": q3, "4": q4})
        tcp = gripper[:3, :3] @ TCP_IN_GRIPPER + gripper[:3, 3]
        approach = gripper[:3, :3] @ [0.0, 0.0, -1.0]  # the jaws point along -Z of the gripper link
        point, pitch = forward3d(q1, q2, q3, q4)
        assert tuple(point) == pytest.approx((-tcp[1], -tcp[0], tcp[2]), abs=1e-4)  # forward -Y, right -X, up +Z
        along = np.array([-math.sin(q1), -math.cos(q1), 0.0])  # the turned plane's forward direction in the base
        assert angle_gap(pitch, math.atan2(approach[2], approach @ along)) < 1e-4


def test_pan_is_zero_straight_ahead_of_the_axis_and_positive_to_the_right():
    ahead = Point(PAN_AXIS[0] + 0.3, PAN_AXIS[1], 0.1)
    assert pan_angle(ahead) == 0.0
    assert pan_angle(ahead._replace(right=PAN_AXIS[1] + 0.3)) == pytest.approx(math.pi / 4)


def test_straight_ahead_in_the_arms_plane_needs_no_pan():
    point, pitch = forward3d(0.0, 0.3, -0.5, 0.2)
    assert inverse3d(point, pitch, current=(0.0, 0.3, -0.5, 0.2))[0] == pytest.approx(0.0, abs=1e-9)


def test_inverse3d_undoes_forward3d():
    for q in random_joints4(seed=3, n=200):
        if forward(*q[1:]).reach - PAN_AXIS[0] <= 0.005:
            continue  # the arm folded back behind its axis: the point's direction is the pan turned by 180°
        point, pitch = forward3d(*q)
        assert inverse3d(point, pitch, current=q) == pytest.approx(tuple(q), abs=1e-6)


def test_inverse3d_outside_the_pan_limits_gives_none():
    assert inverse3d(Point(-0.1, 0.05, 0.1), -math.pi / 2) is None  # 153 degrees to the right


def test_the_home_point_is_reached_with_the_jaws_straight_down():
    q, pitch = inverse_steepest(Point(0.25, 0.0, 0.08))
    assert pitch == -math.pi / 2
    assert tuple(forward3d(*q)[0]) == pytest.approx((0.25, 0.0, 0.08), abs=1e-9)


def test_the_pitch_relaxes_where_straight_down_is_out_of_reach():
    far = Point(0.37, 0.0, 0.08)
    assert inverse3d(far, -math.pi / 2) is None
    q, pitch = inverse_steepest(far)
    assert -math.pi / 2 < pitch <= -math.pi / 4
    assert tuple(forward3d(*q)[0]) == pytest.approx(tuple(far), abs=1e-9)


def test_inverse_steepest_gives_none_out_of_reach():
    assert inverse_steepest(Point(0.6, 0.0, 0.08)) is None


def test_the_steepest_pitch_and_the_joints_change_smoothly_along_the_reach():
    # 1 mm steps away from the base at the hover height: across the edge of the top-down area the jaws start to
    # tilt, without a jump in the pitch (a plain 5 degree search would jump) or in the joints
    q, pitch = inverse_steepest(Point(0.20, 0.0, 0.08))
    assert pitch == -math.pi / 2
    for reach_mm in range(201, 380):
        q_next, pitch_next = inverse_steepest(Point(reach_mm / 1000, 0.0, 0.08), current=q)
        assert abs(pitch_next - pitch) < math.radians(1.0), f"pitch jump at {reach_mm} mm"
        assert max(abs(a - b) for a, b in zip(q_next, q)) < 0.035, f"joint jump at {reach_mm} mm"
        assert abs(q_next[2] - ELBOW_STRAIGHT) >= MIN_ELBOW_BEND - 1e-9  # never near the stretched arm
        q, pitch = q_next, pitch_next
    assert pitch > math.radians(-65)  # tilted well up at the far end


def test_a_stretched_arm_is_reached_by_the_plain_inverse_but_not_for_the_teleoperation():
    stretched = (0.0, 1.0, ELBOW_STRAIGHT + 0.05, 0.6)  # the elbow 3 degrees from straight
    point, pitch = forward3d(*stretched)
    assert inverse3d(point, pitch, current=stretched) == pytest.approx(stretched, abs=1e-6)
    found = inverse_steepest(point, pitch_range=(pitch, pitch), current=stretched)
    assert found is None
