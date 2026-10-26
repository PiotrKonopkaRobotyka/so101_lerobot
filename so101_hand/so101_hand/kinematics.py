"""Planar kinematics of the SO-101 arm with the shoulder pan (joint "1") and the wrist roll (joint "5") at 0.

Joints "2" (shoulder lift), "3" (elbow flex) and "4" (wrist flex) then all turn about the X axis of the `base`
frame, so the arm moves in the vertical YZ plane of the base. Plane coordinates: reach = forward (-Y of the base:
the arm at zero points that way), height = up (+Z). The geometry comes from the third-party URDF (checked by tests).
"""

import math
from typing import NamedTuple

import numpy as np

# Tool centre point (TCP) in the frame of the `gripper` link: between the closed jaws, 7 mm behind their tips.
TCP_IN_GRIPPER = np.array([0.0, 0.0, -0.098])

# Plane coordinates (reach, height) in metres with all joints at 0
SHOULDER = np.array([0.05347, 0.14908])  # axis of joint "2"
ELBOW_0 = np.array([0.08147, 0.26165])  # axis of joint "3"
WRIST_0 = np.array([0.21637, 0.26685])  # axis of joint "4"
TCP_0 = np.array([0.37547, 0.26685])  # with the gripper pointing straight forward (pitch 0)

UPPER_ARM = float(np.hypot(*(ELBOW_0 - SHOULDER)))  # 0.116 m
FOREARM = float(np.hypot(*(WRIST_0 - ELBOW_0)))  # 0.135 m

JOINT_LIMITS = {"2": (-1.74533, 1.74533), "3": (-1.74533, 1.5708), "4": (-1.65806, 1.65806)}


class Pose(NamedTuple):
    """TCP in the plane: reach (forward) and height (up) in metres; pitch = angle of the jaws above horizontal
    in radians (0 = pointing forward, -pi/2 = pointing straight down)."""

    reach: float
    height: float
    pitch: float


def forward(q2: float, q3: float, q4: float) -> Pose:
    """TCP pose for joint angles in radians.

    The joints turn about +X of the base, which is clockwise in the (reach, height) plane, so a positive angle turns
    everything after the joint downwards. Each link keeps its shape: its vector at zero, turned by the sum of
    the joint angles before it.
    """
    turn_upper, turn_fore, turn_gripper = q2, q2 + q3, q2 + q3 + q4
    elbow = SHOULDER + _rotation(-turn_upper) @ (ELBOW_0 - SHOULDER)
    wrist = elbow + _rotation(-turn_fore) @ (WRIST_0 - ELBOW_0)
    tcp = wrist + _rotation(-turn_gripper) @ (TCP_0 - WRIST_0)
    return Pose(float(tcp[0]), float(tcp[1]), _wrap(-turn_gripper))


def solutions(pose: Pose) -> list[tuple[float, float, float]]:
    """All joint angles (q2, q3, q4) inside the joint limits that reach `pose`: none, one or two.

    1. The pitch fixes the gripper's turn, so the wrist is the TCP minus the turned gripper vector.
    2. Shoulder → elbow → wrist is a triangle with known sides (upper arm, forearm, shoulder-wrist distance):
       the law of cosines gives the angle at the shoulder, and the elbow lies on either side of the
       shoulder-wrist line ("elbow up" or "elbow down").
    3. The joint angles follow from the absolute angles of the links.
    """
    turn_gripper = -pose.pitch
    wrist = np.array([pose.reach, pose.height]) - _rotation(-turn_gripper) @ (TCP_0 - WRIST_0)
    to_wrist = wrist - SHOULDER
    distance = float(np.hypot(*to_wrist))
    if not abs(UPPER_ARM - FOREARM) <= distance <= UPPER_ARM + FOREARM:
        return []
    cos_shoulder = (UPPER_ARM**2 + distance**2 - FOREARM**2) / (2 * UPPER_ARM * distance)
    shoulder_angle = math.acos(min(max(cos_shoulder, -1.0), 1.0))
    found = []
    for side in (1.0, -1.0):
        upper = _angle(to_wrist) + side * shoulder_angle  # absolute angle of the upper arm
        elbow = SHOULDER + UPPER_ARM * np.array([math.cos(upper), math.sin(upper)])
        q2 = _wrap(_angle(ELBOW_0 - SHOULDER) - upper)
        q3 = _wrap(_angle(WRIST_0 - ELBOW_0) - _angle(wrist - elbow) - q2)
        q4 = _wrap(turn_gripper - q2 - q3)
        q = (q2, q3, q4)
        if _within_limits(q) and q not in found:
            found.append(q)
    return found


def inverse(pose: Pose, current=(0.0, 0.0, 0.0)) -> tuple[float, float, float] | None:
    """Joint angles reaching `pose`, or None when it is out of reach or the joint limits.

    Of two solutions, the one closest to `current` (the joints now): during teleoperation the arm then never
    jumps between elbow up and elbow down.
    """
    candidates = solutions(pose)
    if not candidates:
        return None
    return min(candidates, key=lambda q: sum((a - b) ** 2 for a, b in zip(q, current)))


def _rotation(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def _angle(vector) -> float:
    return math.atan2(vector[1], vector[0])


def _wrap(angle: float) -> float:
    """The same angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _within_limits(q, tolerance: float = 1e-9) -> bool:
    return all(lo - tolerance <= qi <= hi + tolerance for qi, (lo, hi) in zip(q, JOINT_LIMITS.values()))
