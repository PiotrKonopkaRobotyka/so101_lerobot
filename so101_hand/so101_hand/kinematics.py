"""Kinematics of the SO-101 arm: the shoulder pan (joint "1") turns the plane in which the shoulder lift, elbow flex
and wrist flex (joints "2"–"4") move the gripper; the wrist roll (joint "5") stays at 0.

Plane coordinates: reach = distance forward from the base axis, height = up (+Z of the base). With the pan at 0 the
arm points along -Y of the base, so the plane is the base's YZ plane. Over the table (ADR 0006): forward (-Y of the
base), right (-X of the base) and height, in metres; the pan is positive to the robot's right, because the joint's
axis is -Z of the base. The geometry comes from the third-party URDF (checked by tests).
"""

import math
from typing import NamedTuple

import numpy as np

# Tool centre point (TCP) in the frame of the `gripper` link: the middle of the open jaws, 12 mm off the wrist roll
# axis, where a top-down grasp centres the cube. A TCP on the axis put the fixed jaw on the cube: 0-4 of 20 grasps in
# MuJoCo lifted it, 19-20 of 20 with this one (ROB-38; it is so101-nexus's `gripperframe` site, 0.2 mm out of the
# arm's plane left out). The closing jaw then pushes the cube about 6 mm towards the fixed jaw.
TCP_IN_GRIPPER = np.array([0.012, 0.0, -0.098127])

# Plane coordinates (reach, height) in metres with all joints at 0
SHOULDER = np.array([0.05347, 0.14908])  # axis of joint "2"
ELBOW_0 = np.array([0.08147, 0.26165])  # axis of joint "3"
WRIST_0 = np.array([0.21637, 0.26685])  # axis of joint "4"
TCP_0 = np.array([0.37560, 0.27885])  # with the gripper pointing straight forward (pitch 0)

UPPER_ARM = float(np.hypot(*(ELBOW_0 - SHOULDER)))  # 0.116 m
FOREARM = float(np.hypot(*(WRIST_0 - ELBOW_0)))  # 0.135 m

PAN_LIMITS = (-1.91986, 1.91986)  # joint "1"
JOINT_LIMITS = {"2": (-1.74533, 1.74533), "3": (-1.74533, 1.5708), "4": (-1.65806, 1.65806)}

# The pan axis (joint "1") in (forward, right) coordinates: 2.3 cm in front of the base origin and 2.1 cm to its
# left; the arm's plane runs 0.2 mm to the right of the axis (URDF, checked by tests)
PAN_AXIS = np.array([0.023075, -0.020791])
PLANE_OFFSET = 0.000175

# Joint "3" with the upper arm and the forearm in one line (the arm stretched): there a millimetre of reach turns
# the elbow by degrees (a singularity), so the teleoperation keeps the elbow bent by at least MIN_ELBOW_BEND
_UPPER_0, _FORE_0 = ELBOW_0 - SHOULDER, WRIST_0 - ELBOW_0
ELBOW_STRAIGHT = math.atan2(_FORE_0[1], _FORE_0[0]) - math.atan2(_UPPER_0[1], _UPPER_0[0])  # -1.288 rad
MIN_ELBOW_BEND = math.radians(30)  # 20° let the elbow turn 0.18 rad per 5 mm of reach near the edge, 30° 0.13


class Pose(NamedTuple):
    """TCP in the plane: reach (forward) and height (up) in metres; pitch = angle of the jaws above horizontal
    in radians (0 = pointing forward, -pi/2 = pointing straight down)."""

    reach: float
    height: float
    pitch: float


class Point(NamedTuple):
    """TCP over the table: forward (-Y of the base), right (-X of the base) and height (+Z), in metres."""

    forward: float
    right: float
    height: float


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


# --- over the table: the pan turns the plane (ADR 0006) ---


def pan_angle(point: Point) -> float:
    """Direction of `point` seen from the pan axis: 0 straight ahead, positive to the robot's right (radians)."""
    return math.atan2(point.right - PAN_AXIS[1], point.forward - PAN_AXIS[0])


def forward3d(q1: float, q2: float, q3: float, q4: float) -> tuple[Point, float]:
    """TCP point over the table and the jaws' pitch (in the turned plane) for joint angles in radians.

    The pan turns the TCP's offset from its axis: along the plane (the planar reach minus the axis' forward
    position) and across it (the plane's offset); a positive pan turns forward towards the right.
    """
    plane = forward(q2, q3, q4)
    along, c, s = plane.reach - PAN_AXIS[0], math.cos(q1), math.sin(q1)
    forward_m = PAN_AXIS[0] + along * c - PLANE_OFFSET * s
    right_m = PAN_AXIS[1] + along * s + PLANE_OFFSET * c
    return Point(float(forward_m), float(right_m), plane.height), plane.pitch


def inverse3d(point: Point, pitch: float, current=(0.0, 0.0, 0.0, 0.0)) -> tuple[float, float, float, float] | None:
    """Joint angles (q1, q2, q3, q4) reaching `point` with the jaws at `pitch`, or None.

    The pan points the plane at the target (the offset from the axis, less the plane's own offset, undoes
    `forward3d`), then the planar IK works in that plane; `current` picks between elbow up and down as in `inverse`.
    """
    d_forward, d_right = point.forward - PAN_AXIS[0], point.right - PAN_AXIS[1]
    along_squared = d_forward**2 + d_right**2 - PLANE_OFFSET**2
    if along_squared <= 0.0:
        return None  # on the pan axis
    along = math.sqrt(along_squared)
    q1 = math.atan2(d_right, d_forward) - math.atan2(PLANE_OFFSET, along)
    if not PAN_LIMITS[0] - 1e-9 <= q1 <= PAN_LIMITS[1] + 1e-9:
        return None
    q = inverse(Pose(float(PAN_AXIS[0] + along), point.height, pitch), current=tuple(current[1:]))
    return None if q is None else (q1, *q)


def inverse_steepest(
    point: Point,
    pitch_range: tuple[float, float] = (-math.pi / 2, -math.pi / 4),
    current=(0.0, 0.0, 0.0, 0.0),
    step: float = math.radians(5),
    refinements: int = 10,
    min_elbow_bend: float = MIN_ELBOW_BEND,
) -> tuple[tuple[float, float, float, float], float] | None:
    """Joint angles and the jaws' pitch for `point` with the steepest pitch the arm reaches there (ADR 0006).

    The wrist flex limit lets the jaws point straight down only near the base (to about 0.25–0.28 m from the pan
    axis below 0.1 m of height); farther out they tilt. The search goes from `pitch_range[0]` (straight down) towards
    `pitch_range[1]` in steps of `step`, and the first pitch that works is refined by bisection against the last one
    that did not, so the pitch changes smoothly as the point moves. A pitch works when the IK has a solution with
    the elbow bent by at least `min_elbow_bend`: without that, the far targets would ride the stretched-arm
    singularity and the elbow would twitch. None when no pitch of the range works.
    """

    def solve(pitch: float):
        q = inverse3d(point, pitch, current)
        return q if q is not None and abs(q[2] - ELBOW_STRAIGHT) >= min_elbow_bend else None

    pitch, rejected = pitch_range[0], None
    while pitch <= pitch_range[1] + 1e-9:
        q = solve(pitch)
        if q is not None:
            break
        rejected, pitch = pitch, pitch + step
    else:
        return None
    if rejected is not None:
        for _ in range(refinements):
            middle = (rejected + pitch) / 2
            q_middle = solve(middle)
            if q_middle is None:
                rejected = middle
            else:
                pitch, q = middle, q_middle
    return q, pitch


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
