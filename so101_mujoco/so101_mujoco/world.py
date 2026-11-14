"""so101_hand's table frame and joints ↔ MuJoCo's world and so101-nexus's joints (ADR 0009).

so101_hand works in the base frame of the Gazebo URDF: points over the table as (forward, right, height) in metres.
so101-nexus simulates the official SO-101 model (MuJoCo Menagerie), whose base is MuJoCo's world. The two bases
differ by a shift only: (forward, left, up) + TABLE_TO_WORLD = (x, y, z) of the world; it follows from where the
two URDFs put the shoulder pan joint (a test checks it against MuJoCo's kinematics). Joints 1–4 and the gripper
are the same in both models; the wrist roll's zero is not.
"""

import numpy as np

from so101_hand.kinematics import PAN_AXIS, Point

TABLE_TO_WORLD = np.array([0.0157608, -0.0207909, -0.0324817])  # metres
FLOOR_HEIGHT = float(-TABLE_TO_WORLD[2])  # MuJoCo's floor in so101_hand's frame: 32.5 mm up
WRIST_ROLL_OFFSET = 0.0486795  # rad: so101-nexus's wrist_roll at the Gazebo zero of joint "5" (2.79°)
PAN_AXIS_WORLD = (float(PAN_AXIS[0] + TABLE_TO_WORLD[0]), float(-PAN_AXIS[1] + TABLE_TO_WORLD[1]))  # (x, y)


def to_world(point: Point) -> np.ndarray:
    """A point of so101_hand's table frame in MuJoCo's world (x, y, z)."""
    return np.array([point.forward, -point.right, point.height]) + TABLE_TO_WORLD


def to_table(xyz) -> Point:
    """A point of MuJoCo's world in so101_hand's table frame."""
    forward, left, up = np.asarray(xyz, dtype=float) - TABLE_TO_WORLD
    return Point(float(forward), float(-left), float(up))


def joint_targets(arm: tuple[float, float, float, float], gripper_rad: float) -> np.ndarray:
    """so101-nexus's action in `pd_joint_pos` mode for joints 1–4 of so101_hand and the gripper's goal: the wrist
    roll at the Gazebo zero, as the Gazebo node keeps joint "5"."""
    return np.array([*arm, WRIST_ROLL_OFFSET, gripper_rad], dtype=np.float32)
