"""Reference for the kinematics tests: generic forward kinematics of the third-party URDF, as TF computes it."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# so101_ws/src/lerobot_ws: third-party URDF next to this repository in the workspace
URDF = Path(__file__).resolve().parents[3] / "lerobot_ws/src/lerobot_description/urdf/so101_base.xacro"


def _floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def _origin(xyz, rpy) -> np.ndarray:
    """4x4 transform of a URDF <origin>: translation xyz, rotation roll-pitch-yaw (fixed axes X, then Y, then Z)."""
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = xyz
    return t


def _about_axis(axis, angle: float) -> np.ndarray:
    """4x4 rotation by `angle` about a unit `axis` (Rodrigues' formula)."""
    a = np.asarray(axis, dtype=float)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    t = np.eye(4)
    t[:3, :3] = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k
    return t


class UrdfArm:
    def __init__(self, path: Path = URDF):
        self._by_child = {}
        for joint in ET.parse(path).getroot().findall("joint"):
            origin, axis, limit = joint.find("origin"), joint.find("axis"), joint.find("limit")
            self._by_child[joint.find("child").get("link")] = {
                "name": joint.get("name"),
                "parent": joint.find("parent").get("link"),
                "origin": _origin(_floats(origin.get("xyz")), _floats(origin.get("rpy"))),
                "axis": _floats(axis.get("xyz")) if axis is not None else None,
                "limits": (float(limit.get("lower")), float(limit.get("upper"))) if limit is not None else None,
            }

    def link_pose(self, link: str, joints: dict) -> np.ndarray:
        """4x4 pose of `link` in the root frame; `joints` maps joint names to angles (missing joints are 0)."""
        if link not in self._by_child:
            return np.eye(4)
        j = self._by_child[link]
        pose = self.link_pose(j["parent"], joints) @ j["origin"]
        if j["axis"] is not None:
            pose = pose @ _about_axis(j["axis"], joints.get(j["name"], 0.0))
        return pose

    def limits(self, joint: str) -> tuple[float, float]:
        return next(j["limits"] for j in self._by_child.values() if j["name"] == joint)
