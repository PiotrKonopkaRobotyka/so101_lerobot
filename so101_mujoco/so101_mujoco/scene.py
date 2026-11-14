"""The MuJoCo scene and the operator's views of it (ADR 0009).

The scene is so101-nexus's PickAndPlace-v2 with the cube and the target disc sampled in our workspace sector (the
Bounds of so101_hand around the pan axis). The operator sees it from above (the base at the bottom and the robot's
right on the right, like the mirrored preview) and from the robot's right side, with the sector and the target
drawn on the floor of these two views only: the cameras so101-nexus records (ROB-39) never show them.
"""

import math

import gymnasium as gym
import mujoco
import numpy as np

import so101_nexus.mujoco  # noqa: F401  (registers the MuJoCo* environment ids)
from so101_hand.session import Settings
from so101_hand.teleop import Bounds
from so101_mujoco.world import FLOOR_HEIGHT, PAN_AXIS_WORLD, to_world
from so101_nexus.config import SO101_JOINT_NAMES, PickAndPlaceV2Config

ENV_ID = "MuJoCoPickAndPlace-v2"
MAX_EPISODE_S = 300.0  # so101-nexus ends an episode after 1024 steps (20 s); the operator needs more
HOVER_ABOVE_FLOOR = 0.07  # m: the TCP between grasps, well above the 25 mm cube
GRASP_ABOVE_FLOOR = 0.0125  # m: the TCP at the cube's centre (a top-down grasp lifts it from 8 to 18 mm)
LOWEST_ABOVE_FLOOR = 0.008  # m: the jaw tips, 7 mm below the TCP, stay off the floor
GREY, GREEN, RED = (0.55, 0.55, 0.55, 1.0), (0.1, 0.8, 0.2, 1.0), (0.9, 0.1, 0.1, 1.0)


def mujoco_settings() -> Settings:
    """The node's settings with the heights above MuJoCo's floor, which is 32.5 mm up in so101_hand's frame."""
    defaults = Settings()
    return Settings(
        **{
            **defaults.__dict__,
            "hover_height": FLOOR_HEIGHT + HOVER_ABOVE_FLOOR,
            "grasp_height": FLOOR_HEIGHT + GRASP_ABOVE_FLOOR,
            "bounds_height": (FLOOR_HEIGHT + LOWEST_ABOVE_FLOOR, defaults.bounds_height[1]),
            "floor_height": FLOOR_HEIGHT,
        }
    )


def make_env(bounds: Bounds = Bounds()) -> gym.Env:
    """PickAndPlace-v2 with the cube and the disc inside `bounds` (reach and pan around the pan axis); an episode
    runs on after a success, which `info["success"]` reports at every step."""
    if bounds.pan[0] != -bounds.pan[1]:
        raise ValueError("so101-nexus samples a sector symmetric about straight ahead")
    config = PickAndPlaceV2Config(
        spawn_center=PAN_AXIS_WORLD,
        spawn_min_radius=bounds.reach[0],
        spawn_max_radius=bounds.reach[1],
        spawn_angle_half_range_deg=math.degrees(bounds.pan[1]),
        terminate_on_success=False,
    )
    env = gym.make(ENV_ID, config=config, max_episode_steps=int(MAX_EPISODE_S / 0.02))
    env.reset(seed=0)
    return env


def joint_addresses(model: mujoco.MjModel) -> list[int]:
    """qpos addresses of so101-nexus's six joints (shoulder_pan … gripper), in the order of its actions."""
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in SO101_JOINT_NAMES]
    return [int(model.jnt_qposadr[i]) for i in ids]


def sector_outline(bounds: Bounds, z: float = 0.0005, n: int = 40) -> np.ndarray:
    """The workspace sector's outline on MuJoCo's floor: the outer arc, then the inner one, closed."""
    pans = np.linspace(bounds.pan[0], bounds.pan[1], n)
    arcs = [(bounds.reach[1], a) for a in pans] + [(bounds.reach[0], a) for a in pans[::-1]]
    # pan is positive to the robot's right, which is -y of the world
    points = [(PAN_AXIS_WORLD[0] + r * math.cos(a), PAN_AXIS_WORLD[1] - r * math.sin(a), z) for r, a in arcs]
    return np.array([*points, points[0]])


class Views:
    """The operator's two views, rendered off-screen: from above and from the robot's right side."""

    def __init__(self, model: mujoco.MjModel, size: tuple[int, int] = (480, 480), bounds: Bounds = Bounds()):
        width, height = size
        self.renderers = [mujoco.Renderer(model, height, width) for _ in range(2)]
        middle = (PAN_AXIS_WORLD[0] + sum(bounds.reach) / 2, PAN_AXIS_WORLD[1])
        self.cameras = [
            _free_camera(lookat=(middle[0] - 0.02, middle[1], 0.0), distance=0.82, azimuth=0.0, elevation=-89.9),
            _free_camera(lookat=(middle[0], middle[1], 0.07), distance=0.75, azimuth=90.0, elevation=-12.0),
        ]
        self.outline = sector_outline(bounds)

    def render(self, data: mujoco.MjData, target=None, colour=GREY) -> list[np.ndarray]:
        """RGB images of both views; `target` (a so101_hand Point) is marked on the floor and at its height."""
        images = []
        for renderer, camera in zip(self.renderers, self.cameras):
            renderer.update_scene(data, camera)
            scene = renderer.scene
            for a, b in zip(self.outline[:-1], self.outline[1:]):
                _add_line(scene, a, b, GREY, 0.002)
            if target is not None:
                top = to_world(target)
                floor = np.array([top[0], top[1], 0.0005])
                _add_line(scene, floor, top, colour, 0.0015)
                _add_line(scene, floor - (0.012, 0, 0), floor + (0.012, 0, 0), colour, 0.002)
                _add_line(scene, floor - (0, 0.012, 0), floor + (0, 0.012, 0), colour, 0.002)
            images.append(renderer.render())
        return images

    def close(self) -> None:
        for renderer in self.renderers:
            renderer.close()


def _free_camera(lookat, distance: float, azimuth: float, elevation: float) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = lookat
    camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
    return camera


def _add_line(scene: mujoco.MjvScene, a, b, rgba, radius: float) -> None:
    """A thin capsule from `a` to `b`, drawn in this render only (not part of the model)."""
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                        np.asarray(rgba, dtype=np.float32))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, np.asarray(a, float), np.asarray(b, float))
    scene.ngeom += 1
