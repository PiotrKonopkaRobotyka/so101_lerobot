import math

import numpy as np
import pytest

pytest.importorskip("so101_nexus")

from so101_hand.gripper import CLOSED_RAD, OPEN_RAD  # noqa: E402
from so101_hand.kinematics import PAN_AXIS, Point  # noqa: E402
from so101_hand.session import Session  # noqa: E402
from so101_hand.teleop import Bounds  # noqa: E402
from so101_mujoco.scene import joint_addresses, make_env, mujoco_settings, sector_outline  # noqa: E402
from so101_mujoco.world import FLOOR_HEIGHT, joint_targets, to_table  # noqa: E402


@pytest.fixture(scope="module")
def env():
    return make_env()


def test_the_cube_and_the_disc_start_in_the_sector(env):
    sim, bounds = env.unwrapped, Bounds()
    for seed in range(100):
        env.reset(seed=seed)
        # so101-nexus 0.11.0's own accessors (pinned version): the cube's pose and the disc's centre
        for xyz in (sim._get_object_pose()[:3], sim._get_target_pos()):
            point = to_table(xyz)
            assert bounds.contains(Point(point.forward, point.right, 0.1), tolerance=1e-4), (seed, xyz)


def test_the_sector_drawn_on_the_floor_is_the_bounds():
    bounds = Bounds()
    for xyz in sector_outline(bounds):
        point = to_table(xyz)
        reach = math.hypot(point.forward - PAN_AXIS[0], point.right - PAN_AXIS[1])
        pan = math.atan2(point.right - PAN_AXIS[1], point.forward - PAN_AXIS[0])
        assert bounds.reach[0] - 1e-6 <= reach <= bounds.reach[1] + 1e-6
        assert bounds.pan[0] - 1e-6 <= pan <= bounds.pan[1] + 1e-6


def test_the_heights_are_above_mujocos_floor():
    settings = mujoco_settings()
    assert settings.grasp_height - FLOOR_HEIGHT == pytest.approx(0.0125)  # the cube's centre
    assert settings.bounds_height[0] < settings.grasp_height < settings.hover_height
    assert settings.floor_height == FLOOR_HEIGHT


def test_a_top_down_grasp_at_the_tcp_lifts_the_cube(env):
    # the session's IK straight above the cube, down to the grasp height, close, lift: with the TCP on the wrist
    # roll axis 0-4 cubes of 20 were lifted, with the TCP between the open jaws 37 of 40 (ROB-38)
    sim = env.unwrapped
    addresses, settings = joint_addresses(sim.model), mujoco_settings()
    lifted = 0
    for seed in range(10):
        env.reset(seed=seed)
        session = Session(settings)
        session.joints = tuple(float(sim.data.qpos[a]) for a in addresses[:4])
        cube = sim._get_object_pose()[:3].copy()
        above = to_table(cube)

        def go(height: float, gripper: float, steps: int) -> None:
            q = session.solve(Point(above.forward, above.right, height))
            assert q is not None
            session.arm_command = q
            for _ in range(steps):
                env.step(joint_targets(q, gripper))

        go(settings.hover_height, OPEN_RAD, 60)
        for height in np.linspace(settings.hover_height, settings.grasp_height, 20):
            go(height, OPEN_RAD, 1)
        go(settings.grasp_height, OPEN_RAD, 15)
        go(settings.grasp_height, CLOSED_RAD, 25)
        for height in np.linspace(settings.grasp_height, settings.hover_height, 25):
            go(height, CLOSED_RAD, 1)
        go(settings.hover_height, CLOSED_RAD, 25)
        lifted += sim._get_object_pose()[2] - cube[2] > 0.03
    assert lifted >= 8
