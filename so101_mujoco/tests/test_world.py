import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("so101_nexus")

from so101_hand.kinematics import JOINT_LIMITS, PAN_LIMITS, Point, forward3d  # noqa: E402
from so101_mujoco.scene import joint_addresses, make_env  # noqa: E402
from so101_mujoco.world import PAN_AXIS_WORLD, joint_targets, to_table, to_world  # noqa: E402
from so101_nexus.config import SO101_TCP_SITE_NAME  # noqa: E402


@pytest.fixture(scope="module")
def sim():
    env = make_env()
    return env.unwrapped.model, env.unwrapped.data


def test_to_world_and_back():
    point = Point(forward=0.21, right=-0.05, height=0.10)
    assert tuple(to_table(to_world(point))) == pytest.approx(tuple(point), abs=1e-12)


def test_the_pan_axis_is_shoulder_pan(sim):
    model, data = sim
    mujoco.mj_kinematics(model, data)
    data_anchor = data.xanchor[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder_pan")]
    assert tuple(data_anchor[:2]) == pytest.approx(PAN_AXIS_WORLD, abs=1e-5)


def test_so101_hand_kinematics_put_its_tcp_where_mujoco_does(sim):
    # so101_hand's joints 1-4 sent as they are (and the wrist roll at the Gazebo zero) put so101_hand's TCP on
    # so101-nexus's TCP site, but for the site's 0.2 mm out of the arm's plane
    model, data = sim
    addresses = joint_addresses(model)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SO101_TCP_SITE_NAME)
    lower, upper = zip(PAN_LIMITS, *(JOINT_LIMITS[j] for j in ("2", "3", "4")))
    for q in np.random.default_rng(0).uniform(lower, upper, size=(200, 4)):
        data.qpos[addresses] = joint_targets(q, 0.0)
        mujoco.mj_kinematics(model, data)
        assert np.linalg.norm(to_world(forward3d(*q)[0]) - data.site_xpos[site]) < 0.5e-3
