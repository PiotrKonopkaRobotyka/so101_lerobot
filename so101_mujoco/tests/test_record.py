import csv
import os

import numpy as np
import pytest

pytest.importorskip("lerobot")
pytest.importorskip("so101_nexus")
if os.environ.get("MUJOCO_GL") == "disable":
    pytest.skip("the recorded cameras need rendering", allow_module_level=True)

from so101_hand.gripper import CLOSED_RAD, OPEN_RAD  # noqa: E402
from so101_hand.kinematics import Point  # noqa: E402
from so101_hand.session import Session  # noqa: E402
from so101_mujoco import record  # noqa: E402
from so101_mujoco.scene import FPS, joint_addresses, make_env, mujoco_settings  # noqa: E402
from so101_mujoco.world import joint_targets, to_table  # noqa: E402
from so101_nexus.observations import privileged_state_feature_names  # noqa: E402


def scripted_pick_and_place(env, seed: int):
    """Steps (observation, state, full state, action) of a scripted top-down pick and place, until 1 s after the
    success; the success's step index (or None)."""
    env.reset(seed=seed)
    sim = env.unwrapped
    addresses, settings = joint_addresses(sim.model), mujoco_settings()
    session = Session(settings)
    session.joints = tuple(float(sim.data.qpos[a]) for a in addresses[:4])
    cube, disc = to_table(sim._get_object_pose()[:3]), to_table(sim._get_target_pos())
    obs, _, _, _, info = env.step(joint_targets(session.solve(Point(cube.forward, cube.right, settings.hover_height)),
                                                OPEN_RAD))
    plan = []  # (point over the table, gripper, steps)
    for (forward, right), down_gripper, up_gripper in (((cube.forward, cube.right), OPEN_RAD, CLOSED_RAD),
                                                       ((disc.forward, disc.right), CLOSED_RAD, OPEN_RAD)):
        plan.append((Point(forward, right, settings.hover_height), down_gripper, 30))
        plan += [(Point(forward, right, h), down_gripper, 1) for h in np.linspace(settings.hover_height,
                                                                                    settings.grasp_height, 20)]
        plan.append((Point(forward, right, settings.grasp_height), up_gripper, 15))
        plan += [(Point(forward, right, h), up_gripper, 1) for h in np.linspace(settings.grasp_height,
                                                                                  settings.hover_height, 20)]
    plan.append((plan[-1][0], OPEN_RAD, 60))
    steps, success = [], None
    for point, gripper, n in plan:
        q = session.solve(point)
        session.arm_command = q
        action = joint_targets(q, gripper)
        for _ in range(n):
            steps.append((obs, sim.data.qpos[addresses].copy(), info["privileged_state"], action))
            obs, _, _, _, info = env.step(action)
            if info["success"] and success is None:
                success = len(steps)
            if success is not None and len(steps) >= success + FPS:
                return steps, success
    return steps, success


def test_an_attempt_is_saved_in_the_so101_schema_and_a_discarded_one_is_not(tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    env = make_env(cameras=True)
    sim = env.unwrapped
    names = privileged_state_feature_names(sim.config.observations)
    gripper = (float(env.action_space.low[5]), float(env.action_space.high[5]))
    root = tmp_path / "dataset"
    recorder = record.Recorder(record.open_dataset("local/so101_test", names, root=root), gripper,
                               sim.task_description, tmp_path / "attempts.csv")
    steps, success = scripted_pick_and_place(env, seed=2)
    assert success is not None, "the scripted pick and place did not succeed"
    for step in steps:
        recorder.add(*step)
    assert recorder.save(seed=2, seconds=success / FPS) == 0
    for step in steps[:20]:  # an attempt given up: r
        recorder.add(*step)
    recorder.discard(seed=3, seconds=20 / FPS)
    recorder.close()
    env.close()

    dataset = LeRobotDataset("local/so101_test", root=root)
    assert dataset.meta.total_episodes == 1 and len(dataset) == len(steps)
    assert dataset.fps == FPS and dataset.meta.robot_type == record.ROBOT_TYPE
    assert dataset.features["action"]["names"] == record.JOINTS
    frame = dataset[len(steps) // 2]
    assert tuple(frame["observation.images.wrist"].shape) == (3, 480, 640)  # decoded from the video, CHW
    assert tuple(frame["observation.images.overhead"].shape) == (3, 480, 640)
    actions = np.stack([dataset[i]["action"].numpy() for i in range(0, len(steps), 10)])
    assert np.all(np.abs(actions[:, :5]) < 180) and np.all((actions[:, 5] >= 0) & (actions[:, 5] <= 100))
    assert actions[:, 5].max() > 80 and actions[:, 5].min() < 15  # the gripper open and closed, in percent
    with (tmp_path / "attempts.csv").open() as file:
        rows = list(csv.DictReader(file))
    assert [r["outcome"] for r in rows] == ["saved", "discarded"]
    assert record.next_seed(tmp_path / "attempts.csv") == 4


def test_a_later_session_adds_episodes_to_the_dataset(tmp_path, monkeypatch):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # as `so101_mujoco --record` runs it: no root, so the dataset lives in $HF_HOME/lerobot/<repo_id>; here a
    # temporary one, in every module that keeps its own copy of the constant
    for module in ("lerobot.utils.constants", "lerobot.datasets.dataset_metadata"):
        monkeypatch.setattr(f"{module}.HF_LEROBOT_HOME", tmp_path)
    env = make_env(cameras=True)
    sim = env.unwrapped
    names = privileged_state_feature_names(sim.config.observations)
    gripper = (float(env.action_space.low[5]), float(env.action_space.high[5]))
    steps, _ = scripted_pick_and_place(env, seed=5)
    for session_steps in (steps[:40], steps[40:70]):  # two sessions, one episode each
        recorder = record.Recorder(record.open_dataset("local/so101_test", names), gripper,
                                   sim.task_description, tmp_path / "attempts.csv")
        for step in session_steps:
            recorder.add(*step)
        recorder.save(seed=5, seconds=1.0)
        recorder.close()
    env.close()
    dataset = LeRobotDataset("local/so101_test", root=tmp_path / "local/so101_test")
    assert dataset.meta.total_episodes == 2 and len(dataset) == 70
