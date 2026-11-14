import csv
import os

import numpy as np
import pytest

pytest.importorskip("lerobot")
pytest.importorskip("so101_nexus")
if os.environ.get("MUJOCO_GL") == "disable":
    pytest.skip("the recorded cameras need rendering", allow_module_level=True)

from scripted import pick_and_place_plan  # noqa: E402  (tests/scripted.py)
from so101_mujoco import record  # noqa: E402
from so101_mujoco.scene import FPS, joint_addresses, make_env  # noqa: E402
from so101_nexus.observations import privileged_state_feature_names  # noqa: E402


def scripted_pick_and_place(env, seed: int):
    """Steps (observation, state, full state, action) of the scripted top-down pick and place, until 1 s after the
    success; the success's step index (or None)."""
    env.reset(seed=seed)
    sim = env.unwrapped
    addresses = joint_addresses(sim.model)
    first, *actions = pick_and_place_plan(env)
    obs, _, _, _, info = env.step(first)
    steps, success = [], None
    for action in actions:
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


def test_a_later_session_adds_episodes_to_the_dataset(tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    env = make_env(cameras=True)
    sim = env.unwrapped
    names = privileged_state_feature_names(sim.config.observations)
    gripper = (float(env.action_space.low[5]), float(env.action_space.high[5]))
    root = tmp_path / "dataset"
    steps, _ = scripted_pick_and_place(env, seed=5)
    for session_steps in (steps[:40], steps[40:70]):  # two sessions, one episode each
        recorder = record.Recorder(record.open_dataset("local/so101_test", names, root=root), gripper,
                                   sim.task_description, tmp_path / "attempts.csv")
        for step in session_steps:
            recorder.add(*step)
        recorder.save(seed=5, seconds=1.0)
        recorder.close()
    env.close()
    dataset = LeRobotDataset("local/so101_test", root=root)
    assert dataset.meta.total_episodes == 2 and len(dataset) == 70
