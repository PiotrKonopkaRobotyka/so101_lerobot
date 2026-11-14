import os

import numpy as np
import pytest

pytest.importorskip("so101_nexus")
if os.environ.get("MUJOCO_GL") == "disable":
    pytest.skip("the policy's cameras need rendering", allow_module_level=True)

from scripted import pick_and_place_plan  # noqa: E402  (tests/scripted.py)
from so101_mujoco import evaluate  # noqa: E402
from so101_mujoco.scene import FPS, make_env  # noqa: E402
from so101_nexus.lerobot_dataset import sim_qpos_to_dataset_row  # noqa: E402

HOME_ROW = [0.0, -2.7, 5.4, 87.3, 2.8, 99.0]  # the first frame of every recorded episode: degrees, gripper %


@pytest.fixture(scope="module")
def env():
    env = make_env(cameras=True)
    yield env
    env.close()


class Scripted:
    """The scripted pick and place as a policy in the dataset's units: it reads the scene, not the cameras."""

    def __init__(self, env):
        self.env = env
        self.gripper = (float(env.action_space.low[5]), float(env.action_space.high[5]))
        self.frames = []

    def reset(self):
        self.actions = [sim_qpos_to_dataset_row(a.astype(np.float64), gripper_limits_rad=self.gripper)
                        for a in pick_and_place_plan(self.env)]
        self.step = 0

    def __call__(self, frame):
        self.frames.append(frame)
        action = self.actions[min(self.step, len(self.actions) - 1)]
        self.step += 1
        return action


class Still:
    """Keeps the joints where they are."""

    def reset(self):
        pass

    def __call__(self, frame):
        return frame["observation.state"]


def test_a_pick_and_place_in_the_datasets_units_succeeds(env):
    policy = Scripted(env)
    episode = evaluate.run_episode(env, policy, seed=2, max_steps=20 * FPS)
    assert episode.success and episode.seconds < 15
    first = policy.frames[0]
    assert set(first) == {"observation.state", "observation.images.wrist", "observation.images.overhead"}
    for key in ("observation.images.wrist", "observation.images.overhead"):
        assert first[key].shape == (480, 640, 3) and first[key].dtype == np.uint8  # as the dataset's frames
    np.testing.assert_allclose(first["observation.state"], HOME_ROW, atol=0.1)  # starts as the recordings


def test_a_policy_that_does_not_move_fails_at_the_time_limit(env):
    episode = evaluate.run_episode(env, Still(), seed=2, max_steps=30)
    assert not episode.success and episode.seconds == pytest.approx(1.0)


def test_a_policy_saved_by_lerobot_runs_in_the_scene(env, tmp_path):
    torch = pytest.importorskip("torch")
    pytest.importorskip("lerobot")
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    # an untrained, small ACT with the pilot's inputs, saved as lerobot-train saves it
    image = PolicyFeature(type=FeatureType.VISUAL, shape=(3, 480, 640))
    config = ACTConfig(
        device="cpu",
        input_features={"observation.state": PolicyFeature(type=FeatureType.STATE, shape=(6,)),
                        "observation.images.wrist": image, "observation.images.overhead": image},
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(6,))},
        pretrained_backbone_weights=None, dim_model=32, n_heads=2, dim_feedforward=64, n_encoder_layers=1,
        n_vae_encoder_layers=1, latent_dim=4, chunk_size=10, n_action_steps=10,
    )
    images = {f"observation.images.{key}": {"mean": torch.full((3, 1, 1), 0.5), "std": torch.full((3, 1, 1), 0.25)}
              for key in ("wrist", "overhead")}
    stats = {"observation.state": {"mean": torch.tensor(HOME_ROW), "std": torch.full((6,), 30.0)},
             "action": {"mean": torch.tensor(HOME_ROW), "std": torch.full((6,), 30.0)}, **images}
    preprocessor, postprocessor = make_pre_post_processors(config, dataset_stats=stats)
    ACTPolicy(config).save_pretrained(tmp_path)
    preprocessor.save_pretrained(tmp_path)
    postprocessor.save_pretrained(tmp_path)

    class Recording:
        def __init__(self, inner):
            self.inner, self.actions = inner, []

        def reset(self):
            self.inner.reset()

        def __call__(self, frame):
            self.actions.append(self.inner(frame))
            return self.actions[-1]

    chunks = Recording(evaluate.LeRobotPolicy(tmp_path, device="cpu"))
    episode = evaluate.run_episode(env, chunks, seed=3, max_steps=12)  # two chunks of 10
    assert not episode.success and len(chunks.actions) == 12
    ensembled = Recording(evaluate.LeRobotPolicy(tmp_path, device="cpu", temporal_ensemble=0.01))
    evaluate.run_episode(env, ensembled, seed=3, max_steps=3)  # a chunk every step, averaged
    assert ensembled.inner.policy.config.n_action_steps == 1 and len(ensembled.actions) == 3
    for recording in (chunks, ensembled):
        assert all(a.shape == (6,) and np.all(np.isfinite(a)) for a in recording.actions)
