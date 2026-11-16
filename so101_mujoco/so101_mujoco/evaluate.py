"""A trained policy picks and places in MuJoCo on its own: its success rate (ROB-40).

Every episode is a new cube and disc, one seed each from --seed on (the recordings' seeds start at 0), and starts as
the recorded ones do: the arm at home, the gripper open. Every step the policy gets what a dataset frame holds, the
wrist and overhead cameras and the joints in the dataset's units (degrees, the gripper 0-100), and returns the joint
targets in the same units for one simulation step (1/30 s). An episode ends with so101-nexus's success (the cube
released on the disc) or after --max-seconds. The task does not ask how the cube got there, so each episode also
notes how far the cube rose: carried, or only pushed onto the disc from next to it. Each episode is a row of
$DATA_DIR/so101_mujoco/eval.csv; --video writes what the policy saw (overhead | wrist). LeRobot's ACT runs each
chunk of 100 actions to its end; with --temporal-ensemble it predicts a chunk every step and averages the
overlapping ones (the ACT paper's inference).

    so101_mujoco_eval POLICY_DIR [--episodes 50] [--seed 1000] [--max-seconds 60] [--video OUT.mp4]
                      [--temporal-ensemble 0.01] [--device cuda]

POLICY_DIR is a pretrained_model directory written by lerobot-train: <output_dir>/checkpoints/last/pretrained_model.
"""

import argparse
import csv
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Protocol

os.environ.setdefault("MUJOCO_GL", "egl")  # off-screen rendering on the GPU; set before mujoco is imported

import numpy as np  # noqa: E402

from so101_mujoco.record import CAMERAS, METRICS  # noqa: E402
from so101_mujoco.scene import CAMERA_SIZE, FPS, home_pose, joint_addresses, make_env, reset_at_home  # noqa: E402
from so101_nexus.lerobot_dataset import dataset_row_to_sim_qpos, sim_qpos_to_dataset_row  # noqa: E402
from so101_nexus.observations import privileged_state_feature_names  # noqa: E402

RESULTS = METRICS / "eval.csv"
CSV_FIELDS = ["time", "policy", "seed", "success", "seconds", "lift_mm"]
LIFTED = 0.02  # m: a cube that rose this much was carried; pushed or tipped over, the 25 mm cube rises under 10 mm


class Policy(Protocol):
    """A dataset frame without its action → the action, both in the dataset's keys and units."""

    def reset(self) -> None: ...

    def __call__(self, frame: dict[str, np.ndarray]) -> np.ndarray: ...


@dataclass
class Episode:
    seed: int
    success: bool
    seconds: float  # simulated time to the success, or of the whole episode
    lift: float  # m: the most the cube rose above where it started

    @property
    def carried(self) -> bool:
        """A success with the cube lifted, not pushed onto the disc."""
        return self.success and self.lift >= LIFTED


def run_episode(env, policy: Policy, seed: int, max_steps: int,
                on_step: Callable[[dict, float], None] | None = None) -> Episode:
    """One episode of `policy` from the home pose, until the success or `max_steps` steps; `on_step` gets each new
    observation and its time. `env` is `make_env(cameras=True)`: the policy's images and the simulator's state."""
    sim = env.unwrapped
    addresses = joint_addresses(sim.model)
    low, high = env.action_space.low, env.action_space.high
    gripper = (float(low[5]), float(high[5]))
    height = privileged_state_feature_names(sim.config.observations).index("object_pose_2")  # the cube's z
    obs, info = reset_at_home(env, seed, *home_pose())
    start = top = float(info["privileged_state"][height])
    policy.reset()
    for step in range(1, max_steps + 1):
        state = sim_qpos_to_dataset_row(sim.data.qpos[addresses].astype(np.float64), gripper_limits_rad=gripper)
        frame = {"observation.state": state.astype(np.float32),
                 **{f"observation.images.{key}": obs[source] for key, source in CAMERAS.items()}}
        action = dataset_row_to_sim_qpos(np.asarray(policy(frame), dtype=np.float64), gripper_limits_rad=gripper)
        obs, _, _, _, info = env.step(np.clip(action, low, high).astype(np.float32))
        top = max(top, float(info["privileged_state"][height]))
        if on_step is not None:
            on_step(obs, step / FPS)
        if info["success"]:
            return Episode(seed, True, step / FPS, top - start)
    return Episode(seed, False, max_steps / FPS, top - start)


class LeRobotPolicy:
    """A policy saved by lerobot-train (its pretrained_model directory), with the pre- and postprocessor saved next
    to it: the normalization by the dataset's statistics, the batch dimension, the device."""

    def __init__(self, path: Path, device: str = "cuda", temporal_ensemble: float | None = None):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies import get_policy_class, make_pre_post_processors

        self.torch = torch
        config = PreTrainedConfig.from_pretrained(path)
        config.device = device
        if temporal_ensemble is not None:  # ACT: a chunk every step, the overlapping actions weighted exp(-coeff·age)
            config.temporal_ensemble_coeff, config.n_action_steps = temporal_ensemble, 1
        self.policy = get_policy_class(config.type).from_pretrained(path, config=config)
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            config, pretrained_path=path, preprocessor_overrides={"device_processor": {"device": device}})

    def reset(self) -> None:
        self.policy.reset()  # ACT: drops what is left of the last action chunk

    def __call__(self, frame: dict[str, np.ndarray]) -> np.ndarray:
        torch = self.torch
        batch = {}
        for key, value in frame.items():
            tensor = torch.from_numpy(np.ascontiguousarray(value))
            if key.startswith("observation.images."):  # HWC uint8 → CHW in [0, 1], as the dataset decodes its videos
                tensor = tensor.permute(2, 0, 1).float() / 255.0
            batch[key] = tensor
        with torch.inference_mode():
            action = self.postprocessor(self.policy.select_action(self.preprocessor(batch)))
        return action.squeeze(0).numpy()


def log(policy: Path | str, episode: Episode, path: Path = RESULTS) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with path.open("a", newline="") as file:
        writer = csv.writer(file)
        if new:
            writer.writerow(CSV_FIELDS)
        writer.writerow([datetime.now().isoformat(timespec="seconds"), policy, episode.seed, int(episode.success),
                         f"{episode.seconds:.1f}", f"{1000 * episode.lift:.0f}"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("policy", type=Path, help="a pretrained_model directory written by lerobot-train")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=1000, help="the first episode's seed (the recordings start at 0)")
    parser.add_argument("--max-seconds", type=float, default=60.0, help="an episode without a success ends then")
    parser.add_argument("--video", metavar="OUT.mp4", help="write the policy's cameras (overhead | wrist) to a video")
    parser.add_argument("--temporal-ensemble", type=float, metavar="COEFF",
                        help="ACT: a new chunk every step, averaged with the earlier ones (the ACT paper: 0.01)")
    parser.add_argument("--device", default="cuda", help="where the policy runs: cuda or cpu")
    args = parser.parse_args()

    path = args.policy.expanduser().resolve()  # checkpoints/last is a link: the CSV names the step
    policy = LeRobotPolicy(path, args.device, args.temporal_ensemble)
    label = path if args.temporal_ensemble is None else f"{path} (temporal ensemble {args.temporal_ensemble})"
    env = make_env(cameras=True)
    writer = None
    if args.video:
        import cv2

        width, height = CAMERA_SIZE
        writer = cv2.VideoWriter(args.video, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (2 * width, height))
    episodes: list[Episode] = []
    started = time.monotonic()

    def write(obs, seconds):  # a video frame of the current episode
        writer.write(video_frame(obs, f"seed {seed}  {seconds:4.1f} s"))

    try:
        for seed in range(args.seed, args.seed + args.episodes):
            on_step = write if writer is not None else None
            episode = run_episode(env, policy, seed, round(args.max_seconds * FPS), on_step)
            episodes.append(episode)
            log(label, episode)
            done = sum(e.success for e in episodes)
            print(f"seed {seed}: {'success' if episode.success else 'failure'} after {episode.seconds:.1f} s, the "
                  f"cube rose {1000 * episode.lift:.0f} mm ({done}/{len(episodes)})", flush=True)
    finally:
        env.close()
        if writer is not None:
            writer.release()
    times = [e.seconds for e in episodes if e.success]
    summary = f"{len(times)} of {len(episodes)} episodes succeeded ({100 * len(times) / max(len(episodes), 1):.0f} %)"
    if times:
        summary += (f", {sum(e.carried for e in episodes)} of them with the cube lifted, in {np.mean(times):.1f} s "
                    f"on average ({min(times):.1f}–{max(times):.1f} s)")
    print(f"{summary}; evaluated in {time.monotonic() - started:.0f} s; rows in {RESULTS}")


def video_frame(obs: dict, text: str) -> np.ndarray:
    """The overhead and the wrist camera side by side (BGR for OpenCV), with `text` over them."""
    import cv2

    frame = cv2.cvtColor(np.hstack([obs[CAMERAS["overhead"]], obs[CAMERAS["wrist"]]]), cv2.COLOR_RGB2BGR)
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return frame


if __name__ == "__main__":
    main()
