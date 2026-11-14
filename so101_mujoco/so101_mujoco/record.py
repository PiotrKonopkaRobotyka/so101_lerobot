"""The demonstrations as a LeRobot dataset in the real SO-101 follower's schema (ROB-39, ADR 0010).

Per frame at FPS: the action (the joint targets sent this step) and the observation before it (the joints, the
wrist and overhead cameras, so101-nexus's full state). Joints are degrees and the gripper 0-100, as LeRobot's SO-101
follower and so101-nexus's datasets store them. An episode is a whole attempt, saved after a success or discarded.
Each attempt is also a row of a CSV next to the dataset's name in $DATA_DIR/so101_mujoco, for the metrics.
"""

import csv
import os
from datetime import datetime
from pathlib import Path

import numpy as np

from so101_mujoco.scene import CAMERA_SIZE, FPS
from so101_nexus.config import SO101_JOINT_NAMES
from so101_nexus.lerobot_dataset import sim_qpos_to_dataset_row

ROBOT_TYPE = "so101_follower"  # LeRobot's SO-101 follower
JOINTS = [f"{name}.pos" for name in SO101_JOINT_NAMES]  # shoulder_pan.pos … gripper.pos
CAMERAS = {"wrist": "wrist_camera", "overhead": "overhead_camera"}  # dataset key → so101-nexus observation key
METRICS = Path(os.environ.get("DATA_DIR", "~/data")).expanduser() / "so101_mujoco"
CSV_FIELDS = ["time", "seed", "outcome", "seconds", "frames", "dataset_episode"]


def features(state_names: list[str]) -> dict:
    width, height = CAMERA_SIZE
    image = {"dtype": "video", "shape": (height, width, 3), "names": ["height", "width", "channels"]}
    return {
        "action": {"dtype": "float32", "shape": (len(JOINTS),), "names": JOINTS},
        "observation.state": {"dtype": "float32", "shape": (len(JOINTS),), "names": JOINTS},
        **{f"observation.images.{key}": dict(image) for key in CAMERAS},
        "observation.environment_state": {"dtype": "float32", "shape": (len(state_names),), "names": state_names},
    }


def open_dataset(repo_id: str, state_names: list[str], root: Path | None = None):
    """A new dataset, or the existing one to add episodes to (recording can go on over several sessions)."""
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.utils.constants import HF_LEROBOT_HOME

    writing = dict(image_writer_threads=4 * len(CAMERAS), rgb_encoder=RGBEncoderConfig(vcodec="h264"))
    root = Path(root) if root else HF_LEROBOT_HOME / repo_id  # LeRobot 0.7 resumes only with an explicit root
    if root.exists():
        dataset = LeRobotDataset.resume(repo_id, root=root, **writing)
        if dataset.fps != FPS or dataset.features.get("action", {}).get("names") != JOINTS:
            raise ValueError(f"{repo_id} was recorded with another schema or rate")
        return dataset
    return LeRobotDataset.create(repo_id, FPS, features(state_names), root=root, robot_type=ROBOT_TYPE, **writing)


class Recorder:
    """Frames of the current attempt into the dataset; the attempts into the metrics CSV."""

    def __init__(self, dataset, gripper_limits_rad: tuple[float, float], task: str, csv_path: Path):
        self.dataset = dataset
        self.gripper_limits = gripper_limits_rad
        self.task = task
        self.csv_path = csv_path
        self.frames = 0

    def row(self, qpos_rad) -> np.ndarray:
        """Simulator joint radians → the dataset's degrees and gripper percent."""
        return sim_qpos_to_dataset_row(np.asarray(qpos_rad, dtype=np.float64), gripper_limits_rad=self.gripper_limits)

    def add(self, obs: dict, state_rad, privileged, action_rad) -> None:
        """One frame: the observation `obs` (and the joints, the full state) before `action_rad` was applied."""
        frame = {
            "action": self.row(action_rad).astype(np.float32),
            "observation.state": self.row(state_rad).astype(np.float32),
            **{f"observation.images.{key}": np.array(obs[source], copy=True) for key, source in CAMERAS.items()},
            "observation.environment_state": np.asarray(privileged, dtype=np.float32),
            "task": self.task,
        }
        self.dataset.add_frame(frame)
        self.frames += 1

    def save(self, seed: int, seconds: float) -> int:
        """Keep the attempt as the dataset's next episode; returns its index."""
        index = self.dataset.meta.total_episodes
        self.dataset.save_episode()
        self.log(seed, "saved", seconds, index)
        return index

    def discard(self, seed: int, seconds: float) -> None:
        """Drop the attempt (it counts as a failure in the metrics)."""
        if self.frames:
            self.dataset.clear_episode_buffer()
        self.log(seed, "discarded", seconds, "")

    def log(self, seed: int, outcome: str, seconds: float, index) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        new = not self.csv_path.exists()
        with self.csv_path.open("a", newline="") as file:
            writer = csv.writer(file)
            if new:
                writer.writerow(CSV_FIELDS)
            writer.writerow([datetime.now().isoformat(timespec="seconds"), seed, outcome, f"{seconds:.1f}",
                             self.frames, index])
        self.frames = 0

    def close(self) -> None:
        """Finalize the dataset: without this its parquet files are invalid."""
        self.dataset.finalize()


def metrics_csv(repo_id: str) -> Path:
    return METRICS / f"{repo_id.split('/')[-1]}_attempts.csv"


def next_seed(csv_path: Path) -> int:
    """Continue after the last attempt's seed, so that a new session gets new cube and disc positions."""
    if not csv_path.exists():
        return 0
    with csv_path.open() as file:
        seeds = [int(row["seed"]) for row in csv.DictReader(file)]
    return max(seeds) + 1 if seeds else 0
