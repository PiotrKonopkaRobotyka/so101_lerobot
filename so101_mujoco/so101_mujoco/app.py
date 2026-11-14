"""so101_mujoco: the hand in front of the webcam drives SO-101 in so101-nexus's MuJoCo pick and place (ROB-38/39).

One window: the preview (the mirrored webcam with the workspace sector, as for Gazebo) and the scene from above and
from the robot's right side. The keys of so101_hand.session: space = hover or grasp height, p = follow the hand or
pause, h = home, c / o = calibrate the closed / open hand, q = quit; and r = the next episode (another cube and
disc). The simulation runs in real time with the camera, at 30 steps per second.

With --record REPO_ID every attempt becomes a LeRobot episode (so101_mujoco.record): it starts with the first p,
is saved 1 s after the success and goes on to the next cube; r discards it (a failed attempt). Without --record a
success shows the time since the episode began; with the webcam it is added to
$DATA_DIR/so101_mujoco/pick_and_place.csv.

    so101_mujoco [--record REPO_ID] [--video FILE] [--camera N] [--seed N] [--follow] [--headless OUT.mp4]
"""

import argparse
import csv
import os
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")  # off-screen rendering on the GPU; set before mujoco is imported

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from so101_hand import preview  # noqa: E402
from so101_hand.kinematics import forward3d  # noqa: E402
from so101_hand.session import Session  # noqa: E402
from so101_hand.tracker import DEFAULT_MODEL, HandTracker  # noqa: E402
from so101_mujoco.scene import GREEN, GREY, RED, Views, joint_addresses, make_env, mujoco_settings  # noqa: E402
from so101_mujoco.world import joint_targets  # noqa: E402

WINDOW = "so101_mujoco"
MAX_STEPS_PER_FRAME = 15  # 0.5 s of simulation: a longer pause of the loop is not caught up
AFTER_SUCCESS_S = 1.0  # the recording goes on this long after the success: the release and the gripper rising
LOG = Path(os.environ.get("DATA_DIR", "~/data")).expanduser() / "so101_mujoco" / "pick_and_place.csv"


class App:
    def __init__(self, args: argparse.Namespace):
        self.recorder = None
        if args.record:
            from so101_mujoco import record
            from so101_nexus.observations import privileged_state_feature_names

            self.env = make_env(cameras=True)
            self.sim = self.env.unwrapped
            names = privileged_state_feature_names(self.sim.config.observations)
            csv_path = record.metrics_csv(args.record)
            gripper = (float(self.env.action_space.low[5]), float(self.env.action_space.high[5]))
            self.recorder = record.Recorder(record.open_dataset(args.record, names), gripper,
                                            self.sim.task_description, csv_path)
            self.episode = args.seed if args.seed is not None else record.next_seed(csv_path)
            print(f"recording into {args.record}: {self.recorder.dataset.meta.total_episodes} episodes so far, "
                  f"attempts in {csv_path}")
        else:
            self.env = make_env()
            self.sim = self.env.unwrapped
            self.episode = args.seed or 0
        self.addresses = joint_addresses(self.sim.model)
        self.views = Views(self.sim.model)
        self.session = Session(mujoco_settings())
        self.tracker = HandTracker(Path(args.model) if args.model else DEFAULT_MODEL)
        self.auto_follow = args.follow
        self.headless = args.headless
        self.log = not args.video and not args.headless and not args.record  # plain webcam sessions only
        if args.video:  # a recorded video instead of the webcam
            self.cap = cv2.VideoCapture(args.video)
            self.frame_period_s = 1.0 / (self.cap.get(cv2.CAP_PROP_FPS) or 20.0)
        else:
            self.cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.frame_period_s = 0.0  # the camera sets the pace
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {args.video or f'camera {args.camera}'}")
        self.video = args.video
        self.writer = None
        self.successes: list[float] = []
        self.saved = self.discarded = 0
        self.reset()

    # --- episodes ---

    def reset(self) -> None:
        """A new episode: so101-nexus places the cube and the disc; the arm starts at the home pose, paused."""
        self.env.reset(seed=self.episode)
        q = self.session.go_home()
        model, data = self.sim.model, self.sim.data
        action = joint_targets(q, self.session.gripper_rad)
        data.qpos[self.addresses] = action
        data.qvel[:] = 0.0
        data.ctrl[: len(action)] = action
        mujoco.mj_forward(model, data)
        self.obs, _, _, _, self.info = self.env.step(action)  # a fresh observation of the arm at home
        self.session.joints = tuple(q)
        self.start_s = None  # the episode's clock starts with its first frame
        self.sim_s = 0.0
        self.success_s = None  # simulated time of the success
        self.success_shown = False
        self.recording_s = None  # simulated time the recording started (the first p)

    def next_episode(self) -> None:
        self.episode += 1
        self.reset()

    def save(self) -> None:
        seconds = self.success_s - self.recording_s
        self.show_message("saving the episode ...")
        index = self.recorder.save(self.episode, seconds)
        self.saved += 1
        self.successes.append(seconds)
        print(f"episode {index} saved (seed {self.episode}): pick and place in {seconds:.1f} s")
        self.next_episode()

    def discard(self) -> None:
        self.recorder.discard(self.episode, self.sim_s - self.recording_s)
        self.discarded += 1
        print(f"attempt with seed {self.episode} discarded")

    # --- the loop ---

    def step_to(self, episode_s: float) -> None:
        """Advance the simulation to `episode_s` seconds of the episode with the session's current targets,
        recording each step's observation and action while an attempt is being recorded."""
        action = joint_targets(self.session.current_arm(), self.session.gripper_rad)
        steps = 0
        while self.sim_s < episode_s - 1e-9 and steps < MAX_STEPS_PER_FRAME:
            if self.recording_s is not None:
                state = self.sim.data.qpos[self.addresses]
                self.recorder.add(self.obs, state, self.info["privileged_state"], action)
            self.obs, _, _, _, self.info = self.env.step(action)
            self.sim_s += self.sim.control_dt
            steps += 1
            if self.info.get("success") and self.success_s is None:
                self.success_s = self.sim_s
        if self.sim_s < episode_s - 1e-9:  # the loop stalled: drop the time it lost
            self.start_s += episode_s - self.sim_s
        self.session.joints = tuple(float(self.sim.data.qpos[a]) for a in self.addresses[:4])

    def run(self) -> None:
        frame_index = 0
        try:
            while True:
                ok, frame = self.cap.read()
                if not ok:
                    print(f"no more frames from {self.video or 'the camera'}")
                    break
                # the video's own time when it is processed off-line, else the clock
                t_s = frame_index * self.frame_period_s if self.headless else time.monotonic()
                frame_index += 1
                session = self.session
                session.update(self.tracker.detect(frame, t_s), t_s)
                if self.auto_follow and not session.follower.following and session.palm_px is not None:
                    session.resume()  # like pressing p with the hand inside the sector
                if self.recorder and self.recording_s is None and session.follower.following:
                    self.recording_s = self.sim_s  # the attempt starts with the first p
                if self.start_s is None:
                    self.start_s = t_s
                self.step_to(t_s - self.start_s)
                if self.success_s is not None:
                    if self.recorder is None and not self.success_shown:
                        self.success_shown = True
                        self.on_success(self.success_s)
                    elif self.recorder and self.sim_s >= self.success_s + AFTER_SUCCESS_S:
                        self.save()
                        continue
                if not self.show(frame, t_s):
                    break
                if self.frame_period_s and not self.headless:  # play a video in real time
                    time.sleep(max(0.0, self.frame_period_s - (time.monotonic() - t_s)))
        finally:
            self.close()

    def on_success(self, seconds: float) -> None:
        self.successes.append(seconds)
        print(f"episode {self.episode}: pick and place in {seconds:.1f} s")
        if self.log:
            LOG.parent.mkdir(parents=True, exist_ok=True)
            new = not LOG.exists()
            with LOG.open("a", newline="") as file:
                writer = csv.writer(file)
                if new:
                    writer.writerow(["time", "episode", "seconds"])
                writer.writerow([datetime.now().isoformat(timespec="seconds"), self.episode, f"{seconds:.1f}"])

    # --- the window ---

    def status(self, t_s: float) -> tuple[str, ...]:
        if self.recorder is None:
            line = f"episode {self.episode}: {t_s - self.start_s:.0f} s"
            if self.success_s is not None:
                line += f", SUCCESS in {self.success_s:.1f} s (r = next)"
            return line, "r = next episode"
        saved = f"saved {self.saved}, discarded {self.discarded} ({self.recorder.dataset.meta.total_episodes} in all)"
        if self.recording_s is None:
            return f"seed {self.episode}: p starts the recording", saved
        if self.success_s is not None:
            return f"REC {self.sim_s - self.recording_s:.0f} s: SUCCESS, saving ...", saved
        return f"REC {self.sim_s - self.recording_s:.0f} s (r = discard)", saved

    def show(self, frame: np.ndarray, t_s: float) -> bool:
        """Draw the window (or write it to the video) and handle the keys; False after q."""
        session = self.session
        inside = session.palm_px is not None and session.follower.inside(session.palm_px, session.levels[session.level])
        colour = GREEN if session.follower.following else GREY if inside else RED
        target = session.target
        if target is None and session.current_arm() is not None:  # paused: where the gripper is going
            target = forward3d(*session.current_arm())[0]
        top, side = self.views.render(self.sim.data, target=target, colour=colour)
        view = preview.draw(session, frame, extra=self.status(t_s))
        self.window = np.hstack([cv2.resize(view, (640, 480)),
                                 *(cv2.cvtColor(v, cv2.COLOR_RGB2BGR) for v in (top, side))])
        if self.headless:
            if self.writer is None:
                fps = 1.0 / self.frame_period_s if self.frame_period_s else 20.0
                height, width = self.window.shape[:2]
                self.writer = cv2.VideoWriter(self.headless, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
            self.writer.write(self.window)
            return True
        cv2.imshow(WINDOW, self.window)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            if self.recorder and self.recording_s is not None:
                self.discard()
            self.next_episode()
            return True
        action, message = session.handle_key(key, time.monotonic())
        if message:
            print(message)
        if action == "home":
            session.go_home()
        return action != "quit"

    def show_message(self, text: str) -> None:
        """A line over the last window, e.g. while an episode is encoded (a few seconds)."""
        if self.headless or getattr(self, "window", None) is None:
            return
        window = self.window.copy()
        cv2.putText(window, text, (660, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.imshow(WINDOW, window)
        cv2.waitKey(1)

    def close(self) -> None:
        if self.recorder is not None:
            if self.recording_s is not None and self.success_s is not None:  # q right after a success
                self.recorder.save(self.episode, self.success_s - self.recording_s)
                self.saved += 1
            elif self.recording_s is not None:
                self.discard()  # q (or an error) in the middle of an attempt
            self.recorder.close()
            print(f"dataset: {self.recorder.dataset.meta.total_episodes} episodes; this session {self.saved} saved, "
                  f"{self.discarded} discarded")
        if self.successes:
            print(f"{len(self.successes)} pick and places, {np.mean(self.successes):.1f} s on average")
        self.cap.release()
        self.tracker.close()
        self.views.close()
        self.env.close()
        if self.writer is not None:
            self.writer.release()
        if not self.headless:
            cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--record", metavar="REPO_ID", help="record the attempts into this LeRobot dataset (user/name)")
    parser.add_argument("--video", help="a recorded video instead of the webcam")
    parser.add_argument("--camera", type=int, default=0, help="the webcam's index (/dev/videoN)")
    parser.add_argument("--seed", type=int, help="the first episode's seed (default 0, or after the last attempt)")
    parser.add_argument("--model", help="the Gesture Recognizer model (default $DATA_DIR/models/...)")
    parser.add_argument("--follow", action="store_true", help="follow the hand once it is in the sector")
    parser.add_argument("--headless", metavar="OUT.mp4", help="no window: write the window to this video")
    App(parser.parse_args()).run()
