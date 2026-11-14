# so101_lerobot — project context for Claude Code

Supplements `~/code/CLAUDE.md` (workflow, safety and naming rules). Plan and status live only in the Linear
project [SO-101 Sim Imitation Learning](https://linear.app/robotics-software-engineer/project/so-101-sim-imitation-learning-9a024fd3273a);
setup and commands for people are in [README.md](README.md).

## Goal

Train SO-101 manipulation in simulation without a leader arm: webcam hand teleoperation (M1–M2 in Gazebo, ADR 0001),
pick and place with ~50 demonstrations in MuJoCo (M3), ACT (M4), release (M5); exit criteria in ADR 0005.

## Packages in this repository

- `so101_bringup/` — `sim.launch.py` (alias `so101_sim`): Gazebo, controllers, MoveIt, RViz; the hand node instead of
  MoveIt with `moveit:=false hand:=true [arm:=true]` (alias `so101_hand`); our controllers at 100 Hz (ADR 0004).
- `so101_hand/` — plain Python, no `rclpy` (`COLCON_IGNORE`, `pip install -e`); `tracker` the only MediaPipe import,
  `kinematics` FK/IK (pan + planar), `teleop` table map and follower, `session` one frame of teleoperation and the
  keys (shared by both layers), `preview` its drawing; `tests/urdf_reference.py` = URDF FK.
- `so101_hand_ros/` — node `hand_teleop`: webcam (or `video:=`) → `so101_hand.session` → gripper and arm goals (joint
  5 at 0); p or `~/clutch` follows/pauses, space the height; parameters in `config/hand_teleop.yaml` (= `Settings`).
- `so101_mujoco/` — plain Python like `so101_hand`: the same session drives so101-nexus's PickAndPlace-v2 (`app`,
  command `so101_mujoco`); `world` frame and joint offsets, `scene` env at 30 Hz, heights, the home start and the
  operator's two views; `record` the LeRobot dataset (`--record`, ADR 0010); `evaluate` a trained policy's success
  rate (command `so101_mujoco_eval`).
- `lerobot_mimic/` — legacy YOLO pose (AGPL-3.0) arm mimicking, J2/J3 only; kept as an example.

## Third-party (pinned in `so101_lerobot.repos` or `requirements.txt`, never modified)

- `lerobot_ws` (Pavankv92, Apache-2.0): `lerobot_description` (URDF, Gazebo launch), `lerobot_controller` (JTC
  `arm_controller` on "1"–"5", `gripper_controller` on "6"), `lerobot_moveit` (groups `arm`, `gripper`; KDL IK).
- so101-nexus 0.11.0 (johnsutor, Apache-2.0, PyPI): the MuJoCo scene, task and success check (ADR 0009).

## Build, run, check

- Build: only the `moveit_ws` underlay and the venv sourced, `python -m colcon build` in `so101_ws` (README).
- Run: `so101_sim` or `so101_hand`; without aliases `ws so101` and `ros2 launch so101_bringup sim.launch.py`;
  MuJoCo: `so101_mujoco` (venv; no ROS).
- Check: `ros2 control list_controllers` shows 3 active controllers; Plan & Execute in RViz moves the arm.
- Tests: `python -m pytest src/so101_lerobot/so101_hand` (~2 s) and `…/so101_mujoco` (~30 s; venv, no ROS or camera);
  CI runs both (`tests.yml`), without rendering: the recording and evaluation tests run only locally. MuJoCo check:
  `so101_mujoco --video <file> --follow --headless out.mp4` (`MUJOCO_GL=egl`).
- ACT: train with README's `lerobot-train` command (inputs listed, bf16; 20k steps ≈ 80 min on the RTX 4060), weights
  in `~/data/models/`; evaluate with `so101_mujoco_eval <output_dir>/checkpoints/last/pretrained_model`.
- Scripted checks: `sim.launch.py … headless:=true video:=<file> preview:=false` (the GUI needs OpenGL), e.g. on
  `~/data/hand_depth_spike/plane.avi`; own `ROS_DOMAIN_ID`, `GZ_PARTITION`; `set -m`, `kill -INT -- -<pgid>`; no
  `set -u`; never the webcam unasked. The sandbox blocks multicast, so nothing is found without `GZ_IP=127.0.0.1
  GZ_RELAY=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` exported for the whole check; no `pkill -f` on own
  patterns.

## Decisions (`docs/decisions/`)

- 0001 Hand teleoperation first, on Gazebo; MuJoCo later for data. 0002 Hand tracking with MediaPipe (since 0008 its
  Gesture Recognizer, model `gesture_recognizer.task`).
- 0003 Hand teleoperation: camera setup, axes and control (the side view; setup and arm control superseded by 0006).
- 0004 Own controller configuration for Gazebo: the upstream robot with controllers at 100 Hz instead of 10 Hz.
- 0005 Project scope and exit criteria: pick and place in MuJoCo, ~50 demos, ACT (M3–M5); limit: one-camera depth.
- 0006 Hand from the front over the workspace from above: absolute XY (the base turns), top-down grasp, `space` pauses.
- 0007 Gripper height from a key: one-camera depth is precise with a steady pose, but pose changes read as ≥ 10 cm.
- 0008 Gripper open or closed: a fist / an open palm (gestures) decides at once, else the aperture with hysteresis.
- 0009 Simulator: so101-nexus 0.11.0 (MuJoCo) `PickAndPlace-v2` from our loop, a third package here, one venv on
  LeRobot 0.7; against the Gazebo URDF the wrist roll is +0.0487 rad, `so101_hand` heights 32.5 mm below MuJoCo's floor.
- 0010 Dataset: LeRobot v3, 30 fps, `so101_follower` units (degrees, gripper 0–100), wrist (fixed) + overhead cameras;
  an episode = an attempt from the first `p` to 1 s after the success; attempts in `<name>_attempts.csv`.

## Pitfalls

- The venv has numpy 2.2.6; `cv_bridge` from apt is built for numpy 1.x and crashes there: read images without it.
- Three OpenCV wheels share `cv2` in the venv: always install with `-c src/so101_lerobot/constraints.txt` (ADR 0002).
- MediaPipe imports `matplotlib`; apt's 3.6 is built for numpy 1.x, so the venv has its own (Axes3D warning: harmless).
- Gripper joint "6": 0.0 rad closed, 1.745 rad open (LeRobot datasets: `gripper.pos` 0–100 over −10°…100°). A goal
  beyond the limit leaves the Gazebo jaw stuck there: command it through `so101_hand.gripper` (≤ 1.725 rad).
- Run pip with `env -u PYTHONPATH`: with ROS sourced, `pip check` and `pip freeze` see ROS's packages too.
- MoveIt's SUCCESS means the trajectory time is over, not that the joints arrived: check `/joint_states` (ADR 0004).
- A YAML value under the node name beats a launch dict one (under `/**`): no `arm`, `video_path`, `preview` in YAML.
- LeRobot gives a policy every `observation.*` feature: list the inputs (`--policy.input_features`), or ACT also gets
  `observation.environment_state`, the simulator's cube and disc. `lerobot-train` needs `accelerate` (requirements).

## Current milestone

M3 done (2026-11-14, exit criteria: ADR 0005): the hand drives SO-101 in MuJoCo (ROB-38, ADR 0006–0009) and ROB-39
recorded 50 demonstrations (ADR 0010): teleoperation 50 of 56 (89 %), 20.0 s per pick and place on average. Now M4,
ROB-40: the pilot on 20 episodes (20k steps, 86 min) reached 0–3 of 50 in MuJoCo (it fits the demos, the grasp
misses; one seed's outcome can differ between runs); ACT on the 50 episodes trains 50k steps. Then ROB-46 (release).
