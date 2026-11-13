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
  `kinematics` FK/IK (pan + planar), `teleop` table map and follower; `tests/urdf_reference.py` = URDF FK.
- `so101_hand_ros/` — node `hand_teleop`: webcam (or `video:=`) → `so101_hand` → gripper goals and, with `arm`, arm
  goals (joint 5 at 0); space or `~/clutch` follows/pauses, z the height; parameters in `config/hand_teleop.yaml`.
- `lerobot_teleop/run_teleop.py` — keyboard joint teleoperation through LeRobot (`so101_ros`, `keyboard_joint`).
- `lerobot_mimic/` — legacy YOLO pose (AGPL-3.0) arm mimicking, J2/J3 only; kept as an example.

## Third-party (pinned in `so101_lerobot.repos`, never modified)

- `lerobot_ws` (Pavankv92, Apache-2.0): `lerobot_description` (URDF, Gazebo launch), `lerobot_controller` (JTC
  `arm_controller` on "1"–"5", `gripper_controller` on "6"), `lerobot_moveit` (groups `arm`, `gripper`; KDL IK).
- `lerobot-ros` (ycheng517, Apache-2.0): LeRobot robot `so101_ros` and teleoperators over ROS; venv, `pip -e`.

## Build, run, check

- Build: only the `moveit_ws` underlay and the venv sourced, `python -m colcon build` in `so101_ws` (README).
- Run: `so101_sim` or `so101_hand`; without aliases `ws so101` and `ros2 launch so101_bringup sim.launch.py`.
- Check: `ros2 control list_controllers` shows 3 active controllers; Plan & Execute in RViz moves the arm.
- Tests: `python -m pytest src/so101_lerobot/so101_hand` (venv, ~2 s, no ROS or camera); CI runs them (`tests.yml`).
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

## Pitfalls

- The venv has numpy 2.2.6; `cv_bridge` from apt is built for numpy 1.x and crashes there: read images without it.
- Three OpenCV wheels share `cv2` in the venv: always install with `-c src/so101_lerobot/constraints.txt` (ADR 0002).
- MediaPipe imports `matplotlib`; apt's 3.6 is built for numpy 1.x, so the venv has its own (Axes3D warning: harmless).
- Gripper joint "6": 0.0 rad closed, 1.745 rad open (lerobot-ros normalises 0 = open, 1 = closed). A goal beyond
  the limit leaves the simulated jaw stuck there: command it through `so101_hand.gripper` (≤ 1.725 rad).
- MoveIt's SUCCESS means the trajectory time is over, not that the joints arrived: check `/joint_states` (ADR 0004).
- A YAML value under the node name beats a launch dict one (under `/**`): no `arm`, `video_path`, `preview` in YAML.

## Current milestone

M3 Webcam pick and place in MuJoCo + 50 demos (exit criteria: ADR 0005). ROB-35 built the frontal mapping (ADR 0006,
the height from a key, ADR 0007, the gripper from gestures, ADR 0008), checked with the webcam on 2026-11-13; next
ROB-36 (sector from the IK) or ROB-17 (MuJoCo).
