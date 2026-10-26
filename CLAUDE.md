# so101_lerobot — project context for Claude Code

Supplements `~/code/CLAUDE.md` (workflow, safety and naming rules). Plan and status live only in the Linear
project [SO-101 Sim Imitation Learning](https://linear.app/robotics-software-engineer/project/so-101-sim-imitation-learning-9a024fd3273a);
setup and commands for people are in [README.md](README.md).

## Goal

Train SO-101 manipulation in simulation without a leader arm. Current path (ADR 0001): hand teleoperation
with the laptop webcam in Gazebo, in stages — M1 hand → gripper, M2 hand → arm in the XZ plane of the base;
the simulator for data collection (so101-nexus/MuJoCo) is confirmed in M3.

## Packages in this repository

- `so101_bringup/` — `sim.launch.py`: Gazebo, controllers, MoveIt and RViz with one command (alias `so101_sim`);
  `moveit:=false hand:=true` starts the hand node instead of MoveIt (alias `so101_hand`).
- `so101_hand/` — plain Python, no `rclpy` (`pyproject.toml`, `COLCON_IGNORE`, `pip install -e`): `measures`,
  `calibration`, `filters`, `gripper` and `tracker` (the only MediaPipe import); `pytest` tests in `tests/`.
- `so101_hand_ros/` — node `hand_gripper`: webcam (or `video_path`) → `so101_hand` → goals on
  `/gripper_controller/joint_trajectory`; parameters and my calibration in `config/hand_gripper.yaml`.
- `lerobot_teleop/run_teleop.py` — keyboard joint teleoperation through LeRobot (`so101_ros`, `keyboard_joint`).
- `lerobot_mimic/` — legacy YOLO pose (AGPL-3.0) arm mimicking, J2/J3 only; kept as an example.

## Third-party (pinned in `so101_lerobot.repos`, never modified)

- `lerobot_ws` (Pavankv92, Apache-2.0): `lerobot_description` (URDF, Gazebo launch, `gz_ros2_control`),
  `lerobot_controller` (JTC `arm_controller` on joints "1"–"5", JTC `gripper_controller` on joint "6",
  `update_rate` 10 Hz), `lerobot_moveit` (groups `arm`, `gripper`; KDL IK on the full pose).
- `lerobot-ros` (ycheng517, Apache-2.0): LeRobot robot `so101_ros` over ROS topics, teleoperators `keyboard_joint`
  and `gamepad_6dof` (MoveIt Servo); in the venv with `pip -e`, later for recording LeRobot datasets.

## Build, run, check

- Build in a terminal with only the `moveit_ws` underlay and the venv: `python -m colcon build` in `so101_ws`
  (details in README).
- Run: `so101_sim` or `so101_hand`; without aliases `ws so101` and `ros2 launch so101_bringup sim.launch.py`.
- Check: `ros2 control list_controllers` shows 3 active controllers; Plan & Execute in RViz moves the arm.
- Tests: `python -m pytest src/so101_lerobot/so101_hand` (venv, ~2 s, no ROS or camera).
- Scripted checks: headless Gazebo (`gz sim -s`) and the node on a video (`-p video_path:=<file> -p preview:=false`);
  start `ros2 launch` with `set -m` and stop it with `kill -INT -- -<pgid>` (a background job ignores SIGINT).
  Never open the webcam without asking.

## Decisions (`docs/decisions/`)

- 0001 Hand teleoperation first, on Gazebo; MuJoCo later for data.
- 0002 Hand tracking with MediaPipe Hand Landmarker.
- 0003 Hand teleoperation: camera setup, axes and control (accepted for the gripper; the arm part in ROB-28).

## Pitfalls

- The venv has numpy 2.2.6; `cv_bridge` from apt is built for numpy 1.x and crashes there: read images without it.
- Three OpenCV wheels share `cv2` in the venv (`opencv-contrib-python` is pinned to 4.12.0.88 in `constraints.txt`:
  MediaPipe would pull 5.x). Always install with `-c src/so101_lerobot/constraints.txt` (ADR 0002).
- MediaPipe imports `matplotlib`; the apt one (3.6) is built for numpy 1.x and crashes, so the venv has its own.
  The "Unable to import Axes3D" warning is harmless.
- Gripper joint "6": 0.0 rad closed, 1.745 rad open (lerobot-ros normalises 0 = open, 1 = closed). A goal beyond
  the limit leaves the simulated jaw stuck there: command it through `so101_hand.gripper` (≤ 1.725 rad).
- The simulated joints lag ~1 s behind goals: `gz_ros2_control` gain 0.1 × the 10 Hz controller rate (ROB-30).
- The RViz marker rarely moves the 5-joint arm (KDL solves the full 6D pose): use Goal State or "Approx IK Solutions".
- The third-party RViz node has no `use_sim_time`; `sim.launch.py` sets it for every node.
- Build with the underlay only, never with another overlay sourced.

## Current milestone

M1 Hand → gripper: stage 1 (ROB-27) works with the webcam. Next: ROB-30 (faster simulated joints), then
M2 Hand → arm in the XZ plane (ROB-28).
