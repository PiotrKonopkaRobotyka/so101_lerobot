# so101_lerobot — project context for Claude Code

Supplements `~/code/CLAUDE.md` (workflow, safety and naming rules). Plan and status live only in the Linear
project [SO-101 Sim Imitation Learning](https://linear.app/robotics-software-engineer/project/so-101-sim-imitation-learning-9a024fd3273a);
setup and commands for people are in [README.md](README.md).

## Goal

Train SO-101 manipulation in simulation without a leader arm. Current path (ADR 0001): hand teleoperation
with the laptop webcam in Gazebo, in stages — M1 hand → gripper, M2 hand → arm in the XZ plane of the base;
the simulator for data collection (so101-nexus/MuJoCo) is confirmed in M3.

## Packages in this repository

- `so101_bringup/` — `sim.launch.py`: Gazebo, controllers, MoveIt and RViz with one command (alias `so101_sim`).
- `lerobot_teleop/run_teleop.py` — keyboard joint teleoperation through LeRobot (`so101_ros`, `keyboard_joint`).
- `lerobot_mimic/` — legacy YOLO pose (AGPL-3.0) arm mimicking, J2/J3 only. Reuse its pipeline pattern
  (camera → keypoints → EMA → limits → command, hold on loss, OSD), not its model.
- Planned (ADR 0001, 0003): `so101_hand/` (plain Python, no `rclpy`, `pyproject.toml`, `COLCON_IGNORE`,
  `pip install -e`) and `so101_hand_ros/` (thin ROS 2 node).

## Third-party (pinned in `so101_lerobot.repos`, never modified)

- `lerobot_ws` (Pavankv92, Apache-2.0): `lerobot_description` (URDF, Gazebo launch, `gz_ros2_control`),
  `lerobot_controller` (JTC `arm_controller` on joints "1"–"5", JTC `gripper_controller` on joint "6",
  `update_rate` 10 Hz), `lerobot_moveit` (groups `arm`, `gripper`; KDL IK on the full pose).
- `lerobot-ros` (ycheng517, Apache-2.0): LeRobot robot `so101_ros` over ROS topics and actions, teleoperators
  `keyboard_joint` and `gamepad_6dof` (MoveIt Servo); installed in the venv with `pip -e`. Useful later for
  recording LeRobot datasets from Gazebo.

## Build, run, check

- Build in a terminal with only the `moveit_ws` underlay and the venv: `python -m colcon build` in `so101_ws`
  (details in README).
- Run: `so101_sim`, or `ws so101` and `ros2 launch so101_bringup sim.launch.py`.
- Check: `ros2 control list_controllers` shows 3 active controllers; Plan & Execute in RViz moves the arm.
- Tests: none yet; `so101_hand` gets `pytest` tests on synthetic landmarks.

## Decisions (`docs/decisions/`)

- 0001 Hand teleoperation first, on Gazebo; MuJoCo later for data.
- 0002 Hand tracking with MediaPipe Hand Landmarker.
- 0003 Hand teleoperation: camera setup, axes and control (proposed; pose checked in ROB-19).

## Pitfalls

- The venv has numpy 2.2.6; `cv_bridge` from apt is built for numpy 1.x and crashes there: read images without it.
- Two OpenCV wheels share `cv2` in the venv; `pip install mediapipe` would add `opencv-contrib-python` 5.x (ADR 0002).
- Gripper joint "6": 0.0 rad closed, 1.745 rad open (lerobot-ros normalises 0 = open, 1 = closed).
- The RViz marker rarely moves the 5-joint arm (KDL solves the full 6D pose): use Goal State or "Approx IK Solutions".
- The third-party RViz node has no `use_sim_time`; `sim.launch.py` sets it for every node.
- Build with the underlay only, never with another overlay sourced.

## Current milestone

M0 Foundations; next M1 Hand → gripper: spike ROB-19, then stage 1 (ROB-27).
