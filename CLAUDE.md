# so101_lerobot — project context for Claude Code

Supplements `~/code/CLAUDE.md` (workflow, safety and naming rules). Plan and status live only in the Linear
project [SO-101 Sim Imitation Learning](https://linear.app/robotics-software-engineer/project/so-101-sim-imitation-learning-9a024fd3273a);
setup and commands for people are in [README.md](README.md).

## Goal

Train SO-101 manipulation in simulation without a leader arm (ADR 0001): hand teleoperation with the laptop webcam
in Gazebo — M1 hand → gripper, M2 hand → arm in its vertical plane; data collection in so101-nexus/MuJoCo (M3).

## Packages in this repository

- `so101_bringup/` — `sim.launch.py`: Gazebo, controllers, MoveIt and RViz with one command (alias `so101_sim`);
  `moveit:=false hand:=true [arm:=true]` starts the hand node instead of MoveIt (alias `so101_hand`). The robot is
  `urdf/so101_sim.urdf.xacro`: the upstream one with our `config/so101_controllers.yaml` at 100 Hz (ADR 0004).
- `so101_hand/` — plain Python, no `rclpy` (`pyproject.toml`, `COLCON_IGNORE`, `pip install -e`): `measures`,
  `calibration`, `filters`, `gripper`, `kinematics` (planar FK/IK, TCP), `teleop` (clutch: position, jaws' pitch),
  `tracker` (the only MediaPipe import); `pytest` in `tests/`, `tests/urdf_reference.py` = generic FK of the URDF.
- `so101_hand_ros/` — node `hand_teleop`: webcam (or `video_path`) → `so101_hand` → gripper goals and, with
  `arm`, arm goals (joints 1 and 5 at 0); clutch on space or `~/clutch`; parameters in `config/hand_teleop.yaml`.
- `lerobot_teleop/run_teleop.py` — keyboard joint teleoperation through LeRobot (`so101_ros`, `keyboard_joint`).
- `lerobot_mimic/` — legacy YOLO pose (AGPL-3.0) arm mimicking, J2/J3 only; kept as an example.

## Third-party (pinned in `so101_lerobot.repos`, never modified)

- `lerobot_ws` (Pavankv92, Apache-2.0): `lerobot_description` (URDF, Gazebo launch), `lerobot_controller` (JTC
  `arm_controller` on joints "1"–"5", `gripper_controller` on "6", 10 Hz; we run a copy at 100 Hz), `lerobot_moveit`
  (groups `arm`, `gripper`; KDL IK on the full pose).
- `lerobot-ros` (ycheng517, Apache-2.0): LeRobot robot `so101_ros` and teleoperators over ROS; venv, `pip -e`.

## Build, run, check

- Build: only the `moveit_ws` underlay and the venv sourced, `python -m colcon build` in `so101_ws` (README).
- Run: `so101_sim` or `so101_hand`; without aliases `ws so101` and `ros2 launch so101_bringup sim.launch.py`.
- Check: `ros2 control list_controllers` shows 3 active controllers; Plan & Execute in RViz moves the arm.
- Tests: `python -m pytest src/so101_lerobot/so101_hand` (venv, ~2 s, no ROS or camera).
- Scripted checks: no GUI (it crashes without OpenGL, also `offscreen`), but the upstream launch always opens it:
  include `ros_gz_sim` `gz_sim.launch.py` with `gz_args` `-s …` (plain `gz sim` misses the `gz_ros2_control` plugin
  path); own `ROS_DOMAIN_ID`, `GZ_PARTITION`; the node on a video (`-p video_path:=<file> -p preview:=false`);
  `set -m`, `kill -INT -- -<pgid>` (a background job ignores SIGINT); no `set -u`. Never open the webcam unasked.

## Decisions (`docs/decisions/`)

- 0001 Hand teleoperation first, on Gazebo; MuJoCo later for data.
- 0002 Hand tracking with MediaPipe Hand Landmarker.
- 0003 Hand teleoperation: camera setup, axes and control (accepted; the arm in ROB-28, the jaws' pitch in ROB-31).
- 0004 Own controller configuration for Gazebo: the upstream robot with controllers at 100 Hz instead of 10 Hz.

## Pitfalls

- The venv has numpy 2.2.6; `cv_bridge` from apt is built for numpy 1.x and crashes there: read images without it.
- Three OpenCV wheels share `cv2` in the venv: always install with `-c src/so101_lerobot/constraints.txt` (ADR 0002).
- MediaPipe imports `matplotlib`; apt's 3.6 is built for numpy 1.x, so the venv has its own (Axes3D warning: harmless).
- Gripper joint "6": 0.0 rad closed, 1.745 rad open (lerobot-ros normalises 0 = open, 1 = closed). A goal beyond
  the limit leaves the simulated jaw stuck there: command it through `so101_hand.gripper` (≤ 1.725 rad).
- MoveIt's SUCCESS means the trajectory time is over, not that the joints arrived: check `/joint_states` (ADR 0004).
- The third-party RViz node has no `use_sim_time`; `sim.launch.py` sets it for every node.
- A YAML value under the node name beats a launch dict parameter (written under `/**`): keep `arm` out of the YAML.

## Current milestone

M2 Hand → arm done with ROB-31 (jaws follow the hand's pitch; webcam test 2026-10-27). Next, agreed 2026-10-27
(logs): exit criteria (ADR 0005), a hand-depth spike for the shoulder pan, pick and place in MuJoCo (M3), ACT (M4).
