# 0009. Simulator for data collection: so101-nexus (MuJoCo)

- Status: Accepted
- Date: 2026-11-13
- Follows: ADR 0001 (decision 3), ADR 0005

## Context

ADR 0001 left the simulator for grasping and dataset recording to M3. ADR 0005 made it the first exit criterion: the
hand drives SO-101 in a MuJoCo scene through `so101_hand`, without ROS, with the simulator confirmed hands-on in
ROB-17. The candidate since ROB-20 is so101-nexus (Apache-2.0, beta): SO-101 tasks as Gymnasium environments in
MuJoCo, with LeRobot dataset recording made for a physical leader arm. The spike (ROB-17) ran it on the laptop
(Ubuntu 24.04, Python 3.12, RTX 4060 8 GB) in its own venv; the code is in `~/code/experiments/so101_nexus`, the
results in `~/data/so101_nexus_spike`.

- **Version:** 0.11.0 from PyPI (2026-11-07, tag `0.11.0`, commit `df078051`) with MuJoCo 3.17.0. The simulator
  needs neither LeRobot nor torch; LeRobot (`>=0.7,<0.8`, not 0.5 as planned) comes only with the recording extras.
  Betas break often: the success predicate changed in 0.4.13, the state vector in 0.5.0.
- **Speed:** `MuJoCoPickAndPlace-v1` runs 3,337 steps per second without cameras (67× real time, one core) and 364
  with the wrist and overhead cameras at 640×480 (7.3× real time, one core, 0.7 GB RAM, 1.2 GB GPU memory, EGL).
  A step is 20 ms of simulation.
- **Task:** a 25 mm, 10 g cube and a target disc of 5 cm radius; `info["success"]` at every step. v1 counts the cube
  within 25 mm of the disc's centre, static and released; v2 counts it resting on the visible disc, carried by the
  table and untouched for 0.2 s (Gymnasium marks v1 out of date). The cube and the disc are sampled in a ring sector
  around `spawn_center`; set to ours (the pan axis, 0.12–0.34 m, ±60°), they stay inside it over 200 resets.
- **Published demonstrations** (`johnsutor/MuJoCoPickAndPlace-v1`, 10 episodes with a leader arm, version 0.4.12):
  replayed open loop, the gripper grasps, lifts (66–154 mm) and carries the cube in 10 of 10, and the cube ends 2–13
  mm from where it was recorded. None is a success in 0.11.0: the operator never opened the gripper, and since 0.4.13
  the release is required. With a release added, v1 counts 4 of 10 and v2 8 of 10: four cubes lie on the disc 31–44
  mm from its centre.
- **Robot model:** the Menagerie MJCF, equal to the official `so101_new_calib` URDF (so101_mtc ADR 0002). Against the
  Gazebo URDF of `so101_hand.kinematics`, compared by the poses of the STL parts over 501 configurations, joints 1–4
  are identical (0.00 mm, 0.00°) and the wrist roll's zero differs: so101-nexus `wrist_roll` = Gazebo joint "5" +
  0.0487 rad (2.79°). The Gazebo base frame lies 32.5 mm below MuJoCo's floor level (the Gazebo robot stands 30 mm
  above its ground). So101-nexus's TCP site lies 12 mm across the jaws from ours.
- **Integration:** the `hand_teleop` pipeline without ROS, on a recorded video (`plane.avi`, 397 frames at 19.8 fps),
  drove the arm through `env.step()`: the base turned −0.61…+0.62 rad as in Gazebo, the TCP followed 1.5 mm behind
  its target on average (95 % below 6.6 mm), and a frame took 12 of its 50 ms (MediaPipe 8.1, IK 0.4, physics 0.9,
  render 2.9). So101-nexus's own teleoperation (`LeaderProtocol`, a Gradio app) is built around a physical leader arm
  and its motor bus.
- **One venv:** so101-nexus 0.11.0, `lerobot[dataset]` 0.7.0, MediaPipe 1.1.0, torch 2.10.0, OpenCV 4.12.0.88 and
  `so101_hand` resolve together (`pip install --dry-run`). The workspace venv holds LeRobot 0.4.4 for lerobot-ros
  (`lerobot<0.5`), which only `lerobot_teleop/run_teleop.py` uses.

## Decision

1. **Simulator:** so101-nexus 0.11.0, pinned, MuJoCo backend, task `MuJoCoPickAndPlace-v2` with the cube and the disc
   in our sector (`spawn_center` at the pan axis, radius 0.12–0.34 m, ±60°). Its `success` measures M3 and M4.
2. **Integration:** our own loop calls `so101_hand` and `env.step()` directly: joints 1–4 from the IK, the wrist roll
   at the Gazebo zero (+0.0487 rad), the gripper from the hand. `so101_hand`'s table frame (forward, left, up) maps to
   MuJoCo's world without rotation, shifted by (+15.8, −20.8, −32.5) mm. No `LeaderProtocol`, no Gradio app.
3. **Code and venv:** the MuJoCo teleoperation and recording become a third plain-Python package in this repository,
   next to `so101_hand` (no ROS, `pip install -e`). The workspace venv moves to LeRobot 0.7 and so101-nexus 0.11.0;
   lerobot-ros and `lerobot_teleop/` (the Gazebo keyboard teleoperation) leave the repository at a tag. A separate
   Python project in `~/code/projects/` with its own venv would keep the ROS venv as it is, but split the release (M5)
   over two repositories.

## Consequences

- ROB-38 needs no scene of its own: the sector on the table and the keys are ours; the scene, the physics and the
  success check come from so101-nexus.
- The heights of `hand_teleop.yaml` do not carry over: in `so101_hand`'s frame MuJoCo's floor is at 32.5 mm, so the
  cube's centre (12.5 mm above the floor) is at 45 mm, not at the Gazebo grasp height of 20 mm.
- An upgrade of so101-nexus is a deliberate step: it can change the success predicate and the observation layout,
  which makes success rates incomparable, so the replay and the success check run again.
- so101-nexus records at 30 fps with one 20 ms step per frame (the simulation at 0.6× real time) unless the control
  period equals the frame period (`PhysicsConfig.control_period_s`, as its policy collection does). ROB-39 records
  with the simulation in real time.
- The published demonstrations are no training data for us (no release, an older state vector); they stay a
  reference for the scene and the schema.
- The keyboard teleoperation through LeRobot, kept as a fallback by ADR 0001, goes with lerobot-ros: the hand is the
  only teleoperation, in Gazebo and in MuJoCo.
- Gazebo and ROS 2 stay for M1–M2 (ADR 0001); the MuJoCo path uses neither.
