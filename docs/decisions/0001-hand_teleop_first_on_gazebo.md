# 0001. Hand teleoperation first, on Gazebo; MuJoCo later for data

- Status: Accepted
- Date: 2026-10-13
- Supersedes: the order chosen on 2026-10-11 (so101-nexus as the MVP simulator, hand tracking in M4)
- Amended by ADR 0009 (2026-11-13): the simulator for data is so101-nexus, and the keyboard fallback
  (`lerobot_teleop/`) is dropped

## Context

On 2026-10-11 the project chose so101-nexus (MuJoCo) as the MVP simulator, with gamepad teleoperation in M2
and webcam hand tracking only in M4. Since then:

- No gamepad and no camera rig are at hand; the only sensor is the laptop webcam.
- The ROS 2 simulation works: `ros2 launch so101_bringup sim.launch.py` starts Gazebo, the controllers,
  MoveIt and RViz, and the arm and gripper execute trajectories (ROB-15).
- The first teleoperation stages need no contact physics: closing the hand closes the gripper (stage 1),
  the hand moves the arm in a vertical plane (stage 2). Grasping and data collection come later.
- The largest technical risk is now detecting the hand from the side with one webcam (ROB-19).

## Decision

1. Build the hand teleoperation first, in stages, on Gazebo (ROS 2 Jazzy, ros2_control):
   M1 hand → gripper, M2 hand → arm in the XZ plane of the base.
2. Keep the logic in a Python package without ROS (`so101_hand/`) with a thin ROS 2 node
   (`so101_hand_ros/`), both in this repository.
3. Confirm the simulator for grasping and LeRobot dataset recording in M3 (so101-nexus/MuJoCo, ROB-17 and ROB-18).

## Consequences

- Progress now with the hardware at hand, and the riskiest part (hand detection) is tested first.
- Moving the teleoperation to MuJoCo later costs an adapter; the ROS-free core keeps it small.
- The gamepad leaves the MVP; keyboard joint control (`lerobot_teleop/run_teleop.py`) stays as a fallback.
- Milestones are reordered: M1 hand → gripper, M2 hand → arm (XZ), M3 simulator for data and recording,
  M4 first policy (ACT), M5 public release.
- The third-party Gazebo controllers run at 10 Hz; stage 2 checks whether hand-driven arm motion is smooth enough.
