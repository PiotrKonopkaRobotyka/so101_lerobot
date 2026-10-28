# 0005. Project scope and exit criteria

- Status: Accepted
- Date: 2026-10-27

## Context

M1 and M2 are done: the hand in front of the laptop webcam opens and closes the simulated gripper, moves the arm in
its vertical plane and tilts the jaws in Gazebo (ADR 0003). The goal is to train SO-101 manipulation in simulation
without a leader arm, but the end of the project was not defined. The teleoperation alone could be improved without
end (base rotation, filters, full 3D IK), and the project would stop without a dataset or a trained policy.

The constraints of ADR 0001 still hold:

- The only sensor is the laptop webcam: no real arm, no leader arm, no gamepad, no depth camera. Training runs on
  the laptop GPU, an RTX 4060 with 8 GB.
- One camera measures the hand well in the image plane and poorly in depth (ADR 0003). In the side view the image
  plane is the arm's plane, so turning the base (joint 1) is the one motion left that needs depth.
- Picking a block needs contact physics and a task scene. so101-nexus (MuJoCo, Apache-2.0) has a pick-and-place
  task and LeRobot dataset recording, made for a physical leader arm (ROB-20); the hand can take that arm's place.

## Decision

The project is finished when all four hold (milestones M3–M5):

1. Teleoperation (M3): the hand in front of the webcam drives SO-101 in a MuJoCo scene through `so101_hand`, without
   ROS, and picks a block and places it in a target area. The simulator is confirmed hands-on in ROB-17.
2. Data (M3): about 50 demonstrations recorded as a LeRobot dataset (v3) with simulated camera images, in the real
   SO-101 schema (ROB-18); the time per pick and place and the teleoperation success rate are measured.
3. Policy (M4): ACT trained with LeRobot on that dataset on the RTX 4060; its success rate over 50 episodes in
   MuJoCo, compared with the teleoperation success rate.
4. Release (M5): a README with the results (the metrics, a video) and the limitations, Docker, a LICENSE, the public
   repository and a write-up.

The success rates are reported as measured. A target for the policy is set after the first ACT baseline (M4);
falling short of it does not block the release.

Teleoperation for the task: the arm in its vertical plane (M2), and the base turned by the hand's depth if the spike
ROB-34 finds it precise enough, otherwise by the keys a/d (ROB-35). The IK stays analytic: base rotation plus the
planar IK of ADR 0003. Other teleoperation changes only where the task needs them, such as the reachable area in
the preview (ROB-36) and a One Euro filter with goals at 20 Hz (ROB-37).

Depth from one camera is the method's limit: it is measured (ROB-34) and stated in the README, not removed with
more hardware (a second camera or a depth camera).

Out of scope, the project's non-goals: the real arm until one is available, a gamepad, our own simulator or robot
model, learning directly from human videos without the robot, training VLA models from scratch, more than one task,
photorealistic rendering. Also out: grasping in Gazebo and LeRobot's `RobotKinematics` (placo).

## Consequences

- Every issue in M3–M5 serves one of the four criteria; other ideas wait until the project is finished (a gamepad,
  a VLA with language commands, the real SO-101).
- The project ends with numbers, a dataset and success rates, even if the policy is weak.
- The placo evaluation deferred in ADR 0003 is dropped (ROB-29 canceled); the joint names and zero offsets of the
  official SO-101 model are checked in ROB-18 instead.
- With the keys a/d the other hand turns the base, as it already toggles the clutch.
- Gazebo and ROS 2 stay for M1–M2 and the MoveIt checks; in MuJoCo the teleoperation calls `so101_hand` without ROS,
  and the ROS-free core (ADR 0001) keeps that adapter small.
- The same task with other methods, MoveIt 2 with MoveIt Task Constructor and then reinforcement learning in
  MuJoCo, belongs to separate projects.
- The README gets a Limitations section now; M5 adds the results.
