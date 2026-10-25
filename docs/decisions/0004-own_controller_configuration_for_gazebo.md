# 0004. Own controller configuration for the Gazebo simulation

- Status: Accepted
- Date: 2026-10-25

## Context

The simulated gripper followed the hand about 1 s late (ROB-27). Measured in headless Gazebo with single-point
goals streamed to the gripper controller: 90% of a step after 1.86 s, 0.76 s behind a 4 s sine, and the same
for goal rates of 5–20 Hz, so the lag is not in the hand pipeline.

The cause is in the configuration of the third-party `lerobot_ws`. `gz_ros2_control` drives a joint with a
position command through a velocity: `position_proportional_gain` (default 0.1) × error × the controller manager's
`update_rate`. `lerobot_controller/config/so101_controllers.yaml` sets `update_rate: 10`, so every joint behaves like
a first-order lag with a time constant of about 1 s. The arm goes through the same path: MoveIt reported SUCCESS for
four Plan & Execute goals, but in three of them the joints were still more than 0.02 rad away 3 s after the result.

`lerobot_ws` is pinned in `so101_lerobot.repos` and never modified.

## Decision

- `so101_bringup/urdf/so101_sim.urdf.xacro` is the upstream robot (`so101_base.xacro`, `so101_ros2_control.xacro`)
  with the `gz_ros2_control` plugin pointing to `so101_bringup/config/so101_controllers.yaml`.
- That YAML is the upstream one ("based on", Apache-2.0) with one change: `update_rate: 100` (time constant about
  0.1 s). The position gain keeps its default.
- `sim.launch.py` passes the xacro to the upstream Gazebo launch through its `model` argument. MoveIt keeps the
  upstream URDF: it does not use the plugin block.

## Consequences

- Measured in headless Gazebo on 2026-10-25: a gripper step reaches 90% after 0.28 s (goals at 10 Hz with
  `time_from_start` 0.1 s, as the hand node sends them), 0.20 s behind the sine; four MoveIt Plan & Execute goals
  (arm and gripper) succeed with the joints within 0.02 rad at most 0.08 s after the result; real-time factor
  0.97–1.0, as before.
- The rendered URDF differs from the upstream one only in the plugin block.
- The controllers and the joint state broadcaster run ten times as often (`/joint_states` at 100 Hz); the CPU cost
  is small on this laptop.
- The copied YAML has to follow upstream changes to joints or controllers; the pin in `.repos` makes such changes
  deliberate.
- Rejected: a higher `position_proportional_gain` at 10 Hz. The trajectory controller would still sample at
  10 Hz, and with gain × rate × cycle = 1 every cycle already closes the whole error, leaving no margin for delays.
