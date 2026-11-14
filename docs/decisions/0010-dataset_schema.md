# 0010. Dataset schema: the real SO-101 follower's, as so101-nexus records it

- Status: Accepted
- Date: 2026-11-14
- Follows: ADR 0005 (criterion 2), ADR 0009; answers the ROB-18 spike (folded into ROB-39)

## Context

ADR 0005 asks for about 50 demonstrations as a LeRobot dataset (v3) with simulated camera images, in the real
SO-101 schema, so that a dataset or a policy from the simulation needs no conversion on a real arm. so101-nexus
records its datasets in the units of LeRobot's SO follower (ROB-20, ROB-17): joints in degrees, the gripper as a
percentage of its travel, the cameras by name. LeRobot 0.7's SO-101 follower uses the same names and units
(`use_degrees`) under the robot type `so101_follower`. Against the Gazebo URDF of `so101_hand`, joints 1–4 and the
gripper are the same, and the wrist roll's zero differs by 0.0487 rad (ADR 0009).

## Decision

1. A LeRobot v3 dataset at 30 fps, robot type `so101_follower`. One simulation step per frame: the control period
   is 1/30 s (14 physics steps of 2.4 ms) for the teleoperation, the recording and the policy's evaluation alike.
2. The features, with so101-nexus's own conversion (`sim_qpos_to_dataset_row`):

   | Feature | Simulation | Real SO-101 | Units |
   | --- | --- | --- | --- |
   | `action` | the joint targets sent this step (the IK of `so101_hand`, the wrist roll, the gripper) | the motors' goal positions | `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`, `wrist_flex.pos`, `wrist_roll.pos` in degrees; `gripper.pos` 0–100 % of −10°…100° |
   | `observation.state` | the joint positions before the step | the motors' present positions | as `action` |
   | `observation.images.wrist` | so101-nexus's wrist camera, fixed: 75° field of view, −17.2° pitch, no position noise | a camera on the gripper | RGB 640×480, h264 video |
   | `observation.images.overhead` | so101-nexus's camera above the table | a camera above the table | RGB 640×480, h264 video |
   | `observation.environment_state` | so101-nexus's full state: joints, velocities, TCP pose, grasp, the cube's and the disc's poses (43 values) | none | SI units |

3. An episode is one attempt: from the first `p` (following) to 1 s after the success, which covers the release and
   the gripper rising. An attempt given up with `r` is discarded and counts as a failure. Every attempt is a row of
   `$DATA_DIR/so101_mujoco/<name>_attempts.csv` (saved or discarded, seconds, frames): the source of the time per
   pick and place and of the teleoperation success rate.
4. The dataset lives outside git, in `$HF_HOME/lerobot/<repo_id>` (`~/data/huggingface/lerobot/…`). A later session
   resumes it, and its seeds continue after the last attempt's, so the cube and the disc start elsewhere.

## Consequences

- A policy trained on it outputs the real follower's actions. Only the cameras differ from a real setup (their
  placement, the rendered look); transfer to a real arm is out of scope (ADR 0005).
- The wrist camera does not vary between episodes (no domain randomization): easier for a first policy from 50
  demonstrations, less robust to another mounting.
- The action is the target the teleoperation sends, not the hand: the policy learns joint targets, as from a leader
  arm.
- The control rate is 30 Hz instead of so101-nexus's 50 Hz; the scripted top-down grasp still lifts the cube in at
  least 8 of 10 scenes (tests).
- h264 instead of LeRobot's default AV1, as so101-nexus's recorder: an episode encodes in a few seconds.
- `observation.environment_state` lets an episode be replayed and analysed in the simulator; ACT does not use it.
  Correction (2026-11-14, ROB-40): LeRobot 0.7 gives a policy every `observation.*` feature of the dataset, and ACT
  has an input for the environment state, so the training lists the policy's inputs without it
  (`--policy.input_features`: the joints and the two cameras).
