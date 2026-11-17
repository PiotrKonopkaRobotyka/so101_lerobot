# Teaching SO-101 a pick and place in simulation, with a webcam instead of a leader arm

November 2026. A write-up of this repository: what was built, how, what came out, and where it stops.

## The question

Imitation learning for low-cost arms such as the SO-101 usually starts with a leader arm: a second arm the operator
moves by hand while the follower copies it and a dataset is recorded. Without any robot hardware, only a laptop with
a webcam and an RTX 4060 (8 GB), can the same pipeline run in simulation, with the operator's hand as the leader?
The project set itself four exit criteria (ADR 0005): hand teleoperation of a pick and place, about 50 recorded
demonstrations in the real SO-101 dataset schema, an ACT policy trained on them with its success rate measured, and
a public release with the results and the limits.

## How

**The hand as the leader (Gazebo, ROS 2).** MediaPipe tracks the hand in the webcam image. The core is a plain Python
package with tests (`so101_hand`); the ROS 2 node is a thin layer over it. The hand is seen from the front, over a
sector of the table drawn in the preview: its position in the image sets the gripper's position over the table, and
the inverse kinematics turns the base and points the jaws down (ADR 0006). A fist or an open palm closes or opens the
gripper at once (MediaPipe's gesture recognizer, ADR 0008).

Depth was measured before it was used, and then not used. From one camera the hand's distance is precise while its
pose stays still, but every change of the pose reads as depth: closing the hand alone moves it by 10 cm (ADR 0007).
So the gripper's height comes from a key, two levels, hover and grasp.

**The task (MuJoCo).** Gazebo has no scene to pick from, so the pick and place runs in so101-nexus, a MuJoCo
environment with a cube, a target disc and a success check (ADR 0009). The same teleoperation code drives it, without
ROS. Two models of the same arm had to agree first: comparing their parts' poses showed the base frames 32.5 mm apart
and the wrist roll's zero 0.0487 rad off. Where the grasp point sits mattered most: on the wrist's axis a top-down
grasp lifted the cube in 0–4 of 20 scenes, between the open jaws in 37 of 40.

**The data.** Every attempt became an episode of a LeRobot v3 dataset at 30 fps, in the units of LeRobot's SO-101
follower (joints in degrees, the gripper in percent) with a wrist and an overhead camera (ADR 0010). A dataset or a
policy from the simulation needs no conversion on a real arm; only the pictures would differ.

**The policy.** ACT, trained with `lerobot-train` on the laptop: LeRobot's defaults, mixed precision (bf16, 25 %
faster than fp32 with the same loss), 50,000 steps in 3.5 hours. One setting was not a default: the list of inputs.
LeRobot gives a policy every observation in the dataset, and this dataset also held the simulator's own state, the
exact poses of the cube and the disc. Left in, ACT would have learnt from information a real arm does not have.

**The evaluation.** A loop of its own (`so101_mujoco_eval`) runs the policy in the recording's scene: the same start
pose, new cube and disc positions (seeds the recordings never used), a 60 s limit, so101-nexus's success check.

## Results

| | Success | Time to success |
| --- | --- | --- |
| Teleoperation with the hand | 50 of 56 attempts (89 %) | 20.0 s mean |
| ACT on the 50 demonstrations | 17 and 16 of 50 scenes (33 %) | 31.8 s mean |
| ACT on the first 20 demonstrations (pilot) | 0 to 3 of 50 | |

- **Data mattered more than steps.** On 20 demonstrations the policy fitted them (a similar training loss) and failed
  in the scene; on 50 it succeeded a third of the time. Between 20,000 and 50,000 steps the rate stayed within its
  noise (14, 9, 15, 17 of 50 at 20k, 30k, 40k, 50k).
- **The successes are real.** The task's check only asks that the cube rests on the disc, released, and in 6 of the
  50 scenes the cube starts close enough to be pushed there. The evaluation therefore records how far the cube rose:
  all 16 successes of the second run carried it.
- **A scene's outcome is not repeatable.** Two runs of the same checkpoint on the same 50 scenes agreed on only 8
  successes: tiny numerical differences on the GPU grow in the closed loop. One run of 50 episodes is good to about
  ±13 points, so the results above give two runs and no single "best" checkpoint.
- **The failures** are mostly missed grasps: in 28 of the second run's 34 the cube never left the floor, in 6 it was
  lifted but not placed. ACT's temporal ensembling, which re-plans every step, did worse here (10 of 50).

## What I would keep

- Measure a signal before building on it: the hand's depth looked usable and was not.
- Read what the library feeds the model; the default inputs would have leaked the answer.
- Check the success metric against what it should mean, and run an evaluation twice before trusting a difference.
- Keep the logic in plain, tested Python and make ROS a thin layer: the same session code drove Gazebo and MuJoCo.

## Limits and next steps

The policy works a third of the time on one task, in one simulator, with fixed cameras rendered without noise; no
real SO-101 has been driven. The obvious next steps are more and more varied demonstrations, randomized cameras and
lighting, and a real arm. The README's [Limitations](../README.md#limitations) list the method's limits in detail.
