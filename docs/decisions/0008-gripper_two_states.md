# 0008. Gripper in two states: open or closed, with hysteresis

- Status: Accepted (2026-11-13, after the webcam tests of ROB-35)
- Date: 2026-11-13
- Amends: ADR 0006 (the gripper following the hand's openness in proportion, as in ADR 0003)

## Context

ADR 0003 made the gripper follow the hand in proportion: the aperture (the thumb tip to the index tip over the palm
length, `so101_hand.measures`) between the calibrated closed and open poses mapped linearly to the jaws. From the
side the camera sees the thumb and the index finger in profile, and the gap follows the hand. ADR 0006 kept this for
the hand seen from the front, and the two webcam tests of ROB-35 show that it does not work there:

- 2026-11-12: the relaxed open hand (fingers together) measured 0.65–0.85, below the open pose 1.13 taken from a hand
  spread wide (ROB-34), so the gripper opened only halfway (median 53 %). The open pose calibrated on the relaxed
  hand (0.70) fixed the opening.
- 2026-11-13: moved to the right, the fist turned thumb up, the thumb tip off the index finger: 0.45–0.54, so the
  closed hand held the gripper 34–55 % open for 6 s.

Pick and place needs no half-open jaws: they are open over the block and closed around it.

## Decision

- The gripper has two states, open (1.725 rad, the safety margin under the joint limit) and closed (0 rad), with
  hysteresis: it closes when the smoothed aperture (the EMA of ADR 0003) falls to the closed pose, opens when it
  rises to the open pose, and keeps its state in between.
- The two calibrated poses are the two thresholds, so there is no new parameter: `c` / `o` and
  `config/hand_teleop.yaml` (0.36 / 0.70 for my hand from the front) set them as before.
- The measure stays the aperture. Both tests replayed offline (the measures computed again from the camera image in
  the screen recordings, the thresholds 0.36 / 0.70): the state is right in 100 % / 98 % of the open / closed frames
  on 2026-11-12 and in 100 % / 100 % on 2026-11-13; it switches 6 times in each, at the real changes of the hand,
  and never while the fist is turned.
- The preview shows the state and the threshold that changes it, e.g. `gripper CLOSED (opens at 0.70)`.

## Rejected or deferred

- The fingertips–wrist measure (the mean distance of the four fingertips to the wrist over the palm length, in the
  world landmarks; robust to tilt in ROB-34): 97–100 % right with two states on the same recordings, no better than
  the aperture. It is the fallback if the turned fist (below) gets in the way.
- One threshold: the noise around it would make the jaws chatter; the gap between the poses is the margin.
- A rate limit on the jaws: the controller moves them over the full stroke in `move_time_s` (0.1 s); to be added if
  the jump disturbs a grasp in MuJoCo.

## Consequences

- To close the gripper: a fist towards the camera or the thumb on the index finger, like the jaws. An open hand
  turned straight into a fist with the thumb up stays between the thresholds and keeps the gripper open.
- `so101_hand`: `Calibration.is_open` replaces `openness`, and `gripper.openness_to_joint` is gone; the proportional
  gripper stays in the history (tag `v0.2-side-view`).
- The demonstrations of M3 record the gripper's action with two values; ACT (M4) learns a gripper that switches.
