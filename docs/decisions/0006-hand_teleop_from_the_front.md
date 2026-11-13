# 0006. Hand teleoperation from the front: the hand over the workspace seen from above

- Status: Accepted (agreed on 2026-11-11); the height axis settled in ROB-34 (ADR 0007: a key, no depth), the
  mapping built in ROB-35 (`so101_hand.teleop`, `hand_teleop`) and confirmed with my hand in four webcam tests on
  2026-11-12/13 (the gripper's states: ADR 0008)
- Date: 2026-11-11
- Supersedes: in ADR 0003 the setup, the axes, the relative arm control with the clutch and the jaws following the
  hand's pitch. Its gripper measure and calibration, the EMA, the image scale, the planar IK, the TCP, the workspace
  bounds and the step limit stay.

## Context

M1 and M2 built the side view of ADR 0003: the webcam sees my right forearm across the body, the arm moves in its
vertical plane, the base is fixed and the jaws follow the hand's pitch; the clutch gives relative control (done on
2026-10-27). For M3 (ADR 0005) the hand has to pick a block: aim over the table, lower, close, lift, carry, place.

- Aiming over the table takes two axes, the base angle and the reach. In the side view the base angle is the one
  axis that needs the depth, the axis one camera measures poorly (ADR 0003, ROB-34): half of the aiming would ride
  on it.
- The side pose tires: the forearm across the body, the elbow on the desk in a marked spot, because the image
  scale is latched when the clutch engages (ADR 0003, README). About 50 demonstrations (ADR 0005) mean an hour or
  more of teleoperation.
- ROB-19 (2026-10-22) measured both poses. From the front, with the palm to the camera, the hand was found in 100%
  of the frames and the measures told open from closed in 97.6–100% of them (98.8–99.5% from the side); ADR 0003
  kept the frontal pose as the fallback.
- ROB-34 so far (2026-10-28, on screen recordings, no tape measure yet): the depth from the palm's image scale, the
  one the node has, follows the palm's turn out of the image plane (1 / cos θ) more than the distance; solvePnP and
  a 21-point scale agree within a few per cent; closing the hand shifts them by about 1 cm. The measurements against
  a tape measure are pending.
- The MTC project (so101_mtc, ROB-43, 2026-11-03) found on the same arm that with the jaws straight down the wrist
  flex limit leaves the gripper only about 6 cm of lift: a top-down grasp is reachable in part of the workspace
  (ADR 0003: the reachable pitch depends on the position).

## Decision

Setup: the webcam sees me from the front, as it stands on the laptop. The right hand is open, the palm to the
camera and the fingers up, in front of the chest; nothing lies on the desk and nothing is marked in the room. The
preview is mirrored like a selfie and all computations use the raw frame (as in ADR 0003).

The hand is the gripper seen from above:

- The preview shows the robot's workspace seen from above, from behind the base: the base at the bottom edge,
  forward up the screen, the reachable area at the current height drawn as a ring sector (the base angle × the
  reach, ROB-36). The hand's position in the sector is the gripper's position over the table (absolute mapping,
  set once as a scale and an offset in the preview, not per session): hand to my right → the gripper to the robot's
  right (the base turns); hand up → away from the base (the reach). A hand outside the sector is drawn red and the
  target is rejected.
- The hand's position is the palm's centre (the wrist and the knuckles), which hardly moves when the hand closes,
  taken in metres in the camera frame: x and y from the image at the hand's depth, z the depth. (Amended by
  ADR 0007: in pixels of the image, no depth at all.)
- Height: the gripper hovers at a fixed height above the table while the hand is at its usual distance from the
  camera, and goes down towards the table when the hand comes nearer by more than the dead zone d (ROB-34), as far
  as the push; back out, it rises. The usual distance is the hand's depth when the pause is released (and before
  that, when the hand is first seen), so nothing is calibrated. If ROB-34 finds the depth too noisy for that
  (d > 3 cm), two keys move the gripper down and up at a fixed rate instead.
- Closing the hand closes the gripper: the measure and the two-pose calibration of ADR 0003, with the frontal
  values of ROB-19. (Amended by ADR 0008: the gripper is open or closed, the two poses are its thresholds.)
- The jaws point down (a top-down grasp) and do not follow the hand any more: the IK takes the steepest pitch the
  arm reaches at the target, −90° where it can; the wrist roll stays 0.
- `space` is a pause, no longer a clutch: the arm holds while I rest or move the hand; it resumes only with a
  detected hand inside the sector, and the gripper then goes to the hand's place at the step limit (≤ 0.4 m/s),
  without a jump. `h` still sends the arm home; the `~/clutch` service stays, as the pause.
- The IK stays analytic: the base angle from the target's direction over the table, then the planar IK of ADR 0003
  in the turned plane (joints 2–4).
- The simulator shows two views at once: from above, oriented like the sector in the preview (the base at the
  bottom), to aim; from the side, to see the height and the grasp. In Gazebo for the check (ROB-35), in MuJoCo for
  the data (ROB-38).

## Rejected or deferred

- A rectangle for the workspace in the preview: from above the arm reaches a ring sector, and a rectangle would
  promise corners it cannot reach; the sector also shows the kinematics (the base angle, the reach).
- Relative control with the clutch (M2) from the front: aiming wants the plain picture "my hand is over the
  block"; relative control needs the clutch gymnastics and latches the scale at engage.
- The hand's height on the image for the gripper's height and the depth for the reach: aiming (the base angle and
  the reach) needs both well-measured axes, while the height needs only two levels, hover and table, and
  tolerates the depth's noise with a dead zone.
- The jaws following the hand's pitch (ROB-31): from the front the palm's turn is the depth cue of the image
  scale, and the top-down grasp of a block does not need it.
- A gesture instead of the pause key: one more thing the tracker must tell apart from the grasp.
- Keeping the side view behind a parameter: two interfaces to maintain and test; the side view stays in the git
  history (the last `main` before this change, 9cae4e3, tag `v0.2-side-view`) and in ADR 0003.
- A second camera or a depth camera: out of scope (ADR 0005).

## Consequences

- The ROS-free core stays: the tracker, the openness measure and calibration, the EMA, the planar IK, the TCP, the
  bounds, the step limit. New in ROB-35: the base angle in the IK, the hand's position in the camera frame, the
  absolute mapping, the height from the depth or keys, the pause; ROB-36 draws the sector; ROB-38 gives MuJoCo the
  two views.
- ROB-34 continues in the frontal pose with the same question, how precise the depth is, now for the gripper's
  height; its protocol is recorded from the front.
- The arm control of M2 (ROB-28, ROB-31) is replaced in the node; the README's stage 2 section and the GIFs
  describe the side view until ROB-35 is merged, and the Limitations say so.
- No more fatigue of the side pose and no latched scale; instead the hand must be in the sector to resume, and the
  preview shows where.
- Perspective: a hand pushed towards the camera off the optical axis also moves across the image. With the hand's
  position in metres at its depth the drift is removed; with keys for the height the hand keeps its distance and
  the drift stays small (about 3 cm at the sector's edge for a 10 cm push at f = 500 px).
- The jaws straight down are not reachable everywhere (the wrist flex limit): the sector is drawn for the pitch
  the arm reaches, and near its edge the grasp is tilted.
- The tag `v0.2-side-view` marks the last side-view version.
