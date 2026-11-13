# 0008. Gripper in two states: open or closed, from the hand's gesture and the aperture

- Status: Accepted (2026-11-13, after the webcam tests of ROB-35)
- Date: 2026-11-13
- Amends: ADR 0006 (the gripper following the hand's openness in proportion, as in ADR 0003) and ADR 0002 (the
  model: MediaPipe's Gesture Recognizer, which runs the same Hand Landmarker)

## Context

ADR 0003 made the gripper follow the hand in proportion: the aperture (the thumb tip to the index tip over the palm
length, `so101_hand.measures`) between the calibrated closed and open poses mapped linearly to the jaws. From the
side the camera sees the thumb and the index finger in profile, and the gap follows the hand. ADR 0006 kept this for
the hand seen from the front, and the webcam tests of ROB-35 show that it does not work there:

- 2026-11-12: the relaxed open hand (fingers together) measured 0.65–0.85, below the open pose 1.13 taken from a hand
  spread wide (ROB-34), so the gripper opened only halfway (median 53 %). The open pose calibrated on the relaxed
  hand (0.70) fixed the opening.
- 2026-11-13, 18:26: moved to the right, the fist turned thumb up: 0.45–0.54, so the closed hand held the gripper
  34–55 % open for 6 s.
- 2026-11-13, 19:07, with two states on the aperture (closed at 0.36 or below, open at 0.70 or above): a fist with
  the thumb along the curled index finger measures 0.41–0.52 and did not close the gripper. In 48 s of curled fingers
  the gripper stayed open for 8.8 s, in four spells of 1–1.9 s; a fist held for 4 s never closed it.

Pick and place needs no half-open jaws, and closing must not depend on where the thumb lies.

## Decision

- The gripper has two states, open (1.725 rad, the safety margin under the joint limit) and closed (0 rad).
- MediaPipe's Gesture Recognizer replaces the Hand Landmarker in `so101_hand.tracker`. It runs the same landmark
  model (the same 21 points for the arm) and classifies the whole hand from them into canned gestures with a score.
  A fist (`Closed_Fist`, and `Thumb_Up` / `Thumb_Down`: a fist with the thumb out) closes the gripper and an open
  palm (`Open_Palm`) opens it, at once, with a score of 0.5 or more.
- Any other hand (`None`, `Pointing_Up`, `Victory`, `ILoveYou`, a lower score) is left to the aperture with
  hysteresis: the smoothed aperture closes the gripper at the closed pose or below, opens it at the open pose or
  above, and keeps the state in between. The two calibrated poses (`c` / `o`, `config/hand_teleop.yaml`:
  0.36 / 0.70 for my hand from the front) are the thresholds. Without a hand the state stays.
- The goal goes out at once when the state changes, not with the next goal at `rate_hz`.
- The preview shows the gesture with its score, the state and what decided it, e.g. `gripper CLOSED by Closed_Fist`
  or `gripper OPEN by aperture (closes at 0.36)`.

Replayed offline, the share of the frames with the right state, open / closed hand, in %:

| Recording (labels)                                         | Aperture | Gestures | Gestures, then aperture |
|------------------------------------------------------------|----------|----------|-------------------------|
| 2026-11-12, closing with the "beak" (read from the video)  | 100 / 98 | 100 / 0  | 100 / 97                |
| 2026-11-13 18:26, fists (read from the video)              | 100 / 100| 100 / 100| 100 / 100               |
| 2026-11-13 19:07, varied fists (two whole-hand measures)   | 100 / 82 | 77 / 100 | 100 / 94                |
| ROB-34 `grip`, the raw camera image (its cues)             | 100 / 68 | 100 / 100| 100 / 100               |

The webcam tests were cut out of the screen recordings of the preview. On 19:07 the gestures closed the gripper
within 0.5 s after the fingers curled in all 67 cases; the aperture alone was later in 17 and not within 3 s in 3.
In Gazebo (headless, `grip`) joint 6 took only 0 and 1.725 rad and followed every cue. Scripts and numbers:
`~/code/experiments/gripper_gesture_spike/`.

## Rejected or deferred

- Gestures alone: the model calls the closing "beak" (the thumb on the fingertips, 2026-11-12) and some open hands
  near the face or tilted (19:07: five spells of 0.6–2 s) `None`; with `None` keeping the state, the gripper did not
  close, or stayed closed under an open hand.
- The aperture alone: it misses the fists with the thumb along the index finger (above).
- Whole-hand measures from the 3D landmarks over the palm length, the mean distance of the five fingertips to the
  wrist or the straightness of the four fingers: they caught those fists at once, but are noisier (d′ 2.2–2.5
  against 4.0 for the aperture on 19:07) and need their own calibration; the trained classifier does that job.
- One threshold on the aperture: the noise around it would make the jaws chatter; the gap between the poses is the
  margin.
- A rate limit on the jaws: the controller moves them over the full stroke in `move_time_s` (0.1 s); to be added if
  the jump disturbs a grasp in MuJoCo.

## Consequences

- To close the gripper: a fist, whatever the thumb does, or the "beak"; to open it: an open hand. A hand in between
  keeps the state.
- The model is `gesture_recognizer.task` (8.4 MB) in `$DATA_DIR/models`, downloaded as the README says. MediaPipe is
  Apache-2.0; the model card names no license for the model file itself, so it stays out of the repository.
- `so101_hand`: `Hand` carries the gesture and its score, `gripper.next_state` decides, `Calibration.is_open` is the
  aperture's hysteresis; `openness` and `openness_to_joint` are gone. The proportional gripper stays in the history
  (tag `v0.2-side-view`).
- The combination is still wrong in 6 % of the closed frames on 19:07, partly the labels' uncertainty; the webcam
  test shows whether it matters.
- The demonstrations of M3 record the gripper's action with two values; ACT (M4) learns a gripper that switches.
