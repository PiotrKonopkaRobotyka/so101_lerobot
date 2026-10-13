# 0003. Hand teleoperation: camera setup, axes and control

- Status: Proposed — the pose is checked in ROB-19, the control in ROB-27 (stage 1) and ROB-28 (stage 2)
- Date: 2026-10-13

## Context

Only the laptop webcam is available. A single camera measures motion in the image plane well and depth
poorly. The idea: show the right arm to the camera from the side, with the hand posed like the SO-101 gripper
tilted down, and let the image play the role of a side view of the robot. The robot then moves at most in the XZ plane
of its base (shoulder pan fixed at 0), and closing the hand closes the gripper.

## Decision

Setup:

```
Desk from above                              Mirrored preview ≈ Gazebo view from the robot's left side

     [======== laptop ========]                 +Z
               (cam)                             ^
                 |  40-60 cm                     |   hand ≈ gripper        elbow ≈ base
                 v                               |      V <====== forearm ======o
   hand <====== forearm ====== elbow             |
   (my left)    (right arm)    (my right)        +-------------------------------------
                 me                       +X <-- robot forward is to the left
```

- My right forearm is held across the body, 40–60 cm in front of the webcam: elbow on my right, hand on my
  left. The camera sees the arm from the side; in the mirrored preview the elbow is where the robot base is
  in Gazebo (right) and the hand where the gripper is (left).
- The hand is posed like the gripper tilted down: fingers pointing down, thumb and fingers as the two jaws,
  palm not facing the camera.
- Gazebo is viewed from the robot's left side, so robot +X (forward) is to the left on the screen and +Z is up.
- Risk: with the palm down the camera sees the little-finger side, so the thumb and index finger can be
  hidden. ROB-19 compares this side pose with a frontal pose and several aperture measures.

Axes (computed on the raw frame: `u` to the right, `v` down; robot base: X forward, Z up):

- hand to my left → image right (+u) → robot +X → the gripper moves left on the screen;
- hand up → image up (−v) → robot +Z;
- hand towards or away from the camera (depth) → not used (Y = 0);
- closing the hand → the thumb–index gap shrinks → the gripper closes (joint "6" towards 0.0 rad).

The preview window shows the frame mirrored, so the hand on the screen moves like the robot in Gazebo;
all computations use the raw frame.

Gripper (stage 1):

- Measure, chosen in ROB-19 from: thumb tip – index tip distance, thumb tip – mean of the four fingertips
  ("beak", robust when the index finger is hidden), mean fingertip–wrist distance / palm length ("open ratio",
  open hand vs fist), all from world landmarks, and the image thumb–index distance / palm length.
- Two-pose calibration (open, closed) maps the measure to 0…1, then to joint "6" (1.745 rad open, 0.0 rad closed).
- EMA filter first (One Euro filter if lag or jitter is a problem); no hand for more than 0.5 s → stop
  sending goals, the controller holds the last one.

Arm (stage 2):

- Relative control with a clutch: while a key is held, TCP target = TCP pose at the press + gain × hand
  displacement since the press; releasing the key stops the robot (like LeRobot's `EEReferenceAndDelta`
  with a latched reference).
- Scale in metres per pixel at the hand's depth = palm length in world landmarks / palm length in pixels.
- Workspace bounds in XZ and a maximum step per cycle (like LeRobot's `EEBoundsAndSafety`).
- Own planar IK: with the pan at 0 and a fixed wrist roll, shoulder lift, elbow flex and wrist flex form a
  planar 3R chain; target x, z and pitch → wrist point → two-link solution (law of cosines) → wrist angle.
  Zero offsets come from the URDF and are checked by a forward-kinematics test against TF.
- The TCP frame (between the jaws) is defined in our code; the third-party URDF has none.

## Rejected or deferred

- Absolute mapping with a hand-size template and depth from apparent hand size: relative control with a
  clutch and the world/image scale make both unnecessary.
- MoveIt Plan & Execute in the loop: it plans whole trajectories and stops after each (stop-and-go).
  MoveIt stays for verification.
- MoveIt Servo: real-time, but ROS-only; it would not carry over to MuJoCo.
- LeRobot `RobotKinematics` (placo) now: a new dependency, a different URDF (`gripper_frame_link`, named
  joints) and degrees. Evaluated later for full 3D in ROB-29, next to the planar IK and behind the same
  interface (end-effector pose → joints), selected by a parameter.

## Consequences

- No depth estimation and no calibration template: one camera is enough for stages 1–2.
- Holding the forearm across the body tires the shoulder (resting the elbow on the desk helps); the spike
  (ROB-19) can change the pose.
- The planar IK stays as the explained baseline even when full 3D is added.
