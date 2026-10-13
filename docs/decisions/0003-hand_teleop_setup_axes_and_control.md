# 0003. Hand teleoperation: camera setup, axes and control

- Status: Proposed — the pose is checked in ROB-19, the control in ROB-27 (stage 1) and ROB-28 (stage 2)
- Date: 2026-10-13

## Context

Only the laptop webcam is available. A single camera measures motion in the image plane well and depth
poorly. The idea: show the right hand to the camera from the side, posed like the SO-101 gripper tilted down,
and let the image play the role of a side view of the robot. The robot then moves at most in the XZ plane
of its base (shoulder pan fixed at 0), and closing the hand closes the gripper.

## Decision

Setup:

```
Desk from above                          Gazebo view from the robot's left side

   [======== laptop ========]                     +Z
             (cam)                                 ^      ____
              |                                    |     /    \
              v  40-60 cm                          |    /      \___[#] base
        [ right hand ]                             |   V gripper
              ^                           +X <-----+--------------------
             me                           (robot forward is to the left)
```

- Right hand 40–60 cm in front of the webcam, forearm towards the screen, fingers pointing down like a beak,
  palm facing my left, thumb towards the camera: the camera sees the thumb and index finger from the side
  and the gap between them opens left–right in the image.
- Gazebo is viewed from the robot's left side, so robot +X (forward) is to the left on the screen and +Z is up.

Axes (computed on the raw frame: `u` to the right, `v` down; robot base: X forward, Z up):

- hand to my left → image right (+u) → robot +X → the gripper moves left on the screen;
- hand up → image up (−v) → robot +Z;
- hand towards or away from the camera (depth) → not used (Y = 0);
- closing the hand → the thumb–index gap shrinks → the gripper closes (joint "6" towards 0.0 rad).

The preview window shows the frame mirrored, so the hand on the screen moves like the robot in Gazebo;
all computations use the raw frame.

Gripper (stage 1):

- Measure: thumb tip – index tip distance from world landmarks (metres); ROB-19 compares it with the image
  distance divided by the palm length.
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
- The hand pose is unusual and may tire the wrist; the spike (ROB-19) can change it.
- The planar IK stays as the explained baseline even when full 3D is added.
