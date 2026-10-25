# 0003. Hand teleoperation: camera setup, axes and control

- Status: Accepted for the setup, the axes and the gripper (pose confirmed in ROB-19, gripper control in ROB-27);
  the arm control stays proposed until ROB-28 (stage 2)
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
  hidden. ROB-19 (2026-10-22) confirmed the side view: the hand was found in 100% of frames and every measure
  told open from closed in 98.8–99.5% of frames. The frontal pose is steadier and stays the fallback if the side
  pose tires the arm.

Axes (computed on the raw frame: `u` to the right, `v` down; robot base: X forward, Z up):

- hand to my left → image right (+u) → robot +X → the gripper moves left on the screen;
- hand up → image up (−v) → robot +Z;
- hand towards or away from the camera (depth) → not used (Y = 0);
- closing the hand → the thumb–index gap shrinks → the gripper closes (joint "6" towards 0.0 rad).

The preview window shows the frame mirrored, so the hand on the screen moves like the robot in Gazebo;
all computations use the raw frame.

Note (ROB-28): in the `lerobot_ws` URDF the arm at zero points along −Y of the `base` frame and joints 2–4 turn
about its X axis, so the motion plane is the base YZ plane. "X forward" in this ADR means the arm's forward
direction (−Y of `base`); the code calls the plane coordinates reach (forward) and height (up).

Gripper (stage 1):

- Measure (chosen in ROB-19): the image distance thumb tip – index tip divided by the palm length (wrist –
  middle-finger MCP), in pixels. It had the lowest noise with the hand closed and the smallest frame-to-frame
  jumps; dividing by the palm length cancels the distance to the camera. Fallback: thumb–index from world
  landmarks. Also compared: thumb – mean of the four fingertips ("beak") and fingertips–wrist / palm length
  ("open ratio").
- Two-pose calibration (keys `c` and `o`: quartiles of the measure over the last second, so noise keeps the
  gripper at its ends) maps the measure to 0…1, then to joint "6": 0.0 rad closed, 1.725 rad open — the URDF
  limit 1.745 minus a 0.02 rad margin, because a goal beyond the limit leaves the simulated jaw stuck.
  At 0.0 rad the jaw tips are still 16 mm apart (their meshes): grasping thin objects later (M3) needs goals
  below 0.0, down to the lower limit −0.17 rad.
- EMA filter (alpha 0.4, about 50 ms of lag at 20 fps; One Euro filter if lag or jitter is a problem); goals at
  10 Hz with `time_from_start` 0.1 s; no hand for more than 0.5 s → stop sending goals, the controller holds
  the last one.

Arm (stage 2):

- Relative control with a clutch: while engaged, TCP target = TCP pose at engage + gain × hand displacement
  since engage; releasing stops the robot (like LeRobot's `EEReferenceAndDelta` with a latched reference).
  Built in ROB-28 (`so101_hand/teleop.py`) with space toggling the clutch instead of a held key: the OpenCV
  window reports key presses, not held keys. The service `~/clutch` does the same without the keyboard. The
  jaws keep the pitch they had at engage; the arm starts from a home pose (TCP 0.25 m forward, 0.15 m up,
  jaws 45° down).
- Scale in metres per pixel at the hand's depth = palm length in world landmarks / palm length in pixels.
- Workspace bounds in XZ and a maximum step per cycle (like LeRobot's `EEBoundsAndSafety`).
- Own planar IK (`so101_hand/kinematics.py`): with the pan at 0 and a fixed wrist roll, shoulder lift, elbow
  flex and wrist flex form a planar 3R chain; target reach, height and pitch → wrist point → two-link solution
  (law of cosines) → wrist angle; of the two solutions (elbow up or down) the one closest to the current
  joints. The link vectors at zero come from the URDF; the forward kinematics matches a generic FK of the URDF
  and TF from `robot_state_publisher` to 0.005 mm.
- The TCP is defined in our code (the third-party URDF has none): between the closed jaws, 7 mm behind their
  tips, at (0, 0, −0.098) m in the `gripper` link frame.

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
- Holding the forearm across the body tires the shoulder (resting the elbow on the desk helps).
- With the upstream controllers (10 Hz) the simulated gripper lagged about 1 s behind the goals; with our
  configuration (ADR 0004) the whole chain from the hand to the jaw lags about 0.3 s, of which the EMA and the
  10 Hz goals are about 0.1 s.
- The planar IK stays as the explained baseline even when full 3D is added.
