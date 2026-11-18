# 0002. Hand tracking with MediaPipe Hand Landmarker

- Status: Accepted; detection in the side view confirmed in ROB-19 (2026-10-22). Amended by ADR 0008: the node uses
  MediaPipe's Gesture Recognizer (`gesture_recognizer.task`), which runs this Hand Landmarker and adds the gesture.
- Date: 2026-10-13

## Context

The webcam teleoperation needs hand keypoints in real time on the laptop CPU. Candidates:

- MediaPipe Hand Landmarker (Tasks API), Apache-2.0: 21 hand landmarks in the image and the same landmarks in
  metric 3D coordinates centred on the hand ("world landmarks"); real time on a CPU.
- Ultralytics YOLO pose, AGPL-3.0, used by the legacy `lerobot_mimic` script: the body model
  (`yolo26m-pose.pt`) has no finger keypoints, and the AGPL would cover a public repository that ships it.

A dry run on 2026-10-13 showed that `pip install mediapipe` (1.1.0) in the `so101_ws` venv would also install
`opencv-contrib-python` 5.0.0.93. That venv already has `opencv-python` and `opencv-python-headless`
4.12.0.88, which share the `cv2` module; a third variant from a new major version would replace it.
numpy 2.2.6 would stay.

## Decision

- Use MediaPipe Hand Landmarker through the Tasks API in VIDEO mode; the legacy `mp.solutions` API is not used.
- Keep the model file (`hand_landmarker.task`) in `$DATA_DIR/models` (default `~/data/models`), outside git;
  the ROS node takes its path as a parameter.
- Do not install `mediapipe` into the `so101_ws` venv with its default dependencies: `opencv-contrib-python` is
  pinned to the version of the other OpenCV wheels, 4.12.0.88, in `constraints.txt` (ROB-27). The ROB-19 spike
  used its own venv in `~/code/experiments/`.

## Consequences

- The new code path is Apache-2.0 only; `lerobot_mimic` stays as a legacy example (license question in M5).
  Resolved (2026-11-17, ROB-46): the repository is released under Apache-2.0 and `lerobot_mimic` with the
  `ultralytics` dependency was removed; it stays in the history (tag `v0.3-gazebo`).
- World landmarks give hand distances in metres without a calibration template.
- Handedness is not used: on raw webcam frames the right hand is labelled "Right", but in the side view the label
  flickers to "Left" in 3–11% of frames (ROB-19), so the code takes the one hand in view.
- MediaPipe imports `matplotlib`; the apt one is built for numpy 1.x, so the venv has its own.
- If the side view proves unreliable (ROB-19), the hand pose in ADR 0003 changes, not the library.
