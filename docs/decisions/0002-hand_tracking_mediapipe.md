# 0002. Hand tracking with MediaPipe Hand Landmarker

- Status: Accepted; detection quality in the side view is measured in ROB-19
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
- Do not install `mediapipe` into the `so101_ws` venv with its default dependencies: pin
  `opencv-contrib-python` to the version of the other OpenCV wheels, or install it without dependencies
  (decided in the stage 1 issue, ROB-27). The ROB-19 spike uses its own venv in `~/code/experiments/`.

## Consequences

- The new code path is Apache-2.0 only; `lerobot_mimic` stays as a legacy example (license question in M5).
- World landmarks give hand distances in metres without a calibration template.
- MediaPipe reports handedness for a mirrored (selfie) image: on raw webcam frames a right hand is labelled "Left".
- If the side view proves unreliable (ROB-19), the hand pose in ADR 0003 changes, not the library.
