"""ROS 2 node: the operator's hand seen by the webcam → goals for the simulated SO-101 gripper (stage 1, ADR 0003).

The logic lives in the so101_hand package (plain Python, tested with pytest). This node only reads frames with
OpenCV (not cv_bridge: it is built for numpy 1.x), shows the preview and publishes the goals.

Keys in the preview window: c = calibrate the closed hand, o = calibrate the open hand (hold the pose for about
a second before pressing), q = quit.
"""

import time
from collections import deque

import cv2
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_hand.calibration import Calibration
from so101_hand.filters import HoldingEma
from so101_hand.gripper import openness_to_joint
from so101_hand.measures import INDEX_TIP, MIDDLE_MCP, THUMB_TIP, WRIST, aperture
from so101_hand.tracker import DEFAULT_MODEL, HandTracker

WINDOW = "so101_hand"
GRIPPER_JOINT = "6"
CALIBRATION_WINDOW_S = 1.0  # c / o take the apertures of the last second


class HandGripperNode(Node):
    """Frame → HandTracker → aperture → Calibration → HoldingEma → joint "6" → /gripper_controller/joint_trajectory."""

    def __init__(self):
        super().__init__("hand_gripper")
        model_path = self.declare_parameter("model_path", "").value or str(DEFAULT_MODEL)
        camera_index = self.declare_parameter("camera_index", 0).value
        video_path = self.declare_parameter("video_path", "").value
        self.preview = self.declare_parameter("preview", True).value
        self.period_s = 1.0 / self.declare_parameter("rate_hz", 10.0).value
        self.move_time = Duration(seconds=self.declare_parameter("move_time_s", 0.1).value).to_msg()
        self.calibration = Calibration(
            closed=self.declare_parameter("aperture_closed", 0.2).value,
            open=self.declare_parameter("aperture_open", 1.6).value,
        )
        self.filter = HoldingEma(
            alpha=self.declare_parameter("ema_alpha", 0.4).value,
            hold_s=self.declare_parameter("hold_s", 0.5).value,
        )

        self.tracker = HandTracker(model_path)
        self.source = video_path or f"camera {camera_index}"
        if video_path:  # a recorded video instead of the webcam, replayed in real time
            self.cap = cv2.VideoCapture(video_path)
            self.frame_period_s = 1.0 / (self.cap.get(cv2.CAP_PROP_FPS) or 20.0)
        else:
            self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.frame_period_s = 0.0  # the camera sets the pace
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {self.source}")

        self.publisher = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.recent = deque(maxlen=60)  # (time, aperture) of the last frames with a hand, for c / o
        self.get_logger().info(
            f"reading {self.source}, goals at {1.0 / self.period_s:.0f} Hz to /gripper_controller/joint_trajectory; "
            f"calibration: closed {self.calibration.closed:.2f}, open {self.calibration.open:.2f}"
        )

    def run(self) -> None:
        last_goal_s = float("-inf")
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().info(f"no more frames from {self.source}")
                break
            t_s = time.monotonic()
            landmarks = self.tracker.detect(frame, t_s)
            ap = None if landmarks is None else aperture(landmarks)
            if ap is not None:
                self.recent.append((t_s, ap))
            openness = self.filter.update(t_s, None if ap is None else self.calibration.openness(ap))
            # None: the hand is lost for longer than hold_s, so no goals; the controller keeps the last one.
            # 0.9: frames arrive with jitter; without the margin a frame 1 ms early would wait for the next one.
            if openness is not None and t_s - last_goal_s >= 0.9 * self.period_s:
                self.send_goal(openness_to_joint(openness))
                last_goal_s = t_s
            if self.preview and not self.show(frame, landmarks, ap, openness):
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.frame_period_s:
                time.sleep(max(0.0, self.frame_period_s - (time.monotonic() - t_s)))

    def send_goal(self, joint_rad: float) -> None:
        point = JointTrajectoryPoint(positions=[joint_rad], time_from_start=self.move_time)
        self.publisher.publish(JointTrajectory(joint_names=[GRIPPER_JOINT], points=[point]))

    def show(self, frame, landmarks, ap, openness) -> bool:
        """Draw the measured points, show the mirrored preview and handle the keys; False after q."""
        if landmarks is not None:
            p = {i: (int(landmarks[i][0]), int(landmarks[i][1])) for i in (WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP)}
            cv2.line(frame, p[WRIST], p[MIDDLE_MCP], (255, 255, 255), 2)  # palm length: the unit of the aperture
            cv2.line(frame, p[THUMB_TIP], p[INDEX_TIP], (0, 0, 255), 3)  # the measured gap
        view = cv2.flip(frame, 1)  # mirrored like a selfie; drawn before the flip, so the lines stay on the hand
        lines = [
            f"aperture {ap:.2f}" if ap is not None else "no hand",
            f"gripper {100 * openness:.0f}% open" if openness is not None else "hand lost: the gripper stays",
            f"calibration: closed {self.calibration.closed:.2f}, open {self.calibration.open:.2f}",
            "c = closed hand, o = open hand (hold 1 s first), q = quit",
        ]
        for i, text in enumerate(lines):
            cv2.putText(view, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("c"), ord("o")):
            self.calibrate("closed" if key == ord("c") else "open")
        return key != ord("q")

    def calibrate(self, pose: str) -> None:
        now = time.monotonic()
        samples = [ap for t_s, ap in self.recent if now - t_s <= CALIBRATION_WINDOW_S]
        try:
            if pose == "closed":
                self.calibration = self.calibration.with_closed(samples)
            else:
                self.calibration = self.calibration.with_open(samples)
        except ValueError as error:
            self.get_logger().warning(f"{pose} hand not calibrated: {error}")
            return
        self.get_logger().info(
            f"{pose} hand calibrated from {len(samples)} frames: aperture_closed {self.calibration.closed:.2f}, "
            f"aperture_open {self.calibration.open:.2f} (to keep them, put them in config/hand_gripper.yaml)"
        )

    def close(self) -> None:
        self.cap.release()
        self.tracker.close()
        if self.preview:
            cv2.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = HandGripperNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
