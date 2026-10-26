"""ROS 2 node: the operator's hand seen by the webcam → goals for the simulated SO-101 (ADR 0003).

Stage 1: the hand's openness drives the gripper. Stage 2 (parameter `arm`): while the clutch is engaged, moving
the hand moves the gripper in the arm's plane (so101_hand.teleop), through our planar IK (so101_hand.kinematics).
The logic lives in the so101_hand package (plain Python, tested with pytest). This node only reads frames with
OpenCV (not cv_bridge: it is built for numpy 1.x), shows the preview and publishes the goals.

Keys in the preview window: c / o = calibrate the closed / open hand (hold the pose for about a second first),
space = engage or release the clutch, h = arm back to the home pose, q = quit. The service ~/clutch
(std_srvs/SetBool) engages or releases the clutch without the keyboard.
"""

import math
import time
from collections import deque

import cv2
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_hand.calibration import Calibration
from so101_hand.filters import HoldingEma
from so101_hand.gripper import openness_to_joint
from so101_hand.kinematics import TCP_IN_GRIPPER, Pose, forward, inverse, solutions
from so101_hand.measures import INDEX_TIP, MIDDLE_MCP, THUMB_TIP, WRIST, aperture, metres_per_pixel
from so101_hand.teleop import Bounds, Clutch
from so101_hand.tracker import DEFAULT_MODEL, HandTracker

WINDOW = "so101_hand"
GRIPPER_JOINT = "6"
ARM_JOINTS = ["1", "2", "3", "4", "5"]  # 1 (shoulder pan) and 5 (wrist roll) stay at 0: the arm moves in its plane
CALIBRATION_WINDOW_S = 1.0  # c / o take the apertures of the last second
HOME = Pose(reach=0.25, height=0.15, pitch=-math.pi / 4)  # TCP at start and after h: in front, jaws 45° down
HOME_TIME_S = 2.0


class HandTeleopNode(Node):
    """Frame → HandTracker → aperture → gripper goal; with `arm`: wrist → Clutch → IK → arm goal."""

    def __init__(self):
        super().__init__("hand_teleop")
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
        alpha = self.declare_parameter("ema_alpha", 0.4).value
        hold_s = self.declare_parameter("hold_s", 0.5).value
        self.filter = HoldingEma(alpha=alpha, hold_s=hold_s)
        self.wrist_filters = (HoldingEma(alpha=alpha, hold_s=hold_s), HoldingEma(alpha=alpha, hold_s=hold_s))
        self.arm = self.declare_parameter("arm", False).value
        self.clutch = Clutch(
            gain=self.declare_parameter("gain", 1.0).value,
            bounds=Bounds(
                reach=tuple(self.declare_parameter("bounds_reach", [0.10, 0.40]).value),
                height=tuple(self.declare_parameter("bounds_height", [0.02, 0.40]).value),
            ),
            max_step_m=self.declare_parameter("max_step_m", 0.02).value,
            reachable=lambda pose: bool(solutions(pose)),
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
        self.hand = None  # the last detected hand, for engaging the clutch
        if self.arm:
            self.arm_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
            self.joints = {}  # the latest /joint_states
            self.arm_command = None  # the last commanded (q2, q3, q4)
            self.home_sent = False
            self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
            self.create_service(SetBool, "~/clutch", self.on_clutch)
            self.publish_tcp_frame()
        self.get_logger().info(
            f"reading {self.source}, goals at {1.0 / self.period_s:.0f} Hz; calibration: closed "
            f"{self.calibration.closed:.2f}, open {self.calibration.open:.2f}; arm {'on' if self.arm else 'off'}"
        )

    def run(self) -> None:
        last_goal_s = float("-inf")
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().info(f"no more frames from {self.source}")
                break
            t_s = time.monotonic()
            self.hand = self.tracker.detect(frame, t_s)
            ap = None if self.hand is None else aperture(self.hand.image)
            if ap is not None:
                self.recent.append((t_s, ap))
            openness = self.filter.update(t_s, None if ap is None else self.calibration.openness(ap))
            target = self.arm_target(t_s) if self.arm else None
            # None: the hand is lost for longer than hold_s (or the clutch is released), so no goals; the controller
            # keeps the last one. 0.9: frames arrive with jitter; without the margin a frame 1 ms early would wait.
            if t_s - last_goal_s >= 0.9 * self.period_s and (openness is not None or target is not None):
                if openness is not None:
                    self.send_goal(openness_to_joint(openness))
                if target is not None:
                    self.send_arm(target, self.move_time)
                last_goal_s = t_s
            if self.preview and not self.show(frame, ap, openness, target):
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.frame_period_s:
                time.sleep(max(0.0, self.frame_period_s - (time.monotonic() - t_s)))

    def send_goal(self, joint_rad: float) -> None:
        point = JointTrajectoryPoint(positions=[joint_rad], time_from_start=self.move_time)
        self.publisher.publish(JointTrajectory(joint_names=[GRIPPER_JOINT], points=[point]))

    # --- stage 2: the arm ---

    def arm_target(self, t_s: float) -> Pose | None:
        """The clutch's TCP target for the smoothed wrist position, or None (released, hand lost)."""
        wrist = None if self.hand is None else self.hand.image[WRIST]
        u = self.wrist_filters[0].update(t_s, None if wrist is None else float(wrist[0]))
        v = self.wrist_filters[1].update(t_s, None if wrist is None else float(wrist[1]))
        return self.clutch.update(None if u is None or v is None else (u, v))

    def send_arm(self, target: Pose, move_time) -> None:
        q = inverse(target, current=self.current_arm() or (0.0, 0.0, 0.0))
        if q is None:  # the clutch only passes reachable targets; HOME is reachable
            return
        self.arm_command = q
        point = JointTrajectoryPoint(positions=[0.0, *q, 0.0], time_from_start=move_time)
        self.arm_publisher.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def current_arm(self) -> tuple[float, float, float] | None:
        """Where the arm is going (the last command), or where it is (/joint_states), or None before both."""
        if self.arm_command is not None:
            return self.arm_command
        if all(j in self.joints for j in ("2", "3", "4")):
            return (self.joints["2"], self.joints["3"], self.joints["4"])
        return None

    def go_home(self) -> None:
        self.clutch.release()
        self.send_arm(HOME, Duration(seconds=HOME_TIME_S).to_msg())
        self.get_logger().info(f"arm to the home pose (TCP reach {HOME.reach} m, height {HOME.height} m)")

    def engage(self) -> str | None:
        """Engage the clutch at the current hand and TCP; returns why it could not, or None."""
        q = self.current_arm()
        if self.hand is None or q is None:
            return "no hand in view" if self.hand is None else "no joint states yet"
        # the smoothed wrist, as in arm_target(): with the raw one the first target would jump by the filter's lag
        wrist = (self.wrist_filters[0].value, self.wrist_filters[1].value)
        self.clutch.engage(wrist, metres_per_px=metres_per_pixel(self.hand.image, self.hand.world), tcp=forward(*q))
        return None

    def on_joint_states(self, msg: JointState) -> None:
        self.joints.update(zip(msg.name, msg.position))
        if not self.home_sent:  # the first message: the controllers are up, the publisher is matched
            self.home_sent = True
            self.go_home()

    def on_clutch(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            problem = self.engage()
            response.success, response.message = problem is None, problem or "engaged"
        else:
            self.clutch.release()
            response.success, response.message = True, "released"
        return response

    def publish_tcp_frame(self) -> None:
        """Static TF frame `tcp` on the gripper link, to see our TCP in RViz (the third-party URDF has none)."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = "gripper", "tcp"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, TCP_IN_GRIPPER)
        tf.transform.rotation.w = 1.0
        self.tcp_broadcaster = StaticTransformBroadcaster(self)
        self.tcp_broadcaster.sendTransform(tf)

    # --- preview ---

    def show(self, frame, ap, openness, target) -> bool:
        """Draw the measured points, show the mirrored preview and handle the keys; False after q."""
        if self.hand is not None:
            lm = self.hand.image
            p = {i: (int(lm[i][0]), int(lm[i][1])) for i in (WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP)}
            cv2.line(frame, p[WRIST], p[MIDDLE_MCP], (255, 255, 255), 2)  # palm length: the unit of the aperture
            cv2.line(frame, p[THUMB_TIP], p[INDEX_TIP], (0, 0, 255), 3)  # the measured gap
            if self.arm:
                cv2.circle(frame, p[WRIST], 8, (0, 255, 0) if self.clutch.engaged else (128, 128, 128), -1)
        view = cv2.flip(frame, 1)  # mirrored like a selfie; drawn before the flip, so the lines stay on the hand
        lines = [
            f"aperture {ap:.2f}" if ap is not None else "no hand",
            f"gripper {100 * openness:.0f}% open" if openness is not None else "hand lost: the gripper stays",
            f"calibration: closed {self.calibration.closed:.2f}, open {self.calibration.open:.2f}",
        ]
        if self.arm:
            state = "ENGAGED" if self.clutch.engaged else "released"
            where = f": TCP {100 * target.reach:.0f} cm forward, {100 * target.height:.0f} cm up" if target else ""
            lines.append(f"arm {state}{where}")
            lines.append("c / o = closed / open hand, space = clutch, h = home, q = quit")
        else:
            lines.append("c = closed hand, o = open hand (hold 1 s first), q = quit")
        for i, text in enumerate(lines):
            cv2.putText(view, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("c"), ord("o")):
            self.calibrate("closed" if key == ord("c") else "open")
        elif self.arm and key == ord(" "):
            if self.clutch.engaged:
                self.clutch.release()
            else:
                problem = self.engage()
                if problem:
                    self.get_logger().warning(f"clutch not engaged: {problem}")
        elif self.arm and key == ord("h"):
            self.go_home()
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
            f"aperture_open {self.calibration.open:.2f} (to keep them, put them in config/hand_teleop.yaml)"
        )

    def close(self) -> None:
        self.cap.release()
        self.tracker.close()
        if self.preview:
            cv2.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = HandTeleopNode()
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
