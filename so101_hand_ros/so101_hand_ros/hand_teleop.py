"""ROS 2 node: the operator's hand seen by the webcam → goals for the simulated SO-101 (ADR 0006, ADR 0007).

Stage 1: the hand opens and closes the gripper (two states, ADR 0008). Stage 2 (parameter `arm`): the hand over the
workspace seen from above drives the gripper over the table: where the palm is in the preview, inside the drawn
sector, is where the gripper goes (so101_hand.teleop), the base turning with it (so101_hand.kinematics); the height
is a level toggled with the key z (hover or grasp) and the jaws point down.
The logic lives in the so101_hand package (plain Python, tested with pytest). This node only reads frames with
OpenCV (not cv_bridge: it is built for numpy 1.x), shows the preview and publishes the goals.

Keys in the preview window: c / o = calibrate the closed / open hand (hold the pose for about a second first),
space = follow the hand or pause, z = hover or grasp height, h = arm back to the home pose, q = quit. Without the
keyboard: the services ~/clutch (std_srvs/SetBool: true follows, false pauses) and ~/grasp (true = grasp height,
false = hover).
"""

import math
import time
from collections import deque

import cv2
import numpy as np
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
from so101_hand.gripper import CLOSED_RAD, OPEN_RAD, next_state
from so101_hand.kinematics import PAN_AXIS, PLANE_OFFSET, TCP_IN_GRIPPER, Point, forward3d, inverse_steepest
from so101_hand.measures import INDEX_TIP, MIDDLE_MCP, THUMB_TIP, WRIST, aperture, palm_centre
from so101_hand.teleop import Bounds, Follower, TableMap
from so101_hand.tracker import DEFAULT_MODEL, HandTracker

WINDOW = "so101_hand"
GRIPPER_JOINT = "6"
ARM_JOINTS = ["1", "2", "3", "4", "5"]  # 5 (wrist roll) stays at 0
CALIBRATION_WINDOW_S = 1.0  # c / o take the apertures of the last second
HOME_REACH = 0.20  # TCP straight ahead of the pan axis at the hover height, jaws down: at start and after h
HOME_TIME_S = 2.0
GREEN, GREY, RED, WHITE, YELLOW = (0, 255, 0), (128, 128, 128), (0, 0, 255), (255, 255, 255), (0, 255, 255)


class HandTeleopNode(Node):
    """Frame → HandTracker → aperture → gripper state → goal; with `arm`: palm → Follower → IK → arm goal."""

    def __init__(self):
        super().__init__("hand_teleop")
        model_path = self.declare_parameter("model_path", "").value or str(DEFAULT_MODEL)
        camera_index = self.declare_parameter("camera_index", 0).value
        video_path = self.declare_parameter("video_path", "").value
        self.preview = self.declare_parameter("preview", True).value
        self.period_s = 1.0 / self.declare_parameter("rate_hz", 10.0).value
        self.move_time = Duration(seconds=self.declare_parameter("move_time_s", 0.1).value).to_msg()
        self.calibration = Calibration(
            closed=self.declare_parameter("aperture_closed", 0.4).value,
            open=self.declare_parameter("aperture_open", 1.1).value,
        )
        alpha = self.declare_parameter("ema_alpha", 0.4).value
        hold_s = self.declare_parameter("hold_s", 0.5).value
        self.filter = HoldingEma(alpha=alpha, hold_s=hold_s)  # the aperture
        self.gripper_open = True  # the gripper's state (ADR 0008); it starts open
        self.gripper_by = "start"  # what decided the state: a gesture or "aperture"
        self.palm_filters = (HoldingEma(alpha=alpha, hold_s=hold_s), HoldingEma(alpha=alpha, hold_s=hold_s))
        self.arm = self.declare_parameter("arm", False).value
        self.levels = {  # TCP heights in metres; z toggles between them
            "hover": self.declare_parameter("hover_height", 0.08).value,
            "grasp": self.declare_parameter("grasp_height", 0.02).value,
        }
        self.level = "hover"
        self.pitch_range = tuple(self.declare_parameter("pitch_range", [-math.pi / 2, -math.pi / 4]).value)
        self.follower = Follower(
            table=TableMap(
                px_per_m=self.declare_parameter("px_per_m", 800.0).value,
                axis_px=tuple(self.declare_parameter("axis_px", [320.0, 440.0]).value),
            ),
            bounds=Bounds(
                reach=tuple(self.declare_parameter("bounds_reach", [0.12, 0.34]).value),
                pan=tuple(self.declare_parameter("bounds_pan", [-math.pi / 3, math.pi / 3]).value),
                height=tuple(self.declare_parameter("bounds_height", [0.02, 0.40]).value),
            ),
            max_step_m=self.declare_parameter("max_step_m", 0.02).value,
            max_step_height_m=self.declare_parameter("max_step_height_m", 0.005).value,
            reachable=lambda point: self.solve(point) is not None,
        )
        self.home = Point(float(PAN_AXIS[0] + HOME_REACH), float(PAN_AXIS[1] + PLANE_OFFSET), self.levels["hover"])

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
        self.hand = None  # the last detected hand
        self.palm_px = None  # the smoothed palm centre in the raw frame, or None without a hand
        if self.arm:
            self.arm_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
            self.joints = {}  # the latest /joint_states
            self.arm_command = None  # the last commanded (q1, q2, q3, q4)
            self.home_sent = False
            self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
            self.create_service(SetBool, "~/clutch", self.on_clutch)
            self.create_service(SetBool, "~/grasp", self.on_grasp)
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
            smoothed = self.filter.update(t_s, ap)  # the aperture's EMA, held while the hand is briefly lost
            if self.hand is not None:  # without a hand the state stays
                was_open = self.gripper_open
                self.gripper_open, self.gripper_by = next_state(
                    was_open, self.hand.gesture, self.hand.score, smoothed, self.calibration
                )
                if self.gripper_open != was_open:  # at once, not with the next regular goal (up to 1 / rate_hz)
                    self.send_goal(OPEN_RAD if self.gripper_open else CLOSED_RAD)
            target = self.arm_target(t_s) if self.arm else None
            # None: the hand is lost for longer than hold_s (or the arm is paused), so no goals; the controller
            # keeps the last one. 0.9: frames arrive with jitter; without the margin a frame 1 ms early would wait.
            if t_s - last_goal_s >= 0.9 * self.period_s and (smoothed is not None or target is not None):
                if smoothed is not None:
                    self.send_goal(OPEN_RAD if self.gripper_open else CLOSED_RAD)
                if target is not None:
                    self.send_arm(target, self.move_time)
                last_goal_s = t_s
            if self.preview and not self.show(frame, ap, smoothed):
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.frame_period_s:
                time.sleep(max(0.0, self.frame_period_s - (time.monotonic() - t_s)))

    def send_goal(self, joint_rad: float) -> None:
        point = JointTrajectoryPoint(positions=[joint_rad], time_from_start=self.move_time)
        self.publisher.publish(JointTrajectory(joint_names=[GRIPPER_JOINT], points=[point]))

    # --- stage 2: the arm ---

    def arm_target(self, t_s: float) -> Point | None:
        """The follower's TCP target for the smoothed palm centre, or None (paused, hand lost)."""
        palm = None if self.hand is None else palm_centre(self.hand.image)
        u = self.palm_filters[0].update(t_s, None if palm is None else float(palm[0]))
        v = self.palm_filters[1].update(t_s, None if palm is None else float(palm[1]))
        self.palm_px = None if u is None or v is None else (u, v)
        return self.follower.update(self.palm_px, self.levels[self.level])

    def solve(self, point: Point) -> tuple[float, float, float, float] | None:
        """Joints 1–4 for `point` with the steepest jaws the arm reaches there (ADR 0006), or None."""
        found = inverse_steepest(point, self.pitch_range, current=self.current_arm() or (0.0, 0.0, 0.0, 0.0))
        return None if found is None else found[0]

    def send_arm(self, target: Point, move_time) -> None:
        q = self.solve(target)
        if q is None:  # the follower only passes reachable targets; the home point is reachable
            return
        self.arm_command = q
        point = JointTrajectoryPoint(positions=[*q, 0.0], time_from_start=move_time)
        self.arm_publisher.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def current_arm(self) -> tuple[float, float, float, float] | None:
        """Where the arm is going (the last command), or where it is (/joint_states), or None before both."""
        if self.arm_command is not None:
            return self.arm_command
        if all(j in self.joints for j in ("1", "2", "3", "4")):
            return tuple(self.joints[j] for j in ("1", "2", "3", "4"))
        return None

    def go_home(self) -> None:
        self.follower.pause()
        self.level = "hover"
        self.send_arm(self.home, Duration(seconds=HOME_TIME_S).to_msg())
        self.get_logger().info(
            f"arm to the home pose (TCP {self.home.forward:.2f} m ahead, {self.home.height:.2f} m up), paused"
        )

    def resume(self) -> str | None:
        """Follow the hand from where the gripper is; returns why it could not, or None."""
        q = self.current_arm()
        if q is None:
            return "no joint states yet"
        return self.follower.resume(self.palm_px, forward3d(*q)[0])

    def on_joint_states(self, msg: JointState) -> None:
        self.joints.update(zip(msg.name, msg.position))
        if not self.home_sent:  # the first message: the controllers are up, the publisher is matched
            self.home_sent = True
            self.go_home()

    def on_clutch(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            problem = self.resume()
            response.success, response.message = problem is None, problem or "following"
        else:
            self.follower.pause()
            response.success, response.message = True, "paused"
        return response

    def on_grasp(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.level = "grasp" if request.data else "hover"
        response.success, response.message = True, f"{self.level}: TCP {self.levels[self.level]:.2f} m up"
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

    def draw_workspace(self, frame) -> None:
        """The workspace as the hand sees it in the raw frame (mirrored with the frame later): the sector of the
        bounds, the ring where the gripper is (or is going), the dot where the palm is."""
        bounds, table = self.follower.bounds, self.follower.table
        pans = np.linspace(bounds.pan[0], bounds.pan[1], 40)
        arcs = [(bounds.reach[1], a) for a in pans] + [(bounds.reach[0], a) for a in pans[::-1]]  # outer, then inner
        outline = [table.to_pixels(*(PAN_AXIS + reach * np.array([math.cos(a), math.sin(a)]))) for reach, a in arcs]
        cv2.polylines(frame, [np.array(outline, dtype=np.int32)], isClosed=True, color=GREY, thickness=2)
        q = self.current_arm()
        if q is not None:
            tcp = forward3d(*q)[0]
            u, v = table.to_pixels(tcp.forward, tcp.right)
            cv2.circle(frame, (int(u), int(v)), 12, WHITE, 2)
        if self.palm_px is not None:
            inside = self.follower.inside(self.palm_px, self.levels[self.level])
            colour = RED if not inside else GREEN if self.follower.following else GREY
            cv2.circle(frame, (int(self.palm_px[0]), int(self.palm_px[1])), 8, colour, -1)

    def show(self, frame, ap, smoothed) -> bool:
        """Draw the measured points, show the mirrored preview and handle the keys; False after q."""
        if self.arm:
            self.draw_workspace(frame)
        if self.hand is not None:
            lm = self.hand.image
            p = {i: (int(lm[i][0]), int(lm[i][1])) for i in (WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP)}
            cv2.line(frame, p[WRIST], p[MIDDLE_MCP], WHITE, 2)  # palm line: the aperture's unit
            cv2.line(frame, p[THUMB_TIP], p[INDEX_TIP], RED, 3)  # the measured gap
        view = cv2.flip(frame, 1)  # mirrored like a selfie; drawn before the flip, so the lines stay on the hand
        if smoothed is None:
            gripper = "hand lost: the gripper stays"
        else:
            gripper = f"gripper {'OPEN' if self.gripper_open else 'CLOSED'} by {self.gripper_by}"
            if self.gripper_by == "aperture":  # the threshold the smoothed aperture must reach to change it
                change, at = ("closes", self.calibration.closed) if self.gripper_open else ("opens", self.calibration.open)
                gripper += f" ({change} at {at:.2f})"
        hand = self.hand
        lines = [
            f"aperture {ap:.2f}, gesture {hand.gesture} {hand.score:.2f}" if hand is not None else "no hand",
            gripper,
            f"calibration (c / o): closed {self.calibration.closed:.2f}, open {self.calibration.open:.2f}",
        ]
        keys = "c = closed hand, o = open hand (hold 1 s first), q = quit"
        if self.arm:  # lines longer than about 58 characters are cut at 640 px
            state = "FOLLOWING" if self.follower.following else "paused"
            lines.append(f"arm {state}, height {self.level.upper()} {100 * self.levels[self.level]:.0f} cm (z)")
            q = self.current_arm()
            if q is not None:
                tcp, pitch = forward3d(*q)
                side = "right" if tcp.right >= 0 else "left"
                lines.append(  # OpenCV's fonts have no degree sign
                    f"TCP {100 * tcp.forward:.0f} cm ahead, {100 * abs(tcp.right):.0f} cm {side}, "
                    f"jaws {math.degrees(pitch):.0f} deg"
                )
            keys = "space = follow/pause, z = height, h = home, q = quit"
        for i, text in enumerate(lines):
            cv2.putText(view, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)
        cv2.putText(view, keys, (10, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("c"), ord("o")):
            self.calibrate("closed" if key == ord("c") else "open")
        elif self.arm and key == ord(" "):
            if self.follower.following:
                self.follower.pause()
            else:
                problem = self.resume()
                if problem:
                    self.get_logger().warning(f"not following: {problem}")
        elif self.arm and key == ord("z"):
            self.level = "hover" if self.level == "grasp" else "grasp"
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
