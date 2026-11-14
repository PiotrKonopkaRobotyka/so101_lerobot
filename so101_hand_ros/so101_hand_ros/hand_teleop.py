"""ROS 2 node: the operator's hand seen by the webcam → goals for the simulated SO-101 (ADR 0006–0008).

Stage 1: the hand opens and closes the gripper (two states, ADR 0008). Stage 2 (parameter `arm`): the hand over the
workspace seen from above drives the gripper over the table: where the palm is in the preview, inside the drawn
sector, is where the gripper goes, the base turning with it; the height is a level toggled with space (hover or
grasp) and the jaws point down. The logic and the preview live in so101_hand (plain Python, tested with pytest;
so101_hand.session, shared with the MuJoCo app). This node reads frames with OpenCV (not cv_bridge: it is built for
numpy 1.x), shows the preview and publishes the goals.

Keys in the preview window: c / o = calibrate the closed / open hand (hold the pose for about a second first),
space = hover or grasp height, p = follow the hand or pause, h = arm back to the home pose, q = quit. Without the
keyboard: the services ~/clutch (std_srvs/SetBool: true follows, false pauses) and ~/grasp (true = grasp height,
false = hover).
"""

import time

import cv2
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_hand import preview
from so101_hand.kinematics import TCP_IN_GRIPPER
from so101_hand.session import Session, Settings
from so101_hand.tracker import DEFAULT_MODEL, HandTracker

WINDOW = "so101_hand"
GRIPPER_JOINT = "6"
ARM_JOINTS = ["1", "2", "3", "4", "5"]  # 5 (wrist roll) stays at 0
HOME_TIME_S = 2.0


class HandTeleopNode(Node):
    """Frame → HandTracker → Session (gripper state; with `arm`, the TCP target and its joints) → goals."""

    def __init__(self):
        super().__init__("hand_teleop")
        model_path = self.declare_parameter("model_path", "").value or str(DEFAULT_MODEL)
        camera_index = self.declare_parameter("camera_index", 0).value
        video_path = self.declare_parameter("video_path", "").value
        self.preview = self.declare_parameter("preview", True).value
        self.period_s = 1.0 / self.declare_parameter("rate_hz", 10.0).value
        self.move_time = Duration(seconds=self.declare_parameter("move_time_s", 0.1).value).to_msg()
        self.arm = self.declare_parameter("arm", False).value
        defaults = Settings()

        def parameter(name: str):
            default = getattr(defaults, name)
            value = self.declare_parameter(name, list(default) if isinstance(default, tuple) else default).value
            return tuple(value) if isinstance(default, tuple) else value

        settings = Settings(**{name: parameter(name) for name in Settings.__dataclass_fields__ if name != "floor_height"})
        self.session = Session(settings, arm=self.arm)

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
        if self.arm:
            self.arm_publisher = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
            self.joints = {}  # the latest /joint_states
            self.home_sent = False
            self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
            self.create_service(SetBool, "~/clutch", self.on_clutch)
            self.create_service(SetBool, "~/grasp", self.on_grasp)
            self.publish_tcp_frame()
        calibration = self.session.calibration
        self.get_logger().info(
            f"reading {self.source}, goals at {1.0 / self.period_s:.0f} Hz; calibration: closed "
            f"{calibration.closed:.2f}, open {calibration.open:.2f}; arm {'on' if self.arm else 'off'}"
        )

    def run(self) -> None:
        last_goal_s = float("-inf")
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().info(f"no more frames from {self.source}")
                break
            t_s = time.monotonic()
            session = self.session
            if session.update(self.tracker.detect(frame, t_s), t_s):
                self.send_gripper()  # at once, not with the next regular goal (up to 1 / rate_hz)
            # without the hand for longer than hold_s (or with the arm paused) no goals: the controllers keep the
            # last ones. 0.9: frames arrive with jitter; without the margin a frame 1 ms early would wait.
            if t_s - last_goal_s >= 0.9 * self.period_s and (session.smoothed is not None or session.target is not None):
                if session.smoothed is not None:
                    self.send_gripper()
                if session.target is not None and session.arm_command is not None:
                    self.send_arm(session.arm_command, self.move_time)
                last_goal_s = t_s
            if self.preview and not self.show(frame):
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.frame_period_s:
                time.sleep(max(0.0, self.frame_period_s - (time.monotonic() - t_s)))

    def send_gripper(self) -> None:
        point = JointTrajectoryPoint(positions=[self.session.gripper_rad], time_from_start=self.move_time)
        self.publisher.publish(JointTrajectory(joint_names=[GRIPPER_JOINT], points=[point]))

    def send_arm(self, q, move_time) -> None:
        point = JointTrajectoryPoint(positions=[*q, 0.0], time_from_start=move_time)
        self.arm_publisher.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def go_home(self) -> None:
        q = self.session.go_home()
        if q is not None:  # the home point is reachable
            self.send_arm(q, Duration(seconds=HOME_TIME_S).to_msg())
        home = self.session.home
        self.get_logger().info(
            f"arm to the home pose (TCP {home.forward:.2f} m ahead, {home.height:.2f} m up), paused"
        )

    def on_joint_states(self, msg: JointState) -> None:
        self.joints.update(zip(msg.name, msg.position))
        if all(j in self.joints for j in ("1", "2", "3", "4")):
            self.session.joints = tuple(self.joints[j] for j in ("1", "2", "3", "4"))
        if not self.home_sent:  # the first message: the controllers are up, the publisher is matched
            self.home_sent = True
            self.go_home()

    def on_clutch(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            problem = self.session.resume()
            response.success, response.message = problem is None, problem or "following"
        else:
            self.session.pause()
            response.success, response.message = True, "paused"
        return response

    def on_grasp(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        session = self.session
        session.level = "grasp" if request.data else "hover"
        response.success, response.message = True, f"{session.level}: TCP {session.levels[session.level]:.2f} m up"
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

    def show(self, frame) -> bool:
        """Show the preview and handle its keys; False after q."""
        cv2.imshow(WINDOW, preview.draw(self.session, frame))
        action, message = self.session.handle_key(cv2.waitKey(1) & 0xFF, time.monotonic())
        if message:
            self.get_logger().info(message)
        if action == "home":
            self.go_home()
        return action != "quit"

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
