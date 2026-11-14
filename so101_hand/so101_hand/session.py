"""Hand teleoperation frame by frame, without ROS or a simulator (ADR 0006–0008).

Per camera frame: the hand → the gripper's state (a gesture, else the smoothed aperture with hysteresis) and, with
the arm, the palm → the follower → the TCP target over the table and its joints (the steepest jaws the arm reaches).
The preview's keys act on it. The Gazebo node (so101_hand_ros) and the MuJoCo app (so101_mujoco) are thin layers
around it: they read the frames, send the targets to their arm and draw the preview with so101_hand.preview.
"""

from collections import deque
from dataclasses import dataclass

from so101_hand.calibration import Calibration
from so101_hand.filters import HoldingEma
from so101_hand.gripper import CLOSED_RAD, OPEN_RAD, next_state
from so101_hand.kinematics import PAN_AXIS, PLANE_OFFSET, Point, forward3d, inverse_steepest
from so101_hand.measures import aperture, palm_centre
from so101_hand.teleop import Bounds, Follower, TableMap

CALIBRATION_WINDOW_S = 1.0  # c / o take the apertures of the last second
HOME_REACH = 0.20  # the home pose: the TCP this far straight ahead of the pan axis, at the hover height, jaws down
ARM_KEYS = "space = height, p = follow/pause, h = home, q = quit"
GRIPPER_KEYS = "c = closed hand, o = open hand (hold 1 s first), q = quit"


@dataclass(frozen=True)
class Settings:
    """The teleoperation's parameters, explained in so101_hand_ros/config/hand_teleop.yaml; the defaults are its
    values (a test keeps them equal). Heights are metres up in so101_hand's table frame; `floor_height` is where the
    simulator's floor is in it (0 in Gazebo, 0.0325 in MuJoCo, ADR 0009), for the heights shown in the preview."""

    aperture_closed: float = 0.36
    aperture_open: float = 0.70
    ema_alpha: float = 0.4
    hold_s: float = 0.5
    px_per_m: float = 800.0
    axis_px: tuple[float, float] = (320.0, 440.0)
    bounds_reach: tuple[float, float] = (0.12, 0.34)
    bounds_pan: tuple[float, float] = (-1.0472, 1.0472)
    bounds_height: tuple[float, float] = (0.02, 0.40)
    hover_height: float = 0.08
    grasp_height: float = 0.02
    pitch_range: tuple[float, float] = (-1.5708, -0.7854)
    max_step_m: float = 0.02
    max_step_height_m: float = 0.005
    floor_height: float = 0.0


class Session:
    """The teleoperation's state between frames; with `arm` False only the gripper follows the hand (stage 1)."""

    def __init__(self, settings: Settings = Settings(), arm: bool = True):
        s = self.settings = settings
        self.arm = arm
        self.calibration = Calibration(closed=s.aperture_closed, open=s.aperture_open)
        self.aperture_filter = HoldingEma(alpha=s.ema_alpha, hold_s=s.hold_s)
        self.palm_filters = (HoldingEma(alpha=s.ema_alpha, hold_s=s.hold_s), HoldingEma(alpha=s.ema_alpha, hold_s=s.hold_s))
        self.levels = {"hover": s.hover_height, "grasp": s.grasp_height}  # space toggles between them
        self.level = "hover"
        self.follower = Follower(
            table=TableMap(px_per_m=s.px_per_m, axis_px=tuple(s.axis_px)),
            bounds=Bounds(reach=tuple(s.bounds_reach), pan=tuple(s.bounds_pan), height=tuple(s.bounds_height)),
            max_step_m=s.max_step_m,
            max_step_height_m=s.max_step_height_m,
            reachable=lambda point: self.solve(point) is not None,
        )
        self.home = Point(float(PAN_AXIS[0] + HOME_REACH), float(PAN_AXIS[1] + PLANE_OFFSET), s.hover_height)
        self.gripper_open = True  # the gripper starts open
        self.gripper_by = "start"  # what decided its state: a gesture or "aperture"
        self.hand = None  # the hand of the last frame (so101_hand.tracker.Hand) or None
        self.aperture: float | None = None  # its aperture
        self.smoothed: float | None = None  # the aperture's EMA; None when the hand has been lost for over hold_s
        self.palm_px: tuple[float, float] | None = None  # the smoothed palm centre in the raw frame
        self.target: Point | None = None  # the follower's TCP target of the last frame; None: paused or no hand
        self.arm_command: tuple[float, float, float, float] | None = None  # joints 1–4 last commanded
        self.joints: tuple[float, float, float, float] | None = None  # where the arm is (set by the layer)
        self.recent = deque(maxlen=60)  # (time, aperture) of the last frames with a hand, for c / o

    @property
    def gripper_rad(self) -> float:
        """The gripper's goal for joint "6" (so101_hand.gripper)."""
        return OPEN_RAD if self.gripper_open else CLOSED_RAD

    def update(self, hand, t_s: float) -> bool:
        """One frame: `hand` from HandTracker.detect (None without a hand) at `t_s` seconds of a monotonic clock.

        Returns True when the gripper's state changed, so that the layer sends its goal at once.
        """
        self.hand = hand
        self.aperture = None if hand is None else aperture(hand.image)
        if self.aperture is not None:
            self.recent.append((t_s, self.aperture))
        self.smoothed = self.aperture_filter.update(t_s, self.aperture)
        changed = False
        if hand is not None:  # without a hand the state stays
            was_open = self.gripper_open
            self.gripper_open, self.gripper_by = next_state(
                was_open, hand.gesture, hand.score, self.smoothed, self.calibration
            )
            changed = self.gripper_open != was_open
        if self.arm:
            palm = None if hand is None else palm_centre(hand.image)
            u = self.palm_filters[0].update(t_s, None if palm is None else float(palm[0]))
            v = self.palm_filters[1].update(t_s, None if palm is None else float(palm[1]))
            self.palm_px = None if u is None or v is None else (u, v)
            self.target = self.follower.update(self.palm_px, self.levels[self.level])
            if self.target is not None:
                q = self.solve(self.target)
                if q is not None:  # the follower passes reachable targets only
                    self.arm_command = q
        return changed

    def current_arm(self) -> tuple[float, float, float, float] | None:
        """Where the arm is going (the last command), or where it is (`joints`), or None before both."""
        return self.arm_command if self.arm_command is not None else self.joints

    def solve(self, point: Point) -> tuple[float, float, float, float] | None:
        """Joints 1–4 for `point` with the steepest jaws the arm reaches there (ADR 0006), or None."""
        current = self.current_arm() or (0.0, 0.0, 0.0, 0.0)
        found = inverse_steepest(point, tuple(self.settings.pitch_range), current=current)
        return None if found is None else found[0]

    def resume(self) -> str | None:
        """Follow the hand from where the gripper is; returns why it cannot, or None."""
        q = self.current_arm()
        if q is None:
            return "no joint states yet"
        return self.follower.resume(self.palm_px, forward3d(*q)[0])

    def pause(self) -> None:
        self.follower.pause()

    def toggle_height(self) -> None:
        self.level = "hover" if self.level == "grasp" else "grasp"

    def go_home(self) -> tuple[float, float, float, float] | None:
        """Pause, back to the hover height, command the home pose; returns its joints for the layer to send."""
        self.follower.pause()
        self.level = "hover"
        self.arm_command = self.solve(self.home)
        return self.arm_command

    def calibrate(self, pose: str, now_s: float) -> str:
        """c / o: the closed or open pose from the apertures of the last second; returns a message for the log."""
        samples = [ap for t_s, ap in self.recent if now_s - t_s <= CALIBRATION_WINDOW_S]
        try:
            if pose == "closed":
                self.calibration = self.calibration.with_closed(samples)
            else:
                self.calibration = self.calibration.with_open(samples)
        except ValueError as error:
            return f"{pose} hand not calibrated: {error}"
        return (
            f"{pose} hand calibrated from {len(samples)} frames: aperture_closed {self.calibration.closed:.2f}, "
            f"aperture_open {self.calibration.open:.2f} (to keep them, put them in config/hand_teleop.yaml)"
        )

    def handle_key(self, key: int, now_s: float) -> tuple[str | None, str | None]:
        """A key of the preview (cv2.waitKey's code, 255 or -1 for none).

        Returns (an action for the layer: "quit", "home" or None; a message for the log or None). space toggles the
        height, p follows or pauses, c / o calibrate the closed / open hand; the layer sends the home pose itself.
        """
        if key == ord("q"):
            return "quit", None
        if key in (ord("c"), ord("o")):
            return None, self.calibrate("closed" if key == ord("c") else "open", now_s)
        if not self.arm:
            return None, None
        if key == ord(" "):
            self.toggle_height()
        elif key == ord("p"):
            if self.follower.following:
                self.pause()
            else:
                problem = self.resume()
                if problem:
                    return None, f"not following: {problem}"
        elif key == ord("h"):
            return "home", None
        return None, None
