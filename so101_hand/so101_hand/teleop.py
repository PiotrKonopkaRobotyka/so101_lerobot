"""The hand over the workspace seen from above (ADR 0006, ADR 0007): absolute mapping with a pause.

The preview shows the robot's workspace from above, from behind the base: the pan axis near the bottom edge, forward
up the screen. Where the palm is in the raw frame is where the gripper goes over the table (TableMap: pixels per
metre and the pan axis' pixel, set once). `space` pauses: the arm holds; resuming needs a hand inside the workspace,
and the gripper then catches up at the step limit. The height is a level chosen with a key (hover or grasp), not
measured from the hand.
"""

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from so101_hand.kinematics import PAN_AXIS, Point, pan_angle


@dataclass(frozen=True)
class TableMap:
    """Raw frame pixels ↔ the table: `px_per_m` pixels per metre and `axis_px`, where the arm's pan axis is.

    The camera faces me, so my right is the image's left (u decreasing) and forward is up the image (v decreasing);
    the mirrored preview then shows the hand moving like the gripper in the view from above.
    """

    px_per_m: float = 800.0
    axis_px: tuple[float, float] = (320.0, 440.0)

    def to_table(self, hand_px) -> tuple[float, float]:
        """(forward, right) in metres of a point of the raw frame."""
        u, v = hand_px
        forward = PAN_AXIS[0] + (self.axis_px[1] - float(v)) / self.px_per_m
        right = PAN_AXIS[1] + (self.axis_px[0] - float(u)) / self.px_per_m
        return float(forward), float(right)

    def to_pixels(self, forward: float, right: float) -> tuple[float, float]:
        """(u, v) in the raw frame of a point over the table, e.g. to draw the workspace."""
        u = self.axis_px[0] - (right - PAN_AXIS[1]) * self.px_per_m
        v = self.axis_px[1] - (forward - PAN_AXIS[0]) * self.px_per_m
        return float(u), float(v)


@dataclass(frozen=True)
class Bounds:
    """Ring sector over the table that targets never leave: the reach (horizontal distance from the pan axis) and
    the pan (joint "1", positive to the robot's right), and the height range (metres, radians).

    The defaults are reachable with the jaws 45° down or steeper up to 0.10 m of height (checked by a test).
    """

    reach: tuple[float, float] = (0.12, 0.34)
    pan: tuple[float, float] = (-math.pi / 3, math.pi / 3)
    height: tuple[float, float] = (0.02, 0.40)

    def clip(self, point: Point) -> Point:
        reach = _clamp(math.hypot(point.forward - PAN_AXIS[0], point.right - PAN_AXIS[1]), *self.reach)
        pan = _clamp(pan_angle(point), *self.pan)
        forward, right = PAN_AXIS + reach * np.array([math.cos(pan), math.sin(pan)])
        return Point(float(forward), float(right), _clamp(point.height, *self.height))

    def contains(self, point: Point, tolerance: float = 1e-6) -> bool:
        return all(abs(a - b) <= tolerance for a, b in zip(self.clip(point), point))


class Follower:
    def __init__(
        self,
        table: TableMap = TableMap(),
        bounds: Bounds = Bounds(),
        max_step_m: float = 0.02,
        max_step_height_m: float = 0.005,
        reachable: Callable[[Point], bool] = lambda point: True,
    ):
        self.table = table
        self.bounds = bounds
        self.max_step_m = max_step_m  # over the table, per update: a lost and found hand cannot jerk the arm
        self.max_step_height_m = max_step_height_m  # up or down, per update: the gripper comes down gently
        self.reachable = reachable  # e.g. "the IK has a solution"
        self._target: Point | None = None  # the last target; None = paused

    @property
    def following(self) -> bool:
        return self._target is not None

    def desired(self, hand_px, height: float) -> Point:
        """Where the hand puts the gripper: its palm on the table at `height`, clipped to the bounds."""
        forward, right = self.table.to_table(hand_px)
        return self.bounds.clip(Point(forward, right, height))

    def inside(self, hand_px, height: float) -> bool:
        """Whether the hand maps inside the bounds (no clipping needed)."""
        forward, right = self.table.to_table(hand_px)
        return self.bounds.contains(Point(forward, right, height))

    def resume(self, hand_px, tcp: Point) -> str | None:
        """Follow the hand from the gripper's current point `tcp`; returns why it cannot, or None.

        The hand must be in view and inside the workspace, so that the gripper never sets off for a target outside.
        """
        if hand_px is None:
            return "no hand in view"
        if not self.inside(hand_px, tcp.height):
            return "hand outside the workspace"
        self._target = tcp
        return None

    def pause(self) -> None:
        self._target = None

    def update(self, hand_px, height: float) -> Point | None:
        """The next TCP target, or None while paused or without a hand (the arm then stays where it is).

        Towards the hand's point at `height`: at most `max_step_m` over the table and `max_step_height_m` up or
        down per update. A step the arm cannot reach is tried again without the height change, then without the
        move over the table; when neither is reachable, the target stays.
        """
        if not self.following or hand_px is None:
            return None
        last = self._target
        step = np.array(self.desired(hand_px, height)) - np.array(last)
        across = float(np.hypot(step[0], step[1]))
        if across > self.max_step_m:
            step[:2] *= self.max_step_m / across
        forward, right = float(last.forward + step[0]), float(last.right + step[1])
        new_height = float(last.height + _clamp(step[2], -self.max_step_height_m, self.max_step_height_m))
        for candidate in (
            Point(forward, right, new_height),
            Point(forward, right, last.height),
            Point(last.forward, last.right, new_height),
        ):
            if self.reachable(candidate):
                self._target = candidate
                break
        return self._target


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
