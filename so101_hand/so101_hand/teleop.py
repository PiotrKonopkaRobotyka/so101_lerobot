"""Relative control of the arm with a clutch (ADR 0003, stage 2): the hand moves the gripper in the arm's plane.

Like lifting a mouse: engaging latches where the hand is in the image and where the gripper is; from then on
target = gripper at engage + gain × hand displacement, converted from pixels to metres. Releasing stops the
arm, so the hand can move to a comfortable place before engaging again.
"""

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from so101_hand.kinematics import Pose


@dataclass(frozen=True)
class Bounds:
    """Box in the arm's plane (metres) that targets never leave: above the floor and within the arm's reach."""

    reach: tuple[float, float] = (0.10, 0.40)
    height: tuple[float, float] = (0.02, 0.40)


class Clutch:
    def __init__(
        self,
        gain: float = 1.0,
        bounds: Bounds = Bounds(),
        max_step_m: float = 0.02,
        reachable: Callable[[Pose], bool] = lambda pose: True,
    ):
        self.gain = gain
        self.bounds = bounds
        self.max_step_m = max_step_m  # per update: a lost and found hand, or a tracking glitch, cannot jerk the arm
        self.reachable = reachable  # e.g. "the IK has a solution"
        self._hand = None  # hand position at engage, pixels; None = released
        self._metres_per_px = 0.0
        self._start = None  # TCP at engage
        self._target = None  # the last target

    @property
    def engaged(self) -> bool:
        return self._hand is not None

    def engage(self, hand_px, metres_per_px: float, tcp: Pose) -> None:
        """Latch the hand position in the raw image (pixels), the image scale at the hand and the current TCP."""
        self._hand = np.asarray(hand_px, dtype=float)
        self._metres_per_px = metres_per_px
        self._start = self._target = tcp

    def release(self) -> None:
        self._hand = None

    def update(self, hand_px) -> Pose | None:
        """The next TCP target, or None while released or without a hand (the arm then stays where it is)."""
        if not self.engaged or hand_px is None:
            return None
        du, dv = (np.asarray(hand_px, dtype=float) - self._hand) * self._metres_per_px * self.gain
        # ADR 0003 axes on the raw frame: hand to my left = image right (+u) → forward; hand up (-v) → up
        desired = np.clip(
            [self._start.reach + du, self._start.height - dv],
            [self.bounds.reach[0], self.bounds.height[0]],
            [self.bounds.reach[1], self.bounds.height[1]],
        )
        current = np.array([self._target.reach, self._target.height])
        step = desired - current
        length = float(np.hypot(*step))
        if length > self.max_step_m:
            step *= self.max_step_m / length
        candidate = Pose(float(current[0] + step[0]), float(current[1] + step[1]), self._start.pitch)
        if self.reachable(candidate):
            self._target = candidate
        return self._target
