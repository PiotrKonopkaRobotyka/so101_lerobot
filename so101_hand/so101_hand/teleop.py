"""Relative control of the arm with a clutch (ADR 0003, stage 2): the hand moves the gripper in the arm's plane.

Like lifting a mouse: engaging latches where the hand is in the image and where the gripper is; from then on
target = gripper at engage + gain × hand displacement, converted from pixels to metres. Releasing stops the
arm, so the hand can move to a comfortable place before engaging again. With the hand's pitch (the palm line,
measures.hand_pitch) the jaws turn too: pitch at engage + pitch_gain × the hand's turn since engage (ROB-31).
"""

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from so101_hand.kinematics import Pose


@dataclass(frozen=True)
class Bounds:
    """Box in the arm's plane (metres) that targets never leave: above the floor and within the arm's reach;
    and the range of the jaws' pitch (radians): from straight down to 30° up."""

    reach: tuple[float, float] = (0.10, 0.40)
    height: tuple[float, float] = (0.02, 0.40)
    pitch: tuple[float, float] = (-math.pi / 2, math.pi / 6)


class Clutch:
    def __init__(
        self,
        gain: float = 1.0,
        bounds: Bounds = Bounds(),
        max_step_m: float = 0.02,
        reachable: Callable[[Pose], bool] = lambda pose: True,
        pitch_gain: float = 1.0,
        max_step_rad: float = 0.05,
    ):
        self.gain = gain
        self.bounds = bounds
        self.max_step_m = max_step_m  # per update: a lost and found hand, or a tracking glitch, cannot jerk the arm
        self.reachable = reachable  # e.g. "the IK has a solution"
        self.pitch_gain = pitch_gain  # 0: the jaws keep their pitch whatever the hand does
        self.max_step_rad = max_step_rad  # largest pitch change per update
        self._hand = None  # hand position at engage, pixels; None = released
        self._hand_pitch = None  # hand pitch at engage, radians; None = not measured, the jaws keep their pitch
        self._metres_per_px = 0.0
        self._start = None  # TCP at engage
        self._target = None  # the last target

    @property
    def engaged(self) -> bool:
        return self._hand is not None

    def engage(self, hand_px, metres_per_px: float, tcp: Pose, hand_pitch: float | None = None) -> None:
        """Latch the hand position in the raw image (pixels), the image scale at the hand, the current TCP and,
        if measured, the hand's pitch (radians)."""
        self._hand = np.asarray(hand_px, dtype=float)
        self._hand_pitch = hand_pitch
        self._metres_per_px = metres_per_px
        self._start = self._target = tcp

    def release(self) -> None:
        self._hand = None

    def update(self, hand_px, hand_pitch: float | None = None) -> Pose | None:
        """The next TCP target, or None while released or without a hand (the arm then stays where it is).

        A pose out of reach is tried again with the last pitch, so the gripper still moves, then at the last
        position, so the jaws still turn; when neither is reachable, the target stays.
        """
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
        reach, height = float(current[0] + step[0]), float(current[1] + step[1])
        pitch, last = self._next_pitch(hand_pitch), self._target
        for candidate in (Pose(reach, height, pitch), Pose(reach, height, last.pitch), last._replace(pitch=pitch)):
            if self.reachable(candidate):
                self._target = candidate
                break
        return self._target

    def _next_pitch(self, hand_pitch: float | None) -> float:
        """The jaws' pitch for this update: towards pitch at engage + pitch_gain × the hand's turn since engage,
        within the bounds and at most max_step_rad from the last one. Without a hand pitch: the last one."""
        last = self._target.pitch
        if hand_pitch is None or self._hand_pitch is None:
            return last
        turn = math.remainder(hand_pitch - self._hand_pitch, math.tau)  # the short way: +179° to -179° is 2°
        desired = min(max(self._start.pitch + self.pitch_gain * turn, self.bounds.pitch[0]), self.bounds.pitch[1])
        return last + min(max(desired - last, -self.max_step_rad), self.max_step_rad)
