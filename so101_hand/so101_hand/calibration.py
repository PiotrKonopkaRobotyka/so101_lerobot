"""Two-pose calibration: apertures (see measures.py) of the operator's closed and open hand. They are also the
thresholds of the gripper's two states (ADR 0008): at the closed pose or below it closes, at the open pose or above
it opens."""

import math
from dataclasses import dataclass, replace

import numpy as np

# Smallest accepted gap between the open and the closed aperture, in palm lengths. The ROB-19 spike measured about 2
# in the side view and 1 in the frontal view; a smaller gap means both poses were shown with the same hand shape.
MIN_SPAN = 0.3


@dataclass(frozen=True)
class Calibration:
    """Apertures of the closed and the open hand. Frozen: a rejected recalibration cannot leave half a change."""

    closed: float
    open: float

    def __post_init__(self):
        if not self.open - self.closed >= MIN_SPAN:  # "not >=" also rejects NaN: comparisons with NaN are False
            raise ValueError(
                f"open ({self.open:.2f}) must exceed closed ({self.closed:.2f}) by at least {MIN_SPAN}: calibrate again"
            )

    def is_open(self, aperture: float, was_open: bool) -> bool:
        """The gripper's state for a smoothed aperture: two states with hysteresis (ADR 0008).

        Closed at the closed pose or below, open at the open pose or above, unchanged in between: a hand between the
        poses, a trembling one, or a fist turned with the thumb off the index finger (0.45-0.54 in the webcam test of
        2026-11-13) does not move the jaws.
        """
        if math.isnan(aperture):
            raise ValueError("aperture is NaN")
        if aperture <= self.closed:
            return False
        if aperture >= self.open:
            return True
        return was_open

    def with_closed(self, samples) -> "Calibration":
        """A new calibration with the closed pose taken from apertures of a closed hand.

        The upper quartile, not the median: three quarters of the closed-hand frames are at or below it, so the
        smoothed aperture of a closed hand reaches the threshold and closes the gripper.
        """
        return replace(self, closed=_percentile(samples, 75))

    def with_open(self, samples) -> "Calibration":
        """A new calibration with the open pose taken from apertures of an open hand (lower quartile, as above)."""
        return replace(self, open=_percentile(samples, 25))


def _percentile(samples, q: float) -> float:
    values = np.asarray(samples, dtype=float)
    if values.size == 0:
        raise ValueError("no aperture samples: was the hand in view?")
    return float(np.percentile(values, q))
