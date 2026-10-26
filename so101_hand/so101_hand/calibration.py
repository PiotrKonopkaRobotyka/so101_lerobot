"""Two-pose calibration: apertures (see measures.py) of the operator's closed and open hand → openness 0…1."""

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

    def openness(self, aperture: float) -> float:
        """0 at the closed pose, 1 at the open pose, linear in between, clamped to 0…1."""
        return min(max((aperture - self.closed) / (self.open - self.closed), 0.0), 1.0)

    def with_closed(self, samples) -> "Calibration":
        """A new calibration with the closed pose taken from apertures of a closed hand.

        The upper quartile, not the median: three quarters of the closed-hand frames then map to exactly 0,
        so frame-to-frame noise does not make the closed gripper twitch.
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
