"""Smoothing of the hand's measures (the aperture, the palm's centre), and holding them while the hand is briefly
lost."""

import math


class HoldingEma:
    """Exponential moving average (EMA) that holds its value while measurements are missing.

    Each measurement moves the output the fraction `alpha` of the way towards it: frame-to-frame noise is smoothed
    out at the cost of a little lag (alpha = 1: no smoothing). Without a measurement the last output is held for
    `hold_s` seconds, then `update` returns None and the caller stops sending goals (the controller keeps the last
    one). When the hand is back, the output continues from the held value, so the gripper does not jump.
    """

    def __init__(self, alpha: float, hold_s: float):
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        if hold_s < 0.0:
            raise ValueError(f"hold_s must not be negative, got {hold_s}")
        self.alpha = alpha
        self.hold_s = hold_s
        self.value: float | None = None
        self._last_seen_s: float | None = None

    def update(self, t_s: float, measurement: float | None) -> float | None:
        """Output for a frame at time `t_s` (seconds, any monotonic clock); `measurement` None = no hand."""
        if measurement is None:
            if self._last_seen_s is None or t_s - self._last_seen_s > self.hold_s:
                return None
            return self.value
        if math.isnan(measurement):
            raise ValueError("measurement is NaN")
        if self.value is None:
            self.value = measurement
        else:
            self.value += self.alpha * (measurement - self.value)
        self._last_seen_s = t_s
        return self.value
