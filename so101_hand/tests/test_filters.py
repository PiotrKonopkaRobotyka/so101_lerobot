import math

import pytest

from so101_hand.filters import HoldingEma

# Time is passed in by the caller (seconds), so the tests need no sleeping: frames come every 0.05 s (20 fps).


def test_first_measurement_passes_through():
    # no ramp from zero at start: the gripper starts where the hand is
    assert HoldingEma(alpha=0.4, hold_s=0.5).update(0.0, 0.8) == 0.8


def test_each_frame_moves_a_fraction_alpha_towards_the_measurement():
    ema = HoldingEma(alpha=0.5, hold_s=0.5)
    ema.update(0.0, 0.0)
    assert [ema.update(t, 1.0) for t in (0.05, 0.10, 0.15)] == pytest.approx([0.5, 0.75, 0.875])


def test_alpha_one_means_no_smoothing():
    ema = HoldingEma(alpha=1.0, hold_s=0.5)
    ema.update(0.0, 0.2)
    assert ema.update(0.05, 0.9) == pytest.approx(0.9)  # 0.2 + 1.0 * (0.9 - 0.2) = 0.8999999999999999 in floats


def test_frame_to_frame_noise_is_reduced():
    ema = HoldingEma(alpha=0.4, hold_s=0.5)
    noisy = [0.45, 0.55] * 20  # a still hand: ±0.05 around 0.5
    out = [ema.update(0.05 * i, x) for i, x in enumerate(noisy)]
    assert max(out[10:]) - min(out[10:]) < 0.5 * (max(noisy) - min(noisy))


def test_short_loss_holds_the_last_value():
    ema = HoldingEma(alpha=0.4, hold_s=0.5)
    ema.update(0.0, 0.7)
    assert ema.update(0.3, None) == 0.7
    assert ema.update(0.5, None) == 0.7  # exactly hold_s: still held


def test_long_loss_stops_the_output():
    ema = HoldingEma(alpha=0.4, hold_s=0.5)
    ema.update(0.0, 0.7)
    assert ema.update(0.6, None) is None


def test_after_a_long_loss_the_output_continues_from_the_held_value():
    # the gripper stayed at 0.7 while the hand was away: it moves on smoothly instead of jumping to the new value
    ema = HoldingEma(alpha=0.4, hold_s=0.5)
    ema.update(0.0, 0.7)
    ema.update(5.0, None)
    assert ema.update(5.05, 0.2) == pytest.approx(0.7 + 0.4 * (0.2 - 0.7))


def test_no_hand_since_start_gives_none():
    assert HoldingEma(alpha=0.4, hold_s=0.5).update(0.0, None) is None


def test_nan_is_rejected_and_does_not_poison_the_state():
    ema = HoldingEma(alpha=0.4, hold_s=0.5)
    ema.update(0.0, 0.7)
    with pytest.raises(ValueError):
        ema.update(0.05, math.nan)
    assert ema.update(0.10, 0.7) == pytest.approx(0.7)


@pytest.mark.parametrize("alpha", [0.0, -0.1, 1.5])
def test_alpha_outside_the_range_is_rejected(alpha):
    with pytest.raises(ValueError):
        HoldingEma(alpha=alpha, hold_s=0.5)


def test_negative_hold_time_is_rejected():
    with pytest.raises(ValueError):
        HoldingEma(alpha=0.4, hold_s=-1.0)
