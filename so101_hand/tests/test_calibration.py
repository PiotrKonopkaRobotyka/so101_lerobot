import math

import pytest

from so101_hand.calibration import MIN_SPAN, Calibration

# Poses inside the apertures measured in the ROB-19 spike (side view: closed 0.13, open 1.7-2.2)
DEFAULT = Calibration(closed=0.2, open=1.6)
# My hand from the front, as in config/hand_teleop.yaml (the webcam tests of 2026-11-12 and 2026-11-13)
FRONT = Calibration(closed=0.36, open=0.70)


@pytest.mark.parametrize("was_open", [True, False])
@pytest.mark.parametrize("aperture", [0.0, 0.1, 0.2])
def test_at_or_below_the_closed_pose_the_gripper_closes(aperture, was_open):
    assert DEFAULT.is_open(aperture, was_open) is False


@pytest.mark.parametrize("was_open", [True, False])
@pytest.mark.parametrize("aperture", [1.6, 1.7, 5.0])
def test_at_or_above_the_open_pose_the_gripper_opens(aperture, was_open):
    assert DEFAULT.is_open(aperture, was_open) is True


@pytest.mark.parametrize("was_open", [True, False])
@pytest.mark.parametrize("aperture", [0.21, 0.9, 1.59])
def test_between_the_poses_the_gripper_keeps_its_state(aperture, was_open):
    assert DEFAULT.is_open(aperture, was_open) is was_open


def test_nan_is_rejected_instead_of_moving_the_gripper():
    with pytest.raises(ValueError):
        DEFAULT.is_open(math.nan, True)


def replay(calibration, apertures, is_open=True):
    states = []
    for aperture in apertures:
        is_open = calibration.is_open(aperture, is_open)
        states.append(is_open)
    return states


def test_a_fist_turned_thumb_up_keeps_the_gripper_closed():
    # The webcam test of 2026-11-13, one value a second: the open hand, the fist towards the camera, the fist turned
    # thumb up while moving right (0.45-0.54: it held the old proportional gripper 34-55 % open), the hand opened.
    apertures = [0.90, 0.76, 0.19, 0.14, 0.54, 0.51, 0.45, 0.48, 0.32, 1.28]
    assert replay(FRONT, apertures) == [True, True] + [False] * 7 + [True]


def test_a_trembling_open_hand_keeps_the_gripper_open():
    # the relaxed open hand of the 2026-11-12 test: 0.65-0.85, dipping to 0.42 for a moment
    assert all(replay(FRONT, [0.77, 0.65, 0.42, 0.58, 0.80]))


def test_closed_pose_from_samples_uses_the_upper_quartile():
    # 0.40: a frame caught while the hand was still closing; three quarters of the samples are at or below 0.16
    cal = DEFAULT.with_closed([0.10, 0.12, 0.14, 0.16, 0.40])
    assert cal.closed == pytest.approx(0.16)


def test_open_pose_from_samples_uses_the_lower_quartile():
    cal = DEFAULT.with_open([1.2, 1.9, 2.0, 2.1, 2.2])
    assert cal.open == pytest.approx(1.9)


def test_a_closed_hand_calibrated_from_noisy_samples_closes_the_gripper():
    samples = [0.11, 0.13, 0.12, 0.14, 0.12, 0.13, 0.11, 0.13]  # closed hand, frame-to-frame noise
    cal = DEFAULT.with_closed(samples)
    closing = [not cal.is_open(a, was_open=True) for a in samples]
    assert sum(closing) >= 0.75 * len(samples)


def test_recalibrating_one_pose_keeps_the_other():
    assert DEFAULT.with_open([2.0, 2.0, 2.0]).closed == DEFAULT.closed


@pytest.mark.parametrize("closed, open_", [(1.0, 1.0), (1.0, 1.0 + MIN_SPAN / 2), (1.6, 0.2)])
def test_poses_too_close_or_swapped_are_rejected(closed, open_):
    with pytest.raises(ValueError):
        Calibration(closed=closed, open=open_)


def test_rejected_recalibration_leaves_the_old_one_untouched():
    with pytest.raises(ValueError):
        DEFAULT.with_closed([1.6, 1.6, 1.6])  # "closed" shown with an open hand
    assert DEFAULT == Calibration(closed=0.2, open=1.6)


def test_calibrating_without_samples_is_rejected():
    with pytest.raises(ValueError):
        DEFAULT.with_closed([])
