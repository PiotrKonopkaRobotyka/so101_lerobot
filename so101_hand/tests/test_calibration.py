import pytest

from so101_hand.calibration import MIN_SPAN, Calibration

# Defaults of the node, inside the apertures measured in the ROB-19 spike (side view: closed 0.13, open 1.7-2.2)
DEFAULT = Calibration(closed=0.2, open=1.6)


def test_closed_pose_gives_zero_and_open_pose_gives_one():
    assert DEFAULT.openness(0.2) == 0.0
    assert DEFAULT.openness(1.6) == 1.0


def test_halfway_aperture_gives_half():
    assert DEFAULT.openness(0.9) == pytest.approx(0.5)


@pytest.mark.parametrize("aperture, expected", [(0.0, 0.0), (0.1, 0.0), (1.7, 1.0), (5.0, 1.0)])
def test_apertures_beyond_the_calibrated_poses_are_clamped(aperture, expected):
    # a hand opened wider than at calibration must not ask for more than "fully open"
    assert DEFAULT.openness(aperture) == expected


def test_closed_pose_from_samples_uses_the_upper_quartile():
    # 0.40: a frame caught while the hand was still closing; three quarters of the samples are at or below 0.16
    cal = DEFAULT.with_closed([0.10, 0.12, 0.14, 0.16, 0.40])
    assert cal.closed == pytest.approx(0.16)


def test_open_pose_from_samples_uses_the_lower_quartile():
    cal = DEFAULT.with_open([1.2, 1.9, 2.0, 2.1, 2.2])
    assert cal.open == pytest.approx(1.9)


def test_noise_around_a_calibrated_pose_keeps_the_gripper_at_the_end():
    samples = [0.11, 0.13, 0.12, 0.14, 0.12, 0.13, 0.11, 0.13]  # closed hand, frame-to-frame noise
    cal = DEFAULT.with_closed(samples)
    at_zero = [cal.openness(a) == 0.0 for a in samples]
    assert sum(at_zero) >= 0.75 * len(samples)


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
