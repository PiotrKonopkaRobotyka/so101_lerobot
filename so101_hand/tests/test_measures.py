import numpy as np
import pytest

from so101_hand.measures import INDEX_TIP, MIDDLE_MCP, THUMB_TIP, WRIST, aperture


def make_hand(thumb_tip, index_tip, wrist=(0.0, 0.0), middle_mcp=(0.0, -100.0)):
    """21 image landmarks in pixels: everything at the wrist except the four points the measure uses."""
    hand = np.zeros((21, 2))
    hand[:] = wrist
    hand[WRIST] = wrist
    hand[MIDDLE_MCP] = middle_mcp
    hand[THUMB_TIP] = thumb_tip
    hand[INDEX_TIP] = index_tip
    return hand


def test_touching_fingertips_give_zero():
    hand = make_hand(thumb_tip=(30.0, -150.0), index_tip=(30.0, -150.0))
    assert aperture(hand) == 0.0


def test_gap_is_divided_by_palm_length():
    # palm (wrist to middle MCP) = 100 px, thumb-index gap = 50 px -> 0.5
    hand = make_hand(thumb_tip=(0.0, -150.0), index_tip=(50.0, -150.0))
    assert aperture(hand) == pytest.approx(0.5)


def test_same_hand_closer_to_the_camera_gives_the_same_value():
    hand = make_hand(thumb_tip=(10.0, -140.0), index_tip=(60.0, -170.0))
    assert aperture(hand * 3.0) == pytest.approx(aperture(hand))


def test_moving_the_hand_in_the_image_does_not_change_the_value():
    hand = make_hand(thumb_tip=(10.0, -140.0), index_tip=(60.0, -170.0))
    assert aperture(hand + np.array([200.0, 120.0])) == pytest.approx(aperture(hand))


def test_depth_column_is_ignored():
    hand = make_hand(thumb_tip=(0.0, -150.0), index_tip=(50.0, -150.0))
    with_z = np.column_stack([hand, np.random.default_rng(0).normal(size=21)])
    assert aperture(with_z) == pytest.approx(0.5)


@pytest.mark.parametrize("shape", [(20, 2), (21, 1), (21,)])
def test_wrong_shape_is_rejected(shape):
    with pytest.raises(ValueError):
        aperture(np.zeros(shape))


def test_zero_palm_length_is_rejected():
    hand = make_hand(thumb_tip=(0.0, -150.0), index_tip=(50.0, -150.0), middle_mcp=(0.0, 0.0))
    with pytest.raises(ValueError):
        aperture(hand)
