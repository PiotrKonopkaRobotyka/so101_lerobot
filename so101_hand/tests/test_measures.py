import math

import numpy as np
import pytest

from so101_hand.measures import INDEX_TIP, MIDDLE_MCP, THUMB_TIP, WRIST, aperture, hand_pitch, metres_per_pixel


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


def make_world_hand(palm_m):
    """21 world landmarks in metres: the middle-finger MCP `palm_m` above the wrist, the rest at the wrist."""
    world = np.zeros((21, 3))
    world[MIDDLE_MCP] = (0.0, -palm_m, 0.0)
    return world


def test_scale_is_palm_length_in_metres_over_pixels():
    # palm 0.09 m in the world, 100 px in the image -> 0.9 mm per pixel at the hand's depth
    image = make_hand(thumb_tip=(0.0, -150.0), index_tip=(50.0, -150.0))
    assert metres_per_pixel(image, make_world_hand(0.09)) == pytest.approx(0.0009)


def test_hand_twice_as_far_has_twice_the_scale():
    near = make_hand(thumb_tip=(0.0, -150.0), index_tip=(50.0, -150.0))
    world = make_world_hand(0.09)
    assert metres_per_pixel(near * 0.5, world) == pytest.approx(2 * metres_per_pixel(near, world))


def test_scale_with_zero_palm_in_the_image_is_rejected():
    image = make_hand(thumb_tip=(0.0, -150.0), index_tip=(50.0, -150.0), middle_mcp=(0.0, 0.0))
    with pytest.raises(ValueError):
        metres_per_pixel(image, make_world_hand(0.09))


def make_palm(du, dv, wrist=(300.0, 240.0)):
    """Image landmarks with the palm line (wrist → middle-finger MCP) along (du, dv) pixels; image v grows downwards."""
    return make_hand(thumb_tip=wrist, index_tip=wrist, wrist=wrist, middle_mcp=(wrist[0] + du, wrist[1] + dv))


def test_palm_pointing_forward_has_zero_pitch():
    # side view (ADR 0003): forward = image right (+u) of the raw frame
    assert hand_pitch(make_palm(100.0, 0.0)) == pytest.approx(0.0)


def test_palm_pointing_down_has_minus_half_pi_like_the_jaws():
    assert hand_pitch(make_palm(0.0, 100.0)) == pytest.approx(-math.pi / 2)


def test_palm_tilted_up_has_a_positive_pitch():
    assert hand_pitch(make_palm(100.0, -100.0)) == pytest.approx(math.pi / 4)


def test_pitch_does_not_depend_on_the_hand_size_or_place():
    hand = make_palm(80.0, 30.0)
    assert hand_pitch(hand * 2.0 + np.array([50.0, -20.0])) == pytest.approx(hand_pitch(hand))


def test_pitch_with_zero_palm_is_rejected():
    with pytest.raises(ValueError):
        hand_pitch(make_palm(0.0, 0.0))
