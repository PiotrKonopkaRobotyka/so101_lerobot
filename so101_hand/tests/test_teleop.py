import math

import pytest

from so101_hand.kinematics import Pose
from so101_hand.teleop import Bounds, Clutch

TCP = Pose(reach=0.25, height=0.15, pitch=-0.8)  # where the gripper is when the clutch engages
HAND = (300.0, 240.0)  # wrist in the raw image, pixels
MM_PER_PX = 0.001  # 1 mm per pixel at the hand's depth
HAND_PITCH = -1.0  # the palm line at engage, radians (measures.hand_pitch)


def engaged(hand_pitch=None, **kwargs) -> Clutch:
    clutch = Clutch(**kwargs)
    clutch.engage(HAND, metres_per_px=MM_PER_PX, tcp=TCP, hand_pitch=hand_pitch)
    return clutch


def test_released_clutch_gives_no_target():
    clutch = Clutch()
    assert not clutch.engaged
    assert clutch.update(HAND) is None


def test_engaging_keeps_the_gripper_where_it_is():
    assert engaged().update(HAND) == TCP


def test_hand_to_my_left_moves_the_gripper_forward():
    # the camera faces me, so my left is the image's right (+u) in the raw frame (ADR 0003)
    target = engaged(max_step_m=1.0).update((HAND[0] + 50, HAND[1]))
    assert (target.reach, target.height) == pytest.approx((TCP.reach + 0.05, TCP.height))


def test_hand_up_moves_the_gripper_up():
    target = engaged(max_step_m=1.0).update((HAND[0], HAND[1] - 50))  # image v grows downwards
    assert (target.reach, target.height) == pytest.approx((TCP.reach, TCP.height + 0.05))


def test_gain_scales_the_motion():
    target = engaged(gain=2.0, max_step_m=1.0).update((HAND[0] + 20, HAND[1]))
    assert target.reach == pytest.approx(TCP.reach + 0.04)


def test_without_the_hand_pitch_the_jaws_keep_their_pitch():
    assert engaged(max_step_m=1.0).update((HAND[0] + 30, HAND[1] - 30)).pitch == TCP.pitch


def test_a_jump_is_spread_over_several_cycles():
    clutch = engaged(max_step_m=0.02)
    reaches = [clutch.update((HAND[0] + 50, HAND[1])).reach for _ in range(4)]
    assert reaches == pytest.approx([0.27, 0.29, 0.30, 0.30])  # 2 cm per cycle up to the 5 cm the hand moved


def test_targets_stay_inside_the_bounds():
    clutch = engaged(bounds=Bounds(reach=(0.10, 0.30), height=(0.05, 0.20)), max_step_m=1.0)
    target = clutch.update((HAND[0] + 500, HAND[1] - 500))  # 0.5 m forward and up
    assert (target.reach, target.height) == pytest.approx((0.30, 0.20))


def test_unreachable_targets_are_skipped():
    clutch = engaged(max_step_m=1.0, reachable=lambda pose: pose.height < 0.17)
    assert clutch.update((HAND[0], HAND[1] - 10)).height == pytest.approx(0.16)
    assert clutch.update((HAND[0], HAND[1] - 30)).height == pytest.approx(0.16)  # 0.18 is out of reach: stay


def test_lost_hand_gives_no_target():
    clutch = engaged()
    assert clutch.update(None) is None
    assert clutch.update(HAND) == TCP  # back where it was: no jump


def test_release_stops_the_targets_and_engaging_again_does_not_jump():
    clutch = engaged(max_step_m=1.0)
    moved = clutch.update((HAND[0] + 40, HAND[1]))
    clutch.release()
    assert clutch.update((HAND[0] + 90, HAND[1])) is None
    clutch.engage((HAND[0] + 90, HAND[1]), metres_per_px=MM_PER_PX, tcp=moved)  # hand moved while released
    assert clutch.update((HAND[0] + 90, HAND[1])) == moved


def test_tilting_the_hand_down_tilts_the_jaws_down():
    clutch = engaged(hand_pitch=HAND_PITCH, max_step_rad=1.0)
    assert clutch.update(HAND, hand_pitch=HAND_PITCH - 0.2).pitch == pytest.approx(TCP.pitch - 0.2)


def test_tilting_the_hand_does_not_move_the_gripper():
    target = engaged(hand_pitch=HAND_PITCH, max_step_rad=1.0).update(HAND, hand_pitch=HAND_PITCH + 0.3)
    assert (target.reach, target.height) == (TCP.reach, TCP.height)


def test_pitch_gain_scales_the_tilt():
    clutch = engaged(hand_pitch=HAND_PITCH, pitch_gain=0.5, max_step_rad=1.0)
    assert clutch.update(HAND, hand_pitch=HAND_PITCH + 0.2).pitch == pytest.approx(TCP.pitch + 0.1)


def test_zero_pitch_gain_keeps_the_jaws_as_in_stage_2():
    clutch = engaged(hand_pitch=HAND_PITCH, pitch_gain=0.0, max_step_rad=1.0)
    assert clutch.update(HAND, hand_pitch=HAND_PITCH + 0.5).pitch == TCP.pitch


def test_a_quick_tilt_is_spread_over_several_cycles():
    clutch = engaged(hand_pitch=HAND_PITCH, max_step_rad=0.05)
    pitches = [clutch.update(HAND, hand_pitch=HAND_PITCH - 0.12).pitch for _ in range(4)]
    assert pitches == pytest.approx([-0.85, -0.90, -0.92, -0.92])  # 0.05 rad per cycle up to the 0.12 of the hand


def test_the_pitch_stays_inside_its_bounds():
    clutch = engaged(hand_pitch=HAND_PITCH, bounds=Bounds(pitch=(-1.0, 0.0)), max_step_rad=1.0)
    assert clutch.update(HAND, hand_pitch=HAND_PITCH - 0.5).pitch == pytest.approx(-1.0)


def test_a_turn_across_half_a_circle_goes_the_short_way():
    # the palm line at +179° and then at -179° has turned by 2°, not by -358°
    clutch = engaged(hand_pitch=math.radians(179), max_step_rad=1.0)
    assert clutch.update(HAND, hand_pitch=math.radians(-179)).pitch == pytest.approx(TCP.pitch + math.radians(2))


def test_the_gripper_still_moves_when_the_new_pitch_is_out_of_reach():
    clutch = engaged(hand_pitch=HAND_PITCH, max_step_m=1.0, max_step_rad=1.0, reachable=lambda pose: pose.pitch > -0.9)
    target = clutch.update((HAND[0] + 20, HAND[1]), hand_pitch=HAND_PITCH - 0.3)
    assert (target.reach, target.pitch) == pytest.approx((TCP.reach + 0.02, TCP.pitch))


def test_the_jaws_still_tilt_when_the_new_position_is_out_of_reach():
    clutch = engaged(hand_pitch=HAND_PITCH, max_step_m=1.0, max_step_rad=1.0, reachable=lambda pose: pose.height < 0.16)
    target = clutch.update((HAND[0], HAND[1] - 30), hand_pitch=HAND_PITCH + 0.2)  # 3 cm up is out of reach
    assert (target.height, target.pitch) == pytest.approx((TCP.height, TCP.pitch + 0.2))


def test_engaging_again_latches_the_new_hand_pitch():
    clutch = engaged(hand_pitch=HAND_PITCH, max_step_rad=1.0)
    tilted = clutch.update(HAND, hand_pitch=HAND_PITCH - 0.2)
    clutch.release()
    clutch.engage(HAND, metres_per_px=MM_PER_PX, tcp=tilted, hand_pitch=HAND_PITCH + 0.4)  # hand turned while released
    assert clutch.update(HAND, hand_pitch=HAND_PITCH + 0.4) == tilted


def test_when_only_one_can_change_the_position_wins():
    # each change alone is reachable, both together are not: the gripper follows the hand, the jaws wait
    clutch = engaged(hand_pitch=HAND_PITCH, max_step_m=1.0, max_step_rad=1.0,
                     reachable=lambda pose: not (pose.reach > 0.26 and pose.pitch < -0.9))
    target = clutch.update((HAND[0] + 20, HAND[1]), hand_pitch=HAND_PITCH - 0.3)
    assert (target.reach, target.pitch) == pytest.approx((TCP.reach + 0.02, TCP.pitch))
