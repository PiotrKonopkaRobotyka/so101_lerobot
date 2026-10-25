import pytest

from so101_hand.kinematics import Pose
from so101_hand.teleop import Bounds, Clutch

TCP = Pose(reach=0.25, height=0.15, pitch=-0.8)  # where the gripper is when the clutch engages
HAND = (300.0, 240.0)  # wrist in the raw image, pixels
MM_PER_PX = 0.001  # 1 mm per pixel at the hand's depth


def engaged(**kwargs) -> Clutch:
    clutch = Clutch(**kwargs)
    clutch.engage(HAND, metres_per_px=MM_PER_PX, tcp=TCP)
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


def test_the_pitch_stays_as_at_engage():
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
