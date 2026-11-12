import math

import numpy as np
import pytest

from so101_hand.kinematics import PAN_AXIS, Point, inverse_steepest
from so101_hand.teleop import Bounds, Follower, TableMap

TABLE = TableMap(px_per_m=1000.0, axis_px=(320.0, 440.0))  # 1 mm per pixel; the pan axis at the bottom centre
TCP = Point(forward=PAN_AXIS[0] + 0.25, right=PAN_AXIS[1], height=0.08)  # 25 cm straight ahead of the pan axis
HAND = TABLE.to_pixels(TCP.forward, TCP.right)  # the palm right over the gripper: (320, 190)


def following(**kwargs) -> Follower:
    follower = Follower(table=TABLE, **kwargs)
    assert follower.resume(HAND, TCP) is None
    return follower


def moved(forward: float = 0.0, right: float = 0.0, height: float = TCP.height) -> tuple:
    """The TCP point moved by the given metres, as a plain tuple for pytest.approx."""
    return (TCP.forward + forward, TCP.right + right, height)


# --- the table map ---


def test_the_axis_pixel_is_the_pan_axis():
    assert TABLE.to_table((320.0, 440.0)) == pytest.approx(tuple(PAN_AXIS))
    assert HAND == pytest.approx((320.0, 190.0))


def test_up_the_image_is_forward_and_image_left_is_my_right():
    # the camera faces me: v grows downwards, and my right hand moving to my right moves left in the raw frame
    assert TABLE.to_table((320.0, 340.0)) == pytest.approx((PAN_AXIS[0] + 0.1, PAN_AXIS[1]))
    assert TABLE.to_table((220.0, 440.0)) == pytest.approx((PAN_AXIS[0], PAN_AXIS[1] + 0.1))


def test_pixels_undo_table():
    assert TABLE.to_pixels(*TABLE.to_table((100.0, 200.0))) == pytest.approx((100.0, 200.0))


# --- the bounds ---


def test_bounds_keep_a_point_inside_as_it_is():
    point = Point(PAN_AXIS[0] + 0.2, PAN_AXIS[1] + 0.1, 0.05)
    assert tuple(Bounds().clip(point)) == pytest.approx(tuple(point))
    assert Bounds().contains(point)


def test_bounds_clip_the_reach_along_the_same_direction_from_the_pan_axis():
    clipped = Bounds(reach=(0.1, 0.4)).clip(Point(PAN_AXIS[0] + 0.4, PAN_AXIS[1] + 0.3, 0.05))  # reach 0.5
    forward, right = clipped.forward - PAN_AXIS[0], clipped.right - PAN_AXIS[1]
    assert math.hypot(forward, right) == pytest.approx(0.4)
    assert right / forward == pytest.approx(0.3 / 0.4)


def test_bounds_clip_the_pan_and_the_height():
    behind = Point(PAN_AXIS[0] - 0.05, PAN_AXIS[1] + 0.2, 0.0)
    clipped = Bounds(reach=(0.1, 0.4), pan=(-math.pi / 2, math.pi / 2), height=(0.02, 0.4)).clip(behind)
    assert tuple(clipped) == pytest.approx((PAN_AXIS[0], PAN_AXIS[1] + math.hypot(0.05, 0.2), 0.02))
    assert not Bounds().contains(behind)


def test_the_default_bounds_are_reachable_at_the_working_heights():
    bounds = Bounds()
    for reach in np.linspace(*bounds.reach, 12):
        for pan in np.linspace(*bounds.pan, 9):
            for height in (0.02, 0.05, 0.08, 0.10):
                point = Point(PAN_AXIS[0] + reach * math.cos(pan), PAN_AXIS[1] + reach * math.sin(pan), height)
                assert inverse_steepest(point) is not None, f"reach {reach:.2f} m, pan {pan:.2f} rad, {height} m up"


def test_the_joints_change_smoothly_inside_the_default_bounds():
    # 5 mm steps out and back along the middle and along the inner and outer arcs, at the grasp and hover heights:
    # the IK never flips between elbow up and down and stays away from the stretched arm, so a small step of the
    # gripper is never a swing of the arm
    bounds = Bounds()

    def point(reach, pan, height):
        return Point(PAN_AXIS[0] + reach * math.cos(pan), PAN_AXIS[1] + reach * math.sin(pan), height)

    reaches = np.arange(bounds.reach[0], bounds.reach[1] + 1e-9, 0.005)
    for height in (0.02, 0.08):
        paths = [[point(r, 0.0, height) for r in np.concatenate([reaches, reaches[::-1]])]]
        paths += [
            [point(reach, pan, height) for pan in np.arange(bounds.pan[0], bounds.pan[1], 0.005 / reach)]
            for reach in bounds.reach
        ]
        for path in paths:
            q = inverse_steepest(path[0])[0]
            for p in path[1:]:
                q_next = inverse_steepest(p, current=q)[0]
                assert max(abs(a - b) for a, b in zip(q_next, q)) < 0.15
                q = q_next


# --- following ---


def test_paused_follower_gives_no_target():
    follower = Follower(table=TABLE)
    assert not follower.following
    assert follower.update(HAND, TCP.height) is None


def test_resuming_needs_a_hand_inside_the_workspace():
    follower = Follower(table=TABLE)
    assert follower.resume(None, TCP) == "no hand in view"
    assert follower.resume((320.0, 440.0), TCP) == "hand outside the workspace"  # over the pan axis: reach 0
    assert not follower.following


def test_resuming_keeps_the_gripper_where_it_is():
    assert tuple(following().update(HAND, TCP.height)) == pytest.approx(moved())


def test_hand_to_my_right_moves_the_gripper_to_the_robots_right():
    target = following(max_step_m=1.0).update((HAND[0] - 50, HAND[1]), TCP.height)
    assert tuple(target) == pytest.approx(moved(right=0.05))


def test_hand_up_the_image_moves_the_gripper_forward():
    target = following(max_step_m=1.0).update((HAND[0], HAND[1] - 50), TCP.height)
    assert tuple(target) == pytest.approx(moved(forward=0.05))


def test_a_jump_over_the_table_is_spread_over_several_cycles():
    follower = following(max_step_m=0.02)
    hand = (HAND[0] - 80, HAND[1])  # 8 cm to my right
    assert tuple(follower.update(hand, TCP.height)) == pytest.approx(moved(right=0.02))
    for _ in range(3):
        target = follower.update(hand, TCP.height)
    assert tuple(target) == pytest.approx(moved(right=0.08))


def test_the_height_goes_to_the_level_at_its_own_rate():
    follower = following(max_step_height_m=0.005)
    assert tuple(follower.update(HAND, 0.02)) == pytest.approx(moved(height=0.075))  # 5 mm down per update
    for _ in range(11):
        target = follower.update(HAND, 0.02)
    assert tuple(target) == pytest.approx(moved(height=0.02))
    assert tuple(follower.update(HAND, 0.02)) == pytest.approx(moved(height=0.02))  # and stays there


def test_the_height_change_does_not_slow_the_move_over_the_table():
    follower = following(max_step_m=0.02, max_step_height_m=0.005)
    target = follower.update((HAND[0] - 50, HAND[1]), 0.02)
    assert tuple(target) == pytest.approx(moved(right=0.02, height=0.075))


def test_targets_stay_inside_the_bounds():
    follower = following(bounds=Bounds(reach=(0.1, 0.3)), max_step_m=1.0)
    target = follower.update((HAND[0], 0.0), TCP.height)  # the top edge of the image: 44 cm ahead of the pan axis
    assert tuple(target) == pytest.approx((PAN_AXIS[0] + 0.3, PAN_AXIS[1], TCP.height))


def test_the_height_still_changes_when_the_move_over_the_table_is_out_of_reach():
    follower = following(max_step_m=1.0, reachable=lambda point: point.right < TCP.right + 0.01)
    target = follower.update((HAND[0] - 50, HAND[1]), 0.02)  # 5 cm right is out of reach
    assert tuple(target) == pytest.approx(moved(height=0.075))


def test_the_gripper_still_moves_when_the_new_height_is_out_of_reach():
    follower = following(max_step_m=1.0, reachable=lambda point: point.height > 0.079)
    target = follower.update((HAND[0] - 50, HAND[1]), 0.02)  # going down is out of reach
    assert tuple(target) == pytest.approx(moved(right=0.05))


def test_the_target_stays_when_nothing_is_reachable():
    follower = following(max_step_m=1.0, reachable=lambda point: False)
    assert follower.update((HAND[0] - 50, HAND[1]), 0.02) == TCP


def test_lost_hand_gives_no_target_but_keeps_following():
    follower = following()
    assert follower.update(None, TCP.height) is None
    assert follower.following
    assert tuple(follower.update(HAND, TCP.height)) == pytest.approx(moved())


def test_pause_stops_the_targets_and_resuming_does_not_jump():
    follower = following(max_step_m=0.02)
    follower.pause()
    assert follower.update(HAND, TCP.height) is None
    elsewhere = (HAND[0] - 80, HAND[1])  # the hand came back 8 cm to my right
    assert follower.resume(elsewhere, TCP) is None
    assert tuple(follower.update(elsewhere, TCP.height)) == pytest.approx(moved(right=0.02))
