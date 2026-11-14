from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from so101_hand.gripper import CLOSED_RAD, OPEN_RAD
from so101_hand.kinematics import forward3d
from so101_hand.session import Session, Settings

NODE_YAML = Path(__file__).resolve().parents[2] / "so101_hand_ros" / "config" / "hand_teleop.yaml"


def hand_at(u: float, v: float, gap: float = 0.8, gesture: str = "None", score: float = 0.0):
    """A hand whose palm centre is at pixel (u, v), with aperture `gap` (thumb-index gap / palm length of 100 px);
    the fields the session reads of so101_hand.tracker.Hand, without MediaPipe."""
    lm = np.tile([u, v], (21, 1)).astype(float)
    lm[0] = (u, v + 50)  # wrist
    lm[9] = (u, v - 50)  # middle finger MCP: the palm is 100 px long
    lm[5], lm[13] = (u - 20, v), (u + 20, v)  # knuckles around the centre; 17 stays at it
    lm[4], lm[8] = (u - 50 * gap, v - 80), (u + 50 * gap, v - 80)  # thumb tip, index tip
    return SimpleNamespace(image=lm, gesture=gesture, score=score)


def following_session() -> tuple[Session, tuple[float, float]]:
    """A session at the home pose, following a hand right over the gripper; returns it and that pixel."""
    session = Session()
    session.joints = session.solve(session.home)
    tcp = forward3d(*session.joints)[0]
    pixel = session.follower.table.to_pixels(tcp.forward, tcp.right)
    session.update(hand_at(*pixel), 0.0)
    assert session.handle_key(ord("p"), 0.0) == (None, None)
    assert session.follower.following
    return session, pixel


@pytest.mark.skipif(not NODE_YAML.is_file(), reason="the node's config is not next to this package")
def test_settings_are_the_node_parameters():
    yaml = pytest.importorskip("yaml")
    parameters = yaml.safe_load(NODE_YAML.read_text())["hand_teleop"]["ros__parameters"]
    defaults = Settings()
    for name in Settings.__dataclass_fields__:
        if name == "floor_height":  # the simulator's floor: set by the layer, not a node parameter
            continue
        value = parameters[name]
        assert (tuple(value) if isinstance(value, list) else value) == getattr(defaults, name), name


def test_a_fist_closes_the_gripper_at_once():
    session = Session(arm=False)
    assert session.update(hand_at(320, 240, gap=0.8), 0.00) is False  # open, as it starts
    assert session.gripper_rad == OPEN_RAD
    assert session.update(hand_at(320, 240, gap=0.8, gesture="Closed_Fist", score=0.9), 0.05) is True
    assert not session.gripper_open and session.gripper_by == "Closed_Fist"
    assert session.gripper_rad == CLOSED_RAD


def test_without_a_hand_the_gripper_keeps_its_state():
    session = Session(arm=False)
    session.update(hand_at(320, 240, gesture="Closed_Fist", score=0.9), 0.0)
    assert session.update(None, 0.1) is False
    assert not session.gripper_open
    assert session.smoothed is not None  # held for hold_s
    session.update(None, 1.0)
    assert session.smoothed is None  # lost for longer: no more goals


def test_the_arm_cannot_follow_before_it_is_known():
    session = Session()
    session.update(hand_at(320, 200), 0.0)
    action, message = session.handle_key(ord("p"), 0.0)
    assert action is None and message == "not following: no joint states yet"
    assert not session.follower.following


def test_p_follows_and_pauses_and_the_targets_get_joints():
    session, pixel = following_session()
    start = forward3d(*session.current_arm())[0]
    session.update(hand_at(pixel[0] - 40, pixel[1]), 0.05)  # the hand 40 px to the image's left: the robot's right
    assert session.target is not None and session.target.right > start.right + 0.01
    assert tuple(forward3d(*session.arm_command)[0]) == pytest.approx(tuple(session.target), abs=1e-6)
    session.handle_key(ord("p"), 0.1)
    assert not session.follower.following
    session.update(hand_at(pixel[0] - 80, pixel[1]), 0.15)
    assert session.target is None  # paused: no target, the arm keeps its last command


def test_space_toggles_the_height_reached_in_steps():
    session, pixel = following_session()
    assert session.level == "hover"
    session.handle_key(ord(" "), 0.0)
    assert session.level == "grasp"
    heights = []
    for i in range(1, 30):
        session.update(hand_at(*pixel), 0.05 * i)
        heights.append(session.target.height)
    steps = np.diff([session.settings.hover_height, *heights])
    assert np.all(steps <= 1e-9) and np.all(steps >= -session.settings.max_step_height_m - 1e-9)
    assert heights[-1] == pytest.approx(session.settings.grasp_height)
    session.handle_key(ord(" "), 2.0)
    assert session.level == "hover"


def test_h_asks_the_layer_for_the_home_pose():
    session, _ = following_session()
    session.handle_key(ord(" "), 0.0)
    assert session.handle_key(ord("h"), 0.0) == ("home", None)
    q = session.go_home()
    assert q is not None and session.arm_command == q
    assert not session.follower.following and session.level == "hover"
    assert tuple(forward3d(*q)[0]) == pytest.approx(tuple(session.home), abs=1e-6)


def test_q_quits_and_a_gripper_only_session_ignores_the_arm_keys():
    session = Session(arm=False)
    assert session.handle_key(ord("q"), 0.0) == ("quit", None)
    for key in (ord(" "), ord("p"), ord("h")):
        assert session.handle_key(key, 0.0) == (None, None)
    assert session.level == "hover" and not session.follower.following


def test_the_preview_is_the_mirrored_frame():
    pytest.importorskip("cv2")
    from so101_hand import preview

    session, pixel = following_session()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    view = preview.draw(session, frame, extra=("extra line",))
    assert view.shape == frame.shape
    u = int(pixel[0])
    assert frame[:, u - 9 : u + 10].any()  # the palm dot and the gripper ring, drawn on the raw frame
    assert np.array_equal(view[:, ::-1][170:440], frame[170:440])  # mirrored (the text is above and below)
