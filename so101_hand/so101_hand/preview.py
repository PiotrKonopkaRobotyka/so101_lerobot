"""The preview window's picture: the mirrored camera frame with the hand's measures, the workspace and the state.

Shared by the Gazebo node and the MuJoCo app; they show it and pass its keys to Session.handle_key.
"""

import math

import cv2
import numpy as np

from so101_hand.kinematics import PAN_AXIS, forward3d
from so101_hand.measures import INDEX_TIP, MIDDLE_MCP, THUMB_TIP, WRIST
from so101_hand.session import ARM_KEYS, GRIPPER_KEYS, Session

GREEN, GREY, RED, WHITE, YELLOW = (0, 255, 0), (128, 128, 128), (0, 0, 255), (255, 255, 255), (0, 255, 255)


def draw(session: Session, frame: np.ndarray, extra: tuple[str, ...] = ()) -> np.ndarray:
    """The preview of `frame` (the raw camera frame, BGR; drawn on): mirrored like a selfie, with the state as text
    and `extra` lines from the layer under it."""
    if session.arm:
        draw_workspace(session, frame)
    if session.hand is not None:
        lm = session.hand.image
        p = {i: (int(lm[i][0]), int(lm[i][1])) for i in (WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP)}
        cv2.line(frame, p[WRIST], p[MIDDLE_MCP], WHITE, 2)  # palm line: the aperture's unit
        cv2.line(frame, p[THUMB_TIP], p[INDEX_TIP], RED, 3)  # the measured gap
    view = cv2.flip(frame, 1)  # drawn before the flip, so the lines stay on the hand
    for i, text in enumerate([*state_lines(session), *extra]):
        cv2.putText(view, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)
    keys = ARM_KEYS if session.arm else GRIPPER_KEYS
    cv2.putText(view, keys, (10, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)
    return view


def draw_workspace(session: Session, frame: np.ndarray) -> None:
    """The workspace as the hand sees it in the raw frame: the sector of the bounds, the ring where the gripper is
    (or is going), the dot where the palm is (red outside the sector, green while following, grey paused)."""
    bounds, table = session.follower.bounds, session.follower.table
    pans = np.linspace(bounds.pan[0], bounds.pan[1], 40)
    arcs = [(bounds.reach[1], a) for a in pans] + [(bounds.reach[0], a) for a in pans[::-1]]  # outer, then inner
    outline = [table.to_pixels(*(PAN_AXIS + reach * np.array([math.cos(a), math.sin(a)]))) for reach, a in arcs]
    cv2.polylines(frame, [np.array(outline, dtype=np.int32)], isClosed=True, color=GREY, thickness=2)
    q = session.current_arm()
    if q is not None:
        tcp = forward3d(*q)[0]
        u, v = table.to_pixels(tcp.forward, tcp.right)
        cv2.circle(frame, (int(u), int(v)), 12, WHITE, 2)
    if session.palm_px is not None:
        inside = session.follower.inside(session.palm_px, session.levels[session.level])
        colour = RED if not inside else GREEN if session.follower.following else GREY
        cv2.circle(frame, (int(session.palm_px[0]), int(session.palm_px[1])), 8, colour, -1)


def state_lines(session: Session) -> list[str]:
    """The hand, the gripper and, with the arm, its state; lines over about 58 characters are cut at 640 px."""
    hand = session.hand
    if session.smoothed is None:
        gripper = "hand lost: the gripper stays"
    else:
        gripper = f"gripper {'OPEN' if session.gripper_open else 'CLOSED'} by {session.gripper_by}"
        if session.gripper_by == "aperture":  # the threshold the smoothed aperture must reach to change it
            calibration = session.calibration
            change, at = ("closes", calibration.closed) if session.gripper_open else ("opens", calibration.open)
            gripper += f" ({change} at {at:.2f})"
    lines = [
        f"aperture {session.aperture:.2f}, gesture {hand.gesture} {hand.score:.2f}" if hand is not None else "no hand",
        gripper,
        f"calibration (c / o): closed {session.calibration.closed:.2f}, open {session.calibration.open:.2f}",
    ]
    if session.arm:
        state = "FOLLOWING" if session.follower.following else "paused"
        above = session.levels[session.level] - session.settings.floor_height
        lines.append(f"arm {state}, height {session.level.upper()} {100 * above:.0f} cm (space)")
        q = session.current_arm()
        if q is not None:
            tcp, pitch = forward3d(*q)
            side = "right" if tcp.right >= 0 else "left"
            lines.append(  # OpenCV's fonts have no degree sign
                f"TCP {100 * tcp.forward:.0f} cm ahead, {100 * abs(tcp.right):.0f} cm {side}, "
                f"jaws {math.degrees(pitch):.0f} deg"
            )
    return lines
