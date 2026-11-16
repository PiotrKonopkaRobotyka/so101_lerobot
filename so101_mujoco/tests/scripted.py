"""A scripted top-down pick and place of so101-nexus's current scene, for the tests (no camera, no hand)."""

import numpy as np

from so101_hand.gripper import CLOSED_RAD, OPEN_RAD
from so101_hand.kinematics import Point
from so101_hand.session import Session
from so101_mujoco.scene import joint_addresses, mujoco_settings
from so101_mujoco.world import joint_targets, to_table


def pick_and_place_plan(env) -> list[np.ndarray]:
    """so101-nexus's joint targets (radians), one per step, from where the arm is: over the cube, down, close, up,
    over the disc, down, open, up, and a wait (the success needs the cube released for 0.2 s)."""
    sim = env.unwrapped
    addresses, settings = joint_addresses(sim.model), mujoco_settings()
    session = Session(settings)
    session.joints = tuple(float(sim.data.qpos[a]) for a in addresses[:4])
    cube, disc = to_table(sim._get_object_pose()[:3]), to_table(sim._get_target_pos())
    targets = [joint_targets(session.solve(Point(cube.forward, cube.right, settings.hover_height)), OPEN_RAD)]
    plan = []  # (point over the table, gripper, steps)
    for (forward, right), down_gripper, up_gripper in (((cube.forward, cube.right), OPEN_RAD, CLOSED_RAD),
                                                       ((disc.forward, disc.right), CLOSED_RAD, OPEN_RAD)):
        plan.append((Point(forward, right, settings.hover_height), down_gripper, 30))
        plan += [(Point(forward, right, h), down_gripper, 1) for h in np.linspace(settings.hover_height,
                                                                                    settings.grasp_height, 20)]
        plan.append((Point(forward, right, settings.grasp_height), up_gripper, 15))
        plan += [(Point(forward, right, h), up_gripper, 1) for h in np.linspace(settings.grasp_height,
                                                                                  settings.hover_height, 20)]
    plan.append((plan[-1][0], OPEN_RAD, 60))
    for point, gripper, steps in plan:
        q = session.solve(point)
        session.arm_command = q
        targets += [joint_targets(q, gripper)] * steps
    return targets
