
import numpy as np
import swift
from spatialmath import SE3
from spatialgeometry import Cylinder
from Rebot_class import ReBot

DT = 0.03
STEPS = 80

# ==================================================
# 1. TAUGHT POSITIONS - REPLACE VALUES WHEN RETAUGHT
# Six arm joint angles in radians
# ==================================================

TEACH = {
    "hover above": [0.9127, -2.2772, -0.7418, -1.6009, -0.5915, -0.0547],
    "pickup": [1.1275, -2.1193, -0.7422, -1.4281, -0.0986, -0.0029],
    "moving": [-1.0779, -1.0714, -0.6594, -0.5476, -0.0948, 0.0189],
    "placing": [-2.6072, -2.0296, -0.6407, -1.4727, 0.2283, -0.0006],
    "lifting away": [-2.6061, -1.3636, -0.9524, -0.5842, 0.0513, -0.0780],
    "returning home": [0.6865, -1.3663, -0.9520, -0.5989, 0.0151, -0.0399]
}


# ==================================================
# 2. VALIDATE TAUGHT POSITIONS
# ==================================================

def get_teach_positions(robot):
    positions = {}

    for name, values in TEACH.items():
        q = np.asarray(values, dtype=float).reshape(6)

        if not np.all(np.isfinite(q)):
            raise ValueError(f"Invalid joint values: {name}")

        if np.any(q < robot.qlim[0]) or np.any(q > robot.qlim[1]):
            raise ValueError(f"Joint limits exceeded: {name}")

        positions[name] = q

    return positions


# ==================================================
# 3. GRIPPER POSE
# ==================================================

def tool_pose(robot):
    links = robot.get_link_transforms(robot.q)
    return robot.get_gripper_transforms(
        links, robot.gripper_opening
    )[0]


# ==================================================
# 4. JOINT TRAJECTORY ANIMATION
# ==================================================

def animate_joints(robot, env, target, bottle=None, attachment=None):
    start = robot.q.copy()
    target = np.asarray(target, dtype=float).reshape(6)

    if not np.all(np.isfinite(target)):
        raise ValueError("Invalid joint target")

    if np.any(target < robot.qlim[0]) or np.any(target > robot.qlim[1]):
        raise ValueError("Joint target exceeds limits")

    for u in np.linspace(0, 1, STEPS):
        blend = 3 * u**2 - 2 * u**3
        robot.q = start + blend * (target - start)

        if bottle is not None and attachment is not None:
            bottle.T = (tool_pose(robot) * attachment).A

        env.step(DT)


def operate_gripper(robot, env, action):
    if action == "open":
        robot.open_gripper()
    elif action == "close":
        robot.close_gripper()
    else:
        raise ValueError("Unknown gripper action")

    env.step(0.1)
    print(f"Gripper: {action}")


# ==================================================
# 5. WATER BOTTLE PICKUP AND PLACEMENT
# ==================================================

def demo():
    env = swift.Swift()
    env.launch(realtime=True)

    robot = ReBot(env)
    poses = get_teach_positions(robot)

    # Place test bottle relative to the taught pickup pose
    robot.q = poses["pickup"]
    pickup_tool = tool_pose(robot)

    bottle = Cylinder(
        radius=0.03,
        length=0.19,
        pose=pickup_tool * SE3(0, 0, 0.10),
        color=(0.2, 0.65, 0.95, 1.0)
    )

    env.add(bottle)

    # Start from robot home
    robot.q = robot.q_home.copy()
    env.step(0.1)

    print("STARTING WATER BOTTLE SEQUENCE")

    # 1. Move from home to hover above bottle
    print("1. Moving above bottle")
    animate_joints(robot, env, poses["hover above"])

    # 2. Open gripper while above bottle
    operate_gripper(robot, env, "open")

    # 3. Lower to bottle pickup position
    print("2. Moving around bottle")
    animate_joints(robot, env, poses["pickup"])

    # 4. Close gripper around bottle
    operate_gripper(robot, env, "close")
    attachment = tool_pose(robot).inv() * SE3(bottle.T)

    # 5. Lift and move with bottle attached
    print("3. Lifting and transferring bottle")
    animate_joints(
        robot, env, poses["moving"], bottle, attachment
    )

    # 6. Lower bottle at collection position
    print("4. Placing bottle")
    animate_joints(
        robot, env, poses["placing"], bottle, attachment
    )

    # 7. Open gripper and release bottle
    operate_gripper(robot, env, "open")
    print("Bottle released")

    # 8. Move away without carrying bottle
    print("5. Lifting away")
    animate_joints(robot, env, poses["lifting away"])

    # 9. Close empty gripper
    operate_gripper(robot, env, "close")

    # 10. Move through taught return position
    print("6. Returning home")
    animate_joints(robot, env, poses["returning home"])

    # 11. Return to home configuration
    animate_joints(robot, env, robot.q_home)

    print("WATER BOTTLE SEQUENCE COMPLETE")

    # Future: repeat sequence for next water bottle
    # demo_next_bottle()

    env.hold()


if __name__ == "__main__":
    demo()
    

def run_rebot_sequence(env, robot, poses, bottle):
    """Replay the taught bottle trajectory in the shared Swift environment."""

    robot.q = robot.q_home.copy()
    robot.open_gripper()
    env.step(DT)

    print("ReBot: Hover above bottle")
    animate_joints(robot, env, poses["hover above"])

    print("ReBot: Open gripper")
    robot.open_gripper()
    env.step(DT)

    print("ReBot: Move to pickup")
    animate_joints(robot, env, poses["pickup"])

    print("ReBot: Close gripper")
    robot.close_gripper()
    env.step(DT)

    attachment = tool_pose(robot).inv() * SE3(bottle.T)

    print("ReBot: Lift and transfer")
    animate_joints(robot, env, poses["moving"], bottle, attachment)

    print("ReBot: Place bottle")
    animate_joints(robot, env, poses["placing"], bottle, attachment)

    print("ReBot: Release bottle")
    robot.open_gripper()
    env.step(DT)

    print("ReBot: Lift away")
    animate_joints(robot, env, poses["lifting away"])

    robot.close_gripper()
    env.step(DT)

    print("ReBot: Return home")
    animate_joints(robot, env, poses["returning home"])
    animate_joints(robot, env, robot.q_home)

    print("ReBot sequence complete")
