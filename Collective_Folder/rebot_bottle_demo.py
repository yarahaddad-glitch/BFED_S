
import numpy as np
import swift
from spatialmath import SE3
from spatialgeometry import Cylinder
from Rebot_class import ReBot

DT = 0.03
STEPS = 80

# ==================================================
# 1. TEACH POSITIONS - ENTER YOUR RECORDED VALUES
# Six joint angles in radians for each position
# ==================================================

TEACH = {
    "approach"= [],   # Gripper above the bottle
    "pickup": None,     # Gripper positioned around the bottle
    "lift": None,       # Bottle lifted clear of the table
    "transfer": None,   # Safe position near collection station
    "place": None,      # Bottle resting at collection station
    "retreat": None     # Gripper withdrawn from bottle
}

# Example format for a taught position:
# TEACH["approach"] = [0.0, -0.4, -0.6, 0.0, 0.0, 0.0]


# ==================================================
# 2. VALIDATE TEACH POSITIONS
# ==================================================

def get_teach_positions(robot):
    positions = {}

    for name, angles in TEACH.items():
        if angles is None:
            raise ValueError(f"Missing taught position: {name}")

        q = np.asarray(angles, dtype=float).reshape(6)

        if not np.all(np.isfinite(q)):
            raise ValueError(f"Invalid values: {name}")

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
# 4. JOINT MOVEMENT IN SWIFT
# ==================================================

def animate_joints(robot, env, target,
                   bottle=None, attachment=None):

    start = robot.q.copy()
    target = np.asarray(target, dtype=float)

    for u in np.linspace(0, 1, STEPS):
        s = 3 * u**2 - 2 * u**3

        robot.q = start + s * (target - start)

        # Bottle follows the gripper after simulated grasp
        if bottle is not None and attachment is not None:
            bottle.T = (tool_pose(robot) * attachment).A

        env.step(DT)


# ==================================================
# 5. WATER BOTTLE PICKUP AND PLACE
# ==================================================

def demo():

    env = swift.Swift()
    env.launch(realtime=True)

    robot = ReBot(env)
    poses = get_teach_positions(robot)

    # Position simulated bottle relative to taught pickup
    robot.q = poses["pickup"]
    pickup_tool = tool_pose(robot)

    bottle = Cylinder(
        radius=0.03,
        length=0.19,
        pose=pickup_tool * SE3(0, 0, 0.10),
        color=(0.2, 0.65, 0.95, 1.0)
    )

    env.add(bottle)
    robot.q = robot.q_home
    env.step(0.1)

    # Move to bottle
    robot.open_gripper()
    animate_joints(robot, env, poses["approach"])
    animate_joints(robot, env, poses["pickup"])

    # Simulate gripping bottle
    robot.close_gripper()
    attachment = tool_pose(robot).inv() * SE3(bottle.T)

    # Lift and transport bottle
    animate_joints(
        robot, env, poses["lift"], bottle, attachment
    )
    animate_joints(
        robot, env, poses["transfer"], bottle, attachment
    )
    animate_joints(
        robot, env, poses["place"], bottle, attachment
    )

    # Release bottle
    robot.open_gripper()
    env.step(0.1)

    # Retreat without carrying the bottle
    animate_joints(robot, env, poses["retreat"])

    print("Simulated bottle pickup and placement complete")
    env.hold()


if __name__ == "__main__":
    demo()
