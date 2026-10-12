
import numpy as np
import swift
import time

from Class_ReBot import ReBot


# ----------------------------------
# Initialise Swift
# ----------------------------------

env = swift.Swift()
env.launch(realtime=True)

robot = ReBot()


# ----------------------------------
# Print joint frame positions
# ----------------------------------

print("\nJoint frame positions at zero configuration:")

T_zero = robot.get_link_transforms(
    np.zeros(6)
)

for i, T in enumerate(T_zero):
    print(
        f"Joint {i + 1}: "
        f"{np.round(T.t, 4)}"
    )


# ----------------------------------
# Print joint rotation axes
# ----------------------------------

print("\nJoint rotation axes at zero configuration:")

T = robot.base

for i, link in enumerate(robot.links):

    axis = T.R[:, 2]

    print(
        f"Joint {i + 1}: "
        f"{np.round(axis, 3)}"
    )

    T = T * link.A(robot.q[i])


# ----------------------------------
# Add robot meshes to Swift
# ----------------------------------

robot.add_to_env(env)

print("\nReBot loaded successfully!")
print(robot)


# ----------------------------------
# Starting configuration
# ----------------------------------

q_start = np.array([
    0,
    -np.pi / 2,
    -np.pi / 2,
    0,
    0,
    0
])

robot.q = q_start.copy()
robot.update_meshes()
env.step(0.05)

time.sleep(2)


# ----------------------------------
# Test each joint individually
# ----------------------------------

steps = 50
test_angle = np.deg2rad(45)

for joint_index in range(6):

    print(f"\nTesting Joint {joint_index + 1}")

    robot.q = q_start.copy()
    robot.update_meshes()
    env.step(0.05)

    time.sleep(1)

    start_angle = q_start[joint_index]

    lower = robot.qlim[0, joint_index]
    upper = robot.qlim[1, joint_index]

    positive_angle = np.clip(
        start_angle + test_angle,
        lower,
        upper
    )

    negative_angle = np.clip(
        start_angle - test_angle,
        lower,
        upper
    )

    # Move towards +45 degrees
    for angle in np.linspace(
        start_angle,
        positive_angle,
        steps
    ):

        q = q_start.copy()
        q[joint_index] = angle

        robot.q = q
        robot.update_meshes()

        env.step(0.02)

    time.sleep(1)

    # Move towards -45 degrees
    for angle in np.linspace(
        positive_angle,
        negative_angle,
        steps * 2
    ):

        q = q_start.copy()
        q[joint_index] = angle

        robot.q = q
        robot.update_meshes()

        env.step(0.02)

    time.sleep(1)

    # Return to start
    for angle in np.linspace(
        negative_angle,
        start_angle,
        steps
    ):

        q = q_start.copy()
        q[joint_index] = angle

        robot.q = q
        robot.update_meshes()

        env.step(0.02)

    print(
        f"Joint {joint_index + 1} test complete"
    )

    time.sleep(1)


# ----------------------------------
# Return to starting configuration
# ----------------------------------

robot.q = q_start.copy()
robot.update_meshes()
env.step(0.05)

print("\nAll six joints tested successfully!")

input("Press Enter to close simulation...")
