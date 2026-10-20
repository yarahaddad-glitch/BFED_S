
import numpy as np
import swift
import time

from Class_HannahBot import HannahBot


# ----------------------------------
# Initialise Swift
# ----------------------------------

env = swift.Swift()
env.launch(realtime=True)

# Create HannahBot
robot = HannahBot()

print("\nJoint frame positions at zero configuration:")

T_zero = robot.get_mesh_joint_transforms(np.zeros(6))

for i, T in enumerate(T_zero):
    print(f"Joint {i + 1}: {np.round(T.t, 4)}")



print("\nJoint rotation axes at zero configuration:")

T = robot.base

for i, link in enumerate(robot.links):
    axis = T.R[:, 2]
    print(f"Joint {i+1}: {np.round(axis, 3)}")
    T = T * link.A(robot.q[i])



# Add robot meshes to Swift
robot.add_to_env(env)

print("HannahBot loaded successfully!")
print(robot)


# ----------------------------------
# Starting configuration
# ----------------------------------

q_start = np.zeros(6)

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

    # Reset all joints
    robot.q = q_start.copy()
    robot.update_meshes()
    env.step(0.05)

    time.sleep(1)

    # Move joint from 0 to +45 degrees
    for angle in np.linspace(0, test_angle, steps):

        q = q_start.copy()
        q[joint_index] = angle

        robot.q = q
        robot.update_meshes()

        env.step(0.02)

    time.sleep(1)

    # Move joint from +45 to -45 degrees
    for angle in np.linspace(test_angle, -test_angle, steps * 2):

        q = q_start.copy()
        q[joint_index] = angle

        robot.q = q
        robot.update_meshes()

        env.step(0.02)

    time.sleep(1)

    # Return joint to zero
    for angle in np.linspace(-test_angle, 0, steps):

        q = q_start.copy()
        q[joint_index] = angle

        robot.q = q
        robot.update_meshes()

        env.step(0.02)

    print(f"Joint {joint_index + 1} test complete")

    time.sleep(1)

    robot.q = np.deg2rad([
    0,
    0,
    0,
    0,
    0,
    90
])

robot.update_meshes()
env.step(0.05)


# ----------------------------------
# Final position
# ----------------------------------

robot.q = q_start.copy()
robot.update_meshes()
env.step(0.05)

print("\nAll six joints tested successfully!")

# Keep simulation open
input("Press Enter to close simulation...")
