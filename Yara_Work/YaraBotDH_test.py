import swift
import numpy as np
from math import pi

from YaraBot_DH import YaraBot

# Start Swift simulator
env = swift.Swift()
env.launch(realtime=True, browser=None)

# Load the EC66 robot
robot = YaraBot()
robot.add_to_env(env)

# Use the home position defined in YaraBot_DH.py
home = robot.home_q.copy()

robot.q = home.copy()
env.step()

print("EC66 ROBOT - ALL JOINT TEST")
print("Home configuration:", home)

input("\nRobot at HOME. Press Enter to begin testing...")

# Test each joint individually
for joint in range(6):

    # Reset to home before each test
    robot.q = home.copy()
    env.step()

    print(f"\nTesting Joint {joint + 1}")

    # Calculate joint axis and position at HOME
    T = robot.base.A.copy()

    for i in range(joint):
        T = T @ robot.links[i].A(home[i]).A

    print("Axis:", np.round(T[:3, 2], 4))
    print("Position:", np.round(T[:3, 3], 4))

    input(f"Press Enter to rotate Joint {joint + 1} by 30 degrees...")

    # Gradually rotate the selected joint
    for angle in np.linspace(0, pi/6, 40):

        q_test = home.copy()
        q_test[joint] += angle

        robot.q = q_test
        env.step(0.05)

    print(f"Joint {joint + 1} test completed!")

    input("Inspect the movement. Press Enter to reset...")

    # Return robot to home
    robot.q = home.copy()
    env.step()

print("\nAll six joints tested!")

env.hold()