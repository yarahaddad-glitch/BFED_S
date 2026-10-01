import swift
from math import pi

from YaraBot_DH import YaraBot


env = swift.Swift()
env.launch(realtime=True, browser=None)

robot = YaraBot()

robot.add_to_env(env)

env.step()

# Wait so we can see the original home position
input("Robot is at home position. Press Enter to move Joint 1...")

# Move Joint 1 by 45 degrees
robot.q = [pi/4, 0, 0, 0, 0, 0]

env.step()

input("Joint 1 moved 45 degrees. Press Enter to close...")