from math import pi

import roboticstoolbox as rtb
import swift

from Maryam_robotclass import MaryamBot


# ============================================================
# CREATE ROBOT
# ============================================================

robot = MaryamBot()


# ============================================================
# SWIFT
# ============================================================

env = swift.Swift()
env.launch(realtime=True)


# Add graphical robot meshes
for mesh in robot.links_3d:
    env.add(mesh)


# Home position
robot.q = [0, 0, 0, 0, 0, 0]

env.step(1.0)


# ============================================================
# TEST J1
# ============================================================

q_start = [0, 0, 0, 0, 0, 0]

q_end = [
    45 * pi / 180,
    0,
    0,
    0,
    0,
    0
]


trajectory = rtb.jtraj(
    q_start,
    q_end,
    100
)


for q in trajectory.q:

    robot.q = q

    env.step(0.03)


env.hold()