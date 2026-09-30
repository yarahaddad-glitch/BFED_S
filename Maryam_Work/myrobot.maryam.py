from math import pi
import os

import roboticstoolbox as rtb
import swift

from ir_support.robots.UTSMeshRobot import UTSMeshRobot


# ============================================================
# TEMPORARY BASE + J1 TEST
# ============================================================

mesh_dir = r"C:\Users\marya\OneDrive - UTS\Industrial Robotics-WRX\Assignment 2"

print("Base exists:",
      os.path.exists(os.path.join(mesh_dir, "RS007NLink0.stl")))

print("J1 exists:",
      os.path.exists(os.path.join(mesh_dir, "RS007NLink1.stl")))


# Temporary fake joint - NOT final RS007N DH
links = [
    rtb.RevoluteDH(
        d=0,
        a=0,
        alpha=0,
        qlim=[-pi, pi]
    )
]


robot = UTSMeshRobot(
    links=links,
    mesh_stem="RS007N",
    mesh_dir=mesh_dir,
    name="RS007N",
    home_q=[0],
    meshes_are_global_at_home=True,
)


# ============================================================
# SWIFT
# ============================================================

env = swift.Swift()
env.launch(realtime=True)


# IMPORTANT:
# Add the graphical meshes themselves
for mesh in robot.links_3d:
    env.add(mesh)


robot.q = [0]

print("Number of graphical meshes:", len(robot.links_3d))
print("Base + J1 added to Swift.")

env.hold()