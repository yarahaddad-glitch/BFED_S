from math import pi
import os

import roboticstoolbox as rtb
import swift

from ir_support.robots.UTSMeshRobot import UTSMeshRobot


# ============================================================
# TEMPORARY RS007N FULL MESH TEST
# Loads Base + J1 + J2 + J3 + J4 + J5 + J6
#
# IMPORTANT:
# These are NOT the final Kawasaki RS007N DH parameters yet.
# This test is only to confirm all graphical meshes load correctly.
# ============================================================


# Folder containing this Python file and all RS007N STL files
mesh_dir = os.path.dirname(os.path.abspath(__file__))

print("Mesh folder:", mesh_dir)


# ============================================================
# CHECK ALL MESH FILES EXIST
# ============================================================

for i in range(7):
    file_name = f"RS007NLink{i}.stl"
    file_path = os.path.join(mesh_dir, file_name)

    print(
        file_name,
        "exists:",
        os.path.exists(file_path)
    )


# ============================================================
# PURPLE LINK COLOURS
# ============================================================

link3d_names = {
    "link0": "RS007NLink0",
    "link1": "RS007NLink1",
    "link2": "RS007NLink2",
    "link3": "RS007NLink3",
    "link4": "RS007NLink4",
    "link5": "RS007NLink5",
    "link6": "RS007NLink6",

    # RGBA colour values
    # Purple = Red 0.55, Green 0.20, Blue 0.80
    "color0": (0.55, 0.20, 0.80, 1.0),
    "color1": (0.55, 0.20, 0.80, 1.0),
    "color2": (0.55, 0.20, 0.80, 1.0),
    "color3": (0.55, 0.20, 0.80, 1.0),
    "color4": (0.55, 0.20, 0.80, 1.0),
    "color5": (0.55, 0.20, 0.80, 1.0),
    "color6": (1.0, 1.0, 1.0, 1.0),
}


# ============================================================
# TEMPORARY 6-JOINT ROBOT
# These are fake DH values ONLY for loading all 7 meshes.
# ============================================================

links = [
    rtb.RevoluteDH(
        d=0,
        a=0,
        alpha=0,
        qlim=[-pi, pi]
    ),

    rtb.RevoluteDH(
        d=0,
        a=0,
        alpha=0,
        qlim=[-pi, pi]
    ),

    rtb.RevoluteDH(
        d=0,
        a=0,
        alpha=0,
        qlim=[-pi, pi]
    ),

    rtb.RevoluteDH(
        d=0,
        a=0,
        alpha=0,
        qlim=[-pi, pi]
    ),

    rtb.RevoluteDH(
        d=0,
        a=0,
        alpha=0,
        qlim=[-pi, pi]
    ),

    rtb.RevoluteDH(
        d=0,
        a=0,
        alpha=0,
        qlim=[-pi, pi]
    ),
]


# ============================================================
# CREATE RS007N
# ============================================================

robot = UTSMeshRobot(
    links=links,

    # Searches for:
    # RS007NLink0.stl
    # RS007NLink1.stl
    # ...
    # RS007NLink6.stl
    mesh_stem="RS007N",

    mesh_dir=mesh_dir,

    name="Kawasaki RS007N",

    # Six joints = six home joint values
    home_q=[0, 0, 0, 0, 0, 0],

    # Our Blender meshes were assembled together
    # in their global home configuration.
    meshes_are_global_at_home=True,

    # Apply mesh names and purple colours
    link3d_names=link3d_names,
)


# ============================================================
# SWIFT
# ============================================================

env = swift.Swift()
env.launch(realtime=True)


# Add every graphical mesh to Swift
for mesh in robot.links_3d:
    env.add(mesh)


# Home position
robot.q = [0, 0, 0, 0, 0, 0]


print()
print("Number of graphical meshes:", len(robot.links_3d))
print("Expected graphical meshes: 7")
print("RS007N full purple mesh model loaded.")


env.hold()