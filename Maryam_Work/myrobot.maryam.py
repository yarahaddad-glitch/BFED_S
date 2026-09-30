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
    "link0": "RS007N-BASE",
    "link1": "RS007N-J1",
    "link2": "RS007N-J2",
    "link3": "RS007N-J3",
    "link4": "RS007N-J4",
    "link5": "RS007N-J5",
    "link6": "RS007N-J6",

    "color0": (0.55, 0.20, 0.80, 1.0),
    "color1": (0.55, 0.20, 0.80, 1.0),
    "color2": (0.55, 0.20, 0.80, 1.0),
    "color3": (0.55, 0.20, 0.80, 1.0),
    "color4": (0.55, 0.20, 0.80, 1.0),
    "color5": (0.55, 0.20, 0.80, 1.0),
    "color6": (0.55, 0.20, 0.80, 1.0),
}



# ============================================================
# KAWASAKI RS007N - STANDARD DH PARAMETERS
# Based on Kawasaki's official RS007N kinematic model
# ============================================================

deg = pi / 180

links = [

    # J1
    rtb.RevoluteDH(
        d=0.360,
        a=0.0,
        alpha=pi / 2,
        offset=-pi / 2,
        flip=True,
        qlim=[-180 * deg, 180 * deg]
    ),

    # J2
    rtb.RevoluteDH(
        d=0.0,
        a=0.355,
        alpha=0.0,
        offset=pi / 2,
        qlim=[-135 * deg, 135 * deg]
    ),

    # J3
    rtb.RevoluteDH(
        d=0.0,
        a=0.0,
        alpha=pi / 2,
        offset=pi / 2,
        flip=True,
        qlim=[-155 * deg, 155 * deg]
    ),

    # J4
    rtb.RevoluteDH(
        d=0.375,
        a=0.0,
        alpha=pi / 2,
        offset=pi,
        qlim=[-200 * deg, 200 * deg]
    ),

    # J5
    rtb.RevoluteDH(
        d=0.0,
        a=0.0,
        alpha=-pi / 2,
        offset=0.0,
        flip=True,
        qlim=[-125 * deg, 125 * deg]
    ),

    # J6
    rtb.RevoluteDH(
        d=0.078,
        a=0.0,
        alpha=0.0,
        offset=pi / 2,
        qlim=[-360 * deg, 360 * deg]
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



# ============================================================
# TEST J1 MOVEMENT
# ============================================================

# Start at zero
robot.q = [0, 0, 0, 0, 0, 0]
env.step(1.0)

print("Moving J1...")


# Create a slow trajectory:
# J1 goes from 0 degrees to 45 degrees
q_start = [0, 0, 0, 0, 0, 0]

q_end = [
    45 * pi / 180,   # J1 = 45 degrees
    0,
    0,
    0,
    0,
    0
]


trajectory = rtb.jtraj(q_start, q_end, 100)


# Animate it slowly
for q in trajectory.q:
    robot.q = q
    env.step(0.03)


print("J1 movement finished.")

env.hold()