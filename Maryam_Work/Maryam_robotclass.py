from math import pi
import os

import roboticstoolbox as rtb

from ir_support.robots.UTSMeshRobot import UTSMeshRobot


class MaryamBot(UTSMeshRobot):
    """
    Kawasaki RS007N 6-DOF industrial robot.

    Uses:
    - Standard DH parameters
    - Custom STL graphical meshes
    - UTSMeshRobot for Swift visualisation
    """

    def __init__(self, base=None):

        # =====================================================
        # MESH DIRECTORY
        # =====================================================

        # STL files are stored in the same folder as this file
        mesh_dir = os.path.dirname(os.path.abspath(__file__))


        # =====================================================
        # LINK MESH NAMES + COLOURS
        # =====================================================

        link3d_names = {
            "link0": "RS007N-BASE",
            "link1": "RS007N-J1",
            "link2": "RS007N-J2",
            "link3": "RS007N-J3",
            "link4": "RS007N-J4",
            "link5": "RS007N-J5",
            "link6": "RS007N-J6",

            # Purple
            "color0": (0.55, 0.20, 0.80, 1.0),
            "color1": (0.55, 0.20, 0.80, 1.0),
            "color2": (0.55, 0.20, 0.80, 1.0),
            "color3": (0.55, 0.20, 0.80, 1.0),
            "color4": (0.55, 0.20, 0.80, 1.0),
            "color5": (0.55, 0.20, 0.80, 1.0),
            "color6": (0.55, 0.20, 0.80, 1.0),
        }


        # =====================================================
        # KAWASAKI RS007N STANDARD DH PARAMETERS
        # =====================================================

        deg = pi / 180

        links = [

            # Joint 1
            rtb.RevoluteDH(
                d=0.360,
                a=0.0,
                alpha=pi / 2,
                offset=-pi / 2,
                flip=True,
                qlim=[-180 * deg, 180 * deg]
            ),

            # Joint 2
            rtb.RevoluteDH(
                d=0.0,
                a=0.355,
                alpha=0.0,
                offset=pi / 2,
                qlim=[-135 * deg, 135 * deg]
            ),

            # Joint 3
            rtb.RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=pi / 2,
                offset=pi / 2,
                flip=True,
                qlim=[-155 * deg, 155 * deg]
            ),

            # Joint 4
            rtb.RevoluteDH(
                d=0.375,
                a=0.0,
                alpha=pi / 2,
                offset=pi,
                qlim=[-200 * deg, 200 * deg]
            ),

            # Joint 5
            rtb.RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=-pi / 2,
                offset=0.0,
                flip=True,
                qlim=[-125 * deg, 125 * deg]
            ),

            # Joint 6
            rtb.RevoluteDH(
                d=0.078,
                a=0.0,
                alpha=0.0,
                offset=pi / 2,
                qlim=[-360 * deg, 360 * deg]
            ),
        ]


        # =====================================================
        # CREATE THE UTS MESH ROBOT
        # =====================================================

        super().__init__(
            links=links,
            mesh_stem="RS007N",
            mesh_dir=mesh_dir,
            name="Kawasaki RS007N",

            home_q=[0, 0, 0, 0, 0, 0],

            meshes_are_global_at_home=True,

            link3d_names=link3d_names,

            base=base,
        )