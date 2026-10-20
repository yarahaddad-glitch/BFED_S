
from math import pi
import os
import numpy as np
import roboticstoolbox as rtb
from ir_support.robots.UTSMeshRobot import UTSMeshRobot


class MaryamBot(UTSMeshRobot):
    """
    Kawasaki RS007N 6-DOF industrial robot.

    Uses:
    - Standard DH parameters
    - Custom STL graphical meshes
    - UTSMeshRobot for Swift visualisation
    - Manufacturer joint limits
    """

    def __init__(self, base=None):

        # =====================================================
        # 1. MESH DIRECTORY
        # =====================================================

        # STL files are stored in the same folder as this file.
        mesh_dir = os.path.dirname(os.path.abspath(__file__))

        # =====================================================
        # 2. LINK MESH NAMES + COLOURS
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
        # 3. KAWASAKI RS007N STANDARD DH PARAMETERS
        # =====================================================

        # Convert degrees into radians.
        # Robotics Toolbox uses radians for revolute joints.
        deg = pi / 180

        links = [

            # JOINT 1 - BASE ROTATION
            rtb.RevoluteDH(
                d=0.360,
                a=0.0,
                alpha=pi / 2,
                offset=-pi / 2,
                flip=True,
                qlim=[-180 * deg, 180 * deg]
            ),

            # JOINT 2 - SHOULDER
            rtb.RevoluteDH(
                d=0.0,
                a=0.355,
                alpha=0.0,
                offset=pi / 2,
                qlim=[-135 * deg, 135 * deg]
            ),

            # JOINT 3 - ELBOW
            rtb.RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=pi / 2,
                offset=pi / 2,
                flip=True,
                qlim=[-155 * deg, 155 * deg]
            ),

            # JOINT 4 - WRIST ROTATION
            rtb.RevoluteDH(
                d=0.375,
                a=0.0,
                alpha=pi / 2,
                offset=pi,
                qlim=[-200 * deg, 200 * deg]
            ),

            # JOINT 5 - WRIST BENDING
            rtb.RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=-pi / 2,
                offset=0.0,
                flip=True,
                qlim=[-125 * deg, 125 * deg]
            ),

            # JOINT 6 - TOOL ROTATION
            rtb.RevoluteDH(
                d=0.078,
                a=0.0,
                alpha=0.0,
                offset=pi / 2,
                qlim=[-360 * deg, 360 * deg]
            ),
        ]

        # =====================================================
        # 4. CREATE THE UTS MESH ROBOT
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

        # =====================================================
        # 5. CONSERVATIVE SOFTWARE JOINT LIMITS
        # =====================================================

        # Smaller than the mechanical joint limits.
        # These are INITIAL simulation limits only.
        # They must be tested against the actual STL geometry.
        # Shape: 2 rows x 6 joints.
        self.safe_qlim = np.deg2rad([
            [-170, -120, -140, -180, -110, -330],
            [ 170,  120,  140,  180,  110,  330]
        ])

    # =========================================================
    # 6. CHECK WHETHER JOINT ANGLES ARE WITHIN SAFE LIMITS
    # =========================================================

    def joints_safe(self, q):
        """
        Returns True if all six joint angles are within the
        conservative software limits.

        This does NOT guarantee the robot is collision-free.
        """
        q = np.asarray(q, dtype=float)

        if q.shape != (6,):
            raise ValueError("Expected 6 joint angles")

        return bool(
            np.all(q >= self.safe_qlim[0]) and
            np.all(q <= self.safe_qlim[1])
        )
