
from pathlib import Path
import numpy as np

from spatialgeometry import Mesh
from spatialmath import SE3


class ReBot:

    def __init__(self, env, mesh_folder=None):

        self.name = "ReBot"
        self.n = 6
        self.env = env

        # ==================================================
        # 1. ROBOT JOINT DEFINITIONS
        # ==================================================

        # Format:
        # (xyz, rpy, axis, lower_limit, upper_limit)

        self.joint_data = [

            # Joint 1
            (
                [-0.00008416, 0, 0.08465],
                [0, 0, 0],
                [0, 0, 1],
                -2.8, 2.8
            ),

            # Joint 2
            (
                [0.020084, 0.031625, 0.05555],
                [-1.5708, 0, 0],
                [0, 0, -1],
                -3.14, 0
            ),

            # Joint 3
            (
                [-0.264, 0, 0],
                [0, 0, 0],
                [0, 0, 1],
                -3.14, 0
            ),

            # Joint 4
            (
                [0.2426, -0.054, -0.001625],
                [0, 0, 0],
                [0, 0, 1],
                -1.87, 1.57
            ),

            # Joint 5
            (
                [0.078308, -0.0375, -0.03],
                [-1.5708, 0, 0],
                [0, 0, 1],
                -1.57, 1.57
            ),

            # Joint 6
            (
                [0.028008, 0, 0.04],
                [0, 1.5708, 0],
                [0, 0, 1],
                -3.14, 3.14
            )
        ]

        # Joint limits
        self.qlim = np.array([
            [joint[3] for joint in self.joint_data],
            [joint[4] for joint in self.joint_data]
        ])

        # ==================================================
        # 2. INITIAL JOINT CONFIGURATION
        # ==================================================

        self.q_home = np.zeros(6)

        self._q = self.q_home.copy()

        # Gripper opening
        # 0.00 = closed
        # 0.05 = open

        self._gripper_opening = 0.0

        self.GRIPPER_MIN = 0.0
        self.GRIPPER_MAX = 0.05

        # ==================================================
        # 3. GRIPPER ATTACHMENT
        # ==================================================

        self.T_gripper = self.origin_transform(
            [0, 0, 0.15539],
            [0, -1.5708, 3.1415]
        )

        # ==================================================
        # 4. MESH FOLDER
        # ==================================================

        if mesh_folder is None:
            mesh_folder = (
                Path(__file__).parent / "ReBot_Meshes"
            )

        self.mesh_folder = Path(mesh_folder)

        # ==================================================
        # 5. LOAD ROBOT MESHES
        # ==================================================

        self.load_meshes()

        # ==================================================
        # 6. SAVE REFERENCE TRANSFORMS
        # ==================================================

        self.home_transforms = (
            self.get_link_transforms(self.q_home)
        )

        self.home_gripper_transforms = (
            self.get_gripper_transforms(
                self.home_transforms,
                opening=0.0
            )
        )

        # Show assembled robot
        self.update_meshes()

    # ======================================================
    # 7. JOINT ANGLE PROPERTY
    # ======================================================

    @property
    def q(self):
        return self._q.copy()

    @q.setter
    def q(self, new_q):

        new_q = np.asarray(
            new_q, dtype=float
        ).reshape(6)

        if not np.all(np.isfinite(new_q)):
            raise ValueError("Joint angles must be finite")

        if np.any(new_q < self.qlim[0]):
            raise ValueError("Joint below lower limit")

        if np.any(new_q > self.qlim[1]):
            raise ValueError("Joint above upper limit")

        self._q = new_q.copy()

        # Update meshes automatically
        self.update_meshes()

    # ======================================================
    # 8. GRIPPER PROPERTY
    # ======================================================

    @property
    def gripper_opening(self):
        return self._gripper_opening

    @gripper_opening.setter
    def gripper_opening(self, opening):

        opening = float(opening)

        if not np.isfinite(opening):
            raise ValueError("Gripper opening must be finite")

        if not self.GRIPPER_MIN <= opening <= self.GRIPPER_MAX:
            raise ValueError(
                "Gripper opening outside limits"
            )

        self._gripper_opening = opening

        # Update meshes automatically
        self.update_meshes()

    # ======================================================
    # 9. TRANSFORMATION FUNCTIONS
    # ======================================================

    @staticmethod
    def origin_transform(xyz, rpy):

        x, y, z = xyz
        roll, pitch, yaw = rpy

        return (
            SE3(x, y, z)
            * SE3.Rx(roll)
            * SE3.Ry(pitch)
            * SE3.Rz(yaw)
        )

    @staticmethod
    def axis_rotation(axis, angle):

        axis = np.array(axis, dtype=float)

        axis = axis / np.linalg.norm(axis)

        return SE3.AngleAxis(angle, axis)

    # ======================================================
    # 10. CALCULATE LINK TRANSFORMS
    # ======================================================

    def get_link_transforms(self, q=None):

        if q is None:
            q = self._q

        q = np.asarray(q, dtype=float).reshape(6)

        T = SE3()

        transforms = []

        for i in range(6):

            xyz, rpy, axis, lower, upper = (
                self.joint_data[i]
            )

            T_origin = self.origin_transform(
                xyz, rpy
            )

            T_rotation = self.axis_rotation(
                axis, q[i]
            )

            T = T * T_origin * T_rotation

            transforms.append(T)

        return transforms

    # ======================================================
    # 11. CALCULATE GRIPPER TRANSFORMS
    # ======================================================

    def get_gripper_transforms(
        self, link_transforms, opening=0.0
    ):

        T6 = link_transforms[5]

        # Fixed gripper base
        T_end = T6 * self.T_gripper

        # Right finger slides along negative Y
        T_right = T_end * SE3(0, -opening, 0)

        # Left finger slides along positive Y
        T_left = T_end * SE3(0, opening, 0)

        return [
            T_end,
            T_right,
            T_left
        ]

    # ======================================================
    # 12. LOAD STL MESHES
    # ======================================================

    def load_meshes(self):

        mesh_names = [

            "IR_ReBot_base.STL",
            "IR_ReBot_link1.STL",
            "IR_ReBot_link2.STL",
            "IR_ReBot_link3.STL",
            "IR_ReBot_link4.STL",
            "IR_ReBot_link5.STL",
            "IR_ReBot_link6.STL",
            "IR_ReBot_gripper_base.STL",

            # Corrected right finger file
            "IR_ReBot_right_finger_changed.STL",

            "IR_ReBot_left_finger.STL"
        ]

        self.meshes = []

        for name in mesh_names:

            path = self.mesh_folder / name

            if not path.exists():
                raise FileNotFoundError(
                    f"Missing mesh: {path}"
                )

            mesh = Mesh(
                str(path),
                pose=SE3()
            )

            self.meshes.append(mesh)

            self.env.add(mesh)

        # Individual mesh references
        self.base_mesh = self.meshes[0]

        self.link_meshes = self.meshes[1:7]

        self.gripper_base = self.meshes[7]
        self.right_finger = self.meshes[8]
        self.left_finger = self.meshes[9]

        self.gripper_meshes = [
            self.gripper_base,
            self.right_finger,
            self.left_finger
        ]

        # Save the original assembled mesh positions
        self.original_mesh_poses = [
            SE3(mesh.T) for mesh in self.meshes
        ]

    # ======================================================
    # 13. UPDATE ALL ROBOT MESHES
    # ======================================================

    def update_meshes(self):

        # Current joint transforms
        current_transforms = (
            self.get_link_transforms(self._q)
        )

        # Current gripper transforms
        current_gripper_transforms = (
            self.get_gripper_transforms(
                current_transforms,
                self._gripper_opening
            )
        )

        # ----------------------------------------------
        # Fixed base
        # ----------------------------------------------

        self.base_mesh.T = (
            self.original_mesh_poses[0].A
        )

        # ----------------------------------------------
        # Six moving links
        # ----------------------------------------------

        for i in range(6):

            # Movement relative to reference pose
            movement = (
                current_transforms[i]
                * self.home_transforms[i].inv()
            )

            new_pose = (
                movement
                * self.original_mesh_poses[i + 1]
            )

            self.link_meshes[i].T = new_pose.A

        # ----------------------------------------------
        # Gripper base and two fingers
        # ----------------------------------------------

        for i, mesh in enumerate(
            self.gripper_meshes
        ):

            movement = (
                current_gripper_transforms[i]
                * self.home_gripper_transforms[i].inv()
            )

            new_pose = (
                movement
                * self.original_mesh_poses[i + 7]
            )

            mesh.T = new_pose.A

    # ======================================================
    # 14. GRIPPER CONTROLS
    # ======================================================

    def open_gripper(self):

        self.gripper_opening = self.GRIPPER_MAX

    def close_gripper(self):

        self.gripper_opening = self.GRIPPER_MIN

    # ======================================================
    # 15. RESET ROBOT
    # ======================================================

    def home(self):

        self.q = self.q_home
        self.close_gripper()
