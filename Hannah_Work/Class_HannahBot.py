import numpy as np
from pathlib import Path
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from spatialgeometry import Mesh


class HannahBot(DHRobot):

    def __init__(self):
        links = [
            RevoluteDH(d=0.147300, a=0.000345, alpha=np.pi / 2, offset=np.pi / 2, qlim=[-3*np.pi/2, 3*np.pi/2]),
            RevoluteDH(d=-0.145400, a=0.000508, alpha=np.pi / 2, offset=np.pi, qlim=[-np.pi, np.pi]),
            RevoluteDH(d=0.580700, a=0.125260, alpha=0, offset=np.pi / 2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
            RevoluteDH(d=0.264800, a=0, alpha=np.pi / 2, offset=np.pi / 2, qlim=[-np.pi, np.pi]),
            RevoluteDH(d=-0.108160, a=0, alpha=np.pi / 2, offset=np.pi, qlim=[-np.pi, np.pi]),
            RevoluteDH(d=0.058200, a=0, alpha=0, offset=0, qlim=[-3*np.pi/2, 3*np.pi/2])]

        super().__init__(links, name="HannahBot")

        self.q = np.zeros(6)

        self.mesh_axes = ["z", "x", "z", "z", "x", "z"]

        self.mesh_pivots = [
            np.array([0.0, 0.0, 0.0]),
            np.array([-0.07002, 0.000345, 0.1473]),
            np.array([-0.14540, -0.000163, 0.5129]),
            np.array([-0.02014, -0.000163, 0.9431]),
            np.array([-0.07864, -0.000163, 0.9928]),
            np.array([-0.12830, -0.000163, 1.0510])
        ]

        self.tool_position = np.array([-0.23330, 0.000497, 1.1150])  #used Ai for this line

        # DH base is aligned with Blender world axes.
        self.base = SE3()

        # The final DH frame at q=0 must be calculated
        # before setting the gripper tool offset.
        T_dh_zero = SE3()

        for link in self.links:
            T_dh_zero = T_dh_zero * link.A(0)

        # Tool offset expressed in the final DH frame.
        self.tool = (
            T_dh_zero.inv()
            * SE3(*self.tool_position)
        )


        mesh_folder = Path(__file__).resolve().parent / "Hannah Meshes"

        self.base_mesh = Mesh(
            filename=str(mesh_folder / "IR_base.stl"),
            pose=SE3()
        )

        self.link_meshes = []


        for i in range(1, 7):

            mesh = Mesh(
                filename=str(mesh_folder / f"IR_link{i}.stl"),
                pose=SE3()
            )

            self.link_meshes.append(mesh)

        # Load gripper mesh once
        self.gripper_mesh = Mesh(
            filename=str(mesh_folder / "IR_gripper.stl"),
            pose=SE3()
        )



        self.update_meshes()



    def get_mesh_joint_transforms(self, q):

        transforms = []

        T_motion = SE3()

        for i, axis in enumerate(self.mesh_axes):

            angle = float(q[i])
            pivot = self.mesh_pivots[i]

            if axis == "x":
                R = SE3.Rx(angle)
            else:
                R = SE3.Rz(angle)

            T_joint = (
                SE3(*pivot)
                * R
                * SE3(*(-pivot))
            )

            T_motion = T_motion * T_joint

            transforms.append(T_motion)

        return transforms


    def update_meshes(self):

        T_mesh = self.get_mesh_joint_transforms(self.q)

        # Base mesh
        self.base_mesh.T = self.base.A

        # Update links 1 to 6
        for i, link_mesh in enumerate(self.link_meshes):

            link_mesh.T = (
                self.base * T_mesh[i]
            ).A

        # Gripper follows joint 6
        self.gripper_mesh.T = (
            self.base * T_mesh[5]
        ).A



    def add_to_env(self, env):

        self.update_meshes()

        env.add(self.base_mesh)

        for mesh in self.link_meshes:
            env.add(mesh)

        env.add(self.gripper_mesh)



    def measured_fkine(self, q):

        T_motion = self.get_mesh_joint_transforms(q)[-1]

        T_tool = SE3(*self.tool_position)

        return self.base * T_motion * T_tool


if __name__ == "__main__":

    robot = HannahBot()

    test_configs = [
        [0, 0, 0, 0, 0, 0],
        [0, 30, 0, 0, 0, 0],
        [0, 0, 30, 0, 0, 0],
        [0, 0, 0, 30, 0, 0],
        [0, 0, 0, 0, 30, 0],
        [0, 0, 0, 0, 0, 30],
        [10, 20, -15, 30, 10, -20]
    ]

    print("\nHANNAHBOT FORWARD KINEMATICS TEST")
    print("=" * 65)

    for angles in test_configs:

        q = np.deg2rad(angles)

        T_DH = robot.fkine(q)
        T_measured = robot.measured_fkine(q)

        position_error = np.linalg.norm(
            T_DH.t - T_measured.t
        )

        rotation_error = np.linalg.norm(
            T_DH.R - T_measured.R
        )

        print(f"\nJoint angles: {angles}")
        print("DH position:", T_DH.t)
        print("Measured position:", T_measured.t)
        print(f"Position error: {position_error:.9f} m")
        print(f"Rotation error: {rotation_error:.9f}")

    print("\nTesting complete.")



