from roboticstoolbox import DHRobot, RevoluteDH
from pathlib import Path
from spatialgeometry import Mesh
from spatialmath import SE3
import numpy as np

class HannahBot(DHRobot):
    def __init__(self):
        links = [
            RevoluteDH(d=0.1451, a=0, alpha=-np.pi/2, offset=0, qlim=[-3*np.pi/2, 3*np.pi/2]),
            RevoluteDH(d=0, a=0.4290, alpha=0, offset=-np.pi/2, qlim=[-np.pi, np.pi]),
            RevoluteDH(d=0, a=0.4115, alpha=0, offset=0, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
            RevoluteDH(d=-0.1222, a=0, alpha=np.pi/2, offset=np.pi/2, qlim=[-np.pi, np.pi]),
            RevoluteDH(d=0.1060, a=0, alpha=np.pi/2, offset=0, qlim=[-np.pi, np.pi]),
            RevoluteDH(d=0.1144, a=0, alpha=0, offset=0, qlim=[-3*np.pi/2 , 3*np.pi/2])
        ]

        super().__init__(links, name="HannahBot")  #creating DH robot
        self.q = np.zeros(6) #starting joint angles



#meshes 
        meshes_path = Path(__file__).parent / "Hannah Meshes"
        self.base_mesh = Mesh(str(meshes_path / "IR_base.stl"), pose=SE3())

        self.link_meshes = [
            Mesh(str(meshes_path / "IR_link1.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link2.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link3.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link4.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link5.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link6.stl"), pose=SE3())
        ]

        self.T_zero = self.get_link_transforms(np.zeros(6))  # Get the transforms for the zero configuration

#link positions
    # def get_link_transforms(self, q):
    #     transforms = []
    #     T = self.base
    #     for i, link in enumerate(self.links):
    #         T = T * link.A(q[i])
    #         transforms.append(T)
    #     return transforms
    
    # def update_meshes(self):
    #     T_current = self.get_link_transforms(self.q)
    #     self.base_mesh.T = self.base.A  # Update the base mesh transform
    #     for i, link_mesh in enumerate(self.link_meshes):
    #         T_change = T_current[i] * self.T_zero[i].inv()  # Calculate the change in transform
    #         link_mesh.T = T_change.A  # Update the mesh transform

    # def add_to_env(self, env):
    #     self.update_meshes()
    #     env.add(self.base_mesh)
    #     for mesh in self.link_meshes:
    #         env.add(mesh)









    
from roboticstoolbox import DHRobot, RevoluteDH
from pathlib import Path
from spatialgeometry import Mesh
from spatialmath import SE3
import numpy as np


class HannahBot(DHRobot):

    def __init__(self):

        # ----------------------------------
        # DH Parameters
        # ----------------------------------

        links = [
            RevoluteDH(
                d=0.1451, a=0, alpha=-np.pi/2,
                offset=0,
                qlim=[-3*np.pi/2, 3*np.pi/2]
            ),

            RevoluteDH(
                d=0, a=0.4290, alpha=0,
                offset=-np.pi/2,
                qlim=[-np.pi, np.pi]
            ),

            RevoluteDH(
                d=0, a=0.4115, alpha=0,
                offset=0,
                qlim=[np.deg2rad(-155), np.deg2rad(155)]
            ),

            RevoluteDH(
                d=-0.1222, a=0, alpha=np.pi/2,
                offset=np.pi/2,
                qlim=[-np.pi, np.pi]
            ),

            RevoluteDH(
                d=0.1060, a=0, alpha=np.pi/2,
                offset=0,
                qlim=[-np.pi, np.pi]
            ),

            RevoluteDH(
                d=0.1144, a=0, alpha=0,
                offset=0,
                qlim=[-3*np.pi/2, 3*np.pi/2]
            )
        ]

        super().__init__(links, name="HannahBot")

        # Starting joint configuration
        self.q = np.zeros(6)

        # ----------------------------------
        # Load STL Meshes
        # ----------------------------------

        meshes_path = Path(__file__).parent / "Hannah Meshes"

        self.base_mesh = Mesh(
            str(meshes_path / "IR_base.stl"),
            pose=SE3()
        )

        self.link_meshes = [
            Mesh(str(meshes_path / "IR_link1.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link2.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link3.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link4.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link5.stl"), pose=SE3()),
            Mesh(str(meshes_path / "IR_link6.stl"), pose=SE3())
        ]

        # Zero configuration transforms
        self.T_zero = self.get_link_transforms(np.zeros(6))

        # Desired mesh rotation axes
        self.mesh_axes = ["z", "x", "z", "z", "x", "z"]

        # # Estimated joint pivot locations
        # self.mesh_pivots = [self.base.t.copy()]

        # for T in self.T_zero[:-1]:
        #     self.mesh_pivots.append(T.t.copy())




        self.mesh_pivots = [
        np.array([0.0, 0.0, 0.0]),

        np.array([0.0, 0.0, 0.1451]),

            # Joint 3 - measured in Blender
        np.array([-0.1454, -0.000163, 0.5129]),

        np.array([0.0, 0.0, 0.9856]),

            # Joint 5 - measured in Blender
        np.array([-0.07864, -0.000163, 0.9928]),

        np.array([-0.1283, -0.000163, 1.0510])
        ]


    # ----------------------------------
    # DH Link Transformations
    # ----------------------------------

    def get_link_transforms(self, q):

        transforms = []
        T = self.base

        for i, link in enumerate(self.links):
            T = T * link.A(q[i])
            transforms.append(T)

        return transforms

    # ----------------------------------
    # Mesh Joint Transformations
    # ----------------------------------

    def get_mesh_joint_transforms(self, q):

        transforms = []
        T_motion = SE3()

        for i, axis in enumerate(self.mesh_axes):

            angle = float(q[i])

            # Select rotation axis
            if axis == "x":
                R = SE3.Rx(angle)
            else:
                R = SE3.Rz(angle)

            # Joint pivot location
            p = self.mesh_pivots[i]

            # Rotate around joint pivot
            T_joint = (
                SE3(*p)
                * R
                * SE3(*(-p))
            )

            # Include previous joint movements
            T_motion = T_motion * T_joint

            transforms.append(T_motion)

        return transforms



    def measured_fkine(self, q):

        # Joint axes measured in Blender
        axes = ["z", "x", "z", "z", "x", "z"]

        # Joint centres in assembled zero configuration
        pivots = [
            [0.0, 0.0, 0.0],
            [-0.07002, 0.000345, 0.1473],
            [-0.14540, -0.000163, 0.5129],
            [-0.02014, -0.000163, 0.9431],
            [-0.07864, -0.000163, 0.9928],
            [-0.12830, -0.000163, 1.0510]
        ]

        # Gripper attachment in zero configuration
        tool_zero = SE3(
            -0.23330,
             0.000497,
             1.1150
        )

        # Cumulative joint movement
        T = SE3()

        for i in range(6):

            p = np.array(pivots[i])
            angle = float(q[i])

            if axes[i] == "x":
                R = SE3.Rx(angle)
            else:
                R = SE3.Rz(angle)

            # Rotation about the physical joint centre
            T_joint = (
                SE3(*p)
                * R
                * SE3(*(-p))
            )

            T = T * T_joint

        return self.base * T * tool_zero

    # ----------------------------------
    # Update Mesh Positions
    # ----------------------------------

    def update_meshes(self):

        T_mesh = self.get_mesh_joint_transforms(self.q)

        # Update base mesh
        self.base_mesh.T = self.base.A

        # Update each link mesh
        for i, link_mesh in enumerate(self.link_meshes):
            link_mesh.T = (self.base * T_mesh[i]).A

    # ----------------------------------
    # Add Robot to Swift
    # ----------------------------------

    def add_to_env(self, env):

        self.update_meshes()

        env.add(self.base_mesh)

        for mesh in self.link_meshes:
            env.add(mesh)
