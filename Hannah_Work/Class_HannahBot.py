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
    def get_link_transforms(self, q):
        transforms = []
        T = self.base
        for i, link in enumerate(self.links):
            T = T * link.A(q[i])
            transforms.append(T)
        return transforms
    
    def update_meshes(self):
        T_current = self.get_link_transforms(self.q)
        self.base_mesh.T = self.base.A  # Update the base mesh transform
        for i, link_mesh in enumerate(self.link_meshes):
            T_change = T_current[i] * self.T_zero[i].inv()  # Calculate the change in transform
            link_mesh.T = T_change.A  # Update the mesh transform

    def add_to_env(self, env):
        self.update_meshes()
        env.add(self.base_mesh)
        for mesh in self.link_meshes:
            env.add(mesh)