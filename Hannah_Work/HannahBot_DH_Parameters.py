from roboticstoolbox import DHRobot, RevoluteDH
from pathlib import Path
from spatialgeometry import Mesh
from spatialmath import SE3
import numpy as np
import swift



hannahbot = DHRobot([
    RevoluteDH(d=0.1451, a=0, alpha=-np.pi/2, offset=0, qlim=[-3*np.pi/2, 3*np.pi/2]),
    RevoluteDH(d=0, a=0.4290, alpha=0, offset=-np.pi/2, qlim=[-np.pi, np.pi]),
    RevoluteDH(d=0, a=0.4115, alpha=0, offset=0, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
    RevoluteDH(d=-0.1222, a=0, alpha=np.pi/2, offset=np.pi/2, qlim=[-np.pi, np.pi]),
    RevoluteDH(d=0.1060, a=0, alpha=np.pi/2, offset=0, qlim=[-np.pi, np.pi]),
    RevoluteDH(d=0.1144, a=0, alpha=0, offset=0, qlim=[-3*np.pi/2 , 3*np.pi/2]),
    ], name="HannahBot")

print(hannahbot)


A_path = (Path(__file__).parent / "Hannah Meshes")

base_path = A_path / "IR_mesh_base.dae"
base_mesh = Mesh(str(base_path), pose=SE3())

link1_path = A_path / "IR_mesh_link1.dae"
link1_mesh = Mesh(str(link1_path), pose=SE3())

link2_path = A_path / "IR_mesh_link2.dae"
link2_mesh = Mesh(str(link2_path), pose=SE3())

link3_path = A_path / "IR_mesh_link3.dae"
link3_mesh = Mesh(str(link3_path), pose=SE3())

link4_path = A_path / "IR_mesh_link4.dae"
link4_mesh = Mesh(str(link4_path), pose=SE3())

link5_path = A_path / "IR_mesh_link5.dae"
link5_mesh = Mesh(str(link5_path), pose=SE3())

link6_path = A_path / "IR_mesh_link6.dae"
link6_mesh = Mesh(str(link6_path), pose=SE3())

meshes = [base_mesh, link1_mesh, link2_mesh, link3_mesh, link4_mesh, link5_mesh, link6_mesh]

env = swift.Swift()
env.launch(realtime=True)
env.add(base_mesh)
for mesh in meshes:
    env.add(mesh)
env.hold()

