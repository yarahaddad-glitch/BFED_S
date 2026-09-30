from pathlib import Path
import swift
from spatialgeometry import Mesh
from spatialmath import SE3

env = swift.Swift()
env.launch(realtime=True)

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

env.add(base_mesh)
env.add(link1_mesh)
env.add(link2_mesh)
env.add(link3_mesh)
env.add(link4_mesh)
env.add(link5_mesh)
env.add(link6_mesh)

env.hold()




