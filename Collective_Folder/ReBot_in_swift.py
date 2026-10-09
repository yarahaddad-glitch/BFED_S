from pathlib import Path
import swift
from spatialgeometry import Mesh
from spatialmath import SE3
env = swift.Swift()
env.launch(realtime=True)

A_path = (Path(__file__).parent / "ReBot_Meshes")

base_path = A_path / "IR_ReBot_base.STL"
base_mesh = Mesh(str(base_path), pose=SE3())

link1_path = A_path / "IR_ReBot_link1.STL"
link1_mesh = Mesh(str(link1_path), pose=SE3())

link2_path = A_path / "IR_ReBot_link2.STL"
link2_mesh = Mesh(str(link2_path), pose=SE3())

link3_path = A_path / "IR_ReBot_link3.STL"
link3_mesh = Mesh(str(link3_path), pose=SE3())

link4_path = A_path / "IR_ReBot_link4.STL"
link4_mesh = Mesh(str(link4_path), pose=SE3())

link5_path = A_path / "IR_ReBot_link5.STL"
link5_mesh = Mesh(str(link5_path), pose=SE3())

link6_path = A_path / "IR_ReBot_link6.STL"
link6_mesh = Mesh(str(link6_path), pose=SE3())

link7_path = A_path / "IR_ReBot_gripper_base.STL"
link7_mesh = Mesh(str(link7_path), pose=SE3())

link8_path = A_path / "IR_ReBot_right_finger.STL"
link8_mesh = Mesh(str(link8_path), pose=SE3())

link9_path = A_path / "IR_ReBot_left_finger.STL"
link9_mesh = Mesh(str(link9_path), pose=SE3())

env.add(base_mesh)
env.add(link1_mesh)
env.add(link2_mesh)
env.add(link3_mesh)
env.add(link4_mesh)
env.add(link5_mesh)
env.add(link6_mesh)
env.add(link7_mesh)
env.add(link8_mesh)
env.add(link9_mesh)

env.hold()
