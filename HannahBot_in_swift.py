from pathlib import Path
import swift
from spatialgeometry import Mesh
from spatialmath import SE3

env = swift.Swift()
env.launch(realtime=True)

A_path = (Path(__file__).parent / "Hannah_Work" / "Hannah Meshes")

base_path = A_path / "IR_mesh_base_scaled_version.dae"

base_mesh = Mesh(str(base_path), pose=SE3())

env.add(base_mesh)

env.hold()