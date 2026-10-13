"""Kawasaki RS007N: visible RMRC and STL collision-check test."""
from pathlib import Path
import numpy as np
import swift
from spatialgeometry import Cuboid, Sphere
from spatialmath import SE3
from Maryam_robotclass import MaryamBot
from maryamrobot_controller import RobotController

root = Path(__file__).resolve().parent
robot = MaryamBot()
robot.q = np.deg2rad([0, -45, 60, 0, 45, 0])

files = [root / f"RS007N-J{i}.STL" for i in range(1, 7)]

# A clear obstacle position first, so we can verify basic RMRC movement.
# Replace with your workcell's actual obstacles after this test succeeds.
obstacle_xyz = [0.6, 0.5, 0.6]
obstacles = [{"type": "sphere", "center": obstacle_xyz, "radius": 0.04}]
controller = RobotController(robot, files, obstacles, stl_scale=1.0)

env = swift.Swift()
env.launch(realtime=True)
for mesh in robot.links_3d:
    env.add(mesh)

# Show the registered obstacle in Swift (previous version omitted this).
red_sphere = Sphere(radius=0.04, pose=SE3(*obstacle_xyz),
                    color=(1.0, 0.0, 0.0, 1.0))
env.add(red_sphere)

# Show cube at the known FK target; virtual pickup is NOT implemented here.
target = robot.fkine(np.deg2rad([25, -55, 75, 0, 45, 0])).t.copy()
cube_xyz = target - [0, 0, 0.035]
cube = Cuboid(scale=[0.055, 0.055, 0.055], pose=SE3(*cube_xyz.tolist()),
              color=(1.0, 0.5, 0.0, 1.0))
env.add(cube)

# Transparent STL-fitted collision bodies for calibration.
guides = []
for pose, size in controller.body_poses(robot.q):
    guide = Cuboid(scale=size.tolist(), pose=pose, color=(0.0, 0.9, 0.9, 0.18))
    env.add(guide)
    guides.append(guide)

def step():
    for guide, (pose, _) in zip(guides, controller.body_poses(robot.q)):
        guide.T = pose.A
    env.step(controller.dt)

step()  # Display the scene before potentially expensive path planning.
print('Target XYZ:', np.round(target, 4), flush=True)
print('Planning RMRC movement...', flush=True)
success = controller.move(target, step_callback=step)
print('RMRC result:', success, flush=True)
env.hold()
