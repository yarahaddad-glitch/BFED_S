import numpy as np
import swift
import sys
from pathlib import Path
from spatialmath import SE3
from spatialgeometry import Mesh

sys.path.append(str(Path(__file__).resolve().parent.parent))

from Feed_Iso import feed_iso
from Maryam_Work.Maryam_robotclass import MaryamBot
from Yara_Work.YaraBot_DH import YaraBot
from Collective_Folder.swift_simulation import create_environment
from Maryam_Work.maryam_colour_camera import detect_tray_colour
from Maryam_Work.maryamrobot_controller import RobotController

env = swift.Swift()
env.launch(realtime=True)
groot = create_environment(env)

project_folder = Path(__file__).resolve().parent.parent
enviro_folder = project_folder / "Yara_Work" / "EnviroParts"


tray_path = enviro_folder / "tray_yellow.dae"
tray = Mesh(
    filename=str(tray_path),
    pose=SE3(12.6, 2.8, 0.90),
)
env.add(tray)

maryambot = MaryamBot()
maryambot.base = SE3(13.86, 3.8, 0.51) * SE3.Rz(np.pi / 2)
maryambot.add_to_env(env)

yarabot = YaraBot()
yarabot.base = SE3(4, 0, 0)
yarabot.add_to_env(env)

for filename in ["isolation_area2.dae", "dining_area2.dae",
                 "meal_area2.dae", "3nuggets.dae"]:
    part = Mesh(str(enviro_folder / filename))
    part.T = SE3(0, 0, 0).A
    env.add(part)

robot_files = [
    project_folder / "Maryam_Work" / f"RS007N-J{i}.STL"
    for i in range(1, 7)
]
controller = RobotController(
    maryambot, robot_files,
    obstacles=[],
    stl_scale=1.0,
)

# Create TurtleBot + HannahBot and approach this exact tray object.
hannahbot, turtlebot, success = feed_iso(env, tray, tray_path)


print("HannahBot feed_iso succeeded:", success)

env.hold()
