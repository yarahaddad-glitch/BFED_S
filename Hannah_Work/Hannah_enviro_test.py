
# import numpy as np
# import swift
# import sys
# from pathlib import Path
# from spatialmath import SE3
# from spatialgeometry import Mesh
# sys.path.append(str(Path(__file__).resolve().parent.parent))

# from Hannah_Work.Class_HannahBot import HannahBot
# from Maryam_Work.Maryam_robotclass import MaryamBot
# from Yara_Work.YaraBot_DH import YaraBot
# from Collective_Folder.swift_simulation import create_environment
# from Maryam_Work.maryam_colour_camera import detect_tray_colour
# from Maryam_Work.maryamrobot_controller import RobotController
# # Create ONE Swift simulation
# env = swift.Swift()
# env.launch(realtime=True)
# # Load Groot, prisoner, Drex, cage and ONE green tray
# groot = create_environment(env)


# project_folder = Path(__file__).resolve().parent.parent

# tray_path = (
#     project_folder
#     / "Yara_Work"
#     / "EnviroParts"
#     / "tray_yellow.dae"
# )

# tray = Mesh(
#     filename=str(tray_path),
#     pose=SE3(12.6, 2.8, 0.89),
# )


# env.add(tray)
# #scene_parts.append((tray, np.array(tray.T, dtype=float)))

# # Hannah's robot
# hannahbot = HannahBot()
# hannahbot.base = SE3(12.4, 2.8, 0) * SE3.Rz(np.pi)
# hannahbot.add_to_env(env)

# # Maryam's Kawasaki
# maryambot = MaryamBot()
# maryambot.base = SE3(13.86, 3.8, 0.51) * SE3.Rz(np.pi / 2)
# maryambot.add_to_env(env)
# # Yara's robot
# yarabot = YaraBot()
# yarabot.base = SE3(4, 0, 0)
# yarabot.add_to_env(env)


# # Load kitchen environment
# assignment_folder = Path(__file__).resolve().parent.parent
# enviro_folder = assignment_folder / "Yara_Work" / "EnviroParts"

# meal_file = Mesh(str(enviro_folder / "meal_area2.dae"))
# dining_file = Mesh(str(enviro_folder / "dining_area2.dae"))
# isolation_file = Mesh(str(enviro_folder / "isolation_area2.dae"))
# nuggets_file = Mesh(str(enviro_folder / "3nuggets.dae"))

# isolation_file.T = SE3(0, 0, 0).A
# meal_file.T = SE3(0, 0, 0).A
# dining_file.T = SE3(0, 0, 0).A
# nuggets_file.T = SE3(0, 0, 0).A

# env.add(isolation_file)
# env.add(dining_file)
# env.add(meal_file)
# env.add(nuggets_file)

# # Set up Kawasaki RMRC and STL collision geometry
# robot_files = [
#     assignment_folder / "Maryam_Work" / f"RS007N-J{i}.STL"
#     for i in range(1, 7)
# ]
# controller = RobotController(
#     maryambot,
#     robot_files,
#     obstacles=[],  # Add the workcell's actual collision obstacles
#     stl_scale=1.0
# )

# env.hold()




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

# The one existing yellow tray is passed to feed_iso below.
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
