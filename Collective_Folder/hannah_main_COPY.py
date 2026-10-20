import numpy as np
import swift
import sys
from pathlib import Path
from spatialmath import SE3
from spatialgeometry import Mesh, Cylinder
 
# Project folder (the one holding Hannah_Work, Maryam_Work, ...) on the path
sys.path.append(str(Path(__file__).resolve().parent.parent))
 
from EnviroParts_S.Full_sim import create_environment as create_full_sim
 
from Hannah_Work.Feed_Iso import feed_iso
from Maryam_Work.Maryam_robotclass import MaryamBot
from Yara_Work.YaraBot_DH import YaraBot
from Maryam_Work.maryam_colour_camera import detect_tray_colour
from Maryam_Work.maryamrobot_controller import RobotController
 
# ReBot and water bottle demonstration
from Rebot_class import ReBot
from rebot_bottle_demo import (
    get_teach_positions,
    animate_joints,
    tool_pose
)
 
# Create ONE Swift simulation
env = swift.Swift()
env.launch(realtime=True)
 
# The whole building, prisoners and the three yellow trays.
# scene is a dict: name -> Mesh (e.g. scene["yellowTray1"])
scene = create_full_sim(env)
 
# Maryam's Kawasaki
maryambot = MaryamBot()
maryambot.base = SE3(13.86, 3.8, 0.51) * SE3.Rz(np.pi / 2)
maryambot.add_to_env(env)
 
# Yara's robot
yarabot = YaraBot()
yarabot.base = SE3(4, 0, 0)
yarabot.add_to_env(env)
 
# ReBot
rebot = ReBot(env)
rebot.base = SE3(12.5, 3.6, 0.66) * SE3.Rz(np.pi / 2)
rebot.update_meshes()
 
env.step(0.05)
 
# ---------------- Hannah: TurtleBot + HannahBot pick up yellowTray1 ----------------
# feed_iso adds HannahBot and the TurtleBot itself, reads the tray's size from
# its own file (including scale = 2), and avoids every other Full_sim mesh and
# the other robots.
# show_obstacles=True draws the boxes it found - set False once they look right.
# Commands at the prompt: auto, drive, pre, forward, pickup, leave, check, quit
tray = scene["yellowTray1"]
 
hannahbot, turtlebot, success = feed_iso(
    env, tray,
    scene_meshes=list(scene.values()),
    other_robots=[maryambot, yarabot, rebot],
    show_obstacles=True,
)
print("HannahBot feed_iso succeeded:", success)
 
env.hold()