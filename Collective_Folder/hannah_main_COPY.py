import numpy as np
import swift
import sys
from pathlib import Path
from spatialmath import SE3
from spatialgeometry import Mesh, Cylinder

# Project folder on the path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from EnviroParts_S.Full_sim import create_environment as create_full_sim

#saftey features
from Collective_Folder.Prison_break import PrisonBreak
from Light_curtain import SafetyLightCurtain
from BFED_S.Hannah_Work.Prisoner_movement import walk_all_prisoners

#Hannah
from Hannah_Work.wall_collision import WallCollision
from Hannah_Work.Feed_Iso import feed_iso

#Maryam
from Maryam_Work.Maryam_robotclass import MaryamBot
from Maryam_Work.maryamrobot_controller import RobotController

#Yara
from Yara_Work.YaraBot_DH import YaraBot

# ReBot and water bottle demonstration
from Rebot_class import ReBot
from rebot_bottle_demo import (
    get_teach_positions,
    animate_joints,
    tool_pose
)

# Creates one Swift simulation
env = swift.Swift()
env.launch(realtime=True)

# Whole simulation 
scene = create_full_sim(env)

#Hannah Collision 
walls = WallCollision(env, scene)
walls.watch_trays([scene["yellowTray1"], scene["yellowTray2"], scene["yellowtray3"]])
walls.test_ball_in_path()   
walls.arm()


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

walk_all_prisoners(env, scene)

#Feed Iso prisoners 
hannahbot, turtlebot, success = feed_iso(env, [scene["yellowTray1"],
                                               scene["yellowTray2"],
                                               scene["yellowtray3"]], walls=walls)
print("HannahBot feed_iso succeeded:", success)

env.hold()