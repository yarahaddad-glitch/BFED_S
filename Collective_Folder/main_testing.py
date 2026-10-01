import numpy as np
import swift
import sys
from pathlib import Path
from spatialmath import SE3


sys.path.append(str(Path(__file__).parent.parent))
from Hannah_Work.Class_HannahBot import HannahBot
from Maryam_Work.Maryam_robotclass import MaryamBot

hannahbot = HannahBot()
hannahbot.base = SE3(2, 0, 0)

maryambot = MaryamBot()
maryambot.base = SE3(3, 0, 0)

env = swift.Swift()
env.launch(realtime=True)

hannahbot.add_to_env(env)
maryambot.add_to_env(env)
env.hold()