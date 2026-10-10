import numpy as np
import swift
import sys
from pathlib import Path
from spatialmath import SE3
from spatialgeometry import Mesh


sys.path.append(str(Path(__file__).parent.parent))
from Hannah_Work.Class_HannahBot import HannahBot
from Maryam_Work.Maryam_robotclass import MaryamBot
from Yara_Work.YaraBot_DH import YaraBot


hannahbot = HannahBot()
hannahbot.base = SE3(2, 0, 0)

maryambot = MaryamBot()
maryambot.base = SE3(3, 0, 0)

# Yara's EC66 robot:
yarabot = YaraBot()
yarabot.base = SE3(4, 0, 0)

env = swift.Swift()
env.launch(realtime=True)

hannahbot.add_to_env(env)
maryambot.add_to_env(env)
yarabot.add_to_env(env)


assignment_folder = Path(__file__).resolve().parent.parent

meal_file = assignment_folder / "Yara_Work" / "EnviroParts" / "meal_area.dae"

print("Kitchen exists:", meal_file.exists())

meal = Mesh(str(meal_file))

# Position kitchen at origin
meal.T = SE3(0, 0, 0).A

env.add(meal)

env.hold()