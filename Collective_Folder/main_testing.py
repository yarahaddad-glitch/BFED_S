
import numpy as np
import swift
import sys
from pathlib import Path
from spatialmath import SE3
from spatialgeometry import Mesh
sys.path.append(str(Path(__file__).resolve().parent.parent))

from Hannah_Work.Class_HannahBot import HannahBot
from Maryam_Work.Maryam_robotclass import MaryamBot
from Yara_Work.YaraBot_DH import YaraBot
from swift_simulation import create_environment
from Maryam_Work.maryam_colour_camera import detect_tray_colour
from Maryam_Work.maryamrobot_controller import RobotController
# Create ONE Swift simulation
env = swift.Swift()
env.launch(realtime=True)
# Load Groot, prisoner, Drex, cage and ONE green tray
groot, green_tray = create_environment(env)
# Hannah's robot
hannahbot = HannahBot()
hannahbot.base = SE3(2, 0, 0)
hannahbot.add_to_env(env)
# Maryam's Kawasaki
maryambot = MaryamBot()
maryambot.base = SE3(13.86, 3.8, 0.51) * SE3.Rz(np.pi / 2)
maryambot.add_to_env(env)
# Yara's robot
yarabot = YaraBot()
yarabot.base = SE3(4, 0, 0)
yarabot.add_to_env(env)


# Load kitchen environment
assignment_folder = Path(__file__).resolve().parent.parent
enviro_folder = assignment_folder / "Yara_Work" / "EnviroParts"

meal_file = Mesh(str(enviro_folder / "meal_area2.dae"))
dining_file = Mesh(str(enviro_folder / "dining_area2.dae"))
isolation_file = Mesh(str(enviro_folder / "isolation_area2.dae"))
nuggets_file = Mesh(str(enviro_folder / "3nuggets.dae"))

isolation_file.T = SE3(0, 0, 0).A
meal_file.T = SE3(0, 0, 0).A
dining_file.T = SE3(0, 0, 0).A
nuggets_file.T = SE3(0, 0, 0).A

env.add(isolation_file)
env.add(dining_file)
env.add(meal_file)
env.add(nuggets_file)

# Set up Kawasaki RMRC and STL collision geometry
robot_files = [
    assignment_folder / "Maryam_Work" / f"RS007N-J{i}.STL"
    for i in range(1, 7)
]
controller = RobotController(
    maryambot,
    robot_files,
    obstacles=[],  # Add the workcell's actual collision obstacles
    stl_scale=1.0
)
# Open laptop webcam and identify colour
colour = detect_tray_colour()
if colour == "GREEN":
    print("GREEN GROOT DETECTED!")
    print("Kawasaki selecting the green tray")
    # Get tray position in world coordinates
    tray_xyz = np.asarray(green_tray.T[:3, 3], dtype=float)
    # Temporary pickup positions: adjust to the actual tray geometry
    above = tray_xyz + np.array([0.0, 0.0, 0.18])
    grasp = tray_xyz + np.array([0.0, 0.0, 0.05])
    lift = above.copy()
    attached = False
    relative_tray = None
    def step():
        # Move the tray with the end effector after grasping
        if attached:
            green_tray.T = (maryambot.fkine(maryambot.q) * relative_tray).A
        env.step(controller.dt)
    # Approach the tray
    if controller.move(above, step_callback=step):
        print("Above tray")
        # Lower toward the tray
        if controller.move(grasp, step_callback=step):
            print("Grasp position reached")
            # Simulate gripping the tray
            relative_tray = maryambot.fkine(maryambot.q).inv() * SE3(green_tray.T)
            attached = True
            # Lift the tray using RMRC
            if controller.move(lift, step_callback=step):
                print("GREEN TRAY LIFTED!")
            else:
                print("Lift failed - holding current position")
        else:
            print("Could not reach grasp position")
    else:
        print("Could not reach the tray")
elif colour is None:
    print("Camera cancelled - no robot movement")
else:
    print(f"Detected {colour}, waiting for GREEN")

env.hold()