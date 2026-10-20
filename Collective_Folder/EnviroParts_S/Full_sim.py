import os
from pathlib import Path
import socketserver
import sys
from types import SimpleNamespace
import swift
import swift.SwiftRoute as swift_route
from spatialgeometry import Mesh
from spatialmath import SE3
from math import pi

assignment_folder = Path(__file__).resolve().parents[2]
if str(assignment_folder) not in sys.path:
    sys.path.insert(0, str(assignment_folder))

from Maryam_Work.Maryam_robotclass import MaryamBot

ROBOT_BASE_X = 14.5  # Moving marayms robot and trays 
ROBOT_FORWARD_Y = -0.05 # maryamas robot movin forward 

# DO NOT CHANGE THIS IN MARYAM BOTS DIRECTION AT ALL
CONVEYOR_DEPTH_SCALE = 0.56
CONVEYOR_ORIGINAL_FAR_EDGE_Y = 2.53473288
CONVEYOR_FAR_EDGE_Y = 2.91736668


def conveyor_y(original_y):
    return CONVEYOR_FAR_EDGE_Y + (original_y - CONVEYOR_ORIGINAL_FAR_EDGE_Y) * CONVEYOR_DEPTH_SCALE


def add_maryam_robot(env, objects):
    """Mount MaryamBot on the bench beside the front bar."""
    bench_pose = SE3(objects["botCounter"].T)
    # The countertop is 0.6327448 m above the bench mesh origin.
    base_pose = SE3(
        ROBOT_BASE_X,
        bench_pose.t[1] + ROBOT_FORWARD_Y,
        bench_pose.t[2] + 0.6327448,
    )
    robot = MaryamBot(base=base_pose * SE3.Rz(pi))
    robot.add_to_env(env)
    return robot


def add_tray_stacks(env, robot):

    tray_folder = assignment_folder / "Yara_Work" / "EnviroParts"
    centre_x = 0.15
    centre_y = -0.0989120895
    tray_height = 0.018043518
    stack_gap = 0.005
    stacks = {}
    for colour, side in (("green", -1), ("blue", 1)):
        path = tray_folder / f"tray_{colour}.dae"
        if not path.is_file():
            raise FileNotFoundError(f"Missing tray mesh: {path}")
        stack_x = robot.base.t[0] + side * 0.45
        # Keep trays at their existing bench position as the robot moves forward.
        stack_y = robot.base.t[1] - ROBOT_FORWARD_Y - 0.20
        for level in range(2):
            tray = Mesh(path.as_posix())
            tray.T = SE3(
                stack_x - centre_x,
                stack_y - centre_y,
                robot.base.t[2] + 0.002 + level * (tray_height + stack_gap),
            ).A
            env.add(tray)
            stacks[f"{colour}_tray_{level + 1}"] = tray
    return stacks

# DO NOT CHNAGE 
def configure_swift_server():
    """Prevent idle browser connections from blocking mesh downloads."""
    class ThreadedSwiftHTTPServer(socketserver.ThreadingTCPServer):
        daemon_threads = True
    swift_route.socketserver = SimpleNamespace(TCPServer=ThreadedSwiftHTTPServer)


def create_environment(env):

    # FOLDER LOCATIONS

    enviro_folder = os.path.dirname(os.path.abspath(__file__))

    isolation_folder = os.path.join(enviro_folder, "isolation_area")

    inside_folder = os.path.join(isolation_folder, "insideISO")
    outside_folder = os.path.join(isolation_folder, "outsideISO")

    meal_folder = os.path.join(enviro_folder, "meal_area")
    tables_folder = os.path.join(enviro_folder, "tables")
    Extra_folder = os.path.join(enviro_folder, "Extra")

    objects = {}



    def add_mesh(name, folder, filename, x=0, y=0, z=0,
                 rz=0, scale=1):

        file_path = os.path.join(folder, filename)

        if not os.path.exists(file_path):
            print(f"Missing file: {file_path}")
            return None

        # Swift sends this filename to its browser's mesh loader. Forward
        # slashes preserve the directory when the loader resolves asset URLs.
        mesh = Mesh(Path(file_path).resolve().as_posix())

        mesh.scale = [scale, scale, scale] if isinstance(scale, (int, float)) else scale

        mesh.T = (
            SE3(x, y, z) *
            SE3.Rz(rz)
        ).A

        print(f"Loading: {name}", flush=True)
        try:
            env.add(mesh)
        except TimeoutError as exc:
            raise RuntimeError(
                f"Swift could not load {name} from {mesh.filename}. "
                "Keep the Swift browser tab open and connected. "
                "Check its browser console (F12) for failed mesh requests."
            ) from exc

        objects[name] = mesh

        print(f"Loaded: {name}")

        return mesh

    # INSIDE ISOLATION AREA

    ISO_cell1 = add_mesh(
        "ISO_cell1", inside_folder, "ISO_cell.dae",
        x=2.7, y=9.37, z=0
    )

    ISO_door1 = add_mesh(
        "ISO_door1", inside_folder, "ISO_door.dae",
        x=4.6, y=10.6, z=0
    )

    ISO_cell2 = add_mesh(
        "ISO_cell2", inside_folder, "ISO_cell.dae",
        x=2.7, y=14.37, z=0
    )

    ISO_door2 = add_mesh(
        "ISO_door2", inside_folder, "ISO_door.dae",
        x=4.6, y=15.6, z=0
    )

    ISO_cell3 = add_mesh(
        "ISO_cell3", inside_folder, "ISO_cell.dae",
        x=2.7, y=19.4, z=0
    )

    ISO_door3 = add_mesh(
        "ISO_door3", inside_folder, "ISO_door.dae",
        x=4.6, y=20.6, z=0
    )

    # OUTSIDE ISOLATION AREA

    backWall = add_mesh(
        "backWall", outside_folder, "backWall.dae",
        x=4.8, y=1.267, z=0
    )

    backWall2 = add_mesh(
        "backWall2", outside_folder, "backWall2.dae",
        x=4.8, y=24.2, z=0
    )

    LeftWall = add_mesh(
        "LeftWall", outside_folder, "LeftWall.dae",
        x=9.4, y=14.5, z=0
    )

    rightWall = add_mesh(
        "rightWall", outside_folder, "rightWall.dae",
        x=0.32, y=12.7, z=0
    )

    # MEAL AREA

    backWall_meal = add_mesh(
        "backWall_meal", meal_folder, "bakcWall.dae",
        x=14.9, y=1.267, z=-0.01
    )

    botCounter = add_mesh(
        "botCounter", meal_folder, "botCounter.dae",
        x=13, y=4.3, z=-0.004
    )

    Conv1_1 = add_mesh(
        "Conv1_1", meal_folder, "Conv1.dae",
        x=13.45, y=conveyor_y(2.75), z=0.68,
        scale=[1, CONVEYOR_DEPTH_SCALE, 1]
    )

    Conv1_2 = add_mesh(
        "Conv1_2", meal_folder, "Conv1.dae",
        x=13.45, y=conveyor_y(3.888), z=0.68,
        scale=[1, CONVEYOR_DEPTH_SCALE, 1]
    )

    Conv2_1 = add_mesh(
        "Conv2_1", meal_folder, "Conv2.dae",
        x=14.96,
        y=conveyor_y(3.429), z=0.68, scale=[1, CONVEYOR_DEPTH_SCALE, 1]
    )

    Conv2_2 = add_mesh(
        "Conv2_2", meal_folder, "Conv2.dae",
        x=11.93,
        y=conveyor_y(3.429), z=0.68, scale=[1, CONVEYOR_DEPTH_SCALE, 1]
    )

    ConvCounter = add_mesh(
        "ConvCounter", meal_folder, "ConvCounter.dae",
        x=13.001,
        y=conveyor_y(3.3), z=0.025, scale=[1, CONVEYOR_DEPTH_SCALE, 1]
    )

    Door = add_mesh(
        "Door", meal_folder, "Door.dae",
        x=15.95, y= 2, z=0
    )

    ExtraCounter = add_mesh(
        "ExtraCounter", meal_folder, "ExtraCounter.dae",
        x=15.72, y=3.68, z=0.01
    )

    frontBar2 = add_mesh(
        "frontBar2", meal_folder, "frontBar2.dae",
        x=15.39, y=4.6, z=0.5
    )

    frontBars = add_mesh(
        "frontBars", meal_folder, "frontBars.dae",
        x=12.15, y=4.6, z=0.5
    )

    HannahCounter = add_mesh(
        "HannahCounter", meal_folder, "HannahCounter.dae",
        x= 10.147, y= 2.7, z=0.02
    )

    kitchen = add_mesh(
        "kitchen", meal_folder, "kitchen.dae",
        x=13.2, y=1.4, z=0
    )

    ledtBars = add_mesh(
        "ledtBars", meal_folder, "ledtBars.dae",
        x=16.02, y=3.55, z=0.67
    )

    OverHead = add_mesh(
        "OverHead", meal_folder, "OverHead.dae",
        x=14.2, y=3.9, z=1.75
    )

    #Tables

    tables = add_mesh(
        "tables", tables_folder, "eating.dae",
        x=0, y=0, z=0
    )

    #Extra 

    extra = add_mesh(
        "extra", Extra_folder, "Yard.dae",
        x= 0, y=0, z=0
    )

    return objects


if __name__ == "__main__":

    configure_swift_server()
    env = swift.Swift()
    env.launch(realtime=True)

    objects = create_environment(env)
    maryambot = add_maryam_robot(env, objects)
    trays = add_tray_stacks(env, maryambot)
    objects.update(trays)
    env.step()

    env.hold()
