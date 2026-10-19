import os
import swift
from spatialgeometry import Mesh
from spatialmath import SE3
from math import pi


def create_environment(env):

    # FOLDER LOCATIONS

    enviro_folder = os.path.dirname(os.path.abspath(__file__))

    isolation_folder = os.path.join(enviro_folder, "isolation_area")

    inside_folder = os.path.join(isolation_folder, "insideISO")
    outside_folder = os.path.join(isolation_folder, "outsideISO")

    meal_folder = os.path.join(enviro_folder, "meal_area")
    tables_folder = os.path.join(enviro_folder, "tables")
    Extra_folder = os.path.join(enviro_folder, "Extra")
    prisoners_folder = os.path.join(enviro_folder, "Prisoners")

    objects = {}



    def add_mesh(name, folder, filename, x=0, y=0, z=0,
                 rz=0, scale=1):

        file_path = os.path.join(folder, filename)

        if not os.path.exists(file_path):
            print(f"Missing file: {file_path}")
            return None

        mesh = Mesh(file_path)

        mesh.scale = [scale, scale, scale]

        mesh.T = (
            SE3(x, y, z) *
            SE3.Rz(rz)
        ).A

        env.add(mesh)

        objects[name] = mesh


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
        x=4.8, y=24.3, z=0
    )

    LeftWall = add_mesh(
        "LeftWall", outside_folder, "LeftWall.dae",
        x=9.4, y=14.5, z=0
    )

    rightWall = add_mesh(
        "rightWall", outside_folder, "rightWall.dae",
        x=0.27, y=12.8, z=0
    )

    # MEAL AREA

    backWall_meal = add_mesh(
        "backWall_meal", meal_folder, "bakcWall.dae",
        x=14.9, y=1.267, z=-0.06
    )

    botCounter = add_mesh(
        "botCounter", meal_folder, "botCounter.dae",
        x=12.78, y=4.47, z=-0.004
    )

    Conv1_1 = add_mesh(
        "Conv1_1", meal_folder, "Conv1.dae",
        x=13.45, y=2.75, z=0.61
    )

    Conv1_2 = add_mesh(
        "Conv1_2", meal_folder, "Conv1.dae",
        x=13.45, y=3.888, z=0.61
    )

    Conv2_1 = add_mesh(
        "Conv2_1", meal_folder, "Conv2.dae",
        x=14.96, y=3.429, z=0.58
    )

    Conv2_2 = add_mesh(
        "Conv2_2", meal_folder, "Conv2.dae",
        x=11.92, y=3.429, z=0.58
    )

    ConvCounter = add_mesh(
        "ConvCounter", meal_folder, "ConvCounter.dae",
        x=13.05, y=3.3, z=0.025
    )

    Door = add_mesh(
        "Door", meal_folder, "Door.dae",
        x=15.95, y= 2.05, z=-0.01
    )

    ExtraCounter = add_mesh(
        "ExtraCounter", meal_folder, "ExtraCounter.dae",
        x=15.76, y=3.68, z=0.02
    )

    frontBar2 = add_mesh(
        "frontBar2", meal_folder, "frontBar2.dae",
        x=10, y=4.65, z=0.5
    )

    frontBars = add_mesh(
        "frontBars", meal_folder, "frontBars.dae",
        x=13.4, y=4.65, z=0.5
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
        x=16.02, y=3.55, z=0.6
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

    #add prisoners 
    Iso_prisoner1 = add_mesh(
        "Iso_prisoner1", prisoners_folder, "ISO_prisoner.dae",
        x = 3, y = 10.7 , z = 0
    )

    Iso_prisoner2 = add_mesh(
        "Iso_prisoner2", prisoners_folder, "drex.stl",
        x = 5, y = 19, z=-0.03,
        rz=pi/2,
        scale = 13
    )

    prisoner1 = add_mesh(
        "prisoner1", prisoners_folder, "blueFoodPrisoner.dae",
        x = 0, y=0, z = 0
        )

    prisoner2 = add_mesh(
        "prisoner2", prisoners_folder, "blueFoodPrisoner2.dae",
        x = 3, y=0, z=0
    )

    return objects


if __name__ == "__main__":

    env = swift.Swift()
    env.launch(realtime=True)

    objects = create_environment(env)

    env.hold()
