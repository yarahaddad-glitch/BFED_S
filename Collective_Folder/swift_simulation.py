import os
import swift
from spatialgeometry import Mesh
from spatialmath import SE3
from math import pi


def create_environment():

    env = swift.Swift()
    env.launch(realtime=True)

    collective_folder = os.path.dirname(os.path.abspath(__file__))
    assignment_folder = os.path.dirname(collective_folder)

    

    maryam_folder = os.path.join(
        assignment_folder,
        "Maryam_Work"
    )

    prisoner_file = os.path.join(
        maryam_folder,
        "prisoner.stl"
    )

    groot_file = os.path.join(
            maryam_folder,
            "groot.stl"
        )
    drex_file = os.path.join(
            maryam_folder,
            "drex.stl"
        )
    solitary_file = os.path.join(
        maryam_folder,
        "solitary.stl"
    )

    yara_folder = os.path.join(
    assignment_folder,
    "Yara_Work"
    )

    tray_file = os.path.join(
    yara_folder,
    "EnviroParts",
    "Tray1.dae"
    )

    print("Prisoner exists:", os.path.exists(prisoner_file))
    print("groot exists:", os.path.exists(groot_file))
    print("drex exists:", os.path.exists(drex_file))
    print("solitary cage exists:", os.path.exists(solitary_file))
    print("Tray exists:", os.path.exists(tray_file))

    prisoner = Mesh(
        prisoner_file,
        color=(0.8, 0.35, 0.05, 1.0)
    )

    prisoner.T = SE3(1.16, 0.14, 0.02) * SE3.Rz(pi).A
    prisoner.scale = [0.5, 0.5, 0.5]
    env.add(prisoner)

    groot = Mesh(
        groot_file,
        color=(0.1, 0.6, 0.2, 1.0)
    )

    groot.T = SE3(0.1, 0.1, 0).A

    env.add(groot)

    drex = Mesh(
        drex_file,
        color=(0.0, 0.0, 0.35, 1.0)
    )

    drex.T = SE3(0.5, 0.1, 0).A

    env.add(drex)

    solitary = Mesh(
            solitary_file,
            color=(0.15, 0.15, 0.15, 1.0)
        )
    
    solitary.T = SE3(1, 0.16, 0).A
    solitary.scale = [0.1, 0.1, 0.1]
    
    env.add(solitary)

    tray = Mesh(
        tray_file,
        color=(0.8, 0.8, 0.8, 1.0)
     )

    tray.T = SE3(0.3, 0.5, 0.1).A

    env.add(tray)

    return env

if __name__ == "__main__":
    env = create_environment()
    env.hold()