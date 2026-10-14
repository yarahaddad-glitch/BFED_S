
import os
import swift
from spatialgeometry import Mesh
from spatialmath import SE3
from math import pi


def create_environment(env):  # Use the existing Swift simulation

    collective_folder = os.path.dirname(os.path.abspath(__file__))
    assignment_folder = os.path.dirname(collective_folder)

    maryam_folder = os.path.join(assignment_folder, "Maryam_Work")
    yara_folder = os.path.join(assignment_folder, "Yara_Work", "EnviroParts")

    # Mesh file locations
    prisoner_file = os.path.join(maryam_folder, "prisoner.stl")
    groot_file = os.path.join(maryam_folder, "groot.stl")
    drex_file = os.path.join(maryam_folder, "drex.stl")
    solitary_file = os.path.join(maryam_folder, "solitary.stl")
    green_tray_file = os.path.join(yara_folder, "tray_green.dae")

    # Verify mesh files
    print("Prisoner exists:", os.path.exists(prisoner_file))
    print("Groot exists:", os.path.exists(groot_file))
    print("Drex exists:", os.path.exists(drex_file))
    print("Solitary cage exists:", os.path.exists(solitary_file))
    print("Green tray exists:", os.path.exists(green_tray_file))

    # Prisoner
    prisoner = Mesh(prisoner_file, color=(0.8, 0.35, 0.05, 1.0))
    prisoner.T = (SE3(1.16, 0.14, 0.02) * SE3.Rz(pi)).A
    prisoner.scale = [0.5, 0.5, 0.5]
    env.add(prisoner)

    # Groot
    groot = Mesh(groot_file, color=(0.1, 0.6, 0.2, 1.0))
    groot.scale = [20, 20, 20]
    groot.T = SE3(13.3, 6.0, 0.0).A
    env.add(groot)

    # Drex
    drex = Mesh(drex_file, color=(0.0, 0.0, 0.35, 1.0))
    drex.T = SE3(0.5, 0.1, 0.0).A
    env.add(drex)

    # Solitary cage
    solitary = Mesh(solitary_file, color=(0.15, 0.15, 0.15, 1.0))
    solitary.scale = [0.1, 0.1, 0.1]
    solitary.T = SE3(1.0, 0.16, 0.0).A
    env.add(solitary)

    # One green tray beside the Kawasaki
    green_tray = Mesh(green_tray_file)
    green_tray.T = SE3(13.3, 3.7, 0.7).A
    env.add(green_tray)

    return groot, green_tray  # Access both objects from main_testing.py


if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    create_environment(env)
    env.hold()
