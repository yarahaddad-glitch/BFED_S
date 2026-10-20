"""Smart MealBot: pick up a PRE-LOADED green tray and return it to its start.

Scene-only Swift demo. No webcam, food-loading animation, or teammate robot motion.
The food starts on the tray at the receiving bench. Only Kawasaki moves.
Trajectory checking uses the current RobotController (joint limits; no
external obstacles configured and no verified self-collision protection).
"""

import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import swift
from spatialgeometry import Mesh
from spatialmath import SE3

# Running from Assignment 2/Maryam_Work, not Collective_Folder.
assignment_folder = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(assignment_folder))
sys.path.insert(0, str(assignment_folder / "Collective_Folder"))

from Maryam_Work.Maryam_robotclass import MaryamBot
from Maryam_Work.maryamrobot_controller import RobotController
from Collective_Folder.swift_simulation import create_environment

# ============================================================
# 1. OPEN YOUR EXISTING SCENE AND KAWASAKI
# ============================================================
env = swift.Swift()
env.launch(realtime=True)

# This returns the tray at its ORIGINAL position beside Kawasaki.
# Save that pose before relocating the tray to the bench.
groot, green_tray = create_environment(env)
original_tray_pose = SE3(green_tray.T)

maryambot = MaryamBot()
maryambot.base = SE3(13.75, 3.5, 0.7) * SE3.Rz(np.pi)
maryambot.add_to_env(env)

enviro_folder = assignment_folder / "Yara_Work" / "EnviroParts"

# Use your existing .dae scene files. No new scene is generated.
meal_path = enviro_folder / "meal_area_updated.dae"
if not meal_path.is_file():
    raise FileNotFoundError(f"Missing scene: {meal_path}")

meal_file = Mesh(str(meal_path))
dining_file = Mesh(str(enviro_folder / "dining_area2.dae"))
isolation_file = Mesh(str(enviro_folder / "isolation_area2.dae"))
nuggets_path = enviro_folder / "3nuggets.dae"
nuggets_file = Mesh(str(nuggets_path))

# Food must NOT stay at the original (0,0,0) scene origin.
for item in (meal_file, dining_file, isolation_file):
    item.T = SE3().A
    env.add(item)
print("[SCENE] Loaded existing scene:", meal_path.name)

# ============================================================
# 2. START WITH NUGGETS ALREADY ON THE GREEN TRAY AT THE BENCH
# ============================================================
BENCH_TRAY_ORIGIN_XYZ = [13.3, 2.95, 0.65]  # Your specified bench position.
NUGGETS_CLEARANCE_Z = 0.001  # Nugget bottoms 1.5 cm above tray origin; adjust if needed.
NUGGETS_XY_NUDGE = (0.1, -0.1)  # Fine-tune if the tray's visible centre is offset.


def nugget_mesh_offset(dae_path):
    """Place nuggets using the *transformed* mesh bounds in 3nuggets.dae.

    The DAE has 3 nuggets with scene-node matrices scaling their vertices
    by 0.0002 and shifting them to around (11.45, 1.45, 0.35) metres.
    Reading untransformed vertices (about +/-180) gives an incorrect offset.
    This applies each node matrix before computing the visible mesh bounds.
    No .dae mesh is modified.
    """
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(dae_path).getroot()

    # Geometry vertex coordinates are in mesh-local coordinates.
    geometry_vertices = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", ns):
        vertices = []
        for source in geometry.findall("c:mesh/c:source", ns):
            if "position" not in source.get("id", "").lower():
                continue
            array = source.find("c:float_array", ns)
            if array is None or not array.text:
                continue
            values = np.fromstring(array.text, sep=" ")
            if values.size and values.size % 3 == 0:
                vertices.append(values.reshape(-1, 3))
        if vertices:
            geometry_vertices["#" + geometry.get("id")] = np.vstack(vertices)

    all_points = []
    for node in root.findall(".//c:visual_scene//c:node", ns):
        instance = node.find("c:instance_geometry", ns)
        if instance is None:
            continue
        local = geometry_vertices.get(instance.get("url"))
        if local is None:
            continue

        matrix_element = node.find("c:matrix", ns)
        if matrix_element is None:
            matrix = np.eye(4)
        else:
            values = np.fromstring(matrix_element.text or "", sep=" ")
            if values.size != 16:
                raise ValueError("Invalid COLLADA matrix in 3nuggets.dae")
            # Blender-exported file has translation in column 4 as written.
            matrix = values.reshape(4, 4)

        transformed = local @ matrix[:3, :3].T + matrix[:3, 3]
        all_points.append(transformed)

    if not all_points:
        raise ValueError("Couldn't read nugget scene-node geometry")

    world_geometry = np.vstack(all_points)
    bounds_min = world_geometry.min(axis=0)
    bounds_max = world_geometry.max(axis=0)
    visible_center_xy = 0.5 * (bounds_min[:2] + bounds_max[:2])

    # Mesh.T is an additional transform on top of the DAE's internal nodes.
    # Move the visible nugget centre to the tray origin, not mesh origin.
    offset = np.array([
        -visible_center_xy[0] + NUGGETS_XY_NUDGE[0],
        -visible_center_xy[1] + NUGGETS_XY_NUDGE[1],
        -bounds_min[2] + NUGGETS_CLEARANCE_Z,
    ])
    print("[FOOD] Nugget transformed bounds:",
          np.round(bounds_min, 4), "to", np.round(bounds_max, 4))
    print("[FOOD] Corrected DAE mesh offset:", np.round(offset, 4))
    return SE3(*offset)


LIFT_HEIGHT = 0.18
POSITION_TOL = 0.015
ROTATION_TOL_RAD = np.deg2rad(3)

bench = np.asarray(BENCH_TRAY_ORIGIN_XYZ, dtype=float)
if bench.shape != (3,) or not np.all(np.isfinite(bench)):
    raise ValueError("BENCH_TRAY_ORIGIN_XYZ must be a finite XYZ vector")

# Keep the tray's original world orientation; change only its location.
bench_tray_pose = SE3.Rt(original_tray_pose.R, bench)
green_tray.T = bench_tray_pose.A

# Fixed relative pose: the nuggets travel WITH the tray at every frame.
tray_to_nuggets = nugget_mesh_offset(nuggets_path)
nuggets_file.T = (bench_tray_pose * tray_to_nuggets).A
env.add(nuggets_file)

destination = original_tray_pose.t.copy()
print("[TARGET] Loaded tray on bench:", np.round(bench, 3))
print("[TARGET] Return tray beside robot:", np.round(destination, 3))
print("[FOOD] Nuggets already on tray (no food-loading animation)")

# ============================================================
# 3. EXISTING RMRC CONTROLLER; NO EXTRA ROBOTS
# ============================================================
robot_files = [
    assignment_folder / "Maryam_Work" / f"RS007N-J{i}.STL"
    for i in range(1, 7)
]
controller = RobotController(
    maryambot,
    robot_files,
    obstacles=[],  # Same configuration as your existing main.
    stl_scale=1.0,
)

hold_R = None  # Preserve free wrist orientation for approach and grasp.
attached = False
relative_tray = None


def step():
    """Keep tray and nuggets together while simulating Kawasaki movement."""
    if attached:
        green_tray.T = (maryambot.fkine(maryambot.q) * relative_tray).A
    nuggets_file.T = (SE3(green_tray.T) * tray_to_nuggets).A
    env.step(controller.dt)


def move_to(xyz, label):
    print(label, flush=True)
    result = controller.move(
        np.asarray(xyz, dtype=float),
        step_callback=step,
        hold_rotation=hold_R,
    )
    print(label, "OK" if result else "FAILED", flush=True)
    return result


# ============================================================
# 4. PICK UP THE LOADED TRAY FROM THE BENCH
# ============================================================
# Same pickup heights and grip-transform logic as your working main.
above = bench + np.array([0.0, 0.0, LIFT_HEIGHT])
grasp = bench + np.array([0.0, 0.0, 0.05])
step()

# Scene is fully loaded, green tray and nuggets already at the bench.
# NOTHING moves until you switch to the Swift window and press Enter.
print("\n[READY] Look at the Swift scene: nuggets should be on the green tray.")
input("[READY] Press ENTER, then switch to the Swift browser... ")
print("[READY] Starting movement in 8 seconds. Switch to Swift now!", flush=True)
time.sleep(8)

if move_to(above, "Approach loaded tray") and move_to(grasp, "Grasp loaded tray"):
    ee_at_grasp = maryambot.fkine(maryambot.q)
    tray_at_grasp = SE3(green_tray.T)

    # Do NOT rotate the tray by 180 degrees. Preserve its scene orientation.
    relative_tray = ee_at_grasp.inv() * tray_at_grasp
    hold_R = ee_at_grasp.R.copy()
    attached = True
    step()

    if move_to(above, "Lift loaded tray vertically"):
        print("[PICKUP] Tray and nuggets lifted together")

        # Fixed tool orientation = constant world offset to the tray origin.
        ee_offset_world = hold_R @ relative_tray.t
        tray_above_destination = destination + np.array([0.0, 0.0, LIFT_HEIGHT])
        ee_above_destination = tray_above_destination - ee_offset_world
        ee_place = destination - ee_offset_world

        # Preplan both segments BEFORE moving the loaded tray away.
        # A failed path leaves the tray in the gripper; it is never teleported.
        print("[RETURN] Planning return above original tray position...", flush=True)
        approach_path = controller.segment(
            maryambot.q.copy(), ee_above_destination, hold_rotation=hold_R
        )
        placement_path = None
        if approach_path is not None:
            print("[RETURN] Planning vertical placement...", flush=True)
            placement_path = controller.segment(
                approach_path[-1], ee_place, hold_rotation=hold_R
            )

        if approach_path is None or placement_path is None:
            print("[RETURN] No valid fixed-orientation route; loaded tray stays attached")
        elif not all(controller.valid(q) for q in approach_path[1:] + placement_path[1:]):
            print("[RETURN] Joint/obstacle validation failed; loaded tray stays attached")
        else:
            print("[RETURN] Carrying loaded tray to original position...", flush=True)
            for q in approach_path[1:]:
                maryambot.q = q.copy()
                step()
            print("[RETURN] Above original position OK")

            print("[RETURN] Lowering loaded tray...", flush=True)
            for q in placement_path[1:]:
                maryambot.q = q.copy()
                step()

            delivered_pose = maryambot.fkine(maryambot.q) * relative_tray
            position_error = np.linalg.norm(delivered_pose.t - destination)
            rotation_error = np.linalg.norm(
                controller.rotation_error(bench_tray_pose.R, delivered_pose.R)
            )
            print("[RETURN] Delivered tray origin:", np.round(delivered_pose.t, 3))
            print("[RETURN] Position error:", round(float(position_error), 4), "m")
            print("[RETURN] Rotation error:",
                  round(float(np.rad2deg(rotation_error)), 3), "degrees")

            if position_error <= POSITION_TOL and rotation_error <= ROTATION_TOL_RAD:
                attached = False
                step()  # Tray and food now stay at the returned position.
                print("[RETURN] Loaded tray released at ORIGINAL position beside robot")
                ee_retreat = maryambot.fkine(maryambot.q).t.copy()
                ee_retreat[2] += LIFT_HEIGHT
                move_to(ee_retreat, "Retreat vertically")
            else:
                print("[RETURN] Placement tolerance not met; tray stays attached")
    else:
        print("[PICKUP] Lift failed; loaded tray stays attached")
else:
    print("[PICKUP] Could not reach loaded tray at bench")

print("[SCENE] Demo finished; Swift scene remains available for inspection")
env.hold()

