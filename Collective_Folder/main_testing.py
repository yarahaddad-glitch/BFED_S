"""Smart MealBot: colour detection, level tray pickup, transfer and release.

Swift-only demonstration. RMRC validates configured joint limits and any
configured external obstacles; it does not validate robot self-collision.
"""

import sys
from pathlib import Path

import numpy as np
import swift
from spatialgeometry import Mesh
from spatialmath import SE3

sys.path.append(str(Path(__file__).resolve().parent.parent))

from Hannah_Work.Class_HannahBot import HannahBot
from Maryam_Work.Maryam_robotclass import MaryamBot
from Maryam_Work.maryam_colour_camera import detect_tray_colour
from Maryam_Work.maryamrobot_controller import RobotController
from Yara_Work.YaraBot_DH import YaraBot
from swift_simulation import create_environment

# ============================================================
# 1. ONE SWIFT ENVIRONMENT
# ============================================================
env = swift.Swift()
env.launch(realtime=True)

groot, green_tray = create_environment(env)

hannahbot = HannahBot()
hannahbot.base = SE3(2, 0, 0)
# hannahbot.add_to_env(env)  # Intentionally disabled.

maryambot = MaryamBot()
maryambot.base = SE3(13.75, 3.5, 0.7) * SE3.Rz(np.pi )
maryambot.add_to_env(env)

yarabot = YaraBot()
yarabot.base = SE3(4, 0, 0)
yarabot.add_to_env(env)

# ============================================================
# 2. KITCHEN GEOMETRY
# ============================================================
assignment_folder = Path(__file__).resolve().parent.parent
enviro_folder = assignment_folder / "Yara_Work" / "EnviroParts"

meal_path = enviro_folder / "meal_area_updated.dae"
if not meal_path.is_file():
    raise FileNotFoundError(
        f"Missing updated meal area: {meal_path}\n"
        "Put meal_area_updated.dae in Yara_Work/EnviroParts."
    )

meal_file = Mesh(str(meal_path))
dining_file = Mesh(str(enviro_folder / "dining_area2.dae"))
isolation_file = Mesh(str(enviro_folder / "isolation_area2.dae"))
nuggets_file = Mesh(str(enviro_folder / "3nuggets.dae"))

for item in (meal_file, dining_file, isolation_file, nuggets_file):
    item.T = SE3().A
    env.add(item)

print("[SCENE] New meal area loaded:", meal_path.name)
print("[SCENE] Verify both tabletops and the target tray height visually.")

# ============================================================
# 3. EXISTING KAWASAKI RMRC CONTROLLER
# ============================================================
robot_files = [
    assignment_folder / "Maryam_Work" / f"RS007N-J{i}.STL"
    for i in range(1, 7)
]
controller = RobotController(
    maryambot,
    robot_files,
    obstacles=[],  # Still no external workcell obstacles configured.
    stl_scale=1.0,
)

print("\n=== KAWASAKI INITIAL DIAGNOSTICS ===")
print("Robot base:", np.round(maryambot.base.t, 3))
print("Home q (degrees):", np.round(np.rad2deg(maryambot.q), 2))
print("Initial controller pose valid:", controller.valid(maryambot.q))
for i, (pose, size) in enumerate(controller.body_poses(maryambot.q), start=1):
    print(f"Link {i}: center={np.round(pose.t, 3)}, size={np.round(size, 3)}")

# ============================================================
# 4. DETECT AND PICK UP THE GREEN TRAY
# ============================================================
colour = detect_tray_colour()
BENCH_TRAY_ORIGIN_XYZ = [13.3, 2.95, 0.65]  # Your latest target, unchanged.

if colour == "GREEN":
    print("GREEN GROOT DETECTED")

    tray_xyz = SE3(green_tray.T).t.copy()
    above = tray_xyz + np.array([0.0, 0.0, 0.18])
    grasp = tray_xyz + np.array([0.0, 0.0, 0.05])

    print("[TARGET] Tray origin:", np.round(tray_xyz, 3))
    print("[TARGET] Pickup approach:", np.round(above, 3))
    print("[TARGET] Bench tray origin:", BENCH_TRAY_ORIGIN_XYZ)

    hold_R = None  # Free orientation for the previously working pickup.
    attached = False
    relative_tray = None

    def step():
        """Update carried tray using the unchanged relative grip transform."""
        if attached:
            green_tray.T = (maryambot.fkine(maryambot.q) * relative_tray).A
        env.step(controller.dt)

    def move_to(xyz, label):
        """Move using the existing controller and current orientation policy."""
        print(label, flush=True)
        result = controller.move(
            np.asarray(xyz, dtype=float),
            step_callback=step,
            hold_rotation=hold_R,
        )
        print(label, "OK" if result else "FAILED", flush=True)
        return result

    if move_to(above, "Approach") and move_to(grasp, "Grasp"):
        ee_at_grasp = maryambot.fkine(maryambot.q)
        tray_at_grasp = SE3(green_tray.T)

        # NO 180-degree spin. Capture the tray exactly as it appears now.
        # The tray starts level in the scene; holding this EE rotation keeps
        # that initial tray orientation throughout the carried movement.
        relative_tray = ee_at_grasp.inv() * tray_at_grasp
        hold_R = ee_at_grasp.R.copy()
        attached = True
        step()

        if move_to(above, "Lift vertically"):
            print("Tray picked up without any deliberate rotation")

            bench = np.asarray(BENCH_TRAY_ORIGIN_XYZ, dtype=float)
            if bench.shape != (3,) or not np.all(np.isfinite(bench)):
                raise ValueError("BENCH_TRAY_ORIGIN_XYZ must be finite XYZ")

            # Since hold_R is fixed, the tray-to-tool XYZ offset is fixed too.
            # Converting tray origin -> EE origin preserves precise placement.
            ee_offset_world = hold_R @ relative_tray.t
            tray_above_bench = bench + np.array([0.0, 0.0, 0.18])
            ee_above_bench = tray_above_bench - ee_offset_world
            ee_place = bench - ee_offset_world

            print("[TRANSFER] Wrist orientation locked from grasp through release")
            print("[TARGET] Above-bench EE XYZ:", np.round(ee_above_bench, 3))
            print("[TARGET] Placement EE XYZ:", np.round(ee_place, 3))

            # Preplan BOTH segments before any carried transfer movement.
            # An impossible fixed-orientation route must not drop the tray.
            q_lifted = maryambot.q.copy()
            print("[TRANSFER] Planning fixed-orientation bench approach...", flush=True)
            approach_path = controller.segment(
                q_lifted, ee_above_bench, hold_rotation=hold_R
            )

            placement_path = None
            if approach_path is not None:
                print("[TRANSFER] Planning vertical placement...", flush=True)
                placement_path = controller.segment(
                    approach_path[-1], ee_place, hold_rotation=hold_R
                )

            if approach_path is None or placement_path is None:
                print(
                    "[TRANSFER] No valid fixed-orientation path to the bench. "
                    "Holding tray; no release."
                )
            else:
                full_transfer = approach_path[1:] + placement_path[1:]
                if not all(controller.valid(q) for q in full_transfer):
                    print("[TRANSFER] Path validation failed; holding tray")
                else:
                    print("[TRANSFER] Carrying tray without wrist rotation...", flush=True)
                    for q in approach_path[1:]:
                        maryambot.q = q.copy()
                        step()
                    print("Above bench OK")

                    print("[TRANSFER] Lowering vertically...", flush=True)
                    for q in placement_path[1:]:
                        maryambot.q = q.copy()
                        step()

                    delivered_pose = maryambot.fkine(maryambot.q) * relative_tray
                    position_error = np.linalg.norm(delivered_pose.t - bench)
                    rotation_error = np.linalg.norm(
                        controller.rotation_error(tray_at_grasp.R, delivered_pose.R)
                    )
                    print("[TRANSFER] Tray origin:", np.round(delivered_pose.t, 3))
                    print("[TRANSFER] Tray position error:", round(float(position_error), 4), "m")
                    print("[TRANSFER] Tray rotation error:", round(float(np.rad2deg(rotation_error)), 3), "deg")

                    # Controller's default final pose tolerances are ~6 mm / 3 deg.
                    if position_error <= 0.015 and rotation_error <= np.deg2rad(3):
                        attached = False  # Leave the tray at this world pose.
                        print("Tray released on receiving bench")

                        # Retreat vertically while STILL maintaining the same
                        # EE orientation. No home return or spin commands.
                        ee_retreat = maryambot.fkine(maryambot.q).t.copy()
                        ee_retreat[2] += 0.18
                        move_to(ee_retreat, "Retreat vertically")
                    else:
                        print("[TRANSFER] Pose tolerance not met; tray stays attached")
        else:
            print("Lift blocked; tray remains attached")

elif colour is None:
    print("Camera cancelled - no movement")
else:
    print(f"Detected {colour}; waiting for GREEN")
