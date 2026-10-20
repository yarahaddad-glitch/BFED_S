"""Smart MealBot main.

Hold a colour card up to the webcam:
  GREEN / BLUE -> the Kawasaki (RMRC) picks the matching tray, sets it on the
                  conveyor belt, and the belt carries it to the middle, closer
                  to the food area where Yara adds the food.
  YELLOW       -> "ISOLATION PRISONER LOOSE" warning, no movement.
  Press Q in the camera window to finish.

Where things live
  Full_sim.py                  scene, robot and tray stacks
  maryam_meal_dispatch.py      camera + colour -> tray decision + yellow alert
  maryam_tray_delivery.py      RMRC pick / place, tray stacks, belt animation
  maryamrobot_controller.py    your RMRC controller (unchanged)
  main_meal_bot.py (this)      wires everything together

Swift-only demonstration. RMRC validates joint limits and configured obstacles;
it does not validate robot self-collision.
"""
import sys
from pathlib import Path

import numpy as np
import swift
from spatialmath import SE3

try:
    from spatialgeometry import Sphere
except ImportError:          # markers are optional
    Sphere = None

ASSIGNMENT_ROOT = Path(__file__).resolve().parents[2]
if str(ASSIGNMENT_ROOT) not in sys.path:
    sys.path.insert(0, str(ASSIGNMENT_ROOT))

from Maryam_Work.maryam_meal_dispatch import ColourScanner, build_default_dispatcher
from Maryam_Work.maryam_tray_delivery import (
    BeltLayout, ConveyorBelt, TrayDelivery, TrayStacks)
from Maryam_Work.maryamrobot_controller import RobotController
from Collective_Folder.EnviroParts_S.Full_sim import (
    TRAY_SCALE, add_maryam_robot, add_tray_stacks,
    configure_swift_server, create_environment)

# ============================================================
# CONFIG  - the numbers you are most likely to change
# ============================================================
# Where the robot sets the tray down, measured from the ROBOT BASE (metres).
# Keep it close to the robot so RMRC can reach it.
PLACE_FROM_ROBOT = (-0.30, -0.45)      # (dx, dy)
BELT_SURFACE_Z = 0.67                  # tray bottom: lowered 3 cm onto the belt
BELT_MIDDLE_X = 14.96                  # destination toward +X, opposite the previous travel
BELT_SPEED = 0.15                      # metres per second

SHOW_MARKERS = False                   # optional red/yellow debug target markers
RETURN_HOME_AFTER_PLACE = False        # True = robot goes back to its start position each cycle


# ============================================================
# 1. SCENE
# ============================================================
def build_scene():
    configure_swift_server()
    env = swift.Swift()
    env.launch(realtime=True)

    objects = create_environment(env)
    maryambot = add_maryam_robot(env, objects)
    trays = add_tray_stacks(env, maryambot)
    objects.update(trays)
    env.step()
    return env, objects, maryambot, trays


def build_controller(maryambot):
    robot_files = [ASSIGNMENT_ROOT / "Maryam_Work" / f"RS007N-J{i}.STL"
                   for i in range(1, 7)]
    return RobotController(
        maryambot, robot_files,
        obstacles=[],            # no external obstacles configured yet
        stl_scale=1.0,
    )


# ============================================================
# 2. CONVEYOR LAYOUT
# ============================================================
def mesh_xyz(objects, name, fallback):
    mesh = objects.get(name)
    return SE3(mesh.T).t if mesh is not None else np.asarray(fallback, dtype=float)


def build_belt_layout(objects, maryambot):
    """Set-down point near the robot, middle point halfway along the belt."""
    end_a = mesh_xyz(objects, "Conv2_1", (14.96, 3.429, 0.58))   # belt end, robot side
    end_b = mesh_xyz(objects, "Conv2_2", (11.92, 3.429, 0.58))   # belt end, far side
    middle_x = BELT_MIDDLE_X if BELT_MIDDLE_X is not None else 0.5 * (end_a[0] + end_b[0])

    base = maryambot.base.t
    place = np.array([base[0] + PLACE_FROM_ROBOT[0],
                      base[1] + PLACE_FROM_ROBOT[1],
                      BELT_SURFACE_Z])
    middle = np.array([middle_x, place[1], BELT_SURFACE_Z])      # same lane as the set-down
    return BeltLayout(place_centre=place, middle_centre=middle, speed=BELT_SPEED)


def add_marker(env, xyz, colour):
    """Small ball so you can SEE a target in Swift while tuning the config."""
    if Sphere is None:
        return
    try:
        ball = Sphere(radius=0.03, color=colour)
        ball.T = SE3(*np.asarray(xyz, dtype=float)).A
        env.add(ball)
    except Exception as exc:     # never let a debug marker stop the demo
        print("[MARKER] could not add marker:", exc)


# ============================================================
# 3. MAIN LOOP
# ============================================================
def main():
    env, objects, maryambot, trays = build_scene()
    controller = build_controller(maryambot)

    layout = build_belt_layout(objects, maryambot)
    print("\n[LAYOUT] Set-down (tray centre):", np.round(layout.place_centre, 3))
    print("[LAYOUT] Belt middle (tray centre):", np.round(layout.middle_centre, 3))
    if SHOW_MARKERS:
        add_marker(env, layout.place_centre, (1.0, 0.0, 0.0, 1.0))
        add_marker(env, layout.middle_centre, (1.0, 0.9, 0.0, 1.0))
        env.step()

    scanner = ColourScanner()
    dispatcher = build_default_dispatcher()
    stacks = TrayStacks(trays)
    delivery = TrayDelivery(env, maryambot, controller, layout,
                            tray_scale=TRAY_SCALE, return_home=RETURN_HOME_AFTER_PLACE)
    belt = ConveyorBelt(env, layout, tray_scale=TRAY_SCALE, dt=controller.dt)

    while True:
        # ---- camera: which colour is being held up? ----
        result = dispatcher.handle(scanner.scan())

        if result.action == "CANCELLED":
            print("Camera cancelled - finished.")
            break
        if result.action == "ALERT":
            print("Yellow detected: no tray sent. Show GREEN or BLUE, or press Q to finish.")
            continue
        if result.action != "SEND_TRAY":
            print(f"[{result.action}] {result.message}")
            continue

        # ---- pick the matching tray ----
        colour = result.order.tray.colour
        pick = stacks.next_tray(colour)
        if pick is None:
            print(f"No {colour} trays left.")
            dispatcher.reset(colour)
            continue
        key, tray_mesh = pick

        dispatcher.mark_on_conveyor(result.order)
        outcome = delivery.deliver(tray_mesh, label=key)
        print(f"[{outcome.status}] {outcome.message}")

        # ---- belt carries it to the middle ----
        if outcome.on_belt:
            stacks.mark_used(key)
            belt.carry_to_middle(tray_mesh)
            dispatcher.mark_delivered(result.order)
            print(f"{colour} tray delivered. {stacks.remaining(colour)} left in that stack.")
        dispatcher.reset(colour)          # allow the next card of this colour

    env.hold()


if __name__ == "__main__":
    main()
