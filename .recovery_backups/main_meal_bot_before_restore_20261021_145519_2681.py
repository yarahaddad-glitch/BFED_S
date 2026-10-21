# main for maryam movement and meal delivery demo
import sys
from pathlib import Path
import numpy as np
import swift
from spatialmath import SE3
from spatialgeometry import Cuboid
try:
    from spatialgeometry import Sphere
except ImportError:          # markers are optional
    Sphere = None
ASSIGNMENT_ROOT = Path(__file__).resolve().parents[2]
if str(ASSIGNMENT_ROOT) not in sys.path:
    sys.path.insert(0, str(ASSIGNMENT_ROOT))
from Maryam_Work.maryam_meal_dispatch import ColourScanner, build_default_dispatcher
from Maryam_Work.maryam_tray_delivery import (
    BeltLayout, TrayDelivery, TrayStacks, TRAY_CENTRE_UNSCALED)
from Maryam_Work.maryamrobot_controller import RobotController
from Collective_Folder.EnviroParts_S.Full_sim import (
    TRAY_SCALE, add_maryam_robot, add_tray_stacks,
    configure_swift_server, create_environment)
PLACE_FROM_ROBOT = (-0.30, -0.45)      # (dx, dy)
BELT_SURFACE_Z = 0.67                  # tray bottom: lowered 3 cm onto the belt
BELT_MIDDLE_X = 14.96                  # destination toward +X, opposite the previous travel
BELT_SPEED = 0.45                      # metres per second (3x original speed)
BELT_FAR_LANE_Y = 2.75                 # centre of the opposite straight belt

HANDOUT_GAP_LOCAL = (0.82485, 0.0, 0.145)  # bottom 18 mm above bench (Z 0.6267)
HANDOUT_SIDE_CLEARANCE = 0.28          # tray centre distance each side of bars
HANDOUT_INSERTION = -0.12             # front rim protrudes; wrist stays well inside
LOADED_TRAY_HEIGHT = 0.07             # low food props fit above bench, below opening
BAR_ROBOT_LIMIT_Y = 4.64              # all robot link boxes stay inside the bars
ROUTE_CACHE_DIR = ASSIGNMENT_ROOT / ".mealbot_routes"
HANDOUT_PAUSE = 2.0                    # seconds before the recipient takes it
ROBOT_ANIMATION_SPEED = 3.0            # faster Swift playback; keep all planned samples
SHOW_MARKERS = False                   # optional red/yellow debug target markers
RETURN_HOME_AFTER_PLACE = False        # True = robot goes back to its start position each cycle

# Scene
BELT_PROP_X = (11.5, 15.5)        # Conv2 belt span, robot side to far side
BELT_PROP_Y = (2.5, 3.7)          # both lanes, near and far
BELT_PROP_Z = (0.55, 0.85)        # sitting on the belt surface
BELT_TRAY_NAME = "bluetray"       # confirmed by the first run's print
REMOVE_BELT_TRAY = True           # False = leave the blue tray in place

def _ghost():
    """Invisible placeholder so swift_objects keeps its length and indices."""
    ghost = (Sphere(radius=0.001, color=(1.0, 1.0, 1.0, 1.0))
             if Sphere is not None
             else Cuboid(scale=[0.001, 0.001, 0.001], color=(1.0, 1.0, 1.0, 1.0)))
    ghost.T = SE3(-50, -50, -50).A
    ghost._changed = False        # never fire a shape_update for it
    return ghost

def swift_remove(env, obj):
    """env.remove() writes None into env.swift_objects, and env.step() then
    calls _propogate_scene_tree() on that None and crashes. Park a ghost in the
    hole so the remaining indices - which are the Swift object ids - do not
    shift underneath the renderer."""
    holes_before = {i for i, ob in enumerate(env.swift_objects) if ob is None}
    env.remove(obj)
    for i, ob in enumerate(env.swift_objects):
        if ob is None and i not in holes_before:
            env.swift_objects[i] = _ghost()

def belt_props(objects):
    """Static meshes parked on the conveyor belt (the blue tray)."""
    parked = []
    for name, mesh in objects.items():
        if any(tag in name.lower() for tag in ("conv", "belt", "frame", "rail")):
            continue
        xyz = SE3(mesh.T).t
        if (BELT_PROP_X[0] < xyz[0] < BELT_PROP_X[1]
                and BELT_PROP_Y[0] < xyz[1] < BELT_PROP_Y[1]
                and BELT_PROP_Z[0] < xyz[2] < BELT_PROP_Z[1]):
            parked.append((name, xyz))
    return parked

def remove_belt_tray(env, objects):
    """Delete the blue tray parked midway along the belt before the demo starts."""
    if not REMOVE_BELT_TRAY:
        return
    if BELT_TRAY_NAME is not None:
        targets = [(BELT_TRAY_NAME, SE3(objects[BELT_TRAY_NAME].T).t)]
    else:
        targets = belt_props(objects)
    if not targets:
        print("[SCENE] No belt tray matched. Scene objects:",
              sorted(objects.keys()), flush=True)
        return
    for name, xyz in targets:
        swift_remove(env, objects.pop(name))
        print(f"[SCENE] Removed '{name}' parked at {np.round(xyz, 3)}", flush=True)

def build_scene():
    configure_swift_server()
    env = swift.Swift()
    env.launch(realtime=True)
    objects = create_environment(env)
    remove_belt_tray(env, objects)          # <- drop the blue tray off the belt
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
        obstacles=[],            # bar-plane guard below; full scene collision is separate
        stl_scale=1.0,
        barrier_y=BAR_ROBOT_LIMIT_Y,
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

def carry_full_loop(env, objects, tray, layout, dt, colour):
    """Follow all four belt sections, returning to the exact pickup pose."""
    if layout.speed <= 0 or dt <= 0:
        raise ValueError("Belt speed and animation timestep must be positive")
    initial = SE3(tray.T)
    local_centre = TRAY_CENTRE_UNSCALED * TRAY_SCALE
    start = initial.t + initial.R @ local_centre
    left = mesh_xyz(objects, "Conv2_2", (11.92, 3.429, 0.58))[0]
    right = mesh_xyz(objects, "Conv2_1", (14.96, 3.429, 0.58))[0]
    near_y, far_y = layout.place_centre[1], BELT_FAR_LANE_Y
    if near_y <= far_y or not left < start[0] < right:
        raise ValueError("Set-down point must lie between the conveyor ends")
    # Conv2 meshes are straight cross-belts, not semicircular end caps.
    points = [np.array([x, y, start[2]]) for x, y in (
        (right, near_y), (right, far_y), (left, far_y), (left, near_y))]
    points.append(start)
    total_distance = sum(np.linalg.norm(b - a)
                         for a, b in zip([start] + points[:-1], points))
    travelled = 0.0
    payload = []
    current = start.copy()
    print("[BELT] Full lap: near lane, far lane, return to pickup.", flush=True)
    for target in points:
        distance = float(np.linalg.norm(target - current))
        count = max(1, int(np.ceil(distance / (layout.speed * dt))))
        for k in range(1, count + 1):
            centre = current + (target - current) * k / count
            tray.T = SE3.Rt(initial.R, centre - initial.R @ local_centre).A
            if not payload and travelled + distance * k / count >= total_distance / 2:
                print("[MEAL] Halfway around: adding food and paper towel.", flush=True)
                payload = add_placeholder_meal(env, tray, colour, dt)
            for shape, relative in payload:
                shape.T = (SE3(tray.T) * relative).A
            env.step(dt)
        travelled += distance
        current = target
    tray.T = initial.A
    for shape, relative in payload:
        shape.T = (initial * relative).A
    return payload

def add_placeholder_meal(env, tray, colour, dt):
    """Temporary colour-specific meal and folded towel; replace with teammate assets."""
    centre = TRAY_CENTRE_UNSCALED * TRAY_SCALE
    food_colour = ((0.20, 0.65, 0.12, 1.0) if colour.lower() == "green"
                   else (0.65, 0.30, 0.10, 1.0))
    props = []
    towel = Cuboid(scale=[0.13, 0.15, 0.006], color=(0.98, 0.98, 0.94, 1))
    props.append((towel, SE3(*(centre + [0.17, 0, 0.045]))))
    for x, y in ((-0.12, -0.055), (-0.12, 0.055), (-0.025, 0)):
        food = Cuboid(scale=[0.08, 0.07, 0.03], color=food_colour)
        props.append((food, SE3(*(centre + [x, y, 0.055]))))
    for shape, relative in props:
        shape.T = (SE3(tray.T) * SE3(0, 0, 0.12) * relative).A
        env.add(shape)
    steps = max(1, int(np.ceil(0.6 / dt)))
    for k in range(1, steps + 1):
        for shape, relative in props:
            shape.T = (SE3(tray.T) * SE3(0, 0, 0.12 * (1 - k / steps))
                       * relative).A
        env.step(dt)
    return props

def serving_gap_validator(gap):
    """Check the loaded tray envelope at every planned sample crossing the bars."""
    local_centre = TRAY_CENTRE_UNSCALED * TRAY_SCALE
    corners = np.array([local_centre + [x, y, z]
                        for x in (-0.30, 0.30)
                        for y in (-0.20, 0.20)
                        for z in (0.0, LOADED_TRAY_HEIGHT)])
    def fits(pose):
        world = corners @ pose.R.T + pose.t
        lo, hi = world.min(axis=0), world.max(axis=0)
        if lo[2] < 0.6267448 + 0.008:
            return False
        if hi[1] < gap[1] - 0.04 or lo[1] > gap[1] + 0.04:
            return True
        # 8 mm inset from the measured opening; reject tilted/clipping routes.
        return (lo[0] >= gap[0] - 0.40355 + 0.008
                and hi[0] <= gap[0] + 0.40355 - 0.008
                and hi[2] <= 0.7338 - 0.008)
    return fits

def collect_served_meal(env, tray, payload, gap, dt):
    """Recipient pulls the released tray through; robot remains inside the bars."""
    start = SE3(tray.T)
    centre = start.t + start.R @ (TRAY_CENTRE_UNSCALED * TRAY_SCALE)
    distance = gap[1] + 0.30 - centre[1]
    steps = max(1, int(np.ceil(distance / (0.30 * dt))))
    poses = [SE3.Rt(start.R, start.t + [0, distance * k / steps, 0])
             for k in range(1, steps + 1)]
    if not all(serving_gap_validator(gap)(pose) for pose in poses):
        raise RuntimeError("Served meal cannot clear the opening; holding tray.")
    print("[HANDOUT] Recipient takes the tray; robot stays behind the bars.", flush=True)
    for pose in poses:
        tray.T = pose.A
        for shape, relative in payload:
            shape.T = (pose * relative).A
        env.step(dt)

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
                            tray_scale=TRAY_SCALE, return_home=RETURN_HOME_AFTER_PLACE,
                            animation_speed=ROBOT_ANIMATION_SPEED,
                            cache_dir=ROUTE_CACHE_DIR)
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
        # ---- one full lap, temporary meal loading, pickup and handout ----
        if outcome.on_belt:
            stacks.mark_used(key)
            delivery.payload = carry_full_loop(
                env, objects, tray_mesh, layout, controller.dt, colour)
            gap = mesh_xyz(objects, "frontBars", (13.4, 4.7, 0.5))
            gap = gap + np.asarray(HANDOUT_GAP_LOCAL)
            inside = gap + [0, -HANDOUT_SIDE_CLEARANCE, 0]
            handout = gap + [0, HANDOUT_INSERTION, 0]
            # Align at belt height inside the bars, lower, then cross horizontally.
            align = inside.copy()
            align[2] = BELT_SURFACE_Z + 0.18
            outcome = delivery.deliver(tray_mesh, label=f"{key} loaded meal",
                                       destination=handout, waypoints=[align, inside],
                                       carry_validator=serving_gap_validator(gap))
            if outcome.status != "PLACED":
                print(f"[HANDOUT] {outcome.message}. Stopping for inspection; "
                      "the loaded meal has not been marked delivered.")
                break
            for _ in range(max(1, int(np.ceil(HANDOUT_PAUSE / controller.dt)))):
                env.step(controller.dt)
            collect_served_meal(
                env, tray_mesh, delivery.payload, gap, controller.dt)
            # Recipient takes the meal; clear the serving position for the next order.
            for shape, _ in delivery.payload:
                swift_remove(env, shape)
            swift_remove(env, tray_mesh)
            delivery.payload = []
            dispatcher.mark_delivered(result.order)
            print(f"{colour} meal handed out. {stacks.remaining(colour)} trays left. "
                  "Reopening camera for the next colour card.")
        dispatcher.reset(colour)          # allow the next card of this colour
    env.hold()

if __name__ == "__main__":
    main()