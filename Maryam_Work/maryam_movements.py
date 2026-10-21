"""Reusable Maryam movements: blue/green pickup, belt lap and handout."""
import sys
from pathlib import Path
from functools import lru_cache
import xml.etree.ElementTree as ET

import numpy as np
import swift
from spatialmath import SE3
from spatialgeometry import Cuboid, Mesh

try:
    from spatialgeometry import Sphere
except ImportError:          # markers are optional
    Sphere = None

ASSIGNMENT_ROOT = Path(__file__).resolve().parents[1]
if str(ASSIGNMENT_ROOT) not in sys.path:
    sys.path.insert(0, str(ASSIGNMENT_ROOT))

from Maryam_Work.maryam_tray_delivery import (
    BeltLayout, TrayDelivery, TrayStacks, TRAY_CENTRE_UNSCALED)
from Maryam_Work.maryamrobot_controller import RobotController
from Collective_Folder.EnviroParts_S.Full_sim import (
    TRAY_SCALE, add_maryam_robot, add_tray_stacks,
    configure_swift_server, create_environment)

# ============================================================
# CONFIG  - the numbers you are most likely to change
# ============================================================
# Where the robot sets the tray down, measured from the ROBOT BASE (metres).
# Keep it close to the robot so RMRC can reach it.
PLACE_FROM_ROBOT = (0.0, -0.45)        # centre on belt within reach of corner grip
BELT_SURFACE_Z = 0.67                  # tray bottom: lowered 3 cm onto the belt
BELT_MIDDLE_X = 14.96                  # destination toward +X, opposite the previous travel
BELT_SPEED = 0.45                      # metres per second (3x original speed)
BELT_FAR_LANE_Y = 2.75                 # centre of the opposite straight belt
# Opening measured in frontBars.dae: world X 13.8213..14.6284,
# shortened bars start at Z 0.7338.
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


# ============================================================
# 1. SCENE
# ============================================================
def build_scene():
    configure_swift_server()
    env = swift.Swift()
    env.launch(realtime=True)

    objects = create_environment(env)
    # Hide the static display tray; only the robot's stack trays belong here.
    # Park it offstage because Swift.remove leaves holes that break env.step.
    belt_display_tray = objects.pop("bluetray", None)
    if belt_display_tray is not None:
        belt_display_tray.T = SE3(0, 0, -100).A
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
        base_stl=ASSIGNMENT_ROOT / 'Maryam_Work/RS007N-BASE.STL',
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
                payload = add_meal(env, tray, colour, dt)
            for shape, relative in payload:
                shape.T = (SE3(tray.T) * relative).A
            env.step(dt)
        travelled += distance
        current = target
    tray.T = initial.A
    for shape, relative in payload:
        shape.T = (initial * relative).A
    return payload


@lru_cache(maxsize=2)
def food_geometry(colour):
    """Bounds include the Blender node transform in these single-mesh assets."""
    filename = {"blue": "cookies.dae", "green": "lettuce.dae"}[colour.lower()]
    path = ASSIGNMENT_ROOT / "Collective_Folder/EnviroParts_S/food" / filename
    root = ET.parse(path).getroot()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    source_id = root.find('.//c:vertices/c:input', ns).attrib['source'][1:]
    positions = root.find(f'.//c:source[@id="{source_id}"]/c:float_array', ns)
    points = np.fromstring(positions.text, sep=" ").reshape(-1, 3)
    matrix = np.fromstring(root.find('.//c:visual_scene/c:node/c:matrix', ns).text,
                           sep=" ").reshape(4, 4)
    points = points @ matrix[:3, :3].T + matrix[:3, 3]
    lo, hi = points.min(axis=0), points.max(axis=0)
    # Keep food within the existing loaded-tray clearance envelope.
    scale = min(np.array([0.16, 0.13, 0.035]) / (hi - lo))
    bottom_centre = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    return path, scale, bottom_centre * scale


def add_meal(env, tray, colour, dt):
    """Load actual cookies for blue and lettuce for green, plus a folded towel."""
    centre = TRAY_CENTRE_UNSCALED * TRAY_SCALE
    path, scale, bottom_centre = food_geometry(colour)
    props = []
    towel = Cuboid(scale=[0.13, 0.15, 0.006], color=(0.98, 0.98, 0.94, 1))
    props.append((towel, SE3(*(centre + [0.17, 0, 0.045]))))
    for x, y in ((-0.14, -0.075), (-0.14, 0.075), (0.025, 0)):
        food = Mesh(str(path), scale=[scale] * 3)
        props.append((food, SE3(*(centre + [x, y, 0.032] - bottom_centre))))
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


def collect_served_meal(env, tray, payload, gap, dt, clearance=None):
    """Recipient pulls the released tray through; robot remains inside the bars."""
    start = SE3(tray.T)
    centre = start.t + start.R @ (TRAY_CENTRE_UNSCALED * TRAY_SCALE)
    distance = gap[1] + 0.30 - centre[1]
    steps = max(1, int(np.ceil(distance / (0.30 * dt))))
    poses = [SE3.Rt(start.R, start.t + [0, distance * k / steps, 0])
             for k in range(1, steps + 1)]
    if not all(serving_gap_validator(gap)(pose) for pose in poses):
        raise RuntimeError("Served meal cannot clear the opening; holding tray.")
    if clearance is not None and not all(clearance(pose) for pose in poses):
        raise RuntimeError('Recipient tray would hit MaryamBot; holding tray')
    print("[HANDOUT] Recipient takes the tray; robot stays behind the bars.", flush=True)
    for pose in poses:
        if clearance is not None and not clearance(pose):
            raise RuntimeError('Recipient path blocked; holding tray')
        tray.T = pose.A
        for shape, relative in payload:
            shape.T = (pose * relative).A
        env.step(dt)


class MaryamMovements:
    """Four named movements selected by colour, independent of arrival order.

    TrayDelivery reuses verified disk routes for each stack height and loaded
    tray pose. Only a new/changed geometry needs the initial RMRC search.
    """

    def __init__(self, env, objects, robot, trays):
        self.env, self.objects = env, objects
        self.home_q = robot.q.copy()
        self.controller = build_controller(robot)
        self.layout = build_belt_layout(objects, robot)
        if SHOW_MARKERS:
            add_marker(env, self.layout.place_centre, (1.0, 0.0, 0.0, 1.0))
            add_marker(env, self.layout.middle_centre, (1.0, 0.9, 0.0, 1.0))
            env.step()
        self.stacks = TrayStacks(trays)
        self.delivery = TrayDelivery(
            env, robot, self.controller, self.layout, tray_scale=TRAY_SCALE,
            return_home=RETURN_HOME_AFTER_PLACE,
            animation_speed=ROBOT_ANIMATION_SPEED, cache_dir=ROUTE_CACHE_DIR)
        self.delivery.stationary_trays = list(trays.values())
        self.controller.extra_valid = lambda q:self.delivery.queued_trays_clear(
            q, active_tray=getattr(self.delivery, '_planning_tray', None))
        self.pickups = {"blue": self.pick_blue, "green": self.pick_green}
        self.handouts = {"blue": self.handout_blue, "green": self.handout_green}

    def pick_blue(self, tray, label):
        """Blue stack -> conveyor."""
        return self.delivery.deliver(tray, label=label)

    def pick_green(self, tray, label):
        """Green stack -> conveyor, using the same transfer as blue."""
        return self.delivery.deliver(tray, label=label)

    def handout_blue(self, tray, label):
        """Collect the loaded blue tray and hand it through the serving gap."""
        return self._handout(tray, label)

    def handout_green(self, tray, label):
        """Collect the loaded green tray and hand it through the serving gap."""
        return self._handout(tray, label)

    def _handout(self, tray, label):
        gap = mesh_xyz(self.objects, "frontBars", (13.4, 4.7, 0.5))
        gap = gap + np.asarray(HANDOUT_GAP_LOCAL)
        inside = gap + [.06, -HANDOUT_SIDE_CLEARANCE, 0]
        handout = gap + [.06, HANDOUT_INSERTION, 0]
        align = inside.copy()
        align[1] = inside[1] - .15
        align[2] = BELT_SURFACE_Z + 0.18
        low = align.copy()
        low[2] = inside[2]
        outcome = self.delivery.deliver(
            tray, label=f"{label} loaded meal", destination=handout,
            waypoints=[align, low, inside], carry_validator=serving_gap_validator(gap),
            destination_rotation=SE3.Rz(-np.pi/2).R @ SE3(tray.T).R)
        if outcome.status != "PLACED":
            raise RuntimeError(f"Handout failed: {outcome.message}")
        for _ in range(max(1, int(np.ceil(HANDOUT_PAUSE / self.controller.dt)))):
            self.env.step(self.controller.dt)
        collect_served_meal(
            self.env, tray, self.delivery.payload, gap, self.controller.dt,
            clearance=lambda pose:self.delivery.tray_clears_robot(self.delivery.robot.q, pose)
            and self.delivery.tray_clears_obstacles(pose)
            and self.delivery.queued_trays_clear(self.delivery.robot.q, pose, tray))
        self.last_handout_pose = SE3(tray.T)
        for shape, _ in self.delivery.payload:
            shape.T = SE3(0, 0, -100).A
        # Swift.remove leaves None entries which this installed Swift.step
        # dereferences on the next cycle. Park served meshes offstage instead.
        tray.T = SE3(0, 0, -100).A
        self.delivery.payload = []
        self.return_home()
        return outcome

    def return_home(self):
        """Return to the startup joint configuration with prechecked joint motion."""
        robot = self.delivery.robot
        start = robot.q.copy()
        count = max(60, int(np.ceil(np.max(np.abs(start-self.home_q)) / .01)))
        blend = np.linspace(0, 1, count)
        blend = 3*blend**2-2*blend**3
        path = start + blend[:, None]*(self.home_q-start)
        if not all(self.controller.valid(q) for q in path):
            raise RuntimeError('Home path blocked; MaryamBot remains stopped')
        self.delivery._watched_tray = None
        getattr(self, 'activity', lambda message:None)('MaryamBot: returning home')
        self.delivery._execute(path)

    def serve(self, colour, on_conveyor):
        """Run one detected colour; return False only when its stack is empty.

        A motion failure stops the cycle for inspection, rather than scanning
        another prisoner while a tray may still occupy the robot or belt.
        """
        colour = colour.lower()
        if colour not in self.pickups:
            raise ValueError(f"Unsupported tray colour: {colour}")
        pick = self.stacks.next_tray(colour)
        if pick is None:
            print(f"No {colour} trays left.", flush=True)
            return False
        key, tray = pick
        outcome = self.pickups[colour](tray, key)
        if outcome.status != "PLACED":
            raise RuntimeError(f"Pickup failed: {outcome.message}")
        self.stacks.mark_used(key)
        on_conveyor()
        self.delivery.payload = carry_full_loop(
            self.env, self.objects, tray, self.layout, self.controller.dt, colour)
        self.handouts[colour](tray, key)
        print(f"{colour} meal handed out. {self.stacks.remaining(colour)} trays left.",
              flush=True)
        return True



