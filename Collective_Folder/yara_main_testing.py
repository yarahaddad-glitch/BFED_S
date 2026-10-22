import sys
import numpy as np
from itertools import islice
from pathlib import Path
from spatialmath import SE3
from roboticstoolbox import jtraj
from ir_support_extra_robots.robots import Turtlebot3Waffle

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Yara_Work.YaraBot_DH import YaraBot
from Yara_Work.YaraBot_Collision import CollisionBox, check_arm_collision, get_link_positions
from Collective_Folder.E_stop_test import ESTOP

# ---------------- Settings ----------------
DT = 0.05
TURTLE_Z = 0.027
MOUNT_HEIGHT = 0.10

FOOD_STATIONS = {"blue": (13.2, 2.05), "green": (14.6, 2.05)}
FOOD_NAMES = {
    "blue": ["cookies1", "cookies2", "cookies3"],
    "green": ["lettuce1", "lettuce2", "lettuce3"],
}
TEST_FOOD = "cookies3"  # None = nearest available food at the station

CONVEYOR_STOP = (13.2, 2.25)  # same x as the food station, so it drives straight along y
DEBUG = True  # print which link hits which obstacle
BLUE_TRAY_POSITION = np.array([13.0, 2.97, 0.68])
PLACE_FOOD_Z = 0.74
PLACE_DX = 0.17  # nudge the drop point in world x (tray mesh origin is not its centre)
PLACE_NEAR_OFFSET = 0.04  # nudge the drop point towards the robot (world -y)
HOVER_FOOD_Z = 0.98
TURN_ANGLE = np.radians(90)  # arm turn before placing
FOOD_PLACEMENT_TOLERANCE = 0.025

PICKUP_OFFSET = 0.08
LIFT_STEP = 0.05
MAX_LIFT_STEPS = 5
MAX_LIFT_SIDEWAYS = 0.06

# RMRC
DAMPING = 0.06
GAIN = 1.5
MAX_JOINT_SPEED = 0.5
POSITION_TOLERANCE = 0.025
MAX_STEPS = 300
JOINT_LIMIT_MARGIN = np.radians(2)
JOINT_CENTRE_GAIN = 0.08

# Collision
ARM_MARGIN = 0.01
FOOD_RADIUS = 0.035
TURTLE_RADIUS = 0.18
SELF_COLLISION_DISTANCE = 0.055
CHASSIS_CLEARANCE = 0.035

# Alternative paths
PATH_CLEARANCE = 0.12
SIDE_OFFSETS = [0.25, -0.25, 0.45, -0.45]
HEIGHT_OFFSETS = [0.0, 0.05, 0.10]

FOOD_COUNTER = CollisionBox("Food Counter", centre=(13.211, 1.478, 0.283), size=(4.601, 0.476, 0.369))
CONVEYOR_COUNTER = CollisionBox("Conveyor Counter", centre=(13.05, 3.30, 0.35), size=(4.61, 1.53, 0.65))
OBSTACLES = [FOOD_COUNTER, CONVEYOR_COUNTER]
ARM_OBSTACLES = OBSTACLES  # what the arm links are checked against

# ---------------- Globals ----------------
env = objects = turtlebot = yarabot = None
held_food = held_offset = None
carried_food = carried_start = None  # food being carried + where it was before pickup (for E-stop reset)
HOME_Q = None
HOME_TURTLE_BASE = None


def setup_yarabot(shared_env, shared_objects):
    global env, objects, turtlebot, yarabot, held_food, held_offset
    global carried_food, carried_start, HOME_Q, HOME_TURTLE_BASE
    env, objects = shared_env, shared_objects
    held_food = held_offset = None
    carried_food = carried_start = None

    turtlebot = Turtlebot3Waffle()
    turtlebot.base = SE3(13.2, 2.0, TURTLE_Z)
    turtlebot.add_to_env(env)

    yarabot = YaraBot()
    yarabot.base = turtlebot.base * SE3(0, 0, MOUNT_HEIGHT)
    yarabot.add_to_env(env)

    HOME_Q = np.asarray(yarabot.q, dtype=float).copy()
    HOME_TURTLE_BASE = SE3(turtlebot.base.A.copy())

    env.step(DT)


# ---------------- Helpers ----------------
def food_position(q, offset):
    return (yarabot.fkine(q) * offset).t


def update_food(food, offset):
    if food is not None and offset is not None:
        food.T = (yarabot.fkine(yarabot.q) * offset).A


def set_base(position):
    turtlebot.base = SE3(*position)
    yarabot.base = turtlebot.base * SE3(0, 0, MOUNT_HEIGHT)


def segment_distance(p1, q1, p2, q2):
    u, v, w = q1 - p1, q2 - p2, p1 - p2
    a, b, c, d, e = u @ u, u @ v, v @ v, u @ w, v @ w
    den = a * c - b * b
    s = 0.0 if den < 1e-12 else np.clip((b * e - c * d) / den, 0, 1)
    t = 0.0 if c < 1e-12 else np.clip((b * s + e) / c, 0, 1)
    if a > 1e-12:
        s = np.clip((b * t - d) / a, 0, 1)
    if c > 1e-12:
        t = np.clip((b * s + e) / c, 0, 1)
    return np.linalg.norm((p1 + s * u) - (p2 + t * v))


def point_inside_box(point, box, margin=0.0):
    half = np.asarray(box.size) / 2.0 + margin
    return bool(np.all(np.abs(point - np.asarray(box.centre)) <= half))


def joint_limits():
    limits = np.asarray(yarabot.qlim, dtype=float)
    return limits[0], limits[1]


def joints_within_limits(q):
    lo, hi = joint_limits()
    return bool(np.all(q >= lo - 1e-8) and np.all(q <= hi + 1e-8))


def clamp_joints(q):
    lo, hi = joint_limits()
    return np.clip(q, lo + JOINT_LIMIT_MARGIN, hi - JOINT_LIMIT_MARGIN)


# ---------------- Collision checks ----------------
def self_collision(q):
    p = np.asarray(get_link_positions(yarabot, q), dtype=float)
    n = len(p) - 1
    for i in range(n):
        for j in range(i + 2, n):
            if segment_distance(p[i], p[i + 1], p[j], p[j + 1]) < SELF_COLLISION_DISTANCE:
                return True
    return False


def chassis_collision(q):
    p = np.asarray(get_link_positions(yarabot, q), dtype=float)
    base = np.asarray(turtlebot.base.t, dtype=float)
    top = base[2] + MOUNT_HEIGHT
    for pos in p[2:]:
        if np.linalg.norm(pos[:2] - base[:2]) < TURTLE_RADIUS and pos[2] < top - CHASSIS_CLEARANCE:
            return True
    return False


def configuration_safe(q, food=None, offset=None, lifting=False):
    q = np.asarray(q, dtype=float)

    if not joints_within_limits(q):
        return False, "Joint limits"

    hit = check_arm_collision(yarabot, q, ARM_OBSTACLES, ARM_MARGIN, DEBUG)
    if isinstance(hit, tuple):
        hit, name = hit
    else:
        name = "?"
    if hit:
        return False, "Arm collision: " + str(name)

    if self_collision(q):
        return False, "Self collision"
    if chassis_collision(q):
        return False, "TurtleBot chassis"

    if food is not None and offset is not None:
        xyz = food_position(q, offset)
        for obstacle in OBSTACLES:
            if lifting and obstacle is FOOD_COUNTER:
                continue
            if point_inside_box(xyz, obstacle, FOOD_RADIUS):
                return False, "Food collision: " + obstacle.name

    return True, "Safe"


# ---------------- RMRC ----------------
def calculate_joint_velocity(q, velocity):
    J = yarabot.jacob0(q)[:3, :]
    J_pinv = J.T @ np.linalg.inv(J @ J.T + DAMPING ** 2 * np.eye(3))
    q_dot = J_pinv @ velocity

    # Nullspace: drift towards joint centres
    lo, hi = joint_limits()
    centre_vel = JOINT_CENTRE_GAIN * ((lo + hi) / 2.0 - q) / np.maximum(hi - lo, 1e-6)
    q_dot += (np.eye(len(q)) - J_pinv @ J) @ centre_vel

    peak = np.max(np.abs(q_dot))
    if peak > MAX_JOINT_SPEED:
        q_dot *= MAX_JOINT_SPEED / peak
    return q_dot


def plan_rmrc(target, food=None, offset=None, lifting=False):
    target = np.asarray(target, dtype=float)
    q = np.asarray(yarabot.q, dtype=float).copy()
    path = [q.copy()]
    previous_error = float("inf")
    stalled = 0

    for _ in range(MAX_STEPS):
        ESTOP.check()  # let E interrupt planning too

        error_vector = target - yarabot.fkine(q).t
        error = np.linalg.norm(error_vector)

        if error < POSITION_TOLERANCE:
            return np.asarray(path)

        q_next = clamp_joints(q + calculate_joint_velocity(q, GAIN * error_vector) * DT)

        safe, reason = configuration_safe(q_next, food, offset, lifting)
        if not safe:
            print("RMRC blocked:", reason)
            return None

        stalled = stalled + 1 if abs(previous_error - error) < 1e-5 else 0
        if stalled > 35:
            print("RMRC stalled. Error:", round(error, 4))
            return None

        previous_error = error
        q = q_next
        path.append(q.copy())

    print("RMRC failed to converge")
    return None


def execute_path(path, food=None, offset=None):
    """Play a joint path frame by frame. E-stop is checked before every frame."""
    if path is None:
        return False
    for q in np.asarray(path, dtype=float):
        ESTOP.check()
        yarabot.q = q.copy()
        update_food(food, offset)
        env.step(DT)
    return True


# ---------------- Arm movement ----------------
def candidate_paths(start, target):
    candidates = [[target.copy()]]
    height = max(start[2], target[2]) + PATH_CLEARANCE

    raised_start = start.copy()
    raised_start[2] = height
    raised_target = target.copy()
    raised_target[2] = height
    candidates.append([raised_start, raised_target, target.copy()])

    for side in SIDE_OFFSETS:
        for extra in HEIGHT_OFFSETS:
            h = height + extra
            w1 = start.copy()
            w1[2] = h
            w2 = target.copy()
            w2[0] += side
            w2[2] = h
            w3 = target.copy()
            w3[2] = h
            candidates.append([w1, w2, w3, target.copy()])

    return candidates


def move_arm(target, food=None, offset=None):
    target = np.asarray(target, dtype=float)
    start_q = np.asarray(yarabot.q, dtype=float).copy()
    start_pos = yarabot.fkine(start_q).t.copy()

    for n, waypoints in enumerate(candidate_paths(start_pos, target), start=1):
        yarabot.q = start_q.copy()
        segments = []

        for waypoint in waypoints:
            segment = plan_rmrc(waypoint, food, offset)
            if segment is None:
                segments = None
                break
            segments.append(segment)
            yarabot.q = segment[-1].copy()

        yarabot.q = start_q.copy()

        if segments is None:
            continue

        print("Using path", n)
        for segment in segments:
            execute_path(segment, food, offset)
        return True

    print("[PLAN] RMRC candidates exhausted; trying collision-checked IK.", flush=True)
    # RMRC can stall at the upright home singularity (especially green food).
    # Solve positional IK and validate every joint sample before execution.
    rng = np.random.default_rng(7)
    lo, hi = joint_limits()
    for waypoints in candidate_paths(start_pos, target):
        q = start_q.copy()
        segments = []
        for waypoint in waypoints:
            segment = None
            for attempt in range(30):
                ESTOP.check()
                seed = q if attempt == 0 else np.clip(q + rng.normal(0, .7, len(q)), lo, hi)
                solution = yarabot.ikine_LM(SE3(*waypoint), q0=seed,
                                            mask=[1, 1, 1, 0, 0, 0], ilimit=100)
                if not solution.success:
                    continue
                candidate = np.asarray(solution.q)
                segment = safe_trajectory(q, candidate, food, offset)
                if segment is not None:
                    q = candidate
                    break
            if segment is None:
                break
            segments.append(segment)
        else:
            for segment in segments:
                execute_path(segment, food, offset)
            return True
    print("No valid arm path found")
    return False


def move_arm_joints(target_q, food=None, offset=None):
    target_q = np.asarray(target_q, dtype=float)
    if not joints_within_limits(target_q):
        print("Target joints exceed limits")
        return False

    trajectory = jtraj(np.asarray(yarabot.q, dtype=float), target_q, 120).q

    for q in trajectory:
        safe, reason = configuration_safe(q, food, offset)
        if not safe:
            print("Joint movement blocked:", reason)
            return False

    return execute_path(trajectory, food, offset)


# ---------------- TurtleBot movement ----------------
def turtle_position_safe(position):
    for obstacle in OBSTACLES:
        centre = np.asarray(obstacle.centre, dtype=float)
        half = np.asarray(obstacle.size, dtype=float)[:2] / 2.0
        nearest = np.clip(position[:2], centre[:2] - half, centre[:2] + half)
        if np.linalg.norm(position[:2] - nearest) < TURTLE_RADIUS:
            return False
    return True


def move_turtlebot(x, y, food=None, offset=None):
    start_base = SE3(turtlebot.base.A.copy())
    start_arm_base = SE3(yarabot.base.A.copy())
    start = np.asarray(start_base.t, dtype=float)
    end = np.array([float(x), float(y), TURTLE_Z])
    steps = 100
    route = [start * (1 - i / steps) + end * (i / steps) for i in range(steps + 1)]

    # Validate whole route first
    for position in route:
        if not turtle_position_safe(position):
            print("TurtleBot path blocked at:", np.round(position, 3))
            turtlebot.base, yarabot.base = start_base, start_arm_base
            return False

        set_base(position)
        safe, reason = configuration_safe(yarabot.q, food, offset)
        if not safe:
            print("TurtleBot movement blocked:", reason)
            turtlebot.base, yarabot.base = start_base, start_arm_base
            return False

    turtlebot.base, yarabot.base = start_base, start_arm_base

    # Execute
    for position in route:
        ESTOP.check()
        set_base(position)
        update_food(food, offset)
        env.step(DT)

    return True


# ---------------- Pick ----------------
def lift_food(food, offset):
    start_food = food_position(yarabot.q, offset)
    counter_top = FOOD_COUNTER.centre[2] + FOOD_COUNTER.size[2] / 2.0
    required_height = counter_top + FOOD_RADIUS + ARM_MARGIN

    if start_food[2] >= required_height:
        return True

    start_xy = start_food[:2].copy()

    for _ in range(MAX_LIFT_STEPS):
        target = yarabot.fkine(yarabot.q).t.copy()
        target[2] += LIFT_STEP

        path = plan_rmrc(target, food, offset, lifting=True)
        if path is None:
            return False

        sideways = max(np.linalg.norm(food_position(q, offset)[:2] - start_xy) for q in path)
        if sideways > MAX_LIFT_SIDEWAYS:
            print("Lift rejected: too much sideways movement")
            return False

        execute_path(path, food, offset)

        if food_position(yarabot.q, offset)[2] >= required_height:
            return True

    print("Food did not reach clearance height")
    return False


def pick_food(food):
    if food is None:
        return False

    xyz = SE3(food.T).t.copy()
    target = xyz.copy()
    target[2] += PICKUP_OFFSET

    if not move_arm(target):
        print("Could not reach food")
        return False

    # Food transform relative to end effector
    offset = yarabot.fkine(yarabot.q).inv() * SE3(food.T)

    if not lift_food(food, offset):
        return False

    return offset


# ---------------- Transport ----------------
def safe_trajectory(q_a, q_b, food=None, offset=None, steps=60):
    """Joint-space path from q_a to q_b, or None if any step is unsafe."""
    trajectory = jtraj(q_a, q_b, steps).q
    for q in trajectory:
        if not configuration_safe(q, food, offset)[0]:
            return None
    return trajectory


def ik_food_configs(target_food, food, offset, seed_q, tries=150):
    """Yield safe joint configurations that put the FOOD at target_food."""
    rng = np.random.default_rng(1)
    lo, hi = joint_limits()

    for i in range(tries):
        ESTOP.check()
        q0 = seed_q if i == 0 else np.clip(seed_q + rng.normal(0, 0.5, len(seed_q)), lo, hi)
        target_ee = target_food - (food_position(q0, offset) - yarabot.fkine(q0).t)

        for _ in range(4):  # correct for the food offset rotating with the arm
            sol = yarabot.ikine_LM(SE3(*target_ee), q0=q0, mask=[1, 1, 1, 0, 0, 0])
            if not sol.success:
                break
            q = np.asarray(sol.q, dtype=float)
            error = target_food - food_position(q, offset)
            if np.linalg.norm(error) <= FOOD_PLACEMENT_TOLERANCE:
                if joints_within_limits(q) and configuration_safe(q, food, offset)[0]:
                    yield q
                break
            target_ee = target_ee + error
            q0 = q


def ik_plan(q_start, hover_food, place_food, food, offset):
    """Joint-space route: start -> hover -> place, using inverse kinematics."""
    for q_hover in islice(ik_food_configs(hover_food, food, offset, q_start), 40):
        up = safe_trajectory(q_start, q_hover, food, offset)
        if up is None:
            continue
        for q_place in ik_food_configs(place_food, food, offset, q_hover, tries=30):
            down = safe_trajectory(q_hover, q_place, food, offset)
            if down is not None:
                return [up, down]
    return None


def rmrc_plan(q_start, targets, food, offset):
    """Straight-line route through each food target using RMRC."""
    segments = []
    try:
        for target in targets:
            q_now = np.asarray(yarabot.q, dtype=float).copy()
            target_ee = target - (food_position(q_now, offset) - yarabot.fkine(q_now).t)
            for _ in range(6):
                yarabot.q = q_now.copy()
                path = plan_rmrc(target_ee, food, offset)
                if path is None:
                    return None
                error = target - food_position(path[-1], offset)
                if np.linalg.norm(error) <= FOOD_PLACEMENT_TOLERANCE:
                    segments.append(path)
                    yarabot.q = path[-1].copy()
                    break
                target_ee = target_ee + error
            else:
                return None
        return segments
    finally:
        yarabot.q = q_start.copy()


def plan_place(q_start, hover_food, place_food, food, offset):
    """Try IK then RMRC. If the counter blocks the arm, retry without it (the tray sits on it)."""
    global DEBUG, ARM_OBSTACLES
    debug, DEBUG = DEBUG, False
    try:
        for obstacles in (OBSTACLES, [FOOD_COUNTER]):
            ARM_OBSTACLES = obstacles
            segments = ik_plan(q_start, hover_food, place_food, food, offset)
            if segments is None:
                segments = rmrc_plan(q_start, [hover_food, place_food], food, offset)
            if segments is not None:
                if obstacles is not OBSTACLES:
                    print("Note: placed with the conveyor counter ignored for the arm links")
                return segments
    finally:
        DEBUG, ARM_OBSTACLES = debug, OBSTACLES
    return None


def turn_arm(food, offset, aim):
    """Turn joint 1 by TURN_ANGLE, in whichever direction brings the food closer to aim."""
    q = np.asarray(yarabot.q, dtype=float).copy()
    lo, hi = joint_limits()
    options = []
    for turn in (TURN_ANGLE, -TURN_ANGLE):
        target = q.copy()
        target[0] += turn
        if lo[0] <= target[0] <= hi[0]:
            dist = np.linalg.norm(food_position(target, offset)[:2] - aim[:2])
            options.append((dist, target))

    if not options:
        print("Turn exceeds joint 1 limits")
        return False

    return move_arm_joints(min(options, key=lambda o: o[0])[1], food, offset)


def transport_food(food, offset, place_target=None):
    global held_food, held_offset, carried_food, carried_start

    # 1. Straight up into the upright pose
    if not move_arm_joints(HOME_Q, food, offset):
        print("Could not return arm upright")
        return False

    # 2. Drive to the conveyor while upright
    if not move_turtlebot(*CONVEYOR_STOP, food, offset):
        print("Could not reach conveyor")
        return False

    # 3. Turn 90 degrees to face the tray
    aim = BLUE_TRAY_POSITION if place_target is None else place_target
    if not turn_arm(food, offset, aim):
        return False

    # 4. Plan: up over the tray, then straight down (all collision checked)
    place_food = np.array([BLUE_TRAY_POSITION[0] + PLACE_DX,
                           BLUE_TRAY_POSITION[1] - PLACE_NEAR_OFFSET,
                           PLACE_FOOD_Z])
    if place_target is not None:
        place_food = np.asarray(place_target, dtype=float)
    hover_food = place_food.copy()
    hover_food[2] = HOVER_FOOD_Z

    segments = plan_place(np.asarray(yarabot.q, dtype=float).copy(),
                          hover_food, place_food, food, offset)
    if segments is None:
        print("Could not find any way to reach the tray")
        return False

    # 5. Execute
    for segment in segments:
        execute_path(segment, food, offset)

    actual = food_position(yarabot.q, offset)
    if np.linalg.norm(actual - place_food) > FOOD_PLACEMENT_TOLERANCE:
        print("Placement error too large")
        return False

    # 6. Release food, keeping its world transform
    food.T = (yarabot.fkine(yarabot.q) * offset).A.copy()
    held_food = held_offset = None
    carried_food = carried_start = None  # food is on the tray now, don't move it on reset
    env.step(DT)

    # 7. Retreat along the verified path, then go home
    for segment in reversed(segments):
        execute_path(segment[::-1])
    if not move_arm_joints(HOME_Q):
        print("Warning: could not return arm home")
    return True


# ---------------- Reset / Task ----------------
def reset_yarabot():
    if yarabot is None or turtlebot is None:
        print("YaraBot has not been set up")
        return False

    if held_food is not None:
        print("Cannot reset while holding food")
        return False

    if not move_arm_joints(HOME_Q):
        return False

    home = HOME_TURTLE_BASE.t
    return move_turtlebot(home[0], home[1])


def run_task(colour, tray=None, food_bottom_centre=None):
    global held_food, held_offset, carried_food, carried_start

    if yarabot is None or turtlebot is None:
        print("YaraBot has not been set up")
        return False

    colour = str(colour).strip().lower()
    if colour not in FOOD_STATIONS:
        print("Unsupported tray colour:", colour)
        return False

    available = [n for n in FOOD_NAMES[colour] if n in objects]
    if not available:
        print("No food available for", colour.upper())
        return False

    name = TEST_FOOD if TEST_FOOD in available else min(
        available, key=lambda n: abs(SE3(objects[n].T).t[0] - FOOD_STATIONS[colour][0]))
    food = objects[name]
    print("Task:", colour.upper(), "->", name)

    if not move_turtlebot(*FOOD_STATIONS[colour]):
        print("Could not reach food station")
        return False

    # Remember where the food started so an E-stop reset can put it back
    carried_food, carried_start = food, SE3(food.T)

    offset = pick_food(food)
    if offset is False:
        print("Pickup unsuccessful")
        carried_food = carried_start = None
        return False

    held_food, held_offset = food, offset

    place_target = None
    if tray is not None:
        from Maryam_Work.maryam_tray_delivery import TRAY_CENTRE_UNSCALED
        from Collective_Folder.EnviroParts_S.Full_sim import TRAY_SCALE
        bottom = np.zeros(3) if food_bottom_centre is None else np.asarray(food_bottom_centre)
        # Food occupies the left half; ReBot's towel goes on the right half.
        place_target = (SE3(tray.T) * SE3(*(TRAY_CENTRE_UNSCALED * TRAY_SCALE
                        + [-0.12, 0, 0.032]))).t - bottom

    if not transport_food(food, offset, place_target):
        print("Transport unsuccessful")
        carried_food = carried_start = None
        return False

    carried_food = carried_start = None
    FOOD_NAMES[colour].remove(name)
    print("Task completed:", name)
    return True