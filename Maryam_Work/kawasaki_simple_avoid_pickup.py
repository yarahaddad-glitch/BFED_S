"""Kawasaki RS007N: cube pickup with collision-aware RMRC detours.

Put next to Maryam_robotclass.py. Uses Swift + spatialgeometry already in the UTS environment.
Simulation demo only: approximate collision bodies, no certified safety, no gripper physics.
"""
import numpy as np
import swift
from spatialgeometry import Cuboid, Sphere, Cylinder
from spatialmath import SE3
from Maryam_robotclass import MaryamBot

# ----------------- Robot and scene -----------------
robot = MaryamBot()
env = swift.Swift()
env.launch(realtime=True)
for mesh in robot.links_3d:
    env.add(mesh)

START_Q = np.deg2rad([0, -45, 60, 0, 45, 0])
robot.q = START_Q.copy()
QMIN = robot.qlim[0] + np.deg2rad(5)
QMAX = robot.qlim[1] - np.deg2rad(5)
CUBE_SIZE = np.array([0.05, 0.05, 0.05])
GRIP_GAP = 0.025
SPHERE_RADIUS = 0.035
SAFETY_GAP = 0.015
DT = 0.035
MAX_QDOT = 0.5
POSITION_TOL = 0.008
rng = np.random.default_rng(12)

# Actual CAD housing widths cannot be established from the available screenshot.
# Each entry: start DH-frame index, end index, approximate body radius (metres).
# Combining frames 2->4 avoids the zero-length DH-3 segment problem.
# Tune to actual STL geometry; cylinders are not required to display these checks.
# Each body is (first DH frame, last DH frame, radius, local SIDE shift, trim length).
# These are *not* verified CAD measurements. Tune them against the visible STL.
# The final body covers ONLY the last 0.11 m near J6, not the whole 4->6 span.
# Moving the forearm sideways shifts its centre toward the actual housing.
BODIES = [
    (0, 1, 0.085, 0.00, None),   # shoulder
    (1, 2, 0.080, 0.00, None),   # upper arm
    (2, 4, 0.105, 0.035, None),  # wider forearm; 3.5 cm sideways shift (trial)
    (4, 6, 0.045, 0.00, 0.11),   # shorter J6 / wrist volume
]


def frames(q):
    T = robot.base.A.copy()
    transforms = [T.copy()]
    for link, angle in zip(robot.links, q):
        T = T @ link.A(float(angle)).A
        transforms.append(T.copy())
    return transforms


# Visualization of approximate collision capsules: same segment endpoints/radii
# as the collision checker. They are NOT measurements from Kawasaki drawings.
COLLISION_VISUAL_ALPHA = 0.10
collision_guides = []


def segment_transform(a, b):
    """Pose with cylinder local Z-axis aligned to a world-space segment."""
    direction = np.asarray(b, dtype=float) - np.asarray(a, dtype=float)
    length = float(np.linalg.norm(direction))
    z_axis = direction / length if length > 1e-9 else np.array([0.0, 0.0, 1.0])
    helper = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z_axis, helper)) > 0.95:
        helper = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(helper, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    centre = (np.asarray(a, dtype=float) + np.asarray(b, dtype=float)) / 2.0
    return SE3.Rt(np.column_stack([x_axis, y_axis, z_axis]), centre), max(length, 0.001)


def body_segment(fs, definition):
    """One common geometry for BOTH collision maths and Swift visuals.

    Side direction comes from segment_transform's local X axis.
    Last entry of BODIES optionally caps length near the END DH frame.
    """
    i, j, radius, side_shift, max_length = definition
    a = np.asarray(fs[i][:3, 3], dtype=float).copy()
    b = np.asarray(fs[j][:3, 3], dtype=float).copy()
    if max_length is not None:
        v = b - a
        length = np.linalg.norm(v)
        if length > max_length:
            a = b - (max_length / length) * v
    pose, _ = segment_transform(a, b)
    shift = pose.R[:, 0] * side_shift
    return a + shift, b + shift, radius


def create_collision_guides():
    fs = frames(robot.q)
    tint = (0.15, 0.95, 0.95, COLLISION_VISUAL_ALPHA)
    for definition in BODIES:
        a, b, radius = body_segment(fs, definition)
        pose, length = segment_transform(a, b)
        body = Cylinder(radius=float(radius), length=float(length), pose=pose, color=tint)
        end_a = Sphere(radius=float(radius), pose=SE3(*a.tolist()), color=tint)
        end_b = Sphere(radius=float(radius), pose=SE3(*b.tolist()), color=tint)
        for shape in (body, end_a, end_b):
            env.add(shape)
        collision_guides.append((body, end_a, end_b, definition))
    env.step(0.05)


def update_collision_guides():
    fs = frames(robot.q)
    for body, end_a, end_b, definition in collision_guides:
        a, b, _ = body_segment(fs, definition)
        pose, _ = segment_transform(a, b)
        body.T = pose.A
        end_a.T = SE3(*a.tolist()).A
        end_b.T = SE3(*b.tolist()).A


def point_segment_distance(p, a, b):
    ab = b - a
    length2 = float(ab @ ab)
    if length2 < 1e-12:
        return np.linalg.norm(p - a)
    t = np.clip(((p - a) @ ab) / length2, 0.0, 1.0)
    return np.linalg.norm(p - (a + t * ab))


# The moving cube must also clear the obstacle once virtually grasped.
def safe(q, obstacle_xyz, attached_offset=None):
    if np.any(q < QMIN) or np.any(q > QMAX):
        return False
    fs = frames(q)
    for definition in BODIES:
        a, b, radius = body_segment(fs, definition)
        if point_segment_distance(obstacle_xyz, a, b) <= (
            SPHERE_RADIUS + radius + SAFETY_GAP
        ):
            return False
    ee = robot.fkine(q)
    if np.linalg.norm(ee.t - obstacle_xyz) <= SPHERE_RADIUS + SAFETY_GAP:
        return False
    if attached_offset is not None:
        centre = (ee * attached_offset).t
        # Bounding sphere approximates cube of side 5 cm, conservatively.
        cube_radius = 0.5 * np.linalg.norm(CUBE_SIZE)
        if np.linalg.norm(centre - obstacle_xyz) <= SPHERE_RADIUS + SAFETY_GAP + cube_radius:
            return False
    return True


def safe_step(a, b, obstacle_xyz, offset=None):
    # Evaluate samples between joint configurations, not only endpoints.
    return all(safe((1 - t) * a + t * b, obstacle_xyz, offset)
               for t in np.linspace(0, 1, 5))


def joint_velocity(q, desired_xyz_velocity):
    J = robot.jacob0(q)[:3, :]
    sig_min = np.linalg.svd(J, compute_uv=False)[-1]
    damp = 0.035 if sig_min < 0.04 else 0.0
    if damp:
        qdot = J.T @ np.linalg.solve(J @ J.T + damp**2 * np.eye(3), desired_xyz_velocity)
    else:
        qdot = np.linalg.pinv(J) @ desired_xyz_velocity
    peak = np.max(np.abs(qdot))
    if peak > MAX_QDOT:
        qdot *= MAX_QDOT / peak
    return qdot


def plan_segment(q0, goal, obstacle_xyz, offset=None):
    """Shadow-simulate RMRC. Never alter robot.q while planning."""
    q = q0.copy()
    start = robot.fkine(q).t
    path = [q.copy()]
    steps = max(50, int(np.linalg.norm(goal - start) / 0.0025))
    steps = min(125, steps)
    for k in range(steps + 100):
        actual = robot.fkine(q).t
        if k < steps:
            u = k / steps
            un = (k + 1) / steps
            smooth = lambda t: 3 * t*t - 2 * t*t*t
            wanted = start + smooth(u) * (goal - start)
            feed = (smooth(un) - smooth(u)) * (goal - start) / DT
        else:
            wanted, feed = goal, np.zeros(3)
        if k >= steps and np.linalg.norm(actual - goal) < POSITION_TOL:
            return path
        qn = q + DT * joint_velocity(q, feed + 2.0 * (wanted - actual))
        if not safe_step(q, qn, obstacle_xyz, offset):
            return None
        q = qn
        path.append(q.copy())
    return path if np.linalg.norm(robot.fkine(q).t - goal) < POSITION_TOL else None


def try_route(q0, target, obstacle_xyz, offset=None):
    """Try direct route, then alternative Cartesian via-points around obstacle."""
    direct = plan_segment(q0, target, obstacle_xyz, offset)
    if direct is not None:
        return direct, 'DIRECT'

    start = robot.fkine(q0).t
    delta = target - start
    length = np.linalg.norm(delta)
    if length < 1e-6:
        return None, 'NO ROUTE'
    axis = delta / length
    side = np.cross(axis, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(side) < 1e-6:
        side = np.cross(axis, np.array([0.0, 1.0, 0.0]))
    side /= np.linalg.norm(side)
    up = np.array([0., 0., 1.])
    middle = (start + target) / 2

    # Search limited side/height detours. Both stages must pass collision checks.
    candidates = []
    for amount in (0.12, 0.20, 0.30, 0.40):
        for direction in (side, -side, up, -up):
            via = middle + amount * direction
            if via[2] > 0.12:
                candidates.append(via)
    for via in candidates:
        first = plan_segment(q0, via, obstacle_xyz, offset)
        if first is None:
            continue
        second = plan_segment(first[-1], target, obstacle_xyz, offset)
        if second is not None:
            return first + second[1:], 'DETOUR'
    return None, 'NO ROUTE FOUND'


# ----------------- Cube and obstacle placement -----------------
# Generate a demonstrable scenario from reachable joint poses.
# Avoid arbitrary XYZ guesses: pick targets using the robot's own FK.
FAR_AWAY = np.array([5., 5., 5.])
GOAL_Q = None
obstacle_xyz = None
PICKUP_XYZ = None

# Find a scene with an obstructed DIRECT path and reachable cube pickup.
# Do not require the entire pickup sequence to have a proven detour before
# showing the obstacle: that earlier requirement caused the RuntimeError.
# We still validate each motion before executing it.

candidate_poses = [
    [25, -55, 75, 0, 45, 0],
    [35, -75, 90, 0, 45, 0],
    [-35, -65, 85, 0, 45, 0],
    [45, -55, 100, 0, 45, 0],
    [-45, -70, 95, 0, 45, 0],
    [40, -80, 105, 0, 55, 0],
    [-40, -80, 105, 0, 55, 0],
]

# Additional nearby goals can help if the approximate bodies reject
# the fixed candidates; these are still checked by FK and joint limits.
for _ in range(35):
    candidate_poses.append(
        (np.array([25, -55, 75, 0, 45, 0]) +
         rng.uniform([-30, -20, -20, -15, -10, -10],
                     [30, 20, 20, 15, 10, 10])).tolist()
    )

attempts = {"open_path": 0, "clear_endpoints": 0, "blocks_direct": 0,
            "detour_found": 0}
scene_route_verified = False

for candidate_q_deg in candidate_poses:
    candidate_q = np.deg2rad(candidate_q_deg)
    candidate_pickup = robot.fkine(candidate_q).t.copy()
    candidate_approach = candidate_pickup + np.array([0.0, 0.0, 0.07])
    candidate_lift = candidate_pickup + np.array([0.0, 0.0, 0.11])

    # First establish that the cube is reachable with NO obstacle.
    open_path = plan_segment(START_Q, candidate_approach, FAR_AWAY)
    if open_path is None or len(open_path) < 25:
        continue
    open_descend = plan_segment(open_path[-1], candidate_pickup, FAR_AWAY)
    if open_descend is None:
        continue
    open_lift = plan_segment(open_descend[-1], candidate_lift, FAR_AWAY)
    if open_lift is None:
        continue
    attempts["open_path"] += 1

    cube_candidate = candidate_pickup - np.array(
        [0.0, 0.0, CUBE_SIZE[2] / 2 + GRIP_GAP]
    )
    path_length = len(open_path) - 1

    # Place the sphere at a point on the ACTUAL predicted direct RMRC
    # approach, not somewhere arbitrarily nearby.
    for fraction in (0.30, 0.40, 0.50, 0.60, 0.70):
        q_mid = open_path[int(fraction * path_length)]
        centre = robot.fkine(q_mid).t.copy()
        if np.linalg.norm(centre - cube_candidate) < 0.14:
            continue
        if not safe(START_Q, centre):
            continue
        # The ending pickup and lift poses must remain clear.
        if not safe(open_descend[-1], centre) or not safe(open_lift[-1], centre):
            continue
        attempts["clear_endpoints"] += 1
        if plan_segment(START_Q, candidate_approach, centre) is not None:
            continue
        attempts["blocks_direct"] += 1

        # Prefer a verified detour; do not demand that the entire later
        # cube transport is also solved at scene-generation time.
        trial_path, route_type = try_route(START_Q, candidate_approach, centre)
        if trial_path is None or route_type != "DETOUR":
            continue
        attempts["detour_found"] += 1
        # The descent must be possible from the selected detour endpoint.
        if plan_segment(trial_path[-1], candidate_pickup, centre) is None:
            continue

        GOAL_Q = candidate_q
        PICKUP_XYZ = candidate_pickup
        obstacle_xyz = centre
        scene_route_verified = True
        break
    if scene_route_verified:
        break

print('Scene search diagnostics:', attempts)
if not scene_route_verified:
    raise RuntimeError(
        'No verified detour found with the CURRENT approximate body geometry. '
        'The robot did not move and no misleading fallback was used. '
        'Check the printed search diagnostics; the overly large or '
        'misaligned body volumes may block every candidate.'
    )
print('VERIFIED: sphere blocks the direct approach; a detour + descent was found.')

CUBE_XYZ = PICKUP_XYZ - np.array([0., 0., CUBE_SIZE[2] / 2 + GRIP_GAP])
cube = Cuboid(scale=CUBE_SIZE.tolist(), pose=SE3(*[float(x) for x in CUBE_XYZ]), color=(1.0, 0.55, 0.05, 1.0))
env.add(cube)
obstacle = Sphere(radius=SPHERE_RADIUS, pose=SE3(*[float(x) for x in obstacle_xyz]), color=(1.0, 0.0, 0.0, 1.0))
env.add(obstacle)
env.step(.05)
create_collision_guides()


# ----------------- Motion + reactive check -----------------
def execute(target, name, offset=None):
    print(f'\n{name}: planning...')
    q0 = np.asarray(robot.q).copy()
    if not safe(q0, obstacle_xyz, offset):
        print('Initial configuration is inside the approximate obstacle region.')
        print('  Stage:', name)
        print('  Joints (deg):', np.round(np.rad2deg(q0), 2))
        print('  EE XYZ:', np.round(robot.fkine(q0).t, 4))
        print('  Obstacle XYZ:', np.round(obstacle_xyz, 4))
        fs = frames(q0)
        for definition in BODIES:
            a, b, radius = body_segment(fs, definition)
            d = point_segment_distance(obstacle_xyz, a, b)
            clearance = d - (SPHERE_RADIUS + radius + SAFETY_GAP)
            print(f'  Body {definition[0]}->{definition[1]} clearance (mm): {clearance * 1000:.1f}')
        if offset is not None:
            cube_centre = (robot.fkine(q0) * offset).t
            cube_radius = 0.5 * np.linalg.norm(CUBE_SIZE)
            cube_gap = np.linalg.norm(cube_centre - obstacle_xyz) - (SPHERE_RADIUS + SAFETY_GAP + cube_radius)
            print('  Attached cube clearance (mm):', round(cube_gap * 1000, 1))
        return False
    path, label = try_route(q0, np.asarray(target), obstacle_xyz, offset)
    if path is None:
        print('No checked route found. Robot stays where it is.')
        return False
    print(f'Route: {label} ({len(path)} joint steps)')
    for index in range(1, len(path)):
        qa, qb = np.asarray(robot.q).copy(), path[index]
        if not safe_step(qa, qb, obstacle_xyz, offset):
            print('Obstacle conflict during motion: replanning from current pose...')
            remaining = np.asarray(target)
            new_path, new_label = try_route(qa, remaining, obstacle_xyz, offset)
            if new_path is None:
                print('No alternative route; remaining stationary.')
                return False
            print('New route:', new_label)
            # Execute replanned trajectory (with checks) using common mover.
            return follow_path(new_path, offset)
        robot.q = qb
        update_cube(offset)
        update_collision_guides()
        env.step(DT)
    return True


def follow_path(path, offset):
    for qb in path[1:]:
        qa = np.asarray(robot.q).copy()
        if not safe_step(qa, qb, obstacle_xyz, offset):
            print('Replanned route blocked; halting safely.')
            return False
        robot.q = qb
        update_cube(offset)
        update_collision_guides()
        env.step(DT)
    return True


def update_cube(offset):
    if offset is not None:
        cube.T = (robot.fkine(robot.q) * offset).A


# ----------------- One automatic pickup demonstration -----------------
print('\n=== CUBE PICKUP WITH COLLISION-AWARE DETOUR ===')
print('Obstacle:', np.round(obstacle_xyz, 3))
print('Cube:', np.round(CUBE_XYZ, 3))
print('Initial collision-free:', safe(robot.q, obstacle_xyz))

approach = PICKUP_XYZ + np.array([0, 0, .07])
if execute(approach, 'APPROACH') and execute(PICKUP_XYZ, 'PICKUP'):
    if np.linalg.norm(robot.fkine(robot.q).t - PICKUP_XYZ) <= POSITION_TOL:
        T_ee_cube = robot.fkine(robot.q).inv() * SE3(cube.T)
        print('Cube virtually grasped.')
        lift = PICKUP_XYZ + np.array([0, 0, .11])
        if execute(lift, 'LIFT', T_ee_cube):
            print('SUCCESS: Cube lifted with collision checking and detour search.')
        else:
            print('Lift incomplete; cube remains virtually attached.')
else:
    print('Pickup not completed; a checked route was not found.')

env.hold()
