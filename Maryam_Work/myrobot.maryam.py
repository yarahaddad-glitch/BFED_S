import os
import numpy as np
import swift
from spatialgeometry import Cuboid, Sphere, Mesh
from ir_support import make_ellipsoid
from spatialmath import SE3
from Maryam_robotclass import MaryamBot

# ============================================================
# 1. CREATE KAWASAKI RS007N AND SWIFT ENVIRONMENT
# ============================================================
robot = MaryamBot()
env = swift.Swift()
env.launch(realtime=True)
for mesh in robot.links_3d:
    env.add(mesh)

# ============================================================
# 2. INITIAL ROBOT CONFIGURATION
# ============================================================
q_start = np.deg2rad([0, -45, 60, 0, 45, 0])
robot.q = q_start.copy()
env.step(0.03)
T_start = robot.fkine(q_start)

# ============================================================
# 3. CREATE SIMULATED TRAY
# ============================================================
tray_size = np.array([0.08, 0.08, 0.02])
q_reference = np.deg2rad([25, -55, 75, 0, 45, 0])
reference_xyz = robot.fkine(q_reference).t
pickup_clearance = 0.02
default_tray_xyz = reference_xyz.copy()
default_tray_xyz[2] -= tray_size[2] / 2 + pickup_clearance
tray = Cuboid(scale=tray_size.tolist(), pose=SE3(*default_tray_xyz), color=(1.0, 0.5, 0.0, 1.0))
env.add(tray)

# ============================================================
# 4. SPHERICAL TEST OBSTACLE
# ============================================================
obstacle_center = np.array([0.10, -0.05, 0.95])
obstacle_radius = 0.04
collision_margin = 0.03
obstacle = Sphere(radius=obstacle_radius, pose=SE3(*obstacle_center), color=(1.0, 0.0, 0.0, 1.0))
env.add(obstacle)
env.step(0.05)

# ============================================================
# 5. RMRC AND JOINT-LIMIT PARAMETERS
# ============================================================
Kp = 2.0
max_joint_speed = 0.5
singularity_threshold = 0.04
max_damping = 0.05
joint_margin = np.deg2rad(5)
q_min = robot.qlim[0, :]
q_max = robot.qlim[1, :]
position_tolerance = 0.005
tray_attached = False
T_ee_tray = None

# ============================================================
# 6. JOINT LIMIT CHECKS
# ============================================================
def show_joint_limits():
    print('\n========== JOINT LIMITS ==========')
    for j in range(robot.n):
        print(f'J{j + 1}: {np.rad2deg(q_min[j]):.1f} to {np.rad2deg(q_max[j]):.1f} deg')


def check_joint_limits(q_test):
    q_test = np.asarray(q_test, dtype=float)
    return bool(np.all(q_test >= q_min + joint_margin) and np.all(q_test <= q_max - joint_margin))

# ============================================================
# 7. LAB 6: ELLIPSOID COLLISION MODEL
# ============================================================
# Lab 6 uses D = (x/rx)^2 + (y/ry)^2 + (z/rz)^2.
# First transform a world obstacle point into each link-local
# ellipsoid frame. D <= 1 means inside/on the ellipsoid.
# We ALSO account for the obstacle sphere radius + clearance using
# numerical point-to-ellipsoid surface distance (scipy.brentq).
# This is a simplified model; sizes/offsets need STL validation.
from scipy.optimize import brentq

# Approximate cross-sectional radii for six DH link regions (m).
link_radii = np.array([0.09, 0.08, 0.07, 0.06, 0.03, 0.02])

# Each row offsets the ellipsoid CENTER in its own segment-aligned
# local frame: [along-segment, sideways, sideways], metres.
# Tune these values against Swift STL geometry, especially link 3/4.
collision_local_offsets = np.zeros((6, 3))

# Additional length beyond DH endpoints for each ellipsoid (m).
# Important: ellipsoid tips taper, unlike capsule tips.
ellipsoid_end_padding = 0.04

# Obstacles: only the RED SPHERE is checked in this version.
# The orange tray is intentionally NOT a collision obstacle during pickup.
collision_visuals = []
visual_alpha = 0.09


def get_joint_positions(q):
    """World XYZ of DH frame origins, including offsets/flips."""
    q = np.asarray(q, dtype=float).reshape(robot.n)
    T = robot.base.A.copy()
    positions = [T[:3, 3].copy()]
    for j in range(robot.n):
        T = T @ robot.links[j].A(q[j]).A
        positions.append(T[:3, 3].copy())
    return np.asarray(positions)


def segment_pose(start, end):
    """Local ellipsoid X axis points along DH segment."""
    delta = end - start
    length = float(np.linalg.norm(delta))
    midpoint = (start + end) * 0.5
    if length < 1e-9:
        return SE3(midpoint)
    x_axis = delta / length
    helper = np.array([0., 0., 1.])
    if abs(np.dot(x_axis, helper)) > 0.95:
        helper = np.array([0., 1., 0.])
    y_axis = np.cross(helper, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    return SE3.Rt(np.column_stack((x_axis, y_axis, z_axis)), midpoint)


def ellipsoid_models(q):
    """Return six (pose, radii) pairs for collision and visualisation."""
    positions = get_joint_positions(q)
    models = []
    for j in range(robot.n):
        start, end = positions[j], positions[j + 1]
        length = float(np.linalg.norm(end - start))
        pose = segment_pose(start, end)
        pose = pose * SE3(*collision_local_offsets[j])
        radii = np.array([
            max(link_radii[j], 0.5 * length + ellipsoid_end_padding),
            link_radii[j],
            link_radii[j]
        ], dtype=float)
        models.append((pose, radii))
    return models


def get_algebraic_dist(points_local, radii):
    """Lab 6 algebraic distance: <=1 means inside the ellipsoid."""
    pts = np.atleast_2d(np.asarray(points_local, dtype=float))
    return np.sum((pts / radii) ** 2, axis=1)


def point_to_ellipsoid_distance(point_local, radii):
    """Surface distance for a point OUTSIDE axis-aligned ellipsoid.

    Inside/on ellipsoid returns zero. Outside: solve for the nearest
    surface point using a 1D root of the Lagrange multiplier equation.
    """
    p = np.asarray(point_local, dtype=float)
    r = np.asarray(radii, dtype=float)
    if get_algebraic_dist(p, r)[0] <= 1.0:
        return 0.0
    r2 = r * r
    def f(lam):
        return np.sum((r * p / (lam + r2)) ** 2) - 1.0
    upper = max(1.0, float(np.linalg.norm(p) * np.max(r)))
    while f(upper) > 0.0:
        upper *= 2.0
    lam = brentq(f, 0.0, upper, xtol=1e-12)
    nearest = r2 * p / (lam + r2)
    return float(np.linalg.norm(p - nearest))


def collision_report(q_test, center=None):
    """Test red sphere against six link ellipsoids and EE point.

    The physical sphere intersects an ellipsoid if centre-to-surface
    distance <= sphere radius + margin; algebraic D checks containment.
    """
    if center is None:
        center = obstacle_center
    center = np.asarray(center, dtype=float)
    results = []
    for j, (pose, radii) in enumerate(ellipsoid_models(q_test)):
        # Transform obstacle centre from world into local ellipsoid frame
        local = pose.R.T @ (center - pose.t)
        algebraic_d = float(get_algebraic_dist(local, radii)[0])
        surface_distance = point_to_ellipsoid_distance(local, radii)
        clearance = surface_distance - (obstacle_radius + collision_margin)
        results.append((f'Link ellipsoid {j + 1}', clearance, algebraic_d))
    ee_xyz = robot.fkine(q_test).t
    ee_clearance = float(np.linalg.norm(ee_xyz - center) - obstacle_radius - collision_margin)
    results.append(('EE reference point', ee_clearance, None))
    part, min_clearance, _ = min(results, key=lambda item: item[1])
    return min_clearance <= 0.0, part, min_clearance, results


def check_robot_collision(q_test, verbose=False, center=None):
    collision, closest, clearance, results = collision_report(q_test, center)
    if verbose:
        print('\n========== LAB 6 ELLIPSOID COLLISION CHECK ==========')
        print('Obstacle XYZ:', np.round(obstacle_center if center is None else center, 4))
        for part, gap, algebraic_d in results:
            suffix = '' if algebraic_d is None else f' | algebraic D={algebraic_d:.3f}'
            print(f'{part}: clearance {gap * 1000:.1f} mm{suffix}')
        print('Closest part:', closest)
        print('Minimum clearance (mm):', round(clearance * 1000, 2))
        print('Collision detected:', collision)
        print('NOTE: Approximate link ellipsoids; not STL, tray, self or other robot collisions.')
    return collision


def check_robot_motion(q_current, q_next):
    """Sample joint space before applying an RMRC step."""
    for fraction in np.linspace(0.0, 1.0, 11):
        q_sample = q_current + fraction * (q_next - q_current)
        if check_robot_collision(q_sample):
            print('\nCOLLISION WARNING: Proposed motion rejected.')
            check_robot_collision(q_sample, verbose=True)
            return False
    return True


# ============================================================
# 7B. ACTUAL ELLIPSOID MESHES IN SWIFT
# ============================================================
# The tutor's make_ellipsoid(is_plot=False) provides surface points.
# Swift needs triangles, so we save ONE unit ellipsoid STL mesh and
# load six differently scaled copies through spatialgeometry.Mesh.
# Visual poses/radii come from ellipsoid_models(), exactly like the
# collision calculations. No dependency upgrades are required.


def create_unit_ellipsoid_stl(filename):
    """Use the tutor's surface generator to build a closed triangle STL."""
    if os.path.isfile(filename):
        return

    # Periodic longitude (no duplicate seam), including north/south poles.
    longitude_count = 32
    latitude_count = 20
    u = np.linspace(0.0, 2.0 * np.pi, longitude_count, endpoint=False)
    v = np.linspace(0.0, np.pi, latitude_count)
    X, Y, Z = make_ellipsoid(
        ellipsoid_info=[1.0, 1.0, 1.0],
        center=[0.0, 0.0, 0.0],
        u=u,
        v=v,
        is_plot=False,
    )
    vertices = np.stack((X, Y, Z), axis=-1)

    os.makedirs(os.path.dirname(filename), exist_ok=True)
    triangle_count = 0
    with open(filename, 'w', encoding='ascii') as out:
        out.write('solid mealbot_unit_ellipsoid\n')
        for i in range(longitude_count):
            next_i = (i + 1) % longitude_count
            for j in range(latitude_count - 1):
                # Two triangles between adjacent latitude/longitude samples.
                a = vertices[i, j]
                b = vertices[next_i, j]
                c = vertices[i, j + 1]
                d = vertices[next_i, j + 1]
                for p1, p2, p3 in ((a, b, c), (b, d, c)):
                    cross = np.cross(p2 - p1, p3 - p1)
                    magnitude = np.linalg.norm(cross)
                    if magnitude < 1e-10:  # skip degenerate triangles at poles
                        continue
                    normal = cross / magnitude
                    # Face normals outward (STL-compatible winding).
                    centroid = (p1 + p2 + p3) / 3.0
                    if np.dot(normal, centroid) < 0:
                        p2, p3 = p3, p2
                        normal = -normal
                    out.write(f'  facet normal {normal[0]:.8f} {normal[1]:.8f} {normal[2]:.8f}\n')
                    out.write('    outer loop\n')
                    for point in (p1, p2, p3):
                        out.write(f'      vertex {point[0]:.8f} {point[1]:.8f} {point[2]:.8f}\n')
                    out.write('    endloop\n  endfacet\n')
                    triangle_count += 1
        out.write('endsolid mealbot_unit_ellipsoid\n')
    print(f'Generated ellipsoid STL with {triangle_count} triangles: {filename}')


def create_collision_visuals():
    """Create one transparent *ellipsoid* per Kawasaki DH segment."""
    base_folder = os.path.dirname(os.path.abspath(__file__))
    mesh_filename = os.path.join(base_folder, 'collision_assets', 'unit_ellipsoid.stl')
    create_unit_ellipsoid_stl(mesh_filename)
    colour = (0.2, 0.95, 1.0, visual_alpha)
    for pose, radii in ellipsoid_models(robot.q):
        shape = Mesh(
            filename=mesh_filename,
            scale=radii.tolist(),
            pose=pose,
            color=colour,
        )
        env.add(shape)
        collision_visuals.append(shape)
    env.step(0.05)
    print('Created 6 faint ellipsoid meshes from tutor-generated surface points.')


def update_collision_visuals():
    """Move ellipsoid meshes with FK; their radii stay constant."""
    for shape, (pose, radii) in zip(collision_visuals, ellipsoid_models(robot.q)):
        shape.T = pose.A

# ============================================================
# 8. TRAY POSITION, ATTACHMENT AND RELEASE
# ============================================================
def get_tray_position():
    return np.asarray(tray.T[:3, 3], dtype=float).copy()


def update_tray():
    if tray_attached:
        tray.T = (robot.fkine(robot.q) * T_ee_tray).A


def attach_tray():
    global tray_attached, T_ee_tray
    T_ee = robot.fkine(robot.q)
    T_ee_tray = T_ee.inv() * SE3(tray.T)
    tray_attached = True
    update_tray()
    print('\nTRAY ATTACHED (virtual grasp)')


def release_tray():
    global tray_attached, T_ee_tray
    update_tray()
    tray_attached = False
    T_ee_tray = None
    print('\nTRAY RELEASED')

# ============================================================
# 9. POSITION-ONLY RMRC WITH DLS
# ============================================================
def calculate_qdot(q_current, desired_velocity):
    J = robot.jacob0(q_current)[:3, :]
    sigma_min = np.linalg.svd(J, compute_uv=False)[-1]
    if sigma_min < singularity_threshold:
        damping = max(0.001, max_damping * (1 - sigma_min / singularity_threshold))
        J_inverse = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), np.eye(3))
    else:
        J_inverse = np.linalg.pinv(J)
    qdot = J_inverse @ desired_velocity
    speed = np.max(np.abs(qdot))
    if speed > max_joint_speed:
        qdot *= max_joint_speed / speed
    return qdot

# ============================================================
# 10. COLLISION-CHECKED RMRC MOVEMENT
# ============================================================
def rmrc_move(target_xyz, stage_name, duration=4.0):
    print(f'\n========== {stage_name} ==========')
    target_xyz = np.asarray(target_xyz, dtype=float)
    if target_xyz.shape != (3,) or not np.all(np.isfinite(target_xyz)):
        print('STOP: Invalid target XYZ.')
        return False
    steps = 150
    dt = duration / (steps - 1)
    start_xyz = robot.fkine(robot.q).t.copy()
    if check_robot_collision(robot.q):
        print('STOP: Current robot pose intersects the protected obstacle model.')
        check_robot_collision(robot.q, verbose=True)
        return False
    u = np.linspace(0, 1, steps)
    s = 3 * u**2 - 2 * u**3
    desired_xyz = start_xyz[None, :] + s[:, None] * (target_xyz - start_xyz)[None, :]
    for k in range((steps - 1) + 120):
        q_current = np.asarray(robot.q, dtype=float).copy()
        current_xyz = robot.fkine(q_current).t
        if k < steps - 1:
            desired = desired_xyz[k]
            feedforward = (desired_xyz[k + 1] - desired_xyz[k]) / dt
        else:
            desired = target_xyz
            feedforward = np.zeros(3)
        error = desired - current_xyz
        if k >= steps - 1 and np.linalg.norm(target_xyz - current_xyz) < position_tolerance:
            break
        qdot = calculate_qdot(q_current, feedforward + Kp * error)
        q_next = q_current + qdot * dt
        if not check_joint_limits(q_next):
            print('STOP: Joint limit margin reached.')
            print('Joint angles (deg):', np.round(np.rad2deg(q_current), 2))
            return False
        if not check_robot_motion(q_current, q_next):
            print('Current EE XYZ:', np.round(current_xyz, 4))
            return False
        robot.q = q_next
        update_tray()
        update_collision_visuals()
        env.step(dt)
    actual_xyz = robot.fkine(robot.q).t
    final_error = np.linalg.norm(target_xyz - actual_xyz)
    print('Target XYZ:', np.round(target_xyz, 4))
    print('Actual XYZ:', np.round(actual_xyz, 4))
    print('Position error (mm):', round(final_error * 1000, 3))
    if final_error > position_tolerance:
        print('STOP: Target not reached.')
        return False
    print('Movement completed.')
    return True

# ============================================================
# 11. TRAY PICKUP WAYPOINTS
# ============================================================
def calculate_pickup_waypoints():
    xyz = get_tray_position()
    tray_top_z = xyz[2] + tray_size[2] / 2
    pickup_xyz = np.array([xyz[0], xyz[1], tray_top_z + pickup_clearance])
    approach_xyz = pickup_xyz + np.array([0.0, 0.0, 0.10])
    lift_xyz = pickup_xyz + np.array([0.0, 0.0, 0.12])
    print('\n========== TRAY TARGETS ==========')
    print('Tray:', np.round(xyz, 4))
    print('Approach:', np.round(approach_xyz, 4))
    print('Pickup:', np.round(pickup_xyz, 4))
    print('Lift:', np.round(lift_xyz, 4))
    return approach_xyz, pickup_xyz, lift_xyz


def pickup_and_lift():
    if tray_attached:
        print('Tray already attached.')
        return False
    approach, pickup, lift = calculate_pickup_waypoints()
    if not rmrc_move(approach, 'APPROACH TRAY', duration=5.0):
        return False
    if not rmrc_move(pickup, 'DESCEND TO TRAY', duration=3.0):
        return False
    if np.linalg.norm(robot.fkine(robot.q).t - pickup) > position_tolerance:
        print('Pickup position not reached.')
        return False
    attach_tray()
    if not rmrc_move(lift, 'LIFT TRAY', duration=4.0):
        print('Lift stopped. Tray remains attached.')
        return False
    print('Pickup and lift completed.')
    return True

# ============================================================
# 12. TERMINAL COMMANDS
# ============================================================
def parse_xyz(text):
    cleaned = text.replace('[', ' ').replace(']', ' ').replace(',', ' ')
    values = np.array([float(x) for x in cleaned.split()], dtype=float)
    if values.size != 3 or not np.all(np.isfinite(values)):
        raise ValueError('Enter exactly three finite XYZ values.')
    return values


def show_help():
    print('''
========== SMART MEALBOT COMMANDS ==========
move x,y,z       Move end effector using RMRC
x,y,z            Same as move
tray x,y,z       Reposition orange tray (when released)
obstacle x,y,z   Reposition red sphere (must be clear of arm)
pickup           Approach, virtually attach, and lift tray
release          Release tray
home             Return to starting EE XYZ
status           Show robot, tray and obstacle positions
limits           Show joint limits
collision        Check ALL Lab 6 link ellipsoids + EE against sphere
help             Show these commands
quit             Leave terminal controller
============================================
''')


show_joint_limits()
create_collision_visuals()
show_help()

while True:
    try:
        command = input('\nMealBot > ').strip()
        if not command:
            continue
        parts = command.split(maxsplit=1)
        action = parts[0].lower()
        if action in ('quit', 'exit', 'q'):
            print('Terminal controller closed.')
            break
        elif action == 'help':
            show_help()
        elif action == 'status':
            print('EE XYZ:', np.round(robot.fkine(robot.q).t, 4))
            print('Tray XYZ:', np.round(get_tray_position(), 4))
            print('Obstacle XYZ:', np.round(obstacle_center, 4))
            print('Tray attached:', tray_attached)
            print('Joint angles (deg):', np.round(np.rad2deg(robot.q), 2))
        elif action == 'limits':
            show_joint_limits()
        elif action == 'collision':
            check_robot_collision(robot.q, verbose=True)
        elif action == 'move':
            if len(parts) < 2:
                print('Example: move 0.04,-0.08,1.00')
                continue
            success = rmrc_move(parse_xyz(parts[1]), 'MANUAL RMRC MOVE')
            print('Movement successful:', success)
        elif action == 'tray':
            if tray_attached:
                print('Release tray before repositioning.')
                continue
            if len(parts) < 2:
                print('Example: tray 0.04,-0.08,0.92')
                continue
            target = parse_xyz(parts[1])
            new_transform = tray.T.copy()
            new_transform[:3, 3] = target
            tray.T = new_transform
            env.step(0.05)
            print('Tray moved to:', np.round(get_tray_position(), 4))
        elif action == 'obstacle':
            if len(parts) < 2:
                print('Example: obstacle 0.10,-0.05,0.95')
                continue
            target = parse_xyz(parts[1])
            if check_robot_collision(robot.q, center=target):
                print('REJECTED: Obstacle would overlap a protected ellipsoid or EE point.')
                check_robot_collision(robot.q, verbose=True, center=target)
                continue
            obstacle_center = target.copy()
            obstacle.T = SE3(*obstacle_center).A
            env.step(0.05)
            print('Obstacle moved to:', np.round(obstacle_center, 4))
        elif action == 'pickup':
            success = pickup_and_lift()
            print('Pickup successful:', success)
        elif action == 'release':
            if tray_attached:
                release_tray()
                env.step(0.05)
            else:
                print('No tray is attached.')
        elif action == 'home':
            success = rmrc_move(T_start.t.copy(), 'RETURN HOME', duration=5.0)
            print('Return successful:', success)
        else:
            success = rmrc_move(parse_xyz(command), 'MANUAL RMRC MOVE')
            print('Movement successful:', success)
    except ValueError as error:
        print('Invalid input:', error)
    except np.linalg.LinAlgError as error:
        print('RMRC numerical error:', error)
    except KeyboardInterrupt:
        print('\nTerminal controller interrupted.')
        break

print('\nSwift remains open.')
env.hold()
