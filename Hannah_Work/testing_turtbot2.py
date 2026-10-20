import numpy as np
import swift
import xml.etree.ElementTree as ET
from pathlib import Path
from scipy.spatial.transform import Rotation, Slerp
from spatialmath import SE3
from spatialgeometry import Mesh, Cuboid
from ir_support_extra_robots import Turtlebot3Waffle
from Class_HannahBot import HannahBot

print("\n=== HANNAHBOT STRAIGHT APPROACH TO TRAY (RMRC) ===")
print("Running file:", Path(__file__).resolve())


# ============================================================
# 0. SETTINGS YOU MAY WANT TO TUNE
# ============================================================

# --- approach, measured from the tray's NEAR edge (the side closest to HannahBot)
PRE_DISTANCE   = 0.10   # pre-pickup stops this far BEFORE the near edge (m)
PICKUP_INSET   = 0.05   # forward ends this far PAST the near edge (m, + = into tray)
                        # FIX: was -0.12, which put "pickup" 2 cm BEHIND pre-pickup,
                        #      so 'forward' moved backwards instead of into the tray.
TOOL_HEIGHT    = 0.00   # tool height above the tray's base (m)
TOOL_Y_OFFSET  = 0.00   # sideways offset from the tray's centre line (m)
# (change these while running with:  set standoff|inset|height|yoff <value>)

# --- where the tray + tables sit (metres, added to their base positions)
TRAY_SHIFT = np.array([0.0, 0.09, 0.0])

# --- TurtleBot (the arm rides on top of it)
#USE_WAFFLE_MODEL  = True    # try the course Turtlebot3Waffle model; falls back to a simple block
DECK_Z            = 0.141   # height of the TurtleBot's top deck = height of the arm's base (m)
TURTLE_START_BACK = 0.60    # starts this far BEHIND the working position (m)
TURTLE_SPEED      = 0.15    #speed (m/s)
TABLE_HEIGHT      = 0.50    # height of the table top above the floor (m)

# FIX: this arm can't fold closer than ~0.62 m from its shoulder joint, so with the
# table at 0.5 m the TurtleBot must park further back than before. Tested in
# simulation: the pre-pickup point works best ~0.70 m in front of the arm base
# (0.68-0.72 all fine). From there it can push ~15 cm forward cleanly;
# much past ~0.90 m in front of the base it runs out of reach and stalls short.
PRE_REACH         = 0.70    # arm base -> pre-pickup point, horizontally (m)
WORK_DISTANCE     = None    # tray near edge -> arm base distance when picking up.
                            # None = PRE_REACH + PRE_DISTANCE

# FIX: was -90. READY_POSES_DEG has no -90 entry (KeyError), and a -90 roll asks for
# the gripper pointing UP, 180 deg away from the ready pose. +90 = pointing down.
ROLL_DEG = 90

READY_POSES_DEG = {
    0:  [3, -31, 62, 115, -131, -2],
    90: [-157.6, 28.8, 23.5, -30.7, 142.0, 15.8],
}
Q_REF_DEG = READY_POSES_DEG[0]     # orientation that roll produces

# If the tray mesh can't be read, set its bounds by hand:
# ((xmin, ymin, zmin), (xmax, ymax, zmax)) relative to the mesh origin.
TRAY_BOUNDS_OVERRIDE = None
#TRAY_BOUNDS_FALLBACK = (np.array([-0.15, -0.10, 0.0]), np.array([0.15, 0.10, 0.03]))


# ============================================================
# 1. SWIFT AND ROBOT
# ============================================================

env = swift.Swift()
env.launch(realtime=True)

robot = HannahBot()

q_start = np.deg2rad(READY_POSES_DEG[ROLL_DEG])

robot.q = q_start.copy()
robot.add_to_env(env)
env.step(0.05)


# ============================================================
# 2. TABLES
# ============================================================

scene_parts = []        # (object, original 4x4 pose) so the scene can be shifted live


table_height = TABLE_HEIGHT        # FIX: was 0.65 + DECK_Z (= 0.791 m)
table_length = 0.30
table_width = 0.30
table_thickness = 0.04

table1_x, table1_y = 0.75, 0
table2_x, table2_y = 0.75, -0.30


def create_table(x, y):

    tabletop = Cuboid(
        scale=[table_length, table_width, table_thickness],
        pose=SE3(x, y, table_height - table_thickness / 2),
        color=[0.55, 0.55, 0.55, 1]
    )
    env.add(tabletop)
    scene_parts.append((tabletop, np.array(tabletop.T, dtype=float)))

    leg_height = table_height - table_thickness

    for dx in (-0.12, 0.12):
        for dy in (-0.12, 0.12):
            leg = Cuboid(
                scale=[0.025, 0.025, leg_height],
                pose=SE3(x + dx, y + dy, leg_height / 2),
                color=[0.35, 0.35, 0.35, 1]
            )
            env.add(leg)
            scene_parts.append((leg, np.array(leg.T, dtype=float)))


create_table(table1_x, table1_y)
create_table(table2_x, table2_y)


# ============================================================
# 3. LOAD THE YELLOW TRAY AND MEASURE IT
# ============================================================

project_folder = Path(__file__).resolve().parent.parent

tray_path = (
    project_folder
    / "Yara_Work"
    / "EnviroParts"
    / "Tray.dae"
)

if not tray_path.is_file():
    raise FileNotFoundError(f"Tray mesh not found: {tray_path}")

tray = Mesh(
    filename=str(tray_path),
    pose=SE3(0.65, 0.0, table_height),
)

env.add(tray)
scene_parts.append((tray, np.array(tray.T, dtype=float)))


def dae_bounds(path):
    """Axis-aligned bounds (lo, hi) of a COLLADA (.dae) mesh, in metres,
    in Swift's world axes (Z up). Raises on failure."""
    root = ET.parse(path).getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""
    T = lambda name: ns + name

    unit = 1.0
    u = root.find(f".//{T('unit')}")
    if u is not None and u.get("meter"):
        unit = float(u.get("meter"))
    up = (root.findtext(f".//{T('up_axis')}") or "Z_UP").strip()

    geoms = {}
    for g in root.iter(T("geometry")):
        mesh = g.find(T("mesh"))
        if mesh is None:
            continue
        sources = {s.get("id"): s for s in mesh.findall(T("source"))}
        pos_id = None
        v = mesh.find(T("vertices"))
        if v is not None:
            for inp in v.findall(T("input")):
                if inp.get("semantic") == "POSITION":
                    pos_id = inp.get("source").lstrip("#")
        if pos_id in sources:
            arr = sources[pos_id].find(T("float_array"))
            geoms[g.get("id")] = np.array(arr.text.split(), float).reshape(-1, 3)
    if not geoms:
        raise ValueError("no mesh geometry found")

    def node_matrix(node):
        M = np.eye(4)
        for ch in node:
            tag = ch.tag.replace(ns, "")
            vals = np.array(ch.text.split(), float) if ch.text else None
            if tag == "matrix":
                M = M @ vals.reshape(4, 4)
            elif tag == "translate":
                A = np.eye(4); A[:3, 3] = vals; M = M @ A
            elif tag == "scale":
                M = M @ np.diag([*vals, 1.0])
            elif tag == "rotate":
                ax, ang = vals[:3], np.deg2rad(vals[3])
                ax = ax / np.linalg.norm(ax)
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                A = np.eye(4); A[:3, :3] = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
                M = M @ A
        return M

    pts = []

    def walk(node, parent):
        M = parent @ node_matrix(node)
        for ig in node.findall(T("instance_geometry")):
            gid = ig.get("url").lstrip("#")
            if gid in geoms:
                P = np.c_[geoms[gid], np.ones(len(geoms[gid]))] @ M.T
                pts.append(P[:, :3])
        for child in node.findall(T("node")):
            walk(child, M)

    scene = root.find(f".//{T('visual_scene')}")
    if scene is not None:
        for n in scene.findall(T("node")):
            walk(n, np.eye(4))
    if not pts:
        pts = list(geoms.values())

    P = np.vstack(pts) * unit
    if up == "Y_UP":
        P = np.c_[P[:, 0], -P[:, 2], P[:, 1]]
    return P.min(axis=0), P.max(axis=0)


# Tray bounds relative to the mesh origin
if TRAY_BOUNDS_OVERRIDE is not None:
    tray_lo = np.array(TRAY_BOUNDS_OVERRIDE[0], dtype=float)
    tray_hi = np.array(TRAY_BOUNDS_OVERRIDE[1], dtype=float)
    print("Tray bounds: using TRAY_BOUNDS_OVERRIDE")
else:
    # try:
        tray_lo, tray_hi = dae_bounds(tray_path)
        print("Tray bounds read from the mesh file.")
    # except Exception as e:
    #     tray_lo, tray_hi = TRAY_BOUNDS_FALLBACK
    #     print("WARNING: could not read tray mesh bounds (", e, ")")
    #     print("         Using a guessed tray size. Set TRAY_BOUNDS_OVERRIDE to fix.")

print("Tray size (m):", np.round(tray_hi - tray_lo, 3).tolist(),
      "| bounds relative to mesh origin:",
      np.round(tray_lo, 3).tolist(), np.round(tray_hi, 3).tolist())


def apply_shift():
    for obj, T0 in scene_parts:
        T = T0.copy()
        T[:3, 3] += TRAY_SHIFT
        obj.T = T
    env.step(0.05)


apply_shift()


# ============================================================
# 3b. TURTLEBOT + DRIVING
# ============================================================

waffle = None
turtle_parts = []          # fallback block: (object, original 4x4 pose)
turtle_x = 0.0
_waffle_warned = False


def build_turtlebot():
    global waffle
    #if USE_WAFFLE_MODEL and Turtlebot3Waffle is not None:
    #     try:
    #         waffle = Turtlebot3Waffle()
    #         waffle.add_to_env(env)
    #         print("TurtleBot: using the Turtlebot3Waffle model.")
    #         return
    #     except Exception as e:
    #         waffle = None
    #         print("TurtleBot: could not use Turtlebot3Waffle (", e, ") - using a simple block.")
    # else:
    #     print("TurtleBot: using a simple block (Waffle model off or not installed).")

    # Waffle-sized block: 0.28 x 0.31 m, from 2 cm off the floor up to the deck
    body_h = DECK_Z - 0.02
    body = Cuboid(
        scale=[0.28, 0.31, body_h],
        pose=SE3(0, 0, 0.02 + body_h / 2),
        color=[0.15, 0.15, 0.17, 1]
    )
    env.add(body)
    turtle_parts.append((body, np.array(body.T, dtype=float)))


def place_turtle(x):
    """Put the TurtleBot, and the arm riding on it, at world x (facing +X)."""
    global turtle_x, _waffle_warned
    turtle_x = x

    # arm base sits on the deck
    robot.base = SE3(x, 0.0, DECK_Z)
    robot.update_meshes()

    if waffle is not None:
        try:
            waffle.base = SE3(x, 0.0, 0.0)
            if hasattr(waffle, "_update_3dmodel"):
                waffle._update_3dmodel()
            else:
                waffle.q = waffle.q
        except Exception as e:
            if not _waffle_warned:
                print("WARNING: could not move the Waffle model (", e, ")")
                print("         Set USE_WAFFLE_MODEL = False to use the simple block.")
                _waffle_warned = True

    for obj, T0 in turtle_parts:
        T = T0.copy()
        T[0, 3] += x
        obj.T = T


def drive_to(x_target):
    """Drive straight along world X with a smooth start and stop."""
    x0 = turtle_x
    dist = x_target - x0
    if abs(dist) < 1e-4:
        return True
    print(f"\n=== DRIVE {dist * 100:+.0f} cm along X ===")
    duration = max(abs(dist) / TURTLE_SPEED * 1.5, 1.0)   # smooth profile peaks at 1.5x average
    steps = max(int(duration / dt), 2)
    for k in range(steps + 1):
        u = k / steps
        place_turtle(x0 + dist * (3 * u**2 - 2 * u**3))
        env.step(dt)
    print(f"Turtlebot at x = {turtle_x:.3f} m")
    return True


build_turtlebot()


# ============================================================
# 4. TARGETS FROM THE TRAY
# ============================================================

def tray_world_bounds():
    origin = np.array(tray.T, dtype=float)[:3, 3]
    return origin + tray_lo, origin + tray_hi


def work_x():
    """World x where the TurtleBot stops: a fixed distance from the tray's near edge."""
    return tray_world_bounds()[0][0] - WORK_DISTANCE


def targets():
    """Return (pre_pickup_xyz, pickup_xyz) computed from the tray's near edge."""
    lo, hi = tray_world_bounds()
    edge_x = lo[0]                              # edge closest to the robot
    y = 0.5 * (lo[1] + hi[1]) + TOOL_Y_OFFSET   # centre line of the tray
    z = lo[2] + TOOL_HEIGHT
    pre_xyz = np.array([edge_x - PRE_DISTANCE, y, z])
    pick_xyz = np.array([edge_x + PICKUP_INSET, y, z])
    return pre_xyz, pick_xyz


# ============================================================
# 5. CONTROLLER SETTINGS
# ============================================================

Kp_pos = 2.0
Kp_rot = 2.0
max_joint_speed = 0.8
damping = 0.05
dt = 0.05

q_min = robot.qlim[0, :]
q_max = robot.qlim[1, :]

# Target orientation = reference orientation turned about world X
R_ref = robot.measured_fkine(np.deg2rad(Q_REF_DEG)).R


def make_target_R(roll_deg):
    return Rotation.from_euler("x", roll_deg, degrees=True).as_matrix() @ R_ref


R_target = make_target_R(ROLL_DEG)

# Distance from the tray's near edge to the arm base at the pickup position.
# Default = the distance of the original layout (arm base at x = 0).
if WORK_DISTANCE is None:
    WORK_DISTANCE = PRE_REACH + PRE_DISTANCE
print(f"Working distance (tray near edge to arm base): {WORK_DISTANCE:.3f} m")

# Start further back
place_turtle(work_x() - TURTLE_START_BACK)
env.step(0.05)


# ============================================================
# 6. KINEMATICS
# ============================================================

def ee_pose(q=None):
    return robot.measured_fkine(robot.q if q is None else q)


def jacobian(q):
    q = np.asarray(q, dtype=float)
    J = np.zeros((6, 6))
    eps = 1e-5
    R0 = ee_pose(q).R

    for j in range(6):
        qp, qm = q.copy(), q.copy()
        qp[j] += eps
        qm[j] -= eps
        Tp, Tm = ee_pose(qp), ee_pose(qm)

        J[:3, j] = (Tp.t - Tm.t) / (2 * eps)

        W = ((Tp.R - Tm.R) / (2 * eps)) @ R0.T
        J[3:, j] = 0.5 * np.array([
            W[2, 1] - W[1, 2],
            W[0, 2] - W[2, 0],
            W[1, 0] - W[0, 1],
        ])
    return J


def rotation_error(target_R, current_R):
    return Rotation.from_matrix(target_R @ current_R.T).as_rotvec()


# FIX: only the gripper's POINTING direction (its z-axis) is controlled now; its spin
# about that axis is left free. This arm has no real elbow (joints 3 and 4 both turn
# about the upper arm's own axis), so it cannot hold one exact full 3-D orientation
# near a low table - with the full orientation locked, no joint solution exists there.
def pointing_axis():
    return R_target[:, 2]


def pointing_error(T, axis=None):
    """Angular-velocity vector that turns the tool z-axis onto the target axis."""
    a = T.R[:, 2]
    return np.cross(a, pointing_axis() if axis is None else axis)


def orientation_error_deg(T):
    """Tilt between the gripper's pointing axis and the target axis (deg)."""
    c = np.clip(T.R[:, 2] @ pointing_axis(), -1.0, 1.0)
    return np.rad2deg(np.arccos(c))


def jacobian5(q, T):
    """Jacobian with the spin about the tool axis removed (position + pointing)."""
    J = jacobian(q)
    P = np.eye(3) - np.outer(T.R[:, 2], T.R[:, 2])
    return np.vstack([J[:3], P @ J[3:]])


q_mid = 0.5 * (q_min + q_max)
q_half = 0.5 * (q_max - q_min)


def limit_avoid(q):
    """Joint velocity that gently pulls joints back towards the middle of their range."""
    return -0.5 * (q - q_mid) / q_half**2


def solve_ik(target_xyz, seeds=60, rng=np.random.default_rng(0)):
    """Joint angles putting the tool at target_xyz, pointing along the target axis.
    Tries the current pose first, then random starts. Returns None if unreachable."""
    target_xyz = np.asarray(target_xyz, dtype=float)
    best = None
    for s in range(seeds + 1):
        q = robot.q.copy() if s == 0 else rng.uniform(q_min, q_max)
        for _ in range(150):
            T = ee_pose(q)
            e = np.r_[target_xyz - T.t, pointing_error(T)]
            if np.linalg.norm(e[:3]) < 1e-3 and np.linalg.norm(e[3:]) < 0.01:
                break
            J = jacobian5(q, T)
            q = np.clip(q + 0.6 * J.T @ np.linalg.solve(J @ J.T + 0.01 * np.eye(6), e),
                        q_min, q_max)
        if np.linalg.norm(e[:3]) < 1e-3 and np.linalg.norm(e[3:]) < 0.01:
            margin = np.min(np.minimum(q - q_min, q_max - q))
            if best is None or margin > best[1]:
                best = (q, margin)
            if margin > np.deg2rad(45):      # comfortably inside the limits: good enough
                break
    return None if best is None else best[0]


def joint_move(q_goal, duration=4.0):
    """Smooth joint-space move (used to get into a good arm configuration)."""
    q0 = robot.q.copy()
    steps = max(int(duration / dt), 2)
    for k in range(steps + 1):
        u = k / steps
        robot.q = q0 + (3 * u**2 - 2 * u**3) * (q_goal - q0)
        robot.update_meshes()
        env.step(dt)


# ============================================================
# 7. ONE RMRC MOVE (position + orientation, 6D)
# ============================================================

def move_to(target_xyz, duration, name):
    print(f"\n=== {name} ===")

    target_xyz = np.asarray(target_xyz, dtype=float)

    T0 = ee_pose()
    steps = max(int(duration / dt), 2)

    u = np.linspace(0, 1, steps)
    s = 3 * u**2 - 2 * u**3                      # smooth 0 -> 1

    # Straight line in XYZ
    path_xyz = T0.t + s[:, None] * (target_xyz - T0.t)

    # Smooth turn of the pointing axis from the current one to the target one
    a0, a1 = T0.R[:, 2], pointing_axis()
    turn = np.cross(a0, a1)
    ang = np.arctan2(np.linalg.norm(turn), a0 @ a1)
    turn = turn / np.linalg.norm(turn) * ang if np.linalg.norm(turn) > 1e-9 else np.zeros(3)
    path_axis = [Rotation.from_rotvec(si * turn).apply(a0) for si in s]

    for k in range(steps + 300):                  # settle up to 15 s
        q = robot.q.copy()
        T = ee_pose(q)

        # Stop early once the target is reached
        if (k >= steps - 1
                and np.linalg.norm(T.t - target_xyz) < 0.005
                and orientation_error_deg(T) < 2.0):
            break

        if k < steps - 1:
            goal_xyz, goal_axis = path_xyz[k], path_axis[k]
            v_ff = (path_xyz[k + 1] - path_xyz[k]) / dt
        else:
            goal_xyz, goal_axis = target_xyz, pointing_axis()
            v_ff = np.zeros(3)

        twist = np.concatenate([
            v_ff + Kp_pos * (goal_xyz - T.t),
            Kp_rot * pointing_error(T, goal_axis),
        ])

        J = jacobian5(q, T)
        J_pinv = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(6))
        qdot = J_pinv @ twist
        # use the free spin (null space) to stay away from joint limits
        qdot += (np.eye(6) - J_pinv @ J) @ limit_avoid(q)

        peak = np.max(np.abs(qdot))
        if peak > max_joint_speed:
            qdot *= max_joint_speed / peak

        q_next = q + qdot * dt
        if np.any(q_next < q_min) or np.any(q_next > q_max):
            print("STOP: joint limit reached at", np.round(T.t, 3))
            print("Joints (deg):", np.round(np.rad2deg(robot.q), 1))
            print("Hint: this point may be out of reach. The arm can't get closer "
                  "than ~0.62 m to its shoulder, nor much past ~0.95 m in front "
                  "of its base with the gripper pointing down.")
            return False

        robot.q = q_next
        robot.update_meshes()
        env.step(dt)

    T = ee_pose()
    p_err = np.linalg.norm(T.t - target_xyz) * 1000
    r_err = orientation_error_deg(T)
    print(f"Position error: {p_err:.1f} mm | Orientation error: {r_err:.2f} deg")
    print("Tool XYZ:", np.round(T.t, 4))
    print("Joints (deg):", np.round(np.rad2deg(robot.q), 1))
    return p_err < 10 and r_err < 3


# ============================================================
# 8. COMMANDS
# ============================================================

def pre():
    if abs(turtle_x - work_x()) > 0.02:
        print("The TurtleBot is not at the working position yet. Type 'drive' first.")
        return False
    pre_xyz, _ = targets()
    print("Pre-pickup target:", np.round(pre_xyz, 3).tolist(),
          f"({PRE_DISTANCE * 100:.0f} cm before the tray edge)")

    # FIX: a straight Cartesian line from the side-facing ready pose gets stuck
    # (~40 cm short at 0.5 m table height). Solve for a good arm configuration at
    # the pre-pickup point first and swing there in joint space; RMRC then only
    # does the final settle and the straight approach.
    q_pre = solve_ik(pre_xyz)
    if q_pre is None:
        print("Pre-pickup point is out of reach. Check PRE_REACH / WORK_DISTANCE "
              "(the arm can't get closer than ~0.62 m to its shoulder).")
        return False
    print("\n=== SWING TO PRE-PICKUP CONFIGURATION ===")
    joint_move(q_pre, 4.0)
    return move_to(pre_xyz, 1.0, f"PRE-PICKUP (roll {ROLL_DEG})")


def forward():
    pre_xyz, pick_xyz = targets()
    if np.linalg.norm(ee_pose().t - pre_xyz) > 0.03:     # FIX: was 0.30 (30 cm, not 30 mm)
        print("Not at pre-pickup. Run 'pre' until position error is under 30 mm.")
        return False
    travel = (pick_xyz[0] - pre_xyz[0]) * 100
    print("Pickup target:", np.round(pick_xyz, 3).tolist(), f"(moving {travel:.0f} cm along +X)")
    return move_to(pick_xyz, 6.0, "STRAIGHT APPROACH")


def drive():
    return drive_to(work_x())


def back():
    return drive_to(work_x() - TURTLE_START_BACK)


def auto():
    """Drive in, then pre-pickup, then the straight approach."""
    drive()
    if not pre():
        print("Stopped: pre-pickup failed.")
        return False
    return forward()


def go(x):
    T = ee_pose()
    return move_to(np.array([x, T.t[1], T.t[2]]), 4.0, f"GO TO X={x}")


def goto(x, y, z):
    return move_to(np.array([x, y, z]), 4.0, f"GOTO ({x}, {y}, {z})")


def shift(dx, dy, dz):
    """Slide tables + tray. (dz leaves the legs short - bake it in via TRAY_SHIFT.)"""
    global TRAY_SHIFT
    TRAY_SHIFT = TRAY_SHIFT + np.array([dx, dy, dz])
    apply_shift()
    print("TRAY_SHIFT =", np.round(TRAY_SHIFT, 3).tolist(),
          " (paste into the TRAY_SHIFT line near the top)")
    status()


def set_param(name, value):
    global PRE_DISTANCE, PICKUP_INSET, TOOL_HEIGHT, TOOL_Y_OFFSET
    if name == "standoff":
        PRE_DISTANCE = value
    elif name == "inset":
        PICKUP_INSET = value
    elif name == "height":
        TOOL_HEIGHT = value
    elif name == "yoff":
        TOOL_Y_OFFSET = value
    else:
        print("Use: set standoff|inset|height|yoff <metres>")
        return
    print(f"PRE_DISTANCE={PRE_DISTANCE}  PICKUP_INSET={PICKUP_INSET}  "
          f"TOOL_HEIGHT={TOOL_HEIGHT}  TOOL_Y_OFFSET={TOOL_Y_OFFSET}")
    status()


def home():
    q0 = robot.q.copy()
    for s in np.linspace(0, 1, 150):
        robot.q = q0 + (3 * s**2 - 2 * s**3) * (q_start - q0)
        robot.update_meshes()
        env.step(0.04)


# def set_roll(deg):
#     """Jump to the ready pose for this roll so you can compare."""
#     global ROLL_DEG, q_start, R_target
#     if deg not in READY_POSES_DEG:
#         print("Roll must be one of:", list(READY_POSES_DEG))
#         return
#     ROLL_DEG = deg
#     q_start = np.deg2rad(READY_POSES_DEG[deg])
#     R_target = make_target_R(deg)
#     robot.q = q_start.copy()
#     robot.update_meshes()
#     env.step(0.05)
#     print(f"Roll set to {deg} deg.")
#     status()


def status():
    T = ee_pose()
    lo, hi = tray_world_bounds()
    pre_xyz, pick_xyz = targets()
    print("\n--- STATUS ---")
    print("TurtleBot x:", round(turtle_x, 3), "| working position x:", round(work_x(), 3))
    print("Tool XYZ:", np.round(T.t, 4))
    print("Tray in world: x", np.round([lo[0], hi[0]], 3).tolist(),
          "y", np.round([lo[1], hi[1]], 3).tolist(),
          "z", np.round([lo[2], hi[2]], 3).tolist())
    print("Near edge x:", round(float(lo[0]), 3))
    print("Pre-pickup :", np.round(pre_xyz, 3).tolist())
    print("Pickup     :", np.round(pick_xyz, 3).tolist())
    print("TRAY_SHIFT :", np.round(TRAY_SHIFT, 3).tolist())
    print("Roll", ROLL_DEG, "deg | pointing error:",
          round(orientation_error_deg(T), 2), "deg")
    print("Joints (deg):", np.round(np.rad2deg(robot.q), 1))


# ============================================================
# 9. MAIN INPUT LOOP
# ============================================================

print("""
Commands:
  auto                      drive in, pre-pickup, then straight approach
  drive | back              TurtleBot to the working position / back to the start
  pre                       move to PRE_DISTANCE before the tray's near edge
  forward                   straight approach to the edge (+ inset)
  set standoff|inset|height|yoff <m>   tune the approach
  shift dx dy dz            slide tables + tray
  go <x> | goto x y z       manual moves (keep orientation)
  #roll <0|90>   home   status   quit
""")
status()

while True:
    try:
        cmd = input("\nHannahBot > ").strip().lower()
        parts = cmd.replace(",", " ").split()

        if cmd == "auto":
            auto()
        elif cmd == "drive":
            drive()
        elif cmd == "back":
            back()
        elif cmd == "pre":
            pre()
        elif cmd == "forward":
            forward()
        elif cmd.startswith("go "):
            go(float(parts[1]))
        elif cmd.startswith("goto "):
            goto(*[float(v) for v in parts[1:4]])
        elif cmd.startswith("shift "):
            shift(*[float(v) for v in parts[1:4]])
        elif cmd.startswith("set "):
            set_param(parts[1], float(parts[2]))
        # elif cmd.startswith("roll"):
        #     set_roll(int(float(parts[1])) if len(parts) > 1 else -1)
        elif cmd == "home":
            home()
        elif cmd == "status":
            status()
        elif cmd in ("quit", "exit"):
            break
        elif cmd:
            print("Unknown command.")

    except (ValueError, IndexError, np.linalg.LinAlgError) as e:
        print("Command error:", e)
    except KeyboardInterrupt:
        break

print("Simulation finished.")