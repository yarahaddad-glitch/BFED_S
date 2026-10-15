# import numpy as np
# import swift
# import xml.etree.ElementTree as ET
# from pathlib import Path
# from scipy.spatial.transform import Rotation, Slerp
# from spatialmath import SE3
# from spatialgeometry import Mesh, Cuboid
# from ir_support_extra_robots import Turtlebot3Waffle
# from Class_HannahBot import HannahBot

# Pre_distance = 0.2
# pick_distance = -0.12

# turtle_base = 0.13
# # turtle_start = 0.8
# turtle_speed = 0.15

# roll = 90 #deg

# READY_POSES_DEG = {
#     0:  [3, -31, 62, 115, -131, -2],
#     90: [-157.6, 28.8, 23.5, -30.7, 142.0, 15.8],
# }
# Q_REF_DEG = READY_POSES_DEG[0]     # orientation that roll produces


# env = swift.Swift()
# env.launch(realtime=True)
# turtlebot = Turtlebot3Waffle()
# hannahbot = HannahBot()

# q_start = np.deg2rad(READY_POSES_DEG[roll])

# hannahbot.q = q_start.copy()
# hannahbot.add_to_env(env)
# env.step(0.02)

# tray_length = np.array([-0.15, -0.10, 0.00])
# tray_width = np.array([0.15, 0.10, 0.03])
# tray_height = np.array([0.00, 0.00, 0.05])


# turtlebot = Turtlebot3Waffle()
# turtlebot.base = SE3(1,0,0)  #change in sim
# turtlebot.add_to_env(env)






# import numpy as np
# from scipy.spatial.transform import Rotation, Slerp
# from spatialmath import SE3
# from ir_support_extra_robots import Turtlebot3Waffle
# from Hannah_Work.Class_HannahBot import HannahBot

# READY_POSES_DEG = {
#     0: [3, -31, 62, 115, -131, -2],
#     90: [-157.6, 28.8, 23.5, -30.7, 142.0, 15.8],
# }


# def feed_iso(env, tray, work_distance=0.50, start_back=0.80,
#              pre_distance=0.20, pickup_inset=-0.12,
#              tool_height=0.00, y_offset=0.09, deck_z=0.13, roll=90):
#     """Drive HannahBot on a TurtleBot to an existing yellow tray, then approach it.

#     The tray is already in `env`; this function does not create another tray or
#     another Swift window. The approach is along world +X, from the tray's -X side.
#     Returns (hannahbot, turtlebot, success).
#     """
#     dt = 0.05
#     turtle_speed = 0.15
#     kp_pos, kp_rot = 2.0, 2.0
#     max_joint_speed, damping = 0.8, 0.05

#     # Bounds of tray_yellow.dae, relative to its mesh origin (metres).
#     # Adjust these if the yellow mesh has a different size/origin.
#     tray_lo = np.array([-0.15, -0.10, 0.00])
#     tray_hi = np.array([0.15, 0.10, 0.03])
#     origin = np.asarray(tray.T, dtype=float)[:3, 3]
#     lo, hi = origin + tray_lo, origin + tray_hi

#     edge_x = lo[0]
#     centre_y = (lo[1] + hi[1]) / 2
#     pick_z = hi[2] + tool_height
#     pre_xyz = np.array([edge_x - pre_distance, centre_y, pick_z])
#     pick_xyz = np.array([edge_x + pickup_inset, centre_y, pick_z])
#     working_x = edge_x - work_distance
#     starting_x = working_x - start_back
#     robot_y = centre_y - y_offset

#     if roll not in READY_POSES_DEG:
#         raise ValueError("roll must be 0 or 90")
#     if work_distance <= 0 or start_back < 0 or pre_distance <= 0:
#         raise ValueError("Check work_distance, start_back, pre_distance and pickup_inset")

#     turtlebot = Turtlebot3Waffle()
#     hannahbot = HannahBot()
#     hannahbot.q = np.deg2rad(READY_POSES_DEG[roll]).copy()

#     def place_turtle(x):
#         # Both objects share X and Y. HannahBot's base is raised onto the deck.
#         turtlebot.base = SE3(x, robot_y, 0)
#         if hasattr(turtlebot, "_update_3dmodel"):
#             turtlebot._update_3dmodel()
#         else:
#             turtlebot.q = turtlebot.q
#         hannahbot.base = SE3(x, robot_y, deck_z)
#         hannahbot.update_meshes()

#     # Add both robots once, then update their positions together.
#     turtlebot.base = SE3(starting_x, robot_y, 0)
#     hannahbot.base = SE3(starting_x, robot_y, deck_z)
#     turtlebot.add_to_env(env)
#     hannahbot.add_to_env(env)
#     place_turtle(starting_x)
#     env.step(dt)

#     # This is the existing gripper orientation used by the standalone test.
#     ref_R = hannahbot.measured_fkine(np.deg2rad(READY_POSES_DEG[0])).R
#     target_R = Rotation.from_euler("x", roll, degrees=True).as_matrix() @ ref_R
#     q_min, q_max = hannahbot.qlim[0, :], hannahbot.qlim[1, :]

#     def pose(q=None):
#         return hannahbot.measured_fkine(hannahbot.q if q is None else q)

#     def jacobian(q):
#         J = np.zeros((6, 6))
#         eps = 1e-5
#         R0 = pose(q).R
#         for j in range(6):
#             qp, qm = q.copy(), q.copy()
#             qp[j] += eps
#             qm[j] -= eps
#             Tp, Tm = pose(qp), pose(qm)
#             J[:3, j] = (Tp.t - Tm.t) / (2 * eps)
#             W = ((Tp.R - Tm.R) / (2 * eps)) @ R0.T
#             J[3:, j] = 0.5 * np.array([
#                 W[2, 1] - W[1, 2],
#                 W[0, 2] - W[2, 0],
#                 W[1, 0] - W[0, 1],
#             ])
#         return J

#     def rotation_error(desired, actual):
#         return Rotation.from_matrix(desired @ actual.T).as_rotvec()

#     def move_to(xyz, duration, label):
#         print(f"\nHannahBot: {label} -> {np.round(xyz, 3)}")
#         T0 = pose()
#         steps = max(int(duration / dt), 2)
#         u = np.linspace(0, 1, steps)
#         s = 3 * u**2 - 2 * u**3
#         path_xyz = T0.t + s[:, None] * (xyz - T0.t)
#         path_rot = Slerp(
#             [0, 1], Rotation.from_matrix(np.stack([T0.R, target_R]))
#         )(s).as_matrix()

#         for k in range(steps + 300):
#             q = hannahbot.q.copy()
#             T = pose(q)
#             p_err = np.linalg.norm(T.t - xyz)
#             r_err = np.linalg.norm(rotation_error(target_R, T.R))
#             if k >= steps - 1 and p_err < 0.005 and r_err < np.deg2rad(2):
#                 break

#             if k < steps - 1:
#                 goal_xyz, goal_R = path_xyz[k], path_rot[k]
#                 v_ff = (path_xyz[k + 1] - path_xyz[k]) / dt
#                 w_ff = Rotation.from_matrix(
#                     path_rot[k + 1] @ path_rot[k].T
#                 ).as_rotvec() / dt
#             else:
#                 goal_xyz, goal_R = xyz, target_R
#                 v_ff, w_ff = np.zeros(3), np.zeros(3)

#             twist = np.r_[
#                 v_ff + kp_pos * (goal_xyz - T.t),
#                 w_ff + kp_rot * rotation_error(goal_R, T.R),
#             ]
#             J = jacobian(q)
#             qdot = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), twist)
#             peak = np.max(np.abs(qdot))
#             if peak > max_joint_speed:
#                 qdot *= max_joint_speed / peak
#             q_next = q + dt * qdot
#             if np.any(q_next < q_min) or np.any(q_next > q_max):
#                 print("STOP: HannahBot joint limit reached.")
#                 return False
#             hannahbot.q = q_next
#             hannahbot.update_meshes()
#             env.step(dt)

#         T = pose()
#         p_mm = 1000 * np.linalg.norm(T.t - xyz)
#         r_deg = np.rad2deg(np.linalg.norm(rotation_error(target_R, T.R)))
#         print(f"Position error: {p_mm:.1f} mm; orientation error: {r_deg:.2f} deg")
#         return p_mm < 10 and r_deg < 3

#     # Drive the TurtleBot; changing its base also moves the arm mounted on top.
#     distance = working_x - starting_x
#     duration = max(abs(distance) / turtle_speed * 1.5, 1.0)
#     steps = max(int(duration / dt), 2)
#     print(f"\nTurtleBot: driving {distance:.2f} m toward yellow tray")
#     for k in range(steps + 1):
#         u = k / steps
#         place_turtle(starting_x + distance * (3 * u**2 - 2 * u**3))
#         env.step(dt)

#     # Move above the point just before the tray, then lower vertically.
#     high_xyz = pre_xyz.copy()
#     high_xyz[2] = max(pose().t[2], hi[2] + 0.10)
#     if not move_to(high_xyz, 6.0, "HIGH STAGING"):
#         return hannahbot, turtlebot, False

#     print("HannahBot reached the approach target beside the existing yellow tray.")
#     return hannahbot, turtlebot, True




import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation, Slerp
from spatialmath import SE3
from spatialgeometry import Cuboid
from Hannah_Work.Class_HannahBot import HannahBot
 
try:
    from ir_support_extra_robots import Turtlebot3Waffle
except ImportError:
    Turtlebot3Waffle = None
 
READY_POSES_DEG = {
    0: [3, -31, 62, 115, -131, -2],
    90: [-157.6, 28.8, 23.5, -30.7, 142.0, 15.8],
}
Q_REF_DEG = READY_POSES_DEG[0]
 
# Parked arm base relative to the tray, measured as in the working test:
#   x: WORK_DISTANCE behind the tray's near edge (edge 0.65, base 0.00)
#   y: offset from the tray's centre line (centre -0.009, base 0.00)
WORK_DISTANCE = 0.65
BASE_Y_FROM_CENTRE = 0.009
# Tray origin height above the floor in the working test (0.75 + 0.13 + 0.02).
TRAY_Z_WORKING = 0.90
 
 
def dae_bounds(path):
    """Axis-aligned bounds (lo, hi) of a COLLADA mesh in metres, Z up."""
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
                A = np.eye(4)
                A[:3, :3] = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
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
 
 
def feed_iso(env, tray, tray_path,
             pre_distance=0.18, pickup_inset=-0.08,
             tool_height=0.009, tool_y_offset=0.00,
             deck_z=0.13, start_back=0.60, roll=90,
             use_waffle=False, interactive=True):
    """Drive HannahBot to `tray` (already in env) and approach it along world +X.
 
    tray_path   : path to the tray's .dae file (bounds are read from it)
    The tray should sit at z = 0.90 (as in the working test); the arm approaches
    from the tray's -X side, so keep that side clear.
    interactive : True = command prompt, False = run 'auto' and return.
    Returns (hannahbot, turtle, success).
    """
    dt = 0.05
    turtle_speed = 0.15
    kp_pos, kp_rot = 2.0, 2.0
    max_joint_speed, damping = 0.8, 0.05
 
    if roll not in READY_POSES_DEG:
        raise ValueError("roll must be 0 or 90")
 
    # ---------------- tray ----------------
    tray_lo, tray_hi = dae_bounds(str(tray_path))
    print("Tray size (m):", np.round(tray_hi - tray_lo, 3).tolist(),
          "| bounds from origin:", np.round(tray_lo, 3).tolist(),
          np.round(tray_hi, 3).tolist())
 
    tray_origin = np.asarray(tray.T, dtype=float)[:3, 3]
    if abs(tray_origin[2] - TRAY_Z_WORKING) > 0.005:
        print(f"NOTE: tray is at z = {tray_origin[2]:.3f}; the working test used "
              f"{TRAY_Z_WORKING:.3f}. Reach may differ.")
 
    params = {"standoff": pre_distance, "inset": pickup_inset,
              "height": tool_height, "yoff": tool_y_offset}
 
    def tray_bounds():
        origin = np.asarray(tray.T, dtype=float)[:3, 3]
        return origin + tray_lo, origin + tray_hi
 
    def targets():
        lo, hi = tray_bounds()
        edge_x = lo[0]
        y = 0.5 * (lo[1] + hi[1]) + params["yoff"]
        z = lo[2] + params["height"]
        return (np.array([edge_x - params["standoff"], y, z]),
                np.array([edge_x + params["inset"], y, z]))
 
    # Everything below is soft-coded to the tray already in the environment.
    lo0, hi0 = tray_bounds()
    working_x = lo0[0] - WORK_DISTANCE                       # from the near edge
    robot_y = 0.5 * (lo0[1] + hi0[1]) + BASE_Y_FROM_CENTRE   # from the centre line
    starting_x = working_x - start_back
 
    # ---------------- robots ----------------
    hannahbot = HannahBot()
    q_start = np.deg2rad(READY_POSES_DEG[roll])
    hannahbot.q = q_start.copy()
    hannahbot.base = SE3(starting_x, robot_y, deck_z)
    hannahbot.add_to_env(env)
 
    turtle = None
    if use_waffle and Turtlebot3Waffle is not None:
        try:
            turtle = Turtlebot3Waffle()
            turtle.base = SE3(starting_x, robot_y, 0)
            turtle.add_to_env(env)
        except Exception as e:
            print("Could not use Turtlebot3Waffle (", e, ") - using a block.")
            turtle = None
    block = None
    if turtle is None:
        body_h = deck_z - 0.02
        block = Cuboid(scale=[0.28, 0.31, body_h],
                       pose=SE3(starting_x, robot_y, 0.02 + body_h / 2),
                       color=[0.15, 0.15, 0.17, 1])
        env.add(block)
        turtle = block
 
    state = {"x": starting_x}
 
    def place_turtle(x):
        state["x"] = x
        hannahbot.base = SE3(x, robot_y, deck_z)
        hannahbot.update_meshes()
        if block is not None:
            block.T = SE3(x, robot_y, 0.02 + (deck_z - 0.02) / 2).A
        else:
            turtle.base = SE3(x, robot_y, 0)
            if hasattr(turtle, "_update_3dmodel"):
                turtle._update_3dmodel()
            else:
                turtle.q = turtle.q
 
    place_turtle(starting_x)
    env.step(dt)
 
    # ---------------- kinematics ----------------
    q_min, q_max = hannahbot.qlim[0, :], hannahbot.qlim[1, :]
    R_ref = hannahbot.measured_fkine(np.deg2rad(Q_REF_DEG)).R
    R_target = Rotation.from_euler("x", roll, degrees=True).as_matrix() @ R_ref
 
    def pose(q=None):
        return hannahbot.measured_fkine(hannahbot.q if q is None else q)
 
    def jacobian(q):
        q = np.asarray(q, dtype=float)
        J = np.zeros((6, 6))
        eps = 1e-5
        R0 = pose(q).R
        for j in range(6):
            qp, qm = q.copy(), q.copy()
            qp[j] += eps
            qm[j] -= eps
            Tp, Tm = pose(qp), pose(qm)
            J[:3, j] = (Tp.t - Tm.t) / (2 * eps)
            W = ((Tp.R - Tm.R) / (2 * eps)) @ R0.T
            J[3:, j] = 0.5 * np.array([W[2, 1] - W[1, 2],
                                       W[0, 2] - W[2, 0],
                                       W[1, 0] - W[0, 1]])
        return J
 
    def rotation_error(target_R, current_R):
        return Rotation.from_matrix(target_R @ current_R.T).as_rotvec()
 
    def orientation_error_deg(T):
        return np.rad2deg(np.linalg.norm(rotation_error(R_target, T.R)))
 
    def move_to(target_xyz, duration, name):
        print(f"\n=== {name} ===")
        target_xyz = np.asarray(target_xyz, dtype=float)
        T0 = pose()
        steps = max(int(duration / dt), 2)
        u = np.linspace(0, 1, steps)
        s = 3 * u**2 - 2 * u**3
        path_xyz = T0.t + s[:, None] * (target_xyz - T0.t)
        path_R = Slerp([0, 1], Rotation.from_matrix(np.stack([T0.R, R_target])))(s).as_matrix()
 
        for k in range(steps + 300):
            q = hannahbot.q.copy()
            T = pose(q)
            if (k >= steps - 1
                    and np.linalg.norm(T.t - target_xyz) < 0.005
                    and orientation_error_deg(T) < 2.0):
                break
            if k < steps - 1:
                goal_xyz, goal_R = path_xyz[k], path_R[k]
                v_ff = (path_xyz[k + 1] - path_xyz[k]) / dt
                w_ff = Rotation.from_matrix(path_R[k + 1] @ path_R[k].T).as_rotvec() / dt
            else:
                goal_xyz, goal_R = target_xyz, R_target
                v_ff = w_ff = np.zeros(3)
 
            twist = np.concatenate([v_ff + kp_pos * (goal_xyz - T.t),
                                    w_ff + kp_rot * rotation_error(goal_R, T.R)])
            J = jacobian(q)
            qdot = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), twist)
            peak = np.max(np.abs(qdot))
            if peak > max_joint_speed:
                qdot *= max_joint_speed / peak
            q_next = q + qdot * dt
            if np.any(q_next < q_min) or np.any(q_next > q_max):
                print("STOP: joint limit reached at", np.round(T.t, 3))
                print("Joints (deg):", np.round(np.rad2deg(hannahbot.q), 1))
                return False
            hannahbot.q = q_next
            hannahbot.update_meshes()
            env.step(dt)
 
        T = pose()
        p_err = np.linalg.norm(T.t - target_xyz) * 1000
        r_err = orientation_error_deg(T)
        print(f"Position error: {p_err:.1f} mm | Orientation error: {r_err:.2f} deg")
        print("Tool XYZ:", np.round(T.t, 4))
        print("Joints (deg):", np.round(np.rad2deg(hannahbot.q), 1))
        return p_err < 10 and r_err < 3
 
    # ---------------- commands ----------------
    def drive_to(x_target):
        x0 = state["x"]
        dist = x_target - x0
        if abs(dist) < 1e-4:
            return True
        print(f"\n=== DRIVE {dist * 100:+.0f} cm along X ===")
        duration = max(abs(dist) / turtle_speed * 1.5, 1.0)
        steps = max(int(duration / dt), 2)
        for k in range(steps + 1):
            u = k / steps
            place_turtle(x0 + dist * (3 * u**2 - 2 * u**3))
            env.step(dt)
        print(f"TurtleBot at x = {state['x']:.3f} m")
        return True
 
    def drive():
        return drive_to(working_x)
 
    def back():
        return drive_to(starting_x)
 
    def pre():
        if abs(state["x"] - working_x) > 0.02:
            print("The TurtleBot is not at the working position yet. Type 'drive' first.")
            return False
        pre_xyz, _ = targets()
        print("Pre-pickup target:", np.round(pre_xyz, 3).tolist(),
              f"({params['standoff'] * 100:.0f} cm before the tray edge)")
        return move_to(pre_xyz, 6.0, f"PRE-PICKUP (roll {roll})")
 
    def forward():
        pre_xyz, pick_xyz = targets()
        if np.linalg.norm(pose().t - pre_xyz) > 0.30:
            print("Not at pre-pickup. Run 'pre' first.")
            return False
        travel = (pick_xyz[0] - pre_xyz[0]) * 100
        print("Pickup target:", np.round(pick_xyz, 3).tolist(), f"(moving {travel:.0f} cm along +X)")
        return move_to(pick_xyz, 6.0, "STRAIGHT APPROACH")
 
    def auto():
        drive()
        if not pre():
            print("Stopped: pre-pickup failed.")
            return False
        return forward()
 
    def goto(x, y, z):
        return move_to(np.array([x, y, z]), 4.0, f"GOTO ({x}, {y}, {z})")
 
    def home():
        q0 = hannahbot.q.copy()
        for s in np.linspace(0, 1, 150):
            hannahbot.q = q0 + (3 * s**2 - 2 * s**3) * (q_start - q0)
            hannahbot.update_meshes()
            env.step(0.04)
 
    def status():
        T = pose()
        lo, hi = tray_bounds()
        pre_xyz, pick_xyz = targets()
        print("\n--- STATUS ---")
        print("TurtleBot x:", round(state["x"], 3), "| working position x:", round(working_x, 3))
        print("Tool XYZ:", np.round(T.t, 4))
        print("Tray in world: x", np.round([lo[0], hi[0]], 3).tolist(),
              "y", np.round([lo[1], hi[1]], 3).tolist(),
              "z", np.round([lo[2], hi[2]], 3).tolist())
        print("Pre-pickup :", np.round(pre_xyz, 3).tolist())
        print("Pickup     :", np.round(pick_xyz, 3).tolist())
        print("Params     :", params)
        print("Roll", roll, "deg | orientation error:", round(orientation_error_deg(T), 2), "deg")
        print("Joints (deg):", np.round(np.rad2deg(hannahbot.q), 1))
 
    if not interactive:
        return hannahbot, turtle, auto()
 
    print("""
Commands:
  auto                 drive in, pre-pickup, then straight approach
  drive | back         TurtleBot to the working position / back to the start
  pre                  move to pre-pickup
  forward              straight approach along +X
  set standoff|inset|height|yoff <m>
  goto x y z           manual move (orientation locked)
  home   status   quit
""")
    status()
    success = False
    while True:
        try:
            parts = input("\nHannahBot > ").strip().lower().replace(",", " ").split()
            if not parts:
                continue
            cmd = parts[0]
            if cmd == "auto":
                success = auto()
            elif cmd == "drive":
                drive()
            elif cmd == "back":
                back()
            elif cmd == "pre":
                pre()
            elif cmd == "forward":
                success = forward()
            elif cmd == "set" and len(parts) == 3 and parts[1] in params:
                params[parts[1]] = float(parts[2])
                status()
            elif cmd == "goto" and len(parts) == 4:
                goto(*[float(v) for v in parts[1:4]])
            elif cmd == "home":
                home()
            elif cmd == "status":
                status()
            elif cmd in ("quit", "exit"):
                break
            else:
                print("Unknown command.")
        except (ValueError, IndexError, np.linalg.LinAlgError) as e:
            print("Command error:", e)
        except KeyboardInterrupt:
            break
 
    return hannahbot, turtle, success