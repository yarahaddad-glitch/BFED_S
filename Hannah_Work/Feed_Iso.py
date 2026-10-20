import numpy as np
from scipy.spatial.transform import Rotation
from spatialmath import SE3
from spatialgeometry import Cuboid
 
try:
    from Hannah_Work.Class_HannahBot import HannahBot
except ImportError:
    from Class_HannahBot import HannahBot
 
try:
    from ir_support_extra_robots import Turtlebot3Waffle
except ImportError:
    Turtlebot3Waffle = None
 
# ---------------- settings ----------------
READY_POSE_DEG = [-157.6, 28.8, 23.5, -30.7, 142.0, 15.8]   # gripper sideways (roll 90)
REF_POSE_DEG = [3, -31, 62, 115, -131, -2]
DECK_Z = 0.13           # TurtleBot deck height = arm base height (m)
WORK_DISTANCE = 0.65    # arm base -> tray's near edge when picking up (m)
START_BACK = 0.60       # robot lines up this far behind the working position (m)
PRE_DISTANCE = 0.19    # pre-pickup point: before the tray's near edge (m)
PICKUP_INSET = -0.09    # pickup point: relative to the near edge (m)
LIFT = 0.05             # how far the tray is lifted, and lowered again (m)
LEAVE_DISTANCE = 1.20   # drive this far after turning round (m)
first_turn = -90        # turn after leaving (deg, negative = clockwise)
Travel_1 = 8.2          # how far to drive after first_turn, for tray 1, 2, 3 (m)
Travel_2 = 12.7
Travel_3 = 17
TRAVELS = [Travel_1, Travel_2, Travel_3]
Second_turn = 90        # turn before the last stretch (deg)
second_travel = 1.6       # last stretch before putting the tray down (m)
BACK_OFF = 1         # reverse this far after putting the tray down (m)
 
DT = 0.02
TOLERANCE = 0.03        # arm moves count as reached within 30 mm
 
# Tray corners relative to its origin, before the Mesh's scale (from read_tray_size.py)
TRAY_LO = [0.0, -0.198, 0.0]
TRAY_HI = [0.3, 0.0002, 0.018]
 
 
def tray_points(tray):
    """Pickup geometry for one tray: (edge_x, tray_y, grip_z)."""
    lo, hi = np.array(TRAY_LO), np.array(TRAY_HI)
    scale = getattr(tray, "scale", None)
    scale = np.ones(3) if scale is None else np.resize(np.ravel(np.asarray(scale, float)), 3)
    origin = np.asarray(tray.T, float)[:3, 3]
    lo, hi = origin + lo * scale, origin + hi * scale
    return lo[0], 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])
 
 
def feed_iso(env, trays, **unused):
    if not isinstance(trays, (list, tuple)):
        trays = [trays]
 
    # ---------------- robots, lined up with the first tray ----------------
    edge_x, tray_y, grip_z = tray_points(trays[0])
    robot = {"x": edge_x - WORK_DISTANCE - START_BACK, "y": tray_y, "yaw": 0.0}
    held = {}
 
    hannahbot = HannahBot()
    hannahbot.q = np.deg2rad(READY_POSE_DEG)
    hannahbot.base = SE3(robot["x"], robot["y"], DECK_Z)
    hannahbot.add_to_env(env)
 
    turtle = None
    if Turtlebot3Waffle is not None:
        try:
            turtle = Turtlebot3Waffle()
            turtle.base = SE3(robot["x"], robot["y"], 0)
            turtle.add_to_env(env)
        except Exception:
            turtle = None
    if turtle is None:   # simple stand-in body
        turtle = Cuboid(scale=[0.28, 0.31, DECK_Z - 0.02], color=[0.15, 0.15, 0.17, 1],
                        pose=SE3(robot["x"], robot["y"], DECK_Z / 2 + 0.01))
        env.add(turtle)
 
    # Gripper orientation stays locked (turns only with the robot)
    hannahbot.base = SE3(0, 0, DECK_Z)
    R_lock = Rotation.from_euler("x", 90, degrees=True).as_matrix() @ \
        hannahbot.measured_fkine(np.deg2rad(REF_POSE_DEG)).R
 
    def tool(q=None):
        return hannahbot.measured_fkine(hannahbot.q if q is None else q)
 
    def update():
        """Redraw the TurtleBot, the arm on it and (if held) the tray."""
        x, y, yaw = robot["x"], robot["y"], robot["yaw"]
        hannahbot.base = SE3(x, y, DECK_Z) * SE3.Rz(yaw)
        hannahbot.update_meshes()
        if isinstance(turtle, Cuboid):
            turtle.T = (SE3(x, y, DECK_Z / 2 + 0.01) * SE3.Rz(yaw)).A
        else:
            turtle.base = SE3(x, y, 0) * SE3.Rz(yaw)
            if hasattr(turtle, "_update_3dmodel"):
                turtle._update_3dmodel()
            else:
                turtle.q = turtle.q
        if held:
            Rz = SE3.Rz(yaw - held["yaw"]).R
            T = np.eye(4)
            T[:3, :3] = Rz @ held["R"]
            T[:3, 3] = tool().t + Rz @ held["offset"]
            held["tray"].T = T
        env.step(DT)
 
    def smooth(n):
        u = np.linspace(0, 1, n)
        return 3 * u**2 - 2 * u**3
 
    # ---------------- moves ----------------
    def drive(distance):
        print(f"Drive {distance * 100:.0f} cm")
        x0, y0, yaw = robot["x"], robot["y"], robot["yaw"]
        for s in smooth(max(int(abs(distance) / 3 * 1.5 / DT), 2)):
            robot["x"] = x0 + s * distance * np.cos(yaw)
            robot["y"] = y0 + s * distance * np.sin(yaw)
            update()
 
    def turn(deg):
        if abs(deg) < 0.5:
            return
        print(f"Turn {deg:+.0f} deg")
        yaw0 = robot["yaw"]
        for s in smooth(max(int(abs(deg) / 45 * 1.5 / DT), 2)):
            robot["yaw"] = yaw0 + s * np.deg2rad(deg)
            update()
 
    def face(yaw_deg):
        """Turn on the spot (shortest way) to face a heading."""
        turn((yaw_deg - np.rad2deg(robot["yaw"]) + 180) % 360 - 180)
 
    def go_to(x, y):
        """Turn towards (x, y) and drive straight there."""
        dx, dy = x - robot["x"], y - robot["y"]
        dist = np.hypot(dx, dy)
        if dist < 0.005:
            return
        face(np.rad2deg(np.arctan2(dy, dx)))
        drive(dist)
 
    def arm_to(target, seconds=6.0):
        """Straight-line move of the gripper with its orientation locked (RMRC)."""
        target = np.asarray(target, float)
        R_goal = SE3.Rz(robot["yaw"]).R @ R_lock
        start = tool().t
        steps = int(seconds / DT)
        line = start + smooth(steps)[:, None] * (target - start)
        qmin, qmax = hannahbot.qlim
        for k in range(steps + 200):
            q = hannahbot.q.copy()
            T = tool(q)
            goal = line[min(k, steps - 1)]
            if k >= steps and np.linalg.norm(T.t - target) < 0.005:
                break
            twist = np.r_[2 * (goal - T.t),
                          2 * Rotation.from_matrix(R_goal @ T.R.T).as_rotvec()]
            J = np.zeros((6, 6))                       # numerical Jacobian
            for j in range(6):
                dq = np.zeros(6); dq[j] = 1e-5
                Tp, Tm = tool(q + dq), tool(q - dq)
                J[:3, j] = (Tp.t - Tm.t) / 2e-5
                W = (Tp.R - Tm.R) / 2e-5 @ T.R.T
                J[3:, j] = 0.5 * np.array([W[2, 1] - W[1, 2], W[0, 2] - W[2, 0], W[1, 0] - W[0, 1]])
            qdot = J.T @ np.linalg.solve(J @ J.T + 0.05**2 * np.eye(6), twist)
            qdot *= min(1.0, 0.8 / np.max(np.abs(qdot)))
            q_next = q + qdot * DT
            if np.any(q_next < qmin) or np.any(q_next > qmax):
                print("  stopped at a joint limit")
                break
            hannahbot.q = q_next
            update()
        error = np.linalg.norm(tool().t - target)
        print(f"  error {error * 1000:.1f} mm")
        return error < TOLERANCE
 
    # ---------------- the run ----------------
    ok = True
    update()
    for i, tray in enumerate(trays):
        print(f"\n===== TRAY {i + 1} =====")
        edge_x, tray_y, grip_z = tray_points(tray)
 
        # Line up behind the tray, facing it, then drive in
        go_to(edge_x - WORK_DISTANCE - START_BACK, tray_y)
        face(0)
        drive(START_BACK)
 
        if i == 0:
            # First tray only: bring the arm into its pickup position
            print("Pre-pickup")
            arm_to([edge_x - PRE_DISTANCE, tray_y, grip_z])   # waypoint only
            print("Forward") 
            ok = arm_to([edge_x + PICKUP_INSET, tray_y, grip_z]) and ok
        else:
            # The arm is still in its pickup position from the last tray
            error = np.linalg.norm(tool().t - [edge_x + PICKUP_INSET, tray_y, grip_z])
            print(f"Arm already in place (error {error * 1000:.1f} mm)")
 
        print("Grab tray")
        T_tray = np.asarray(tray.T, float)
        held.update(tray=tray, offset=T_tray[:3, 3] - tool().t,
                    R=T_tray[:3, :3].copy(), yaw=robot["yaw"])
 
        print("Lift")
        lift_start = tool().t
        arm_to(lift_start + [0, 0, LIFT], seconds=3.0)   # may stop early at a joint limit
        lifted = tool().t[2] - lift_start[2]               # how far it actually went up
 
        # Carry it: remember each corner so the way back can retrace it
        route = []
        turn(180)
        drive(LEAVE_DISTANCE)
        route.append((robot["x"], robot["y"]))
        turn(first_turn)
        drive(TRAVELS[i] if i < len(TRAVELS) else TRAVELS[-1])
        route.append((robot["x"], robot["y"]))
        turn(Second_turn)
        drive(second_travel)
 
        # Put it down (the reverse of the pickup)
        print("Lower")
        arm_to(tool().t - [0, 0, lifted], seconds=3.0)
        print("Release tray")
        held.clear()
        print("Back away")
        drive(-BACK_OFF)
        turn(90)
        drive(TRAVELS[i] if i < len(TRAVELS) else TRAVELS[-1])
 
        # # Retrace the route back towards the trays
        # if i < len(trays) - 1:
        #     print("Heading back for the next tray")
        #     for x, y in reversed(route):
        #         go_to(x, y)
 
    print("\nfeed_iso done")
    return hannahbot, turtle, ok
 