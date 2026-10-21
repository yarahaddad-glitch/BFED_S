import numpy as np
from spatialmath import SE3
from scipy.optimize import least_squares
 
DT = 0.03
STEPS = 80
 
TOWELS = ["papertowel1", "papertowel2"]
TOWEL_DROP = (12.5, 3.8, 0.66)   # where the towels are put down (x, y, z)
STACK_HEIGHT = 0.0               # second towel this much higher (0 = same spot)
GRAB_HEIGHT = 0.10               # gripper this far above the towel's origin when holding it
HOVER = 0.15                     # approach from this far above
 
# Gripper direction: the same as in ReBot's original taught pickup pose
PICKUP_POSE = [1.1275, -2.1193, -0.7422, -1.4281, -0.0986, -0.0029]
 
 
# ---------------- ReBot helpers ----------------
def tool_pose(robot, q=None):
    """Gripper pose in the WORLD (ReBot's own kinematics are relative to its base)."""
    links = robot.get_link_transforms(robot.q if q is None else q)
    return robot.base * robot.get_gripper_transforms(links, robot.gripper_opening)[0]
 
 
def animate_joints(robot, env, target, item=None, attachment=None):
    """Smooth joint move to target; `item` follows the gripper if given."""
    start = robot.q.copy()
    target = np.asarray(target, dtype=float).reshape(6)
    for u in np.linspace(0, 1, STEPS):
        robot.q = start + (3 * u**2 - 2 * u**3) * (target - start)
        if item is not None:
            item.T = (tool_pose(robot) * attachment).A
        env.step(DT)
 
 
def solve_joints(robot, xyz, R_goal, q_start):
    """Joint angles that put the gripper at xyz, pointing straight down.
 
    The gripper may turn freely about the vertical (that's what makes every
    spot reachable). It first starts from q_start so the arm moves smoothly;
    if that gets stuck it tries a few other starting poses.
    """
    qmin = np.asarray(robot.qlim[0], float) + 1e-3     # stay just inside the limits
    qmax = np.asarray(robot.qlim[1], float) - 1e-3
    xyz = np.asarray(xyz, float)
    goal = np.r_[xyz, 0.3 * np.array([0.0, 0.0, -1.0])]   # position + "pointing down"
 
    def f(qq):
        T = tool_pose(robot, qq)
        return np.r_[T.t, 0.3 * T.R[:, 2]]
 
    def solve_from(q0):
        q = np.clip(np.asarray(q0, float), qmin, qmax)
        for _ in range(150):
            fq = f(q)
            e = goal - fq
            if np.linalg.norm(e) < 1e-3:
                break
            J = np.empty((6, 6))
            for j in range(6):
                dq = np.zeros(6)
                dq[j] = 1e-5 if q[j] + 1e-5 <= qmax[j] else -1e-5   # nudge away from a limit
                J[:, j] = (f(q + dq) - fq) / dq[j]
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 0.005 * np.eye(6), e), qmin, qmax)
        return q, np.linalg.norm(goal - f(q))
 
    q, miss = solve_from(q_start)
    if miss > 0.005:                                     # stuck: try other starting poses
        rng = np.random.default_rng(1)
        for q0 in [PICKUP_POSE] + [rng.uniform(qmin, qmax) for _ in range(6)]:
            q2, miss2 = solve_from(q0)
            if miss2 < miss:
                q, miss = q2, miss2
            if miss < 0.005:
                break
 
    gap = np.linalg.norm(xyz - tool_pose(robot, q).t) * 1000
    if gap > 10:
        raise RuntimeError(f"ReBot target unreachable: {gap:.0f} mm error at {np.round(xyz, 3).tolist()}")
    return q


FINGER_CONTACT = SE3(0.09, -0.0081, -0.026)
# Measured finger meshes extend along tool X, not tool Z.
GRASP_ROTATION = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])


def contact_pose(robot, q=None):
    return tool_pose(robot, q) * FINGER_CONTACT


def solve_contact(robot, xyz, seed):
    lo, hi = np.asarray(robot.qlim) + np.array([[0.001], [-0.001]])
    def error(q):
        pose = contact_pose(robot, q)
        return np.r_[pose.t - xyz,
                     0.2 * (pose.R[:, 2] - GRASP_ROTATION[:, 2])]
    rng = np.random.default_rng(3)
    best = None
    for q0 in [seed, PICKUP_POSE] + [rng.uniform(lo, hi) for _ in range(12)]:
        result = least_squares(error, np.clip(q0, lo, hi), bounds=(lo, hi),
                               max_nfev=200, ftol=1e-10, xtol=1e-10, gtol=1e-10)
        if best is None or np.linalg.norm(result.fun) < np.linalg.norm(best.fun):
            best = result
        if np.linalg.norm(result.fun[:3]) < .001 and np.linalg.norm(result.fun[3:]) < .002:
            return result.x
    raise RuntimeError(f"ReBot cannot grip towel at {np.round(xyz, 3)} "
                       f"(position error {np.linalg.norm(best.fun[:3])*1000:.1f} mm)")


def animate_gripper(robot, env, opening):
    for value in np.linspace(robot.gripper_opening, opening, 41)[1:]:
        robot.gripper_opening = value
        env.step(DT)
    for _ in range(12):
        env.step(DT)


def place_towel(env, robot, towel, drop_pose):
    """One towel per meal, using the supplied joint solver and animation."""
    print("[REBOT] Planning towel pickup and tray placement.", flush=True)
    start_pose = SE3(towel.T)
    up = np.zeros(3)  # solve for measured finger contact, not the wrist origin
    above = np.array([0, 0, HOVER])
    guess = robot.q.copy()
    targets = [start_pose.t + up + above, start_pose.t + up,
               start_pose.t + up + above, drop_pose.t + up + above,
               drop_pose.t + up]
    route = []
    for target in targets:
        guess = solve_contact(robot, target, guess)
        route.append(guess)
    animate_joints(robot, env, route[0])
    animate_gripper(robot, env, robot.GRIPPER_MAX)
    animate_joints(robot, env, route[1])
    if np.linalg.norm(contact_pose(robot).t - start_pose.t) > .002:
        raise RuntimeError("ReBot fingers have not reached the towel")
    # Project the towel width onto the actual jaw axis, including wrist yaw.
    jaw_axis = tool_pose(robot).R[:, 1]
    width = float(np.abs(start_pose.R.T @ jaw_axis) @ np.asarray(towel.scale))
    opening = (width - 0.0471) / 2
    if not robot.GRIPPER_MIN <= opening <= robot.GRIPPER_MAX:
        raise RuntimeError("Towel does not fit between ReBot's fingers")
    print("[REBOT] Closing fingers onto towel.", flush=True)
    animate_gripper(robot, env, opening)
    attachment = tool_pose(robot).inv() * start_pose
    for q in route[2:]:
        animate_joints(robot, env, q, towel, attachment)
    # Keep the achieved pose, rather than teleporting an unreachable towel.
    if np.linalg.norm(SE3(towel.T).t - drop_pose.t) > 0.015:
        raise RuntimeError("ReBot towel placement missed the tray")
    print("[REBOT] Opening fingers to release towel.", flush=True)
    animate_gripper(robot, env, robot.GRIPPER_MAX)
    env.step(DT)
    print("[REBOT] Towel placed on tray.", flush=True)
    animate_joints(robot, env, route[3])
    animate_joints(robot, env, robot.q_home)
    return towel
 
 
# ---------------- the paper towel job ----------------
def run_rebot_towels(env, robot, scene):
    robot.q = robot.q_home.copy()
    env.step(DT)
 
    # keep the gripper pointing the same way as the original pickup pose
    robot.q = np.asarray(PICKUP_POSE, float)
    R_down = tool_pose(robot).R
    robot.q = robot.q_home.copy()
    q_guess = np.asarray(PICKUP_POSE, float)
 
    for i, name in enumerate(TOWELS):
        towel = scene[name]
        towel_xyz = np.asarray(towel.T, float)[:3, 3]
        drop_xyz = np.array(TOWEL_DROP, float) + [0, 0, i * STACK_HEIGHT]
        up = np.array([0, 0, GRAB_HEIGHT])
 
        q_above_towel = solve_joints(robot, towel_xyz + up + [0, 0, HOVER], R_down, q_guess)
        q_at_towel = solve_joints(robot, towel_xyz + up, R_down, q_above_towel)
        q_above_drop = solve_joints(robot, drop_xyz + up + [0, 0, HOVER], R_down, q_above_towel)
        q_at_drop = solve_joints(robot, drop_xyz + up, R_down, q_above_drop)
 
        print(f"ReBot: picking up {name}")
        animate_joints(robot, env, q_above_towel)
        robot.open_gripper(); env.step(DT)
        animate_joints(robot, env, q_at_towel)
        robot.close_gripper(); env.step(DT)
        hold = tool_pose(robot).inv() * SE3(towel.T)       # towel now held
 
        print(f"ReBot: carrying {name}")
        animate_joints(robot, env, q_above_towel, towel, hold)
        animate_joints(robot, env, q_above_drop, towel, hold)
        animate_joints(robot, env, q_at_drop, towel, hold)
 
        towel.T = SE3(*drop_xyz).A                          # set it down exactly there
        robot.open_gripper(); env.step(DT)
        print(f"ReBot: {name} put down at {np.round(drop_xyz, 3).tolist()}")
        animate_joints(robot, env, q_above_drop)
        q_guess = q_above_drop
 
    robot.close_gripper(); env.step(DT)
    animate_joints(robot, env, robot.q_home)
    print("ReBot: both paper towels moved")
 
 
# ---------------- run on its own: just ReBot ----------------
if __name__ == "__main__":
    import sys
    from pathlib import Path
    import swift
 
    here = Path(__file__).resolve().parent
    sys.path.append(str(here))
    sys.path.append(str(here.parent))
    from EnviroParts_S.Full_sim import create_environment
    from Rebot_class import ReBot
 
    env = swift.Swift()
    env.launch(realtime=True)
    scene = create_environment(env)
 
    rebot = ReBot(env)
    rebot.base = SE3(12.5, 3.6, 0.66) * SE3.Rz(np.pi / 2)
    rebot.update_meshes()
    env.step(DT)
 
    run_rebot_towels(env, rebot, scene)
    env.hold()
 
