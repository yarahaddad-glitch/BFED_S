"""
E_Stop.py

E = STOP: every robot freezes where it is (on its next frame).
H = HOME: (after E) every robot glides to its home pose. Anything a robot is
    holding (a tray, food, a towel) stays in its gripper and goes home with it.
    Nothing else is moved or put back.
R = RESUME: (after E, with or without H) the task carries on exactly where it was.
    If you pressed H, the robots (and what they hold) glide back to where they
    stopped first.

GUI: call stop("message"), go_home() and resume() from the buttons.
They are thread-safe: they only raise flags; the simulation thread does the moving.

Home pose = the pose each robot had at start-up, unless set with set_home(...).

Nothing needs to be added to the individual robot files. In the main script call:
    ESTOP.install(env)     once, after the scene and movements are built
    ESTOP.begin_task()     at the start of each meal, ESTOP.end_task() when it ends

This file must not import anything from the project (every robot file imports it).
"""

import gc
import inspect
import re
import time
from collections import deque
import threading
import numpy as np
from spatialmath import SE3

try:
    from scipy.spatial.transform import Rotation, Slerp
except Exception:  # scipy missing: rotations just snap
    Rotation = Slerp = None

active = False  # True from stop() until the task has resumed

CARRY_DIST = 0.6        # an item can only be held by a robot whose tool is within this distance (m)
HOLD_FRAMES = 6         # an item must stay locked to a tool for this many moving frames to count as held
HOLD_TRANS_TOL = 0.02   # ...within this much wobble (m)
HOLD_ROT_TOL = 0.08     # ...and this much rotation wobble
HOLD_MIN_MOVE = 0.05    # ...during which BOTH the tool and the item travelled at least this far (m)
RELEASE_TOL = 0.04      # a held item that drifts this far from where it was gripped has been put down (m)
HOME_CARRY_YAW_DEG = {"MaryamBot": 90.0}   # on H, turn what this robot holds about Z (flip sign if wrong way)
HOME_CARRY_PULL = 0.6   # on H, what a robot holds is drawn this far (0-1) in over its gripper and kept level
# Robots listed here only FREEZE on E; H / R do not move them. Their meshes are drawn by their own code, so
# moving q from outside breaks their joints. Remove a name once ESTOP.set_refresh(robot, redraw_function) is given.
HOME_SKIP = {"ReBot"}
# Robots in a stay-put group (see group(..., stay=True)): H only folds the ARM to its straight pose
# (default: the pose it started in); the turtlebot never moves. Set a joint list here to override.
STRAIGHT_Q = {}   # e.g. {"HannahBot": [0, -1.57, 0, -1.57, 0, 0]}


def base_name(name):
    return re.sub(r"_\d+$", "", name)


def table_get(table, name, default=None):
    return table.get(name, table.get(base_name(name), default))


class EmergencyStop(RuntimeError):
    """Kept for old code; the E-stop now pauses and resumes instead of raising."""


def looks_like_robot(obj):
    try:
        return (
            not isinstance(obj, type)
            and hasattr(obj, "q")
            and hasattr(obj, "base")
            and (hasattr(obj, "fkine") or hasattr(obj, "update_meshes"))
        )
    except Exception:
        return False


def looks_like_item(obj):
    """A mesh/shape in the scene (tray, food, towel...): has a 4x4 pose in .T"""
    try:
        return (not isinstance(obj, type)
                and not looks_like_robot(obj)
                and np.asarray(obj.T, dtype=float).shape == (4, 4))
    except Exception:
        return False


def _as_pose(T):
    """Return T as a 4x4 array, or None (handles SE3 lists, stacked arrays, bad shapes)."""
    try:
        if isinstance(T, (list, tuple)):
            T = T[0]
        A = np.asarray(getattr(T, "A", T), dtype=float)
        if A.ndim == 3:
            A = A[0]
        return A if A.shape == (4, 4) else None
    except Exception:
        return None


def tool_pose(robot):
    """Gripper / end-effector pose in the world as a 4x4 array, or None if it can't be worked out."""
    try:
        links = robot.get_link_transforms(robot.q)
        T = robot.base * robot.get_gripper_transforms(links, robot.gripper_opening)[0]
        A = _as_pose(T)
        if A is not None:
            return A
    except Exception:
        pass
    try:
        return _as_pose(robot.fkine(robot.q))
    except Exception:
        return None


def lerp_base(b0, b1, s):
    """Base pose between b0 and b1: straight-line position, smooth rotation."""
    t = b0.t + s * (b1.t - b0.t)
    R = np.asarray(b1.R, dtype=float)
    if Slerp is not None:
        try:
            R = Slerp([0.0, 1.0], Rotation.from_matrix([np.asarray(b0.R, float), R]))(s).as_matrix()
        except Exception:
            pass
    return SE3.Rt(R, t)


def frames_of(robot):
    """World poses (4x4) that something can ride on: the base, then each link frame (if the robot can give them)."""
    out = []
    try:
        out.append(np.asarray(robot.base.A, dtype=float))
    except Exception:
        return out
    try:
        for T in robot.fkine_all(robot.q):
            A = _as_pose(T)
            if A is not None:
                out.append(A)
        return out
    except Exception:
        pass
    try:
        A = _as_pose(robot.fkine(robot.q))
        if A is not None:
            out.append(A)
    except Exception:
        pass
    return out


class EStop:
    def __init__(self):
        self.stop_event = threading.Event()     # E
        self.home_event = threading.Event()     # H
        self.resume_event = threading.Event()   # R
        self.home_positions = {}    # name -> (robot, home q, home base or None)
        self.env = None
        self._original_step = None
        self._busy = False          # True while a stop is being handled
        self._handler_thread = None
        self._holding = False       # True once env.hold() is running (end of program)
        self._known = set()
        self._pending = []          # robots just added to the env, snapshotted on the next step
        self._items = []            # every shape added to the env (trays, food, ...)
        self._item_ids = set()
        self._excluded = set()      # scene objects that are never moved by H / R
        self._recording = False     # True between begin_task() and end_task()
        self._rec_items = []        # scene objects watched for being held by a robot
        self._carry = {}            # id(item) -> (item, robot, robot name, item pose relative to the tool)
        self._last_T = {}
        self._last_rel = {}
        self._hist = {}
        self._chassis_robots = []   # robots whose (non-robot) chassis meshes must travel with them
        self._refresh_fns = {}
        self._groups = []           # (arm, root, extra parts, stay) that must move as ONE rigid machine
        self._stay_arms = set()     # ids of arms that only fold up on H (their turtlebot stays put)
        self._fixed_ids = set()     # ids of robots that do not move at all on H / R (turtlebot, plates, ...)
        self._link_items = {}       # id(item) -> (item, arm, link index, pose relative to that link): arm parts learned live
        self._lp_hist = {}
        self._lp_last = {}
        self._part_mounts = []      # (part, parent robot, link index, part base relative to that link), in order
        self.state = "running"      # running / stopped / going home / home / resuming (for the GUI)
        self._mounts = {}           # id(child robot) -> (parent robot, child base relative to parent)
        self._handlers = []         # optional register(home, resume) pairs
        self._stop_callback = None  # GUI hook: called with the stop message

        self._patch_swift()         # hook the simulation as soon as this file is imported

    # ---------------- hooking the simulation ----------------
    def _patch_swift(self):
        try:
            import swift
        except Exception:
            return

        estop = self
        original_step = swift.Swift.step
        original_add = swift.Swift.add
        original_hold = getattr(swift.Swift, "hold", None)
        self._original_step = original_step

        def step(sim, *args, **kwargs):
            estop._snapshot_pending()
            estop._guard()              # blocks here while stopped
            try:
                estop._track()      # notice which items a robot is holding
            except Exception:
                pass
            return original_step(sim, *args, **kwargs)

        def add(sim, ob, *args, **kwargs):
            result = original_add(sim, ob, *args, **kwargs)
            if looks_like_robot(ob):
                estop._pending.append(ob)   # snapshot its start pose on the next step
            elif looks_like_item(ob) and id(ob) not in estop._item_ids:
                estop._item_ids.add(id(ob))
                estop._items.append(ob)
            return result

        swift.Swift.step = step
        swift.Swift.add = add

        if original_hold is not None:
            def hold(sim, *args, **kwargs):
                estop._holding = True
                return original_hold(sim, *args, **kwargs)
            swift.Swift.hold = hold

    def install(self, env):
        """Call once in main, after the scene and robots have been created."""
        self.env = env
        self.scan_robots()
        print(f"[ESTOP] watching {len(self.home_positions)} robots, "
              f"{len(self._items)} scene objects", flush=True)
        self.list_robots()

    # ---------------- registering robots ----------------
    def register_robot(self, name, robot):
        """Save a robot's start pose as its home. Safe to call twice for one robot."""
        if robot is None:
            return
        if id(robot) in self._known:                     # already known: just give it the better name
            for old_name, value in list(self.home_positions.items()):
                if value[0] is robot and old_name != name and not name.startswith("Robot_"):
                    if name not in self.home_positions:
                        self.home_positions[name] = self.home_positions.pop(old_name)
            return

        try:
            home_q = np.asarray(robot.q, dtype=float).copy()
        except Exception:
            return  # not a real robot (or q unreadable): ignore it

        try:
            base = SE3(np.asarray(robot.base.A, dtype=float).copy())
        except Exception:
            base = None  # invalid base: only the joints are moved for this one

        self._known.add(id(robot))
        self.home_positions[name] = (robot, home_q, base)

    def set_home(self, robot, q=None, base=None):
        """Pre-determine a robot's home pose (instead of its start-up pose)."""
        self.register_robot(f"{type(robot).__name__}_{len(self.home_positions)}", robot)
        for name, (known, home_q, home_base) in list(self.home_positions.items()):
            if known is robot:
                new_q = np.asarray(q, dtype=float).copy() if q is not None else home_q
                new_base = SE3(np.asarray(base.A, dtype=float).copy()) if base is not None else home_base
                self.home_positions[name] = (robot, new_q, new_base)
                return
        print("[ESTOP] set_home: robot could not be registered", flush=True)

    def scan_robots(self):
        """Find every robot that exists right now and save its current pose as home."""
        for obj in gc.get_objects():
            if looks_like_robot(obj):
                try:
                    self.register_robot(f"{type(obj).__name__}_{len(self.home_positions)}", obj)
                except Exception as exc:  # never let one odd object break start-up
                    print(f"[ESTOP] skipped {type(obj).__name__}: {exc}", flush=True)

    def list_robots(self):
        """Print every robot being watched (use this to check all of them were found)."""
        self._find_mounts()
        for name, (robot, home_q, home_base) in self.home_positions.items():
            mounted = " (mounted on another robot)" if id(robot) in self._mounts else ""
            print(f"[ESTOP]   {name}: {len(home_q)} joints, "
                  f"{'base ok' if home_base is not None else 'NO BASE'}{mounted}", flush=True)

    def mount(self, child, parent):
        """Tell the E-stop that child is bolted to parent (it then always moves with it)."""
        self.register_robot(f"{type(child).__name__}_{len(self.home_positions)}", child)
        self.register_robot(f"{type(parent).__name__}_{len(self.home_positions)}", parent)
        try:
            rel = SE3(np.asarray(parent.base.A, float)).inv() * SE3(np.asarray(child.base.A, float))
            self._mounts[id(child)] = (parent, rel)
        except Exception as exc:
            print(f"[ESTOP] mount failed: {exc}", flush=True)

    def _find_mounts(self):
        """A robot sitting directly above another one (same x,y, up to 0.3 m higher) is mounted on it."""
        robots = [(robot, base) for robot, _, base in self.home_positions.values() if base is not None]
        for child, cb in robots:
            if id(child) in self._mounts:
                continue
            for parent, pb in robots:
                if parent is child:
                    continue
                dx, dy, dz = cb.t - pb.t
                if abs(dx) < 0.02 and abs(dy) < 0.02 and 0.0 < dz < 0.3:
                    self.mount(child, parent)
                    break

    def _snapshot_pending(self):
        while self._pending:
            robot = self._pending.pop()
            try:
                self.register_robot(f"{type(robot).__name__}_{len(self.home_positions)}", robot)
            except Exception as exc:
                print(f"[ESTOP] skipped {type(robot).__name__}: {exc}", flush=True)

    def register(self, home, resume=None):
        """Optional custom hooks: home() runs on H (its result is kept), resume(result) runs on R."""
        self._handlers.append((home, resume))

    def set_stop_callback(self, callback):
        """GUI hook: callback(message) is called whenever a stop is requested (the stop still happens)."""
        self._stop_callback = callback

    def group(self, arm, root, extras=(), stay=False):
        """arm rides on root (its turtlebot); extras are other parts of the same machine (gripper robots, plates).
        On H / R the whole machine moves as one rigid thing, so the arm can never come away from its turtlebot."""
        for r, n in ((arm, "Arm"), (root, "Root")):
            self.register_robot(f"{n}_{len(self.home_positions)}", r)
        self._groups.append((arm, root, [e for e in extras if e is not arm and e is not root], stay))

    def group_near(self, arm, root, exclude=(), radius=0.8, stay=False):
        """Like group(), picking as extras every other registered robot within radius (m) of the arm's base."""
        try:
            centre = np.asarray(arm.base.A, dtype=float)[:3, 3]
        except Exception:
            centre = None
        extras = []
        skip = {id(x) for x in exclude}
        for robot, _, _ in self.home_positions.values():
            if robot is arm or robot is root or id(robot) in skip or centre is None:
                continue
            try:
                p = np.asarray(robot.base.A, dtype=float)[:3, 3]
            except Exception:
                continue
            if np.linalg.norm(p[:2] - centre[:2]) < radius:
                extras.append(robot)
        self.group(arm, root, extras, stay)
        print(f"[ESTOP] group: {type(arm).__name__} on {type(root).__name__} + {len(extras)} extra part(s)", flush=True)

    def rehome(self, *robots):
        """Make each robot's CURRENT pose its home (use when a robot is only placed once its job starts)."""
        for robot in robots:
            for name, (known, _, _) in list(self.home_positions.items()):
                if known is robot:
                    try:
                        q = np.asarray(robot.q, dtype=float).copy()
                    except Exception:
                        continue
                    try:
                        base = SE3(np.asarray(robot.base.A, dtype=float).copy())
                    except Exception:
                        base = None
                    self.home_positions[name] = (robot, q, base)

    def flag_chassis(self, robot):
        """This robot rides on a chassis that is NOT a robot object (a plain mesh): take it along on H / R."""
        self._chassis_robots.append(robot)

    def _chassis_items(self):
        """(item, robot, pose relative to the robot's base) for meshes sitting right under a flagged robot."""
        out = []
        for robot in self._chassis_robots:
            try:
                Tb = np.asarray(robot.base.A, dtype=float)
            except Exception:
                continue
            for item in self._items:
                if id(item) in self._excluded:
                    continue
                try:
                    T = np.array(item.T, dtype=float)
                except Exception:
                    continue
                d = T[:3, 3] - Tb[:3, 3]
                if abs(d[0]) < 0.3 and abs(d[1]) < 0.3 and -0.5 < d[2] < 0.1:
                    out.append((item, robot, np.linalg.inv(Tb) @ T))
        return out

    def set_refresh(self, robot, fn):
        """Give a robot's own 'redraw my links/gripper' function; it is called every frame while H / R move it."""
        self._refresh_fns[id(robot)] = fn

    def _refresh_meshes(self, robot):
        """Only ever calls update_meshes() (if the robot has it) or a function given to set_refresh()."""
        fn = self._refresh_fns.get(id(robot))
        if fn is None and hasattr(robot, "update_meshes"):
            fn = robot.update_meshes
        if fn is not None:
            try:
                fn()
            except Exception:
                pass

    def describe_robots(self):
        """Print each robot's own (non-toolbox) methods: send this to whoever wires up set_refresh."""
        for name, (robot, _, _) in self.home_positions.items():
            own = []
            for klass in type(robot).__mro__:
                if "roboticstoolbox" in getattr(klass, "__module__", "") or klass is object:
                    continue
                own += [n for n in vars(klass) if not n.startswith("__")]
            print(f"[ESTOP] {name} ({type(robot).__name__}): {list(dict.fromkeys(own))}", flush=True)

    # ---------------- tasks (so carried items can go home and come back) ----------------
    def exclude(self, items):
        """Scene objects that H / R must never move (e.g. walking prisoners)."""
        for item in items:
            self._excluded.add(id(item))

    def begin_task(self):
        """Start watching which scene objects each robot is holding (so H can take them home with it)."""
        self._rec_items = [item for item in self._items if id(item) not in self._excluded]
        self._carry = {}
        self._last_T = {}
        self._last_rel = {}
        self._hist = {}
        self._recording = True

    def end_task(self):
        self._recording = False
        self._carry = {}
        self._hist = {}
        self._last_T = {}
        self._last_rel = {}

    def _learn_link_parts(self):
        """Every frame, for each group arm: find scene objects that ride rigidly on one of its links (a gripper
        piece, a wrist, a tray it holds). They are then moved with that link when the arm folds up."""
        for arm, _, _, _ in self._groups:
            frames = frames_of(arm)
            if len(frames) < 2:
                continue
            pos = np.array([F[:3, 3] for F in frames])
            for item in self._items:
                key = id(item)
                if id(item) in self._excluded:
                    continue
                try:
                    T = np.array(item.T, dtype=float)
                except Exception:
                    continue
                last = self._lp_last.get(key)
                self._lp_last[key] = T

                known = self._link_items.get(key)
                if known is not None:                      # still locked to its link?
                    _, a, idx, rel = known
                    try:
                        cur = np.linalg.inv(frames_of(a)[idx]) @ T
                        if (np.abs(cur[:3, 3] - rel[:3, 3]).max() > RELEASE_TOL
                                or np.abs(cur[:3, :3] - rel[:3, :3]).max() > 2 * HOLD_ROT_TOL):
                            del self._link_items[key]
                    except Exception:
                        self._link_items.pop(key, None)
                    continue

                hist = self._lp_hist.setdefault(key, [])
                d = np.linalg.norm(pos - T[:3, 3], axis=1)
                idx = int(d.argmin())
                if d[idx] > 0.35 or last is None:
                    hist.clear()
                    continue
                moved = np.abs(T - last).max() >= 1e-9
                if not moved:
                    if hist and np.linalg.norm(frames[idx][:3, 3] - hist[-1][3]) > 0.01:
                        hist.clear()
                    continue
                rel = np.linalg.inv(frames[idx]) @ T
                if hist and (hist[-1][0] != idx
                             or np.abs(hist[0][1][:3, 3] - rel[:3, 3]).max() > HOLD_TRANS_TOL
                             or np.abs(hist[0][1][:3, :3] - rel[:3, :3]).max() > HOLD_ROT_TOL):
                    hist.clear()
                hist.append((idx, rel, T[:3, 3].copy(), frames[idx][:3, 3].copy()))
                if len(hist) > 600:
                    del hist[:100]
                if (len(hist) >= HOLD_FRAMES
                        and np.linalg.norm(hist[-1][2] - hist[0][2]) > 0.03
                        and np.linalg.norm(hist[-1][3] - hist[0][3]) > 0.03):
                    self._link_items[key] = (item, arm, idx, rel)

    def _track(self):
        """Every frame: an item that follows a robot's tool (for a few frames) is 'held' by it.
        It stays held, even if both stand still, until it drifts away from the tool."""
        if self._groups and not self._busy:
            try:
                self._learn_link_parts()
            except Exception:
                pass
        if not self._recording or self._busy:
            return

        tools = {}
        for name, (robot, _, _) in self.home_positions.items():
            Tt = tool_pose(robot)
            if Tt is not None:
                tools[id(robot)] = (name, robot, Tt)

        # 1. forget items that have been let go
        for key in list(self._carry):
            item, robot, name, rel = self._carry[key]
            entry = tools.get(id(robot))
            if entry is None:
                continue
            try:
                cur = np.linalg.inv(entry[2]) @ np.array(item.T, dtype=float)
                if (np.abs(cur[:3, 3] - rel[:3, 3]).max() > RELEASE_TOL
                        or np.abs(cur[:3, :3] - rel[:3, :3]).max() > 2 * HOLD_ROT_TOL):
                    del self._carry[key]        # it no longer follows the gripper: it was put down
            except Exception:
                self._carry.pop(key, None)

        # 2. spot items that follow a tool: locked to it (same pose relative to the tool) for a stretch
        #    during which BOTH moved a real distance. Slow carrying is fine; sitting on a belt is not.
        for item in self._rec_items:
            key = id(item)
            if key in self._carry:
                continue
            try:
                T = np.array(item.T, dtype=float)
            except Exception:
                continue
            hist = self._hist.setdefault(key, [])
            last = self._last_T.get(key)
            self._last_T[key] = T
            if last is None:
                continue
            moved = np.abs(T - last).max() >= 1e-9

            best = None
            for name, robot, Tt in tools.values():
                d = np.linalg.norm(Tt[:3, 3] - T[:3, 3])
                if d < CARRY_DIST and (best is None or d < best[0]):
                    best = (d, name, robot, Tt)
            if best is None:
                hist.clear()
                continue
            _, name, robot, Tt = best

            if not moved:
                # the item is still: if the tool has moved off since the streak began, it was not held
                if hist and np.linalg.norm(Tt[:3, 3] - hist[-1][3]) > 0.01:
                    hist.clear()
                continue

            rel = np.linalg.inv(Tt) @ T
            if hist and (hist[-1][0] is not robot
                         or np.abs(hist[0][1][:3, 3] - rel[:3, 3]).max() > HOLD_TRANS_TOL
                         or np.abs(hist[0][1][:3, :3] - rel[:3, :3]).max() > HOLD_ROT_TOL):
                hist.clear()                      # lost the lock: start counting again from here
            hist.append((robot, rel, T[:3, 3].copy(), Tt[:3, 3].copy()))
            if len(hist) > 600:
                del hist[:100]
            if len(hist) >= HOLD_FRAMES:
                item_move = np.linalg.norm(hist[-1][2] - hist[0][2])
                tool_move = np.linalg.norm(hist[-1][3] - hist[0][3])
                if item_move > HOLD_MIN_MOVE and tool_move > HOLD_MIN_MOVE:
                    self._carry[key] = (item, robot, name, rel)

    # ---------------- the three commands (thread-safe) ----------------
    def stop(self, message="Emergency stop"):
        """E: raise the flag. The simulation thread freezes every robot on its next frame."""
        global active
        if self.stop_event.is_set():
            return
        active = True
        self.state = "stopped"
        self.home_event.clear()
        self.resume_event.clear()
        self.stop_event.set()
        print("\n" + "=" * 44)
        print(f"!!! {message}")
        print("!!! EMERGENCY STOP - ALL ROBOTS FROZEN")
        print("!!! H = send robots home    R = resume the task")
        print("=" * 44, flush=True)
        if self._stop_callback is not None:
            try:
                self._stop_callback(message)
            except Exception as exc:
                print(f"[ESTOP] GUI callback failed: {exc}", flush=True)

    def go_home(self):
        """H: (only while stopped) glide every robot to its home pose."""
        if not self.stop_event.is_set():
            print("Press E first.", flush=True)
            return
        self.home_event.set()
        print("HOME REQUESTED", flush=True)

    def resume(self):
        """R: (only while stopped) carry on with the task where it was."""
        if not self.stop_event.is_set():
            print("Nothing to resume - the E-stop is not active.", flush=True)
            return
        self.resume_event.set()
        print("RESUME REQUESTED", flush=True)

    # ---------------- what happens on the simulation thread ----------------
    def _guard(self):
        """Called on every frame. While stopped, this thread waits here (the task is paused)."""
        if self._holding:
            return

        me = threading.current_thread()
        if self._busy:
            if me is not self._handler_thread:      # someone else is handling the stop: wait for them
                while self._busy:
                    time.sleep(0.01)
            return

        if not self.stop_event.is_set():
            return

        self._busy = True
        self._handler_thread = me
        try:
            self._pause()
        finally:
            self._handler_thread = None
            self._busy = False

    def check(self):
        """Optional: robot loops may still call this; the simulation hook makes it unnecessary."""
        if self._original_step is not None:
            self._guard()

    def _frame(self, dt=0.05):
        """Draw one frame directly (bypasses any lock-step wrapper so nothing else moves)."""
        if self._original_step is not None and self.env is not None:
            self._original_step(self.env, dt)
        elif self.env is not None:
            self.env.step(dt)
        else:
            time.sleep(dt)

    def _pause(self):
        global active
        self._find_mounts()
        for key, (parent, _) in list(self._mounts.items()):      # mounted robots are rigid right now
            for child, _, _ in self.home_positions.values():
                if id(child) == key:
                    try:
                        rel = SE3(np.asarray(parent.base.A, float)).inv() * SE3(np.asarray(child.base.A, float))
                        self._mounts[key] = (parent, rel)
                    except Exception:
                        pass
        self._part_mounts = []
        self._stay_arms, self._fixed_ids = set(), set()
        for arm, root, extras, stay in self._groups:
            try:
                Tr = np.asarray(root.base.A, dtype=float)
                if id(arm) not in self._mounts:
                    self._mounts[id(arm)] = (root, SE3(np.linalg.inv(Tr) @ np.asarray(arm.base.A, dtype=float)))
                pool = [(root, 0, Tr)] + [(arm, i, F) for i, F in enumerate(frames_of(arm))]
                order = sorted(extras, key=lambda p: np.linalg.norm(np.asarray(p.base.A, dtype=float)[:3, 3] - Tr[:3, 3]))
                for part in order:
                    Bp = np.asarray(part.base.A, dtype=float)
                    parent, idx, F = min(pool, key=lambda e: np.linalg.norm(e[2][:3, 3] - Bp[:3, 3]))
                    rel = SE3(np.linalg.inv(F) @ Bp)
                    if parent is root:
                        self._mounts[id(part)] = (root, rel)                 # rides on the chassis
                    else:
                        self._part_mounts.append((part, parent, idx, rel))   # rides on a link of the arm / another part
                    pool += [(part, i, G) for i, G in enumerate(frames_of(part))]
                if stay:
                    self._stay_arms.add(id(arm))
                    self._fixed_ids.add(id(root))
                    self._fixed_ids.update(id(p) for p in extras)
                print(f"[ESTOP] {type(arm).__name__} machine: {len(extras)} extra part(s) locked on "
                      f"({len(self._part_mounts)} follow a link)", flush=True)
            except Exception as exc:
                print(f"[ESTOP] group setup failed: {exc}", flush=True)
        chassis = self._chassis_items()
        self.state = "stopped"

        saved = []
        for name, (robot, home_q, home_base) in self.home_positions.items():
            if table_get({n: True for n in HOME_SKIP}, name, False) and id(robot) not in self._refresh_fns:
                continue                                  # frozen only: H / R do not move it
            if id(robot) in self._fixed_ids:
                continue                                  # stays exactly where the E-stop caught it
            if id(robot) in self._stay_arms:
                home_base = None                          # only the arm folds; its turtlebot does not move
                straight = table_get(STRAIGHT_Q, name)
                if straight is not None:
                    home_q = np.asarray(straight, dtype=float).copy()
            try:
                base_now = SE3(np.asarray(robot.base.A, dtype=float).copy()) if home_base is not None else None
            except Exception:
                base_now = None
            saved.append((robot, np.asarray(robot.q, dtype=float).copy(), base_now, home_q, home_base))

        # parts that ride on an arm's links (found while it was moving) fold up with that arm
        link_items = []
        for key, (item, arm, idx, rel) in list(self._link_items.items()):
            if id(arm) not in self._stay_arms and not any(g[0] is arm for g in self._groups):
                continue
            try:
                cur = np.linalg.inv(frames_of(arm)[idx]) @ np.array(item.T, dtype=float)
                if np.abs(cur[:3, 3] - rel[:3, 3]).max() <= RELEASE_TOL:
                    link_items.append((item, arm, idx, cur))
            except Exception:
                pass
        linked_keys = {id(i[0]) for i in link_items}
        self._active_link_items = link_items
        if link_items:
            print(f"[ESTOP] {len(link_items)} part(s)/item(s) ride on the arm and fold up with it", flush=True)

        # items a robot is holding right now: they go wherever that robot goes
        carried = []
        for item, robot, name, rel in self._carry.values():
            if id(item) in linked_keys:
                continue                                   # moves with the arm's link instead
            try:
                Tt0 = tool_pose(robot)
                if Tt0 is None:
                    continue
                if np.linalg.norm(np.array(item.T, dtype=float)[:3, 3] - Tt0[:3, 3]) > CARRY_DIST:
                    continue                              # nowhere near the gripper: it was put down
                if np.abs(np.linalg.inv(Tt0) @ np.array(item.T, dtype=float) - rel).max() > RELEASE_TOL:
                    continue                              # no longer locked to the gripper
                if table_get({n: True for n in HOME_SKIP}, name, False) and id(robot) not in self._refresh_fns:
                    continue                              # that robot is not going home, so nothing goes with it
                carried.append((item, robot, rel.copy(), np.array(item.T, dtype=float), Tt0))
                print(f"[ESTOP] {name} is holding an item - it will go home with it", flush=True)
            except Exception:
                pass
        if not carried:
            print("[ESTOP] no robot is holding anything right now", flush=True)

        going_home = [(r, q, home_q, b, home_b) for r, q, b, home_q, home_b in saved]
        coming_back = [(r, home_q, q, home_b, b) for r, q, b, home_q, home_b in saved]

        at_home = False
        legacy_state = []

        while not self.resume_event.is_set():
            if self.home_event.is_set():
                self.home_event.clear()
                if at_home:
                    print("Already at home - press R to resume.", flush=True)
                    continue
                print("Sending robots home...", flush=True)
                self.state = "going home"
                self._glide(going_home, carried, chassis, to_home=True)
                legacy_state = [home() for home, _ in self._handlers]
                at_home = True
                self.state = "home"
                print("Robots are home - press R to resume the task.", flush=True)
                continue
            self._frame(0.05)               # keep the window alive; everything stays frozen

        self.state = "resuming"
        if at_home:
            print("Returning robots to where they stopped...", flush=True)
            self._glide(coming_back, carried, chassis, to_home=False)
            for item, _, _, pose, _ in carried:    # land exactly where they were
                try:
                    item.T = pose.copy()
                except Exception:
                    pass
            for (_, resume), state in zip(self._handlers, legacy_state):
                if resume is not None:
                    resume(state)

        self.resume_event.clear()
        self.home_event.clear()
        self.stop_event.clear()
        active = False
        self.state = "running"
        print("RESUMED - the task carries on.", flush=True)

    def _glide(self, robot_moves, carried, chassis=(), to_home=True, steps=60, dt=0.03):
        """Move robots (robot, q0, q1, base0, base1) smoothly, with held items (item, robot, rel, pose) following."""
        groups = {}
        for _, robot, _, pose0, Tt0 in carried:
            groups.setdefault(id(robot), []).append(pose0[:3, 3] - Tt0[:3, 3])
        yaws = {}
        for name, (robot, _, _) in self.home_positions.items():
            yaws[id(robot)] = table_get(HOME_CARRY_YAW_DEG, name, 0.0)
        pulls = {}
        for rid, offsets in groups.items():
            c = np.mean(offsets, axis=0)
            pulls[rid] = np.array([c[0], c[1], 0.0])

        for i in range(1, steps + 1):
            u = i / steps
            s = 3 * u ** 2 - 2 * u ** 3  # smooth start and stop

            bases = {}
            for robot, q0, q1, base0, base1 in robot_moves:
                robot.q = q0 + s * (q1 - q0)
                if base0 is not None and base1 is not None:
                    bases[id(robot)] = lerp_base(base0, base1, s)

            # bases: mounted robots follow their parent exactly, so nothing comes apart
            for robot, *_ in robot_moves:
                base = bases.get(id(robot))
                if base is None:
                    continue
                mount = self._mounts.get(id(robot))
                if mount is not None:
                    if id(mount[0]) not in bases:
                        continue                  # its parent isn't moving, so neither is it: never come apart
                    base = bases[id(mount[0])] * mount[1]
                robot.base = base

            for part, parent, idx, rel in self._part_mounts:   # parts riding on a link follow that link
                try:
                    frames = frames_of(parent)
                    part.base = SE3(frames[idx]) * rel
                except Exception:
                    pass

            for item, arm, idx, rel in getattr(self, "_active_link_items", []):
                try:
                    item.T = frames_of(arm)[idx] @ rel
                except Exception:
                    pass

            for robot, *_ in robot_moves:           # refresh every robot's meshes each frame
                self._refresh_meshes(robot)

            h = s if to_home else 1.0 - s            # 0 = where it stopped, 1 = at home
            for item, robot, rel, pose0, Tt0 in carried:
                # held items stay LEVEL, are turned about Z (so a tray doesn't swing into a wall) and are
                # drawn in over the gripper as the arm folds up; everything one robot holds moves together,
                # so food stays on its tray
                Tt = tool_pose(robot)
                if Tt is None:
                    continue
                ang = np.radians(yaws.get(id(robot), 0.0)) * h
                c, sn = np.cos(ang), np.sin(ang)
                Rz = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
                offset = (pose0[:3, 3] - Tt0[:3, 3]) - HOME_CARRY_PULL * h * pulls[id(robot)]
                T = pose0.copy()
                T[:3, :3] = Rz @ pose0[:3, :3]
                T[:3, 3] = Tt[:3, 3] + Rz @ offset
                item.T = T

            for item, robot, rel in chassis:         # meshes the robot stands on travel with its base
                try:
                    item.T = np.asarray(robot.base.A, dtype=float) @ rel
                except Exception:
                    pass

            self._frame(dt)

    # ---------------- terminal keys (until the GUI exists) ----------------
    def start_listener(self):
        """Listen for terminal commands in another thread."""

        def listen():
            while True:
                try:
                    command = input().strip().upper()
                except (EOFError, OSError):
                    break

                if command == "E":
                    self.stop("Emergency stop (terminal)")
                elif command == "H":
                    self.go_home()
                elif command == "R":
                    self.resume()

        threading.Thread(target=listen, daemon=True).start()

        print("E-STOP READY")
        print("E = STOP ALL ROBOTS")
        print("H = (after E) SEND ROBOTS HOME")
        print("R = (after E) RESUME THE TASK")


ESTOP = EStop()


# ---------------- module-level API (for the GUI and other files) ----------------
def stop(message="Emergency stop"):
    ESTOP.stop(message)


def go_home():
    ESTOP.go_home()


def resume():
    ESTOP.resume()


def set_stop_callback(callback):
    ESTOP.set_stop_callback(callback)


def register(home, resume=None):
    ESTOP.register(home, resume)


def set_home(robot, q=None, base=None):
    ESTOP.set_home(robot, q, base)


def is_active():
    return ESTOP.stop_event.is_set()


def status():
    """For the GUI: 'running', 'stopped', 'going home', 'home' or 'resuming'."""
    return ESTOP.state