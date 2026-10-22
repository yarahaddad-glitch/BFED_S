import sys
import numpy as np
import threading
import time
from pathlib import Path
 
ASSIGNMENT_ROOT = Path(__file__).resolve().parents[2]
for folder in (ASSIGNMENT_ROOT, ASSIGNMENT_ROOT / "Collective_Folder"):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))
 
# Meal system (Maryam / Yara / ReBot)
from Maryam_Work.maryam_meal_dispatch import ColourScanner, build_default_dispatcher
from Maryam_Work.maryam_movements import build_scene
from Collective_Folder.integrated_meal import IntegratedMeal, yara
 
 
# Hannah
from Hannah_Work.wall_collision import WallCollision
from Hannah_Work.Feed_Iso import feed_iso
from Collective_Folder.workcell_runtime import WorkcellRuntime, WorkcellClosed
from Collective_Folder.safety_tests import SafetyDemonstration
from Collective_Folder.maryam_collision_demo import MaryamCollisionDemo
 
# Hannah's safety features
from Collective_Folder.Prison_break import PrisonBreak
from Light_curtain import SafetyLightCurtain
import inspect
import queue
 
DT = 0.05
 
# Switches - turn things off to find what stops the kitchen
RUN_FEED_ISO = True
RUN_PRISONERS = True
USE_WALLS = True
USE_LIGHT_CURTAIN = True
USE_PRISON_BREAK = True
IDLE_GAP = 0.03         # pause between background steps while the camera is open
JOB_TIMEOUT = 0.5      # longest the kitchen waits for feed_iso / prisoners each step
 
 
# ============================================================
# LOCK-STEP: background jobs take one step each time the scene steps
# ============================================================
class LockStep:
    def __init__(self, env, runtime=None):
        self.env = env
        self.real_step = env.step
        self.jobs = {}                    # thread -> job info
        self.turn = threading.RLock()     # only one thing touches the scene at a time
        self.kitchen_idle = False
        self.runtime = runtime
        env.step = self.step
 
    def add_job(self, name, fn):
        job = {"name": name, "go": threading.Event(), "back": threading.Event(), "done": False}
 
        def run():
            job["go"].wait(); job["go"].clear()       # wait for our first turn
            try:
                fn()
                print(f"[{name}] finished", flush=True)
            except Exception as exc:
                print(f"[{name}] stopped: {exc!r}", flush=True)
                if self.runtime:
                    self.runtime.request_stop(f"{name}: {exc}")
            job["done"] = True
            job["back"].set()                         # hand the turn back for good
 
        t = threading.Thread(target=run, daemon=True)
        self.jobs[t] = job
        t.start()
 
    def step(self, dt=DT, *args, **kwargs):
        job = self.jobs.get(threading.current_thread())
        if job is not None:
            # A background job finished one step: hand the turn back, wait for the next
            job["back"].set()
            job["go"].wait(); job["go"].clear()
            return None
        with self.turn:
            if self.runtime:
                self.runtime.checkpoint()
            result = self.real_step(dt, *args, **kwargs)    # draw the frame once
            for job in list(self.jobs.values()):
                if not job["done"]:
                    job["go"].set()                         # let this job do ONE step
                    if job["back"].wait(timeout=JOB_TIMEOUT):
                        job["back"].clear()
                    else:
                        if self.runtime:
                            self.runtime.request_stop(f"{job['name']} missed its step deadline")
                        # Do not advance another frame concurrently with a late job.
                        while not job['back'].wait(0.05):
                            if self.runtime and self.runtime.closed:
                                raise WorkcellClosed()
                        job['back'].clear()
                    if self.runtime:
                        self.runtime.checkpoint()
            return result
 
    def jobs_running(self):
        return any(not j["done"] for j in self.jobs.values())
 
    def start_idle_ticker(self):
        """Keep the background jobs moving while the kitchen waits on the camera."""
        def tick():
            try:
                while not self.runtime or not self.runtime.closed:
                    if self.kitchen_idle:
                        self.step(DT)
                        time.sleep(IDLE_GAP)
                    else:
                        time.sleep(DT)
            except WorkcellClosed:
                pass
        threading.Thread(target=tick, daemon=True).start()
 
 
# ============================================================
# PRISONER 1: walk forward (-y), wait for the tray, walk away with it (+y)
 
QUEUE = ["prisoner1", "prisoner3"]   # served in this order
COUNTER_Y = 5.9              # walk (-y) up to this y at the bench (light curtain is at 4.7)
HOLD = (0.0, -0.30, 0.95)    # where the tray sits relative to the prisoner (in front, hand height)
AWAY_SIDE = 1.5              # after getting the tray: step sideways (+x) ...
AWAY_BACK = 2.5              # ... then walk back (+y)
AWAY_SPACING = 1.0           # each next prisoner goes this much further sideways, so they don't meet
PRISONER_STEPS = 60
TRAY_PULL_BACK = 0.4         # first slide the tray this far back (+y) so it clears the bars
import queue
meals = queue.Queue()        # kitchen puts (tray, riders) here after each handout; None = no more
 
 
class MealTrayTracker:
    """While the kitchen serves, remember where each item was last seen in the scene
    (before Maryam's handout hides it), so the served tray can be put back."""
    def __init__(self, env, items):
        self.env = env
        self.items = list(items)
        self.seen = {id(m): np.array(m.T, float) for m in self.items}
 
    @staticmethod
    def sane(T):
        return abs(T[2, 3]) < 5 and np.linalg.norm(T[:2, 3]) < 100   # not hidden / despawned
 
    def watch(self, extra):
        for m in extra:
            if id(m) not in self.seen:
                self.items.append(m)
                self.seen[id(m)] = np.array(m.T, float)
 
    def __enter__(self):
        self.scene_thread = threading.current_thread()
        self.real_step = self.env.step
 
        def step(*args, **kwargs):
            if threading.current_thread() is self.scene_thread:
                for m in self.items:
                    T = np.array(m.T, float)
                    if self.sane(T):
                        self.seen[id(m)] = T
            return self.real_step(*args, **kwargs)
 
        self.env.step = step
        return self
 
    def __exit__(self, *exc):
        self.env.step = self.real_step
 
    def put_back(self, meshes):
        """Bring meshes back to where they were last seen, if the handout hid them."""
        for mesh in meshes:
            if not self.sane(np.array(mesh.T, float)) and id(mesh) in self.seen:
                mesh.T = self.seen[id(mesh)]
                try:
                    if mesh not in getattr(self.env, "swift_objects", [mesh]):
                        self.env.add(mesh)       # it was removed from the scene: add it again
                except Exception:
                    pass
 
 
 
 
def nearest_tray(prisoner_xyz, trays):
    meshes = list(trays.values()) if isinstance(trays, dict) else list(trays)
    if not meshes:
        return None
    return min(meshes, key=lambda m: np.linalg.norm(np.array(m.T, float)[:3, 3] - prisoner_xyz))
 
 
def walk(env, mesh, direction, distance, carried=()):
    """Slide the prisoner (and anything carried) in a straight line."""
    T = np.array(mesh.T, float)
    start = T[:3, 3].copy()
    items = [(c, np.array(c.T, float)) for c in carried]
    starts = [Ti[:3, 3].copy() for _, Ti in items]
    for s in np.linspace(0, 1, PRISONER_STEPS):
        move = s * distance * np.array(direction, float)
        T[:3, 3] = start + move
        mesh.T = T
        for (c, Ti), st in zip(items, starts):
            Ti[:3, 3] = st + move
            c.T = Ti
        env.step(DT)
 
 
def slide_items(env, items, shift, steps=20):
    """Move several meshes together by `shift` (x, y, z) in a straight line."""
    starts = [np.array(m.T, float) for m in items]
    for s in np.linspace(0, 1, steps)[1:]:
        for m, T0 in zip(items, starts):
            T = T0.copy()
            T[:3, 3] += s * np.asarray(shift, float)
            m.T = T
        env.step(DT)
 
 
def hold_tray(env, mesh, tray, riders):
    """Pull the tray back clear of the bars, then move it into the prisoner's hands."""
    slide_items(env, [tray] + riders, [0.0, TRAY_PULL_BACK, 0.0])
    P = np.array(mesh.T, float)[:3, 3]
    shift = P + np.array(HOLD) - np.array(tray.T, float)[:3, 3]
    slide_items(env, [tray] + riders, [0.0, 0.0, shift[2]])           # up to hand height
    slide_items(env, [tray] + riders, [shift[0], shift[1], 0.0])      # then into her hands
 
 
def wait_for_meal(env):
    while True:
        try:
            return meals.get_nowait()
        except queue.Empty:
            env.step(DT)                  # stand still, keep taking turns
 
 
def prisoners_job(env, objects):
    """Each prisoner in turn: walk up to the bench, wait, take the tray, walk away.
    The next prisoner starts walking up once the one before has walked away."""
    for i, name in enumerate(QUEUE):
        mesh = objects.get(name)
        if mesh is None:
            continue
        y = np.array(mesh.T, float)[1, 3]
        print(f"[{name}] walking up to the bench", flush=True)
        walk(env, mesh, [0, -1, 0], max(y - COUNTER_Y, 0.0))
 
        print(f"[{name}] waiting for a tray...", flush=True)
        meal = wait_for_meal(env)
        if meal is None:
            print(f"[{name}] no more meals - staying put", flush=True)
            return
        tray, riders = meal
        riders = [r for r in riders if r is not mesh]
        hold_tray(env, mesh, tray, riders)
        print(f"[{name}] got the tray! walking away", flush=True)
        walk(env, mesh, [1, 0, 0], AWAY_SIDE + i * AWAY_SPACING, carried=[tray] + riders)
        walk(env, mesh, [0, 1, 0], AWAY_BACK, carried=[tray] + riders)
 
 
def remove_prisoner(env, objects, name):
    """Hide a prisoner far under the floor."""
    if name in objects:
        T = np.array(objects[name].T, float)
        T[:3, 3] = [-500.0, -500.0, -100.0]     # far outside every safety zone
        objects[name].T = T
 
 
 
 
 
def route_alarm_inputs(runtime):
    """Hannah's light curtain / prison break / wall collision use input() to wait.
    With the GUI there's no terminal, so they trigger the GUI e-stop instead and
    wait until it is reset from the GUI."""
    def gui_alarm(prompt=""):
        print(prompt, flush=True)
        runtime.request_stop(str(prompt).strip() or "Safety alarm")
        runtime.checkpoint()          # waits here until the GUI resets / resumes
        return ""
    for name, module in list(sys.modules.items()):
        if module is not None and name.split(".")[-1] in ("E_STOP", "estop", "Prison_break", "Light_curtain"):
            module.input = gui_alarm
 
 
# ============================================================
# KITCHEN
# ============================================================
def run_orders(scanner, dispatcher, movements, sync, env, trays, objects):
    """Finish each meal before scanning again; stop on a failed movement."""
    while True:
        sync.kitchen_idle = True                  # camera waiting: others keep moving
        result = dispatcher.handle(scanner.scan())
        sync.kitchen_idle = False
        if sync.runtime:
            with sync.turn:
                sync.runtime.checkpoint()
 
        if result.action == "CANCELLED":
            print("Camera cancelled - meals finished.", flush=True)
            break
        if result.action == "ALERT":
            if sync.runtime:
                sync.runtime.log('Yellow detected: no tray')
            print("Yellow detected: no tray sent. Show GREEN or BLUE, or press Q to finish.",
                  flush=True)
            continue
        if result.action != "SEND_TRAY":
            print(f"[{result.action}] {result.message}", flush=True)
            continue
 
        order = result.order
        colour = order.tray.colour
        if sync.runtime:
            sync.runtime.log(f'{colour.capitalize()} detected')
        print(f"[MEAL] Starting {colour}: Maryam pickup, Yara food, ReBot towel, handout.", flush=True)
        try:
            # Work out BEFORE serving which tray, towel and food this meal will use
            pick = movements.stacks.next_tray(colour.lower())
            tray = pick[1] if pick else None
            towel = movements.towels[-1] if movements.towels else None
            food_before = list(yara.FOOD_NAMES.get(colour.lower(), []))
            watch = [m for m in [tray, towel] if m is not None] + [objects[n] for n in food_before if n in objects]
 
            with sync.turn, MealTrayTracker(env, watch) as tracker:
                served = movements.serve(colour, lambda: dispatcher.mark_on_conveyor(order))
            if served and tray is not None:
                # The tray that was served, plus the food Yara used and the towel ReBot placed
                used = [n for n in food_before if n not in yara.FOOD_NAMES.get(colour.lower(), [])]
                riders = [objects[n] for n in used if n in objects]
                if towel is not None and towel not in movements.towels:
                    riders.append(towel)
                tracker.put_back([tray] + riders)     # undo the despawn
                meals.put((tray, riders))             # next prisoner in line takes it
                env.step(DT)
        except RuntimeError as exc:
            print(f"[STOPPED] {exc}. Meal is not marked delivered.", flush=True)
            if sync.runtime:
                sync.runtime.request_stop(str(exc))
            raise
        if served:
            dispatcher.mark_delivered(order)
            print("[MEAL] Handout complete. Ready for the next colour card.", flush=True)
        dispatcher.reset(colour)
 
 
# ============================================================
# MAIN
# ============================================================
def run_workcell(runtime, bridge):
    """Build once on the scene thread; Qt owns all desktop/camera widgets."""
    env = None
    try:
        import asyncio
        asyncio.set_event_loop(asyncio.new_event_loop())
        env, objects, robot, trays = build_scene()
        runtime.env, runtime.objects = env, objects
        yellow_trays = [objects["yellowTray1"], objects["yellowTray2"], objects["yellowtray3"]]
        from Collective_Folder import E_STOP
        # WallCollision imports the top-level alias on this entry path.
        import E_STOP as wall_estop
        E_STOP.set_stop_callback(runtime.request_stop)
        wall_estop.set_stop_callback(runtime.request_stop)
        walls = None
        if USE_WALLS:
            walls = WallCollision(env, objects)
            walls.watch_trays(yellow_trays)
            walls.test_ball_in_path()
            walls.arm()
        remove_prisoner(env, objects, "prisoner2")
 
        # ---- Hannah's safety: light curtain (shown) + prison break zones ----
        if USE_LIGHT_CURTAIN:
            curtain = SafetyLightCurtain(env)
            curtain.watch_prisoners([objects[n] for n in QUEUE if n in objects])
            curtain.arm()
        if USE_PRISON_BREAK:
            prison = PrisonBreak(env, objects)
            prison.prisoners.pop("prisoner2", None)        # hidden, don't check it
            prison.arm()
        route_alarm_inputs(runtime)
 
        movements = IntegratedMeal(env, objects, robot, trays, activity=runtime.log)
        runtime.collision = MaryamCollisionDemo(runtime, movements)
        robots = {"MaryamBot": robot, "YaraBot": yara.yarabot, "ReBot": movements.rebot}
        hannah_run = None
        if RUN_FEED_ISO:
            if "prepare_only" in inspect.signature(feed_iso).parameters:
                hannah, turtle, hannah_run = feed_iso(
                    env, yellow_trays, walls=walls, prepare_only=True, activity=runtime.log)
                robots["HannahBot"] = hannah
            else:
                # Hannah's Feed_Iso: loads HannahBot and runs, as a job after Start
                hannah_run = lambda: feed_iso(env, yellow_trays, walls=walls)
        # HANNAH: unsafe-zone/light-curtain monitoring is separate from WallCollision.
        runtime.safety = SafetyDemonstration(env, objects, runtime)
        runtime.safety.arm()
        env.step(DT)  # Flush all robot and scene geometry before reporting Ready.
        runtime.robots = robots
        # TEAM: register collision predicates in workcell_actions.install_collision_checks.
        from Collective_Folder import workcell_actions
        workcell_actions.install_collision_checks(runtime)
        queue_check = movements.controller.extra_valid
        movements.controller.extra_valid = lambda q:queue_check(q) and runtime.configuration_clear('MaryamBot', q)
        movements.delivery.payload_check = lambda q, pose:runtime.configuration_clear('MaryamBot', q, pose)
        movements.delivery.configuration_guard = lambda q, pose:runtime.guard_configuration('MaryamBot', q, pose)
        for name, model in robots.items():
            model.configuration_guard = lambda q, n=name:runtime.guard_configuration(n, q)
        yara.team_collision_check = lambda q:runtime.configuration_clear('YaraBot', q)
        validators = {
            "MaryamBot": movements.controller.valid,
            "YaraBot": lambda q: yara.configuration_safe(q)[0],
        }
        bridge.attach(env, robots, runtime, env.step, validators)
        sync = LockStep(env, runtime)
        with runtime.condition:
            if runtime.state != 'ESTOP':
                runtime.set_state("READY", "Shared Swift scene and all robots loaded. Press Start.")
        runtime.checkpoint()  # No camera, prisoner or automatic motion before Start.
        if hannah_run:
            sync.add_job("FEED ISO", hannah_run)
        if RUN_PRISONERS:
            # HANNAH: add/replace prisoner animation jobs here. They start after START.
            sync.add_job("PRISONERS", lambda: prisoners_job(env, objects))
        sync.start_idle_ticker()
        run_orders(runtime, build_default_dispatcher(), movements, sync, env, trays, objects)
        meals.put(None)                   # no more meals: don't leave anyone waiting forever
        sync.kitchen_idle = False
        while sync.jobs_running():
            env.step(DT)
        runtime.set_state("COMPLETE", "Sequence finished. Manual controls remain available.")
        runtime.checkpoint()  # Keep the existing scene alive and service manual commands.
    except WorkcellClosed:
        pass
    except Exception as exc:
        import traceback
        traceback.print_exc()
        runtime.fail(f"Workcell failed: {exc}. Relaunch after correcting the fault.")
        with runtime.condition:
            while not runtime.closed:
                runtime.condition.wait(timeout=0.1)
    finally:
        if env is not None and runtime.closed:
            try:
                env.close()
            except Exception:
                pass
 
 
def main():
    from PySide6.QtWidgets import QApplication
    gui_folder = ASSIGNMENT_ROOT / "Collective_Folder/MealBot_GUI/MealBot_GUI/MealBot_GUI"
    sys.path.insert(0, str(gui_folder))
    from main_gui import MainWindow
    from theme import apply_theme
    from camera_scanner import CameraScanner
 
    app = QApplication.instance() or QApplication(sys.argv)
    apply_theme(app)
    runtime = WorkcellRuntime()
    window = MainWindow(runtime=runtime)
    camera = CameraScanner(runtime, window)
    window.camera_scanner = camera
    window.show()
    worker = threading.Thread(target=run_workcell, args=(runtime, window.bridge), daemon=True)
    worker.start()
    app.aboutToQuit.connect(runtime.close)
    app.aboutToQuit.connect(camera.release)
    app.exec()
    runtime.close()
    worker.join(timeout=1.0)
 
 
if __name__ == "__main__":
    main()
 