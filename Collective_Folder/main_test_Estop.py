import sys
import numpy as np
import threading
import time
from pathlib import Path
 
ASSIGNMENT_ROOT = Path(__file__).resolve().parents[1]
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
# E-stop: importing this patches Swift once, so ALL robots obey it (no per-robot edits)
from Collective_Folder import E_stop_test as estop
from Collective_Folder.E_stop_test import ESTOP
 
DT = 0.05
 
# Switches - turn things off to find what stops the kitchen
RUN_FEED_ISO = True
RUN_PRISONERS = True
USE_WALLS = True
IDLE_GAP = 0.03         # pause between background steps while the camera is open
JOB_TIMEOUT = 0.5      # longest the kitchen waits for feed_iso / prisoners each step
 
 

# ============================================================
# E-STOP <-> GUI ADAPTER
#   Stop   = freeze every robot where it is (task paused, not cancelled)
#   Home   = (while stopped) robots glide home, still holding trays/food
#   Resume = robots glide back to where they were and the task carries on
# Wire your GUI buttons to runtime.estop_stop / estop_home / estop_resume
# (the E / H / R keys are also bound on the window as shortcuts).
# ============================================================
def wire_estop(runtime, window=None):
    original_checkpoint = runtime.checkpoint
    original_request_stop = runtime.request_stop

    def checkpoint(*a, **k):
        # While our E-stop is active the pause happens inside env.step (it must keep
        # drawing frames to glide home), so don't also block here.
        if ESTOP.stop_event.is_set() or ESTOP.state != "running":
            return None
        return original_checkpoint(*a, **k)

    def estop_stop(message="Emergency stop"):
        ESTOP.stop(message)
        try:
            runtime.log(f"E-STOP: {message}")
        except Exception:
            pass

    def estop_home():
        ESTOP.go_home()

    def estop_resume():
        ESTOP.resume()

    runtime.checkpoint = checkpoint
    runtime.request_stop = estop_stop          # wall hits / job failures / GUI stop all freeze the robots
    runtime.estop_stop, runtime.estop_home, runtime.estop_resume = estop_stop, estop_home, estop_resume
    ESTOP.set_stop_callback(lambda msg=None: None)   # avoid recursion with runtime.request_stop
    runtime._original_request_stop = original_request_stop



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
        self.first_step_hooks = {}        # job name -> function run once after the job's first step
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
                        hook = self.first_step_hooks.pop(job["name"], None)
                        if hook:
                            hook()
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
# PRISONER 1: walk up, then wait for the tray handout
# ============================================================
PRISONER_WALK = 2.0          # metres forward (the way prisoner1 is facing)
PRISONER_STEPS = 60
tray_handed_out = threading.Event()     # set when the kitchen hands the tray over
 
 
def prisoner1_job(env, objects):
    # HANNAH: replace this walking animation / add the other prisoners here.
    # main() schedules this job ONLY AFTER the operator presses Start.
    # Yield with env.step(DT) each animation frame to share the same scene.
    mesh = objects["prisoner1"]
    T = np.array(mesh.T, float)
    start, forward = T[:3, 3].copy(), T[:3, 0]
    print("[PRISONER 1] walking up", flush=True)
    for s in np.linspace(0, 1, PRISONER_STEPS):
        T[:3, 3] = start + s * PRISONER_WALK * forward
        mesh.T = T
        env.step(DT)
 
    print("[PRISONER 1] waiting for the tray...", flush=True)
    while not tray_handed_out.is_set():
        env.step(DT)                      # stand still, keep taking turns
    print("[PRISONER 1] got the tray!", flush=True)
 
 
def remove_prisoner(env, objects, name):
    """Hide a prisoner far under the floor."""
    if name in objects:
        T = np.array(objects[name].T, float)
        T[:3, 3] = [0.0, 0.0, -100.0]
        objects[name].T = T
 
 
# ============================================================
# KITCHEN
# ============================================================
def run_orders(scanner, dispatcher, movements, sync):
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
            ESTOP.begin_task()                    # start watching what each robot is holding
            try:
                with sync.turn:                   # kitchen holds the scene while it moves
                    served = movements.serve(colour, lambda: dispatcher.mark_on_conveyor(order))
            finally:
                ESTOP.end_task()
        except RuntimeError as exc:
            print(f"[STOPPED] {exc}. Meal is not marked delivered.", flush=True)
            if sync.runtime:
                sync.runtime.request_stop(str(exc))
            raise
        if served:
            tray_handed_out.set()                 # prisoner 1 has their tray
            dispatcher.mark_delivered(order)
            print("[MEAL] Handout complete. Ready for the next colour card.", flush=True)
        dispatcher.reset(colour)
 
 
# ============================================================
# MAIN
# ============================================================
def run_workcell(runtime, bridge, scanner=None):
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
        E_STOP.set_stop_callback(lambda m=None: runtime.request_stop(m))      # wall hits now freeze all robots
        wall_estop.set_stop_callback(lambda m=None: runtime.request_stop(m))
        ESTOP.install(env)                                  # hook Swift: every robot is covered
        walls = None
        if USE_WALLS:
            walls = WallCollision(env, objects)
            walls.watch_trays(yellow_trays)
            walls.test_ball_in_path()
            walls.arm()
        remove_prisoner(env, objects, "prisoner2")
        movements = IntegratedMeal(env, objects, robot, trays, activity=runtime.log)
        runtime.collision = MaryamCollisionDemo(runtime, movements)
        robots = {"MaryamBot": robot, "YaraBot": yara.yarabot, "ReBot": movements.rebot}
        hannah_run = None
        if RUN_FEED_ISO:
            hannah, turtle, hannah_run = feed_iso(
                env, yellow_trays, walls=walls, prepare_only=True, activity=runtime.log)
            robots["HannahBot"] = hannah
        # HANNAH: unsafe-zone/light-curtain monitoring is separate from WallCollision.
        runtime.safety = SafetyDemonstration(env, objects, runtime)
        runtime.safety.arm()
        env.step(DT)  # Flush all robot and scene geometry before reporting Ready.
        for name, model in robots.items():                  # name them first, then pick up anything else
            ESTOP.register_robot(name, model)
        ESTOP.scan_robots()
        if hannah_run:                                      # Hannah's arm rides on her TurtleBot
            try:
                others = [robot, yara.yarabot, getattr(yara, "turtlebot", None), movements.rebot]
                if hasattr(turtle, "q") and hasattr(turtle, "base"):
                    ESTOP.register_robot("HannahTurtle", turtle)
                    # turtlebot stays where it stopped; only the ARM (and parts riding on it) folds up on H
                    ESTOP.group_near(hannah, turtle, exclude=[o for o in others if o is not None], stay=True)
                else:
                    ESTOP.flag_chassis(hannah)              # turtlebot is a plain mesh: carry it along
            except Exception as exc:
                print(f"[ESTOP] Hannah group failed: {exc}", flush=True)
        try:                                                # Yara's arm rides on her TurtleBot
            ESTOP.mount(yara.yarabot, yara.turtlebot)
        except Exception:
            pass
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
        ESTOP.describe_robots()                              # prints each robot's own methods (for set_refresh)
        wire_estop(runtime, getattr(bridge, 'window', None))
        with runtime.condition:
            if runtime.state != 'ESTOP':
                runtime.set_state("READY", "Shared Swift scene and all robots loaded. Press Start.")
        runtime.checkpoint()  # No camera, prisoner or automatic motion before Start.
        tray_handed_out.clear()
        if hannah_run:
            sync.add_job("FEED ISO", hannah_run)
            # Hannah is only placed once her job starts, so her home = where she is after her first step
            sync.first_step_hooks["FEED ISO"] = lambda: ESTOP.rehome(hannah, *( [turtle] if hasattr(turtle, "q") else [] ))
        if RUN_PRISONERS:
            # HANNAH: add/replace prisoner animation jobs here. They start after START.
            sync.add_job("PRISONER 1", lambda: prisoner1_job(env, objects))
        sync.start_idle_ticker()
        run_orders(scanner or runtime, build_default_dispatcher(), movements, sync)
        tray_handed_out.set()
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
    from main_gui import MainWindow  # type: ignore  (found at runtime via sys.path)
    from theme import apply_theme  # type: ignore  (found at runtime via sys.path)
    from camera_scanner import CameraScanner  # type: ignore  (found at runtime via sys.path)

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