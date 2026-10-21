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
from Collective_Folder.integrated_meal import IntegratedMeal
 
 
# Hannah
from Hannah_Work.wall_collision import WallCollision
from Hannah_Work.Feed_Iso import feed_iso
 
DT = 0.05
 
# Switches - turn things off to find what stops the kitchen
RUN_FEED_ISO = True
RUN_PRISONERS = True
USE_WALLS = True
IDLE_GAP = 0.03         # pause between background steps while the camera is open
JOB_TIMEOUT = 0.5      # longest the kitchen waits for feed_iso / prisoners each step
 
 
# ============================================================
# LOCK-STEP: background jobs take one step each time the scene steps
# ============================================================
class LockStep:
    def __init__(self, env):
        self.env = env
        self.real_step = env.step
        self.jobs = {}                    # thread -> job info
        self.turn = threading.RLock()     # only one thing touches the scene at a time
        self.kitchen_idle = False
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
            result = self.real_step(dt, *args, **kwargs)    # draw the frame once
            for job in list(self.jobs.values()):
                if not job["done"]:
                    job["go"].set()                         # let this job do ONE step
                    if job["back"].wait(timeout=JOB_TIMEOUT):
                        job["back"].clear()
                    elif not job.get("warned"):
                        job["warned"] = True                # never let a job freeze the kitchen
                        print(f"[{job['name']}] slow to hand back - kitchen carries on", flush=True)
            return result
 
    def jobs_running(self):
        return any(not j["done"] for j in self.jobs.values())
 
    def start_idle_ticker(self):
        """Keep the background jobs moving while the kitchen waits on the camera."""
        def tick():
            while True:
                if self.kitchen_idle and self.jobs_running():
                    self.step(DT)
                    time.sleep(IDLE_GAP)      # give the camera window time to draw
                else:
                    time.sleep(DT)
        threading.Thread(target=tick, daemon=True).start()
 
 
# ============================================================
# PRISONER 1: walk up, then wait for the tray handout
# ============================================================
PRISONER_WALK = 2.0          # metres forward (the way prisoner1 is facing)
PRISONER_STEPS = 60
tray_handed_out = threading.Event()     # set when the kitchen hands the tray over
 
 
def prisoner1_job(env, objects):
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
 
        if result.action == "CANCELLED":
            print("Camera cancelled - meals finished.", flush=True)
            break
        if result.action == "ALERT":
            print("Yellow detected: no tray sent. Show GREEN or BLUE, or press Q to finish.",
                  flush=True)
            continue
        if result.action != "SEND_TRAY":
            print(f"[{result.action}] {result.message}", flush=True)
            continue
 
        order = result.order
        colour = order.tray.colour
        print(f"[MEAL] Starting {colour}: Maryam pickup, Yara food, ReBot towel, handout.", flush=True)
        try:
            with sync.turn:                       # kitchen holds the scene while it moves
                served = movements.serve(colour, lambda: dispatcher.mark_on_conveyor(order))
        except RuntimeError as exc:
            print(f"[STOPPED] {exc}. Meal is not marked delivered.", flush=True)
            break
        if served:
            tray_handed_out.set()                 # prisoner 1 has their tray
            dispatcher.mark_delivered(order)
            print("[MEAL] Handout complete. Ready for the next colour card.", flush=True)
        dispatcher.reset(colour)
 
 
# ============================================================
# MAIN
# ============================================================
def main():
    sys.setswitchinterval(0.001)    # switch threads often so the camera never starves
    # ---- ONE scene, everything loaded BEFORE anything moves ----
    env, objects, robot, trays = build_scene()
    print("[MAIN] scene built", flush=True)
 
    yellow_trays = [objects["yellowTray1"], objects["yellowTray2"], objects["yellowtray3"]]
    walls = None
    if USE_WALLS:
        walls = WallCollision(env, objects)
        walls.watch_trays(yellow_trays)
        walls.test_ball_in_path()
        walls.arm()
 
    remove_prisoner(env, objects, "prisoner2")
    movements = IntegratedMeal(env, objects, robot, trays)   # kitchen robots loaded now
    env.step(DT)
 
    # Open the camera FIRST, before anything else is running
    print("[KITCHEN] opening camera...", flush=True)
    scanner = ColourScanner()
    print("[KITCHEN] camera ready", flush=True)
 
    # ---- Lock-step: feed_iso + prisoners take a step every time the scene steps ----
    sync = LockStep(env)
    if RUN_FEED_ISO:
        sync.add_job("FEED ISO", lambda: feed_iso(env, yellow_trays, walls=walls))
    if RUN_PRISONERS:
        sync.add_job("PRISONER 1", lambda: prisoner1_job(env, objects))
    print("[MAIN] background jobs started", flush=True)
 
    # Give HannahBot a few steps on its own so it loads in fully before the kitchen starts
    for _ in range(5):
        env.step(DT)
    print("[MAIN] warm-up done", flush=True)
    sync.start_idle_ticker()
 
    # ---- Kitchen (camera must stay on the main thread) ----
    print("[KITCHEN] show a colour card to the camera", flush=True)
    try:
        run_orders(scanner, build_default_dispatcher(), movements, sync)
    finally:
        scanner.close()
 
    # ---- Kitchen done: keep stepping until HannahBot and the prisoners finish ----
    tray_handed_out.set()             # no more meals: don't leave prisoner 1 waiting forever
    sync.kitchen_idle = False
    while sync.jobs_running():
        env.step(DT)
    env.hold()
 
 
if __name__ == "__main__":
    main()
 