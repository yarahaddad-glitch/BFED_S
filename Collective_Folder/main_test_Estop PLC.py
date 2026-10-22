
# """
# SMART MEALBOT - CAMERA + PHYSICAL PLC STOP

# GREEN / BLUE:
#     Start meal preparation and delivery.

# YELLOW:
#     Camera alert. No meal order.

# PLC RED / X14:
#     Pause the simulated workcell.

# PLC YELLOW:
#     Remain paused.

# PLC GREEN after reset:
#     Resume the interrupted sequence.

# Simulation only - NOT a safety-rated E-stop.
# """

# import sys
# import time
# import queue
# import threading
# from pathlib import Path

# import numpy as np

# # ============================================================
# # PATHS
# # ============================================================

# ROOT = Path(__file__).resolve().parents[1]

# for folder in (ROOT, ROOT / "Collective_Folder"):
#     if str(folder) not in sys.path:
#         sys.path.insert(0, str(folder))

# # ============================================================
# # IMPORTS
# # ============================================================

# from Maryam_Work.maryam_meal_dispatch import (
#     ColourScanner,
#     build_default_dispatcher,
# )

# from Maryam_Work.maryam_movements import build_scene

# from Collective_Folder.integrated_meal import IntegratedMeal

# from Collective_Folder import yara_main_testing as yara

# from Collective_Folder.PLC_stop import PLCEStopMonitor

# from Hannah_Work.Feed_Iso import feed_iso

# from Hannah_Work.wall_collision import WallCollision

# # ============================================================
# # SETTINGS
# # ============================================================

# DT = 0.05
# PLC_PORT = "COM3"

# RUN_FEED_ISO = True
# RUN_PRISONERS = True
# USE_WALLS = True

# # Leave these disabled for the first integrated PLC test.
# # Re-enable after confirming the physical stop works.
# USE_LIGHT_CURTAIN = False
# USE_PRISON_BREAK = False

# IDLE_GAP = 0.03

# # ============================================================
# # SHARED PLC STATE
# # ============================================================

# class PLCControl:

#     def __init__(self):

#         self.stopped = threading.Event()
#         self.stopped.set()

#         self.reset_requested = threading.Event()

#         self.plc = None
#         self.initial_ready = False

#         self.lock = threading.RLock()

#     def stop(self, message="PLC E-stop"):

#         with self.lock:

#             was_running = not self.stopped.is_set()

#             self.stopped.set()
#             self.reset_requested.clear()

#             if was_running:
#                 print(
#                     f"\n[PLC] STOP: {message}",
#                     flush=True,
#                 )

#     def home(self):

#         # Do not move robots automatically when
#         # the physical button is released.

#         self.stopped.set()

#         print(
#             "[PLC] Yellow - waiting for reset",
#             flush=True,
#         )

#     def resume(self):

#         self.reset_requested.set()

#         print(
#             "[PLC] Reset signal received",
#             flush=True,
#         )

#     def ready(self):

#         plc = self.plc

#         if plc is None or not plc.connected:
#             return False

#         if plc.inputs is None or plc.outputs is None:
#             return False

#         if plc.button_pressed:
#             return False

#         # Red light must be OFF.
#         if plc.outputs[1]:
#             return False

#         # Green light must be ON.
#         if not plc.outputs[3]:
#             return False

#         return True

#     def check(self):

#         if not self.ready():

#             self.stopped.set()
#             return False

#         with self.lock:

#             # Permit initial startup only when the
#             # PLC is already in its normal green state.
#             if not self.initial_ready:

#                 self.initial_ready = True
#                 self.stopped.clear()

#                 print(
#                     "[PLC] Ready - simulation enabled",
#                     flush=True,
#                 )

#             elif self.reset_requested.is_set():

#                 self.reset_requested.clear()
#                 self.stopped.clear()

#                 print(
#                     "\n[PLC] RESET - simulation resumed",
#                     flush=True,
#                 )

#         return not self.stopped.is_set()

#     def wait(self):

#         while not self.check():
#             time.sleep(0.02)


# CONTROL = PLCControl()


# def start_plc():

#     plc = PLCEStopMonitor(
#         stop=CONTROL.stop,
#         home=CONTROL.home,
#         resume=CONTROL.resume,
#         is_active=CONTROL.stopped.is_set,
#         port=PLC_PORT,
#     )

#     CONTROL.plc = plc

#     plc.start()

#     return plc


# # ============================================================
# # LOCKSTEP
# # ============================================================

# class LockStep:

#     def __init__(self, env):

#         self.env = env

#         self.real_step = env.step

#         self.jobs = {}

#         self.turn = threading.RLock()

#         self.kitchen_idle = False

#         self.closed = False

#         env.step = self.step

#     def add_job(self, name, fn):

#         job = {
#             "name": name,
#             "go": threading.Event(),
#             "back": threading.Event(),
#             "done": False,
#         }

#         def run():

#             job["go"].wait()
#             job["go"].clear()

#             try:

#                 CONTROL.wait()

#                 fn()

#                 print(
#                     f"[{name}] finished",
#                     flush=True,
#                 )

#             except Exception as exc:

#                 print(
#                     f"[{name}] error: {exc!r}",
#                     flush=True,
#                 )

#                 CONTROL.stop(
#                     f"{name} failed"
#                 )

#             finally:

#                 job["done"] = True
#                 job["back"].set()

#         thread = threading.Thread(
#             target=run,
#             name=name,
#             daemon=True,
#         )

#         self.jobs[thread] = job

#         thread.start()

#     def step(self, dt=DT, *args, **kwargs):

#         CONTROL.wait()

#         job = self.jobs.get(
#             threading.current_thread()
#         )

#         # Background jobs hand control back to
#         # the main simulation thread.
#         if job is not None:

#             job["back"].set()

#             while not self.closed:

#                 if job["go"].wait(timeout=0.05):

#                     job["go"].clear()

#                     CONTROL.wait()

#                     return None

#             return None

#         with self.turn:

#             CONTROL.wait()

#             result = self.real_step(
#                 dt,
#                 *args,
#                 **kwargs,
#             )

#             for job in list(self.jobs.values()):

#                 if job["done"]:
#                     continue

#                 CONTROL.wait()

#                 job["go"].set()

#                 # Wait for the background job to
#                 # finish its current movement step.
#                 while not job["back"].wait(0.05):

#                     if self.closed:
#                         return result

#                     CONTROL.check()

#                 job["back"].clear()

#             return result

#     def jobs_running(self):

#         return any(
#             not job["done"]
#             for job in self.jobs.values()
#         )

#     def start_idle_ticker(self):

#         def tick():

#             while not self.closed:

#                 if self.kitchen_idle and self.jobs_running():

#                     self.step(DT)

#                     time.sleep(IDLE_GAP)

#                 else:

#                     time.sleep(DT)

#         threading.Thread(
#             target=tick,
#             daemon=True,
#         ).start()


# # ============================================================
# # PRISONER MOVEMENT
# # ============================================================

# QUEUE = [
#     "prisoner1",
#     "prisoner3",
# ]

# COUNTER_Y = 5.9

# HOLD = (0.0, -0.30, 0.95)

# AWAY_SIDE = 1.5
# AWAY_BACK = 2.5
# AWAY_SPACING = 1.0

# PRISONER_STEPS = 60
# TRAY_PULL_BACK = 0.4

# meals = queue.Queue()


# def walk(env, mesh, direction, distance, carried=()):

#     T = np.array(mesh.T, float)

#     start = T[:3, 3].copy()

#     items = [
#         (item, np.array(item.T, float))
#         for item in carried
#     ]

#     starts = [
#         item_T[:3, 3].copy()
#         for _, item_T in items
#     ]

#     for s in np.linspace(
#         0,
#         1,
#         PRISONER_STEPS,
#     ):

#         CONTROL.wait()

#         move = (
#             s
#             * distance
#             * np.array(direction, float)
#         )

#         T[:3, 3] = start + move

#         mesh.T = T

#         for (item, item_T), item_start in zip(
#             items,
#             starts,
#         ):

#             item_T[:3, 3] = item_start + move

#             item.T = item_T

#         env.step(DT)


# def slide_items(env, items, shift, steps=20):

#     starts = [
#         np.array(item.T, float)
#         for item in items
#     ]

#     for s in np.linspace(0, 1, steps)[1:]:

#         CONTROL.wait()

#         for item, start_T in zip(items, starts):

#             T = start_T.copy()

#             T[:3, 3] += (
#                 s * np.asarray(shift, float)
#             )

#             item.T = T

#         env.step(DT)


# def hold_tray(env, prisoner, tray, riders):

#     slide_items(
#         env,
#         [tray] + riders,
#         [0.0, TRAY_PULL_BACK, 0.0],
#     )

#     prisoner_position = np.array(
#         prisoner.T,
#         float,
#     )[:3, 3]

#     tray_position = np.array(
#         tray.T,
#         float,
#     )[:3, 3]

#     shift = (
#         prisoner_position
#         + np.array(HOLD)
#         - tray_position
#     )

#     slide_items(
#         env,
#         [tray] + riders,
#         [0.0, 0.0, shift[2]],
#     )

#     slide_items(
#         env,
#         [tray] + riders,
#         [shift[0], shift[1], 0.0],
#     )


# def wait_for_meal(env):

#     while True:

#         CONTROL.wait()

#         try:

#             return meals.get_nowait()

#         except queue.Empty:

#             env.step(DT)


# def prisoners_job(env, objects):

#     for index, name in enumerate(QUEUE):

#         CONTROL.wait()

#         prisoner = objects.get(name)

#         if prisoner is None:
#             continue

#         y = np.array(
#             prisoner.T,
#             float,
#         )[1, 3]

#         print(
#             f"[{name}] walking to counter",
#             flush=True,
#         )

#         walk(
#             env,
#             prisoner,
#             [0, -1, 0],
#             max(y - COUNTER_Y, 0.0),
#         )

#         print(
#             f"[{name}] waiting for meal",
#             flush=True,
#         )

#         meal = wait_for_meal(env)

#         if meal is None:
#             return

#         tray, riders = meal

#         riders = [
#             item
#             for item in riders
#             if item is not prisoner
#         ]

#         hold_tray(
#             env,
#             prisoner,
#             tray,
#             riders,
#         )

#         walk(
#             env,
#             prisoner,
#             [1, 0, 0],
#             AWAY_SIDE + index * AWAY_SPACING,
#             carried=[tray] + riders,
#         )

#         walk(
#             env,
#             prisoner,
#             [0, 1, 0],
#             AWAY_BACK,
#             carried=[tray] + riders,
#         )


# def remove_prisoner(objects, name):

#     if name not in objects:
#         return

#     T = np.array(
#         objects[name].T,
#         float,
#     )

#     T[:3, 3] = [
#         -500.0,
#         -500.0,
#         -100.0,
#     ]

#     objects[name].T = T


# # ============================================================
# # MEAL TRACKER
# # ============================================================

# class MealTrayTracker:

#     def __init__(self, env, items):

#         self.env = env

#         self.items = list(items)

#         self.seen = {
#             id(item): np.array(item.T, float)
#             for item in self.items
#         }

#     @staticmethod
#     def sane(T):

#         return (
#             abs(T[2, 3]) < 5
#             and np.linalg.norm(T[:2, 3]) < 100
#         )

#     def __enter__(self):

#         self.real_step = self.env.step

#         def step(*args, **kwargs):

#             CONTROL.wait()

#             if threading.current_thread() is threading.main_thread():

#                 for item in self.items:

#                     T = np.array(
#                         item.T,
#                         float,
#                     )

#                     if self.sane(T):
#                         self.seen[id(item)] = T

#             return self.real_step(
#                 *args,
#                 **kwargs,
#             )

#         self.env.step = step

#         return self

#     def __exit__(self, *exc):

#         self.env.step = self.real_step

#     def put_back(self, meshes):

#         for mesh in meshes:

#             CONTROL.wait()

#             if (
#                 not self.sane(
#                     np.array(mesh.T, float)
#                 )
#                 and id(mesh) in self.seen
#             ):

#                 mesh.T = self.seen[id(mesh)]

#                 try:

#                     if mesh not in getattr(
#                         self.env,
#                         "swift_objects",
#                         [mesh],
#                     ):

#                         self.env.add(mesh)

#                 except Exception:
#                     pass


# # ============================================================
# # CAMERA AND MEAL WORKFLOW
# # ============================================================

# def run_orders(
#     scanner,
#     dispatcher,
#     movements,
#     sync,
#     env,
#     trays,
#     objects,
#     start_background_jobs,
# ):

#     while True:

#         # Background jobs can continue between
#         # meal orders after the first valid scan.
#         sync.kitchen_idle = True

#         print(
#             "\n[CAMERA] Show GREEN or BLUE",
#             flush=True,
#         )

#         result = dispatcher.handle(
#             scanner.scan()
#         )

#         sync.kitchen_idle = False

#         if result.action == "CANCELLED":

#             print(
#                 "[CAMERA] Finished",
#                 flush=True,
#             )

#             break

#         if result.action == "ALERT":

#             print(
#                 "[CAMERA] Yellow detected - alert",
#                 flush=True,
#             )

#             continue

#         if result.action != "SEND_TRAY":

#             print(
#                 f"[CAMERA] {result.action}: {result.message}",
#                 flush=True,
#             )

#             continue

#         # A valid camera order has been received.
#         CONTROL.wait()

#         start_background_jobs()

#         order = result.order
#         colour = order.tray.colour

#         print(
#             f"\n[MEAL] Starting {colour}",
#             flush=True,
#         )

#         try:

#             CONTROL.wait()

#             pick = movements.stacks.next_tray(
#                 colour.lower()
#             )

#             tray = pick[1] if pick else None

#             towel = (
#                 movements.towels[-1]
#                 if movements.towels
#                 else None
#             )

#             food_before = list(
#                 yara.FOOD_NAMES.get(
#                     colour.lower(),
#                     [],
#                 )
#             )

#             watch = [
#                 item
#                 for item in (tray, towel)
#                 if item is not None
#             ]

#             watch += [
#                 objects[name]
#                 for name in food_before
#                 if name in objects
#             ]

#             with sync.turn, MealTrayTracker(
#                 env,
#                 watch,
#             ) as tracker:

#                 CONTROL.wait()

#                 served = movements.serve(
#                     colour,
#                     lambda: dispatcher.mark_on_conveyor(
#                         order
#                     ),
#                 )

#             CONTROL.wait()

#             if served and tray is not None:

#                 used = [
#                     name
#                     for name in food_before
#                     if name not in yara.FOOD_NAMES.get(
#                         colour.lower(),
#                         [],
#                     )
#                 ]

#                 riders = [
#                     objects[name]
#                     for name in used
#                     if name in objects
#                 ]

#                 if (
#                     towel is not None
#                     and towel not in movements.towels
#                 ):
#                     riders.append(towel)

#                 tracker.put_back(
#                     [tray] + riders
#                 )

#                 meals.put(
#                     (tray, riders)
#                 )

#                 env.step(DT)

#         except RuntimeError as exc:

#             print(
#                 f"[MEAL] Stopped: {exc}",
#                 flush=True,
#             )

#             break

#         if served:

#             CONTROL.wait()

#             dispatcher.mark_delivered(order)

#             print(
#                 "[MEAL] Handout complete",
#                 flush=True,
#             )

#         dispatcher.reset(colour)


# # ============================================================
# # MAIN
# # ============================================================

# def main():

#     sys.setswitchinterval(0.001)

#     plc = None
#     scanner = None
#     sync = None
#     env = None

#     try:

#         print("\n===================================")
#         print(" SMART MEALBOT")
#         print(" CAMERA + PLC STOP")
#         print("===================================")

#         # ----------------------------------
#         # BUILD SCENE
#         # ----------------------------------

#         env, objects, robot, trays = build_scene()

#         yellow_trays = [
#             objects["yellowTray1"],
#             objects["yellowTray2"],
#             objects["yellowtray3"],
#         ]

#         walls = None

#         if USE_WALLS:

#             walls = WallCollision(
#                 env,
#                 objects,
#             )

#             walls.watch_trays(
#                 yellow_trays
#             )

#             # No deliberate test ball.
#             walls.arm()

#         remove_prisoner(
#             objects,
#             "prisoner2",
#         )

#         # ----------------------------------
#         # PREPARE KITCHEN
#         # ----------------------------------

#         movements = IntegratedMeal(
#             env,
#             objects,
#             robot,
#             trays,
#         )

#         # ----------------------------------
#         # INSTALL LOCKSTEP
#         # ----------------------------------

#         sync = LockStep(env)

#         # ----------------------------------
#         # START PHYSICAL PLC
#         # ----------------------------------

#         print("\n[PLC] Connecting...")

#         plc = start_plc()

#         # Wait for valid communication.
#         while not CONTROL.ready():

#             if plc.connected:
#                 print(
#                     "[PLC] Waiting for GREEN state",
#                     flush=True,
#                 )

#             time.sleep(1)

#         CONTROL.wait()

#         # ----------------------------------
#         # CAMERA
#         # ----------------------------------

#         print("\n[CAMERA] Opening...")

#         scanner = ColourScanner()

#         print(
#             "[CAMERA] Ready",
#             flush=True,
#         )

#         # ----------------------------------
#         # BACKGROUND JOBS
#         # ----------------------------------

#         background_started = False

#         def start_background_jobs():

#             nonlocal background_started

#             if background_started:
#                 return

#             CONTROL.wait()

#             print(
#                 "[MAIN] First valid camera order",
#                 flush=True,
#             )

#             if RUN_FEED_ISO:

#                 sync.add_job(
#                     "FEED ISO",
#                     lambda: feed_iso(
#                         env,
#                         yellow_trays,
#                         walls=walls,
#                     ),
#                 )

#             if RUN_PRISONERS:

#                 sync.add_job(
#                     "PRISONERS",
#                     lambda: prisoners_job(
#                         env,
#                         objects,
#                     ),
#                 )

#             background_started = True

#         sync.start_idle_ticker()

#         # ----------------------------------
#         # RUN CAMERA ORDERS
#         # ----------------------------------

#         run_orders(
#             scanner,
#             build_default_dispatcher(),
#             movements,
#             sync,
#             env,
#             trays,
#             objects,
#             start_background_jobs,
#         )

#         # ----------------------------------
#         # FINISH BACKGROUND JOBS
#         # ----------------------------------

#         meals.put(None)

#         sync.kitchen_idle = False

#         while sync.jobs_running():

#             env.step(DT)

#         print(
#             "\n[MAIN] All jobs finished",
#             flush=True,
#         )

#         while True:

#             env.step(DT)
#             time.sleep(DT)

#     except KeyboardInterrupt:

#         print(
#             "\n[MAIN] Stopping...",
#             flush=True,
#         )

#     except Exception:

#         import traceback
#         traceback.print_exc()

#     finally:

#         if sync is not None:
#             sync.closed = True

#         if scanner is not None:

#             try:
#                 scanner.close()
#             except Exception:
#                 pass

#         if plc is not None:
#             plc.close()

#         if env is not None:

#             try:
#                 env.close()
#             except Exception:
#                 pass


# if __name__ == "__main__":
#     main()