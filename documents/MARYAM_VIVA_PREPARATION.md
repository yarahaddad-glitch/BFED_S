

> My main technical contribution was the Kawasaki RS007N side of Smart MealBot. I implemented its six-joint standard-DH model and connected the STL meshes to its kinematics. I built the tray pickup, conveyor deposit and loaded-tray handout sequence. Numerical inverse kinematics finds a usable grasp, and damped Jacobian-based resolved-rate control plans the Cartesian carrying motion. I track the tray relative to the tool and the food relative to the tray. I added arm and payload collision checks and a separate obstacle scenario that detects a blocked route and searches for a checked detour.
>
> I also built and connected the purple PySide6 GUI, including automatic and manual controls, the webcam colour scanner, Swift camera views, and the simulation stop, reset and recovery workflow. For ReBot, I worked on the simulated towel pickup and placement using its own link transforms, numerical contact-point IK and gripper animation. I helped integrate these components with Yara, Hannah and the shared scene through one entry point.

If asked what you personally did, use specific examples you can explain and demonstrate. Do not claim your teammates' algorithms simply because you connected them. Explain AI-assisted code by its actual mathematics and behaviour; assistance does not replace understanding.

## 2. The complete system in one trace

```text
main_all.main()
  creates QApplication, WorkcellRuntime, MainWindow, CameraScanner
  starts run_workcell() in a worker thread
    _run_workcell_once()
      build_scene() -> one Swift environment, scene objects, Maryam, tray stacks
      IntegratedMeal(...) -> Maryam controller/delivery + Yara setup + ReBot/towels
      prepare Hannah, safety monitors, recovery, collision scenario
      bridge.attach(...) -> GUI controls reference the SAME live robot objects
      LockStep(...) -> coordinate background jobs around shared scene steps
      READY -> checkpoint waits for Start
      run_orders(runtime, dispatcher, movements, ...)
        runtime.scan() <-> Qt CameraScanner.tick()
        MealDispatcher.handle(DetectionResult)
        IntegratedMeal.serve(colour, on_conveyor)
          Maryam pick_blue/pick_green -> TrayDelivery.deliver()
          conveyor travel -> Yara.run_task()
          conveyor travel -> ReBot.place_towel()
          conveyor returns loaded tray
          Maryam handout_blue/handout_green -> _handout()
          recipient collection; Maryam returns home
        mark delivered -> queue meal for prisoner animation -> next scan
```

**Say:** “The GUI requests work, the runtime controls when it is permitted, the bridge adapts robot models, and the task/controller modules generate and execute movement.”

**Inheritance question:** `IntegratedMeal` inherits `MaryamMovements`. Its `super().__init__()` creates the common controller, layout, stacks and delivery object. It **overrides `serve()`**, replacing the standalone food/conveyor demonstration with the integrated Yara and ReBot sequence. Studying only `MaryamMovements.serve()` would describe a different route.

## 3. Function map: what to open when asked

Line numbers are starting locations in the inspected version; use the function name if subsequent edits move them. Paths below are relative to the assignment root. `GUI/` in this guide means `Collective_Folder/MealBot_GUI/MealBot_GUI/MealBot_GUI/`.

| Examiner asks | File:line and function | What to point at |
| --- | --- | --- |
| Where does everything start? | `Collective_Folder/EnviroParts_S/main_all.py:585`, `main` | Qt app, runtime, window, scanner, worker, shutdown |
| Where are the components connected? | Same file:441, `_run_workcell_once` | Scene, task, safety, recovery, bridge, validators and READY gate |
| Where does one order run? | Same file:380, `run_orders` | Scan, dispatch, serve, success/failure, delivery status |
| How are jobs coordinated? | Same file:83, `LockStep`; :122, `step` | Turn lock and go/back events |
| Where is Maryam defined? | `Maryam_Work/Maryam_robotclass.py:20`, `MaryamBot.__init__` | Six `RevoluteDH` links, offsets, limits and mesh names |
| Where is it placed in the room? | `Collective_Folder/EnviroParts_S/Full_sim.py:39`, `add_maryam_robot` | Counter height, world translation and base yaw |
| Where are tray stacks created? | Same file:53, `add_tray_stacks` | Colour, scale, mesh-centre offsets, two heights per stack |
| Where is the controller constructed? | `Maryam_Work/maryam_movements.py:74`, `build_controller` | STL bounds, serving plane and initially empty obstacle list |
| Where is numerical IK? | `Maryam_Work/maryam_tray_delivery.py:466`, `_plan_pick_and_carry` | FK residual, bounded `least_squares`, multiple seeds |
| How is a grip chosen? | Same file:610, `prepare` | Tray centre, rim candidates, reach check, cache or new plan |
| Where is RMRC? | `Maryam_Work/maryamrobot_controller.py:175`, `segment` | Smooth reference, error feedback, Jacobian, damping, integration |
| Where are Cartesian detours? | Same file:249, `plan` | Direct segment, raised/side waypoints, seeded alternative search |
| Where is pickup executed? | `Maryam_Work/maryam_tray_delivery.py:683`, `deliver` | Approach, descent, attach, carry legs, release, retreat |
| Where does the tray follow the tool? | Same file:268, `update_payload` | FK times fixed relative attachment |
| How is execution guarded? | Same file:283, `_execute` | Precheck pending leg and recheck next joint configuration |
| Where is arm collision detection? | `Maryam_Work/maryamrobot_controller.py:96`, `colliding`; :128, `valid` | Link boxes, barrier, spheres/boxes, joint limits |
| Where is box intersection? | `Maryam_Work/collision_geometry.py:5`, `boxes_overlap` | Broad rejection plus separating axes |
| Where is tray collision detection? | `Maryam_Work/maryam_tray_delivery.py:335`, `configuration_clear`; :357, `tray_clears_robot`; :371, `queued_trays_clear`; :440, `tray_clears_obstacles` | Arm, housing, waiting trays and payload envelope |
| Where is the handout? | `Maryam_Work/maryam_movements.py:291`, `_handout`; :204, `serving_gap_validator` | Turning, alignment, height, aperture and insertion |
| Who moves the released tray? | Same file:226, `collect_served_meal` | Recipient animation, with clearance checks |
| Where is the integrated conveyor? | `Collective_Folder/integrated_meal.py:19`, `carry_tray`; :40, `carry_tray_route`; :82, `IntegratedMeal.serve` | Route legs, station stops and rider transforms |
| Where is ReBot built? | `Collective_Folder/Rebot_class.py:11`, `ReBot.__init__`; :228, `get_link_transforms` | Origin/axis joint chain and mesh model |
| Where is its towel task? | `Collective_Folder/rebot_towels.py:124`, `place_towel`; :97, `solve_contact`; :25, `animate_joints` | Contact IK, grasp, joint motion and attachment |
| Where is colour classified? | `Maryam_Work/maryam_meal_dispatch.py:112`, `ColourScanner.classify` | Centre ROI, HSV masks and coverage |
| Where is the active camera popup? | `GUI/camera_scanner.py:68`, `CameraScanner.tick` | Qt timer, stable votes and result event |
| Where is colour converted to an order? | `Maryam_Work/maryam_meal_dispatch.py:358`, `MealDispatcher.handle` | SEND_TRAY / ALERT / REJECTED / CANCELLED |
| Where are buttons connected? | `GUI/main_gui.py:330`, `MainWindow`; :832, `start_sequence`; :814, `stop_all` | Qt signals, runtime requests |
| Where are joint sliders handled? | `GUI/manual_mode.py:125`, `jog`; :156, `flush_jogs`; `GUI/robot_bridge.py:108`, `set_joint` | Pending commands, degrees-to-radians and checked motion |
| Where is manual XYZ motion? | `GUI/robot_bridge.py:116`, `translate` | Numerical Jacobian pose solve, then checked joint motion |
| Where is the simulation camera? | `GUI/main_gui.py:717`, `select_camera_robot`; :753, `update_camera` | Queue Swift `set_camera_pose`, without moving robot |
| Where is the authoritative stop state? | `Collective_Folder/workcell_runtime.py:104`, `request_stop`; :150, `reset`; :219, `checkpoint` | Latched stop and queued scene commands |
| Where is Home recovery? | `Collective_Folder/maryam_recovery.py:208`, `home_robot`; :215, `_recover`; :267, `restore` | Checked Home and restoration of interrupted task poses |
| Where is the cube demonstration? | `Collective_Folder/maryam_collision_demo.py:162`, `TrayCollisionScenario.prepare`; :386, `plan`; :440, `execute`; :488, `finish` | Reference task, retained cube, detour and restoration |
| Where is RRT-Connect? | `Maryam_Work/joint_planner.py:15`, `plan_joint_route` | Checked edges, two trees, bounded search and shortcuts |

## 4. Your robot: DH, meshes and forward kinematics

### Why this robot?

> “The tray task needs position and orientation control: reaching the stacks, keeping a meal level, turning it and presenting it through an opening. A six-revolute-joint industrial arm suits that task. The RS007N model gave me a serial chain I could represent with standard DH and separate link meshes. I placed the robot and stations so the required grasp points were reachable, then checked complete paths.”

This is a technical rationale, not proof of payload sizing or industrial suitability. If asked why you personally selected it, add your real reason. Do not invent a selection history. Payload capacity, reach, speed, model eligibility and manufacturer agreement need their own evidence; a successful simulation alone does not establish them.

### The actual implemented DH table

Lengths are metres; this table shows angles in degrees for revision. Code uses radians.

| Joint | d | a | alpha | theta | Model q limits | Software q limits |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | .360 | 0 | +90° | q1 − 90° | ±180° | ±170° |
| 2 | 0 | .355 | 0° | q2 + 90° | ±135° | ±120° |
| 3 | 0 | 0 | +90° | q3 + 90° | ±155° | ±140° |
| 4 | .375 | 0 | +90° | q4 + 180° | ±200° | ±180° |
| 5 | 0 | 0 | −90° | q5 | ±125° | ±110° |
| 6 | .078 | 0 | 0° | q6 + 90° | ±360° | ±330° |

**Explain each parameter:** `d` translates along the preceding z axis; `theta` rotates about that z axis; `a` translates along the link x axis; `alpha` twists about that x axis. Joint i rotates about z of frame i−1 under this standard-DH convention. Use a labelled frame sketch to explain signs.

```text
A_i = Rz(q_i + offset_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
T_world_tool = T_world_base A_1 A_2 A_3 A_4 A_5 A_6 T_6_tool
```

An offset aligns the chosen zero joint coordinate with the chosen DH frame. It is a constant added to q, not another joint. A zero `a` does not mean “this physical link has no size”: the frame displacement may appear in `d`, with intersecting or coincident axes.

**FK spoken answer:** “Given six joint angles, FK multiplies the link transformations to return a 4×4 tool pose. `robot.fkine(q)` gives translation `.t` and rotation `.R`. The base transform locates the whole chain in the shared room.”

`T = [[R, p], [0, 1]]`: R is 3×3 orientation, p is a 3-vector position. `SE3` represents this rigid pose; `.A` gives its matrix. Multiplication order matters: world translation followed by local rotation is not generally interchangeable with the reverse order.

**Mesh explanation:** `MaryamBot` inherits `UTSMeshRobot`, combining a DH chain with `RS007N-BASE.STL` and J1–J6 meshes. `meshes_are_global_at_home=True` declares that mesh vertices share the zero-configuration CAD frame. Home-relative transformations let links rotate about their kinematic pivots. Meshes provide appearance; DH provides FK and Jacobians. Misaligned pivots, incorrect units or wrong offsets can make the visual and mathematical robots disagree.

`Full_sim.add_maryam_robot()` mounts the base on the counter and applies `Rz(pi)`. Changing this world placement does not change the link lengths. Also distinguish zero joint coordinates, the mesh reference pose and a startup Home saved by the running application.

## 5. Numerical inverse kinematics: read this function confidently

Open `_plan_pick_and_carry` and find `residual`, `least_squares`, `q_grasp`, `relative`, `targets` and `controller.segment`.

> “IK works backwards from a desired tool pose to joint angles. I solve it numerically by repeatedly evaluating FK and minimising the target error, subject to joint bounds. I try several seeds because a local numerical solver may converge to different branches or fail. I accept a candidate only after checking its grasp and the rest of the transfer.”

Actual sequence:

1. `prepare()` computes the tray's geometric centre from its mesh origin and scale, chooses candidate rim grips and applies a necessary reach check.
2. `_plan_pick_and_carry()` uses nearby remembered seeds, current q, zero q and deterministic random seeds. It also tries explicit downward orientation candidates at four yaw angles.
3. Position residual is `fkine(q).t - grasp`. Some candidates add `0.3 * rotation_error(target_R, actual_R)`; the factor weights orientation relative to position in the optimisation.
4. `least_squares(..., bounds=(q_min, q_max))` finds a local numerical candidate. A solver result is not automatically a valid robot route.
5. The code checks limits/obstacles, tool position and tray clearance, saves the grasp orientation and computes the tool-to-tray transform.
6. RMRC plans lift and carrying legs; tray/queued-tray/aperture checks validate them. It checks the empty-arm approach and descent too.
7. It returns the first accepted complete transfer. If none works, it returns failure without committing a grasp.

**Why not just call `ikine`?** “The numerical IK is implemented directly using SciPy and my FK residual, giving me explicit seed selection, joint bounds and grip candidate handling.”

**Why can two answers reach the same point?** Different elbow/wrist configurations may produce the same pose. Position-only IK constrains three coordinates and leaves orientation freedom; a full pose adds orientation requirements. Joint limits, collisions and singularities further restrict feasible solutions.

**What if a target is unreachable?** The residual remains too large or the subsequent route fails validation. `prepare()` has a necessary reach rejection, but being inside that outer bound does not guarantee reachability. `deliver()` returns `NOT_PICKED` for no valid plan or a stale plan; the integrated task rejects failed movement.

**Caching:** `prepare()` can reuse saved routes under `.mealbot_routes`. `_cache_path`, `_cached_plan` and `_save_plan` manage this. Reuse saves expensive searching; it does not permit blindly executing old geometry. `deliver()` compares current joints and tray pose to the prepared start, and `_execute()` checks pending motion again. Do not say every order performs a fresh IK search.

## 6. RMRC: theory matched to your implementation

> “Resolved Motion Rate Control converts the desired instantaneous Cartesian tool velocity into joint velocities using the Jacobian. My implementation plans a sampled trajectory from FK feedback, adds damping near singular configurations, integrates the joint velocities and checks the resulting configurations before execution.”

```text
x_dot = J(q) q_dot
v = v_reference + 2 (p_reference − p_actual)
omega = 2 rotation_error(R_target, R_actual)    # when orientation is held
q_dot = J^T solve(J J^T + lambda² I, [v; omega])
q_next = q + q_dot dt
```

In `RobotController.segment()`:

| Code element | Meaning |
| --- | --- |
| `robot.fkine(q)` | Predicted tool pose at the current planned sample |
| `u*u*(3 - 2*u)` | Cubic smoothstep reference with zero endpoint slope |
| `feedforward` | Reference position difference divided by timestep |
| `2.0 * (desired - actual)` | Proportional position tracking correction |
| `jacob0(q)[:3, :]` | 3×6 translational Jacobian for position-only motion |
| `jacob0(q)` | 6×6 linear/angular Jacobian when holding orientation |
| `Rotation.from_matrix(target_R @ current_R.T).as_rotvec()` | Spatial rotation-error vector |
| `lam = .015` / `.03` | Fixed damping for position-only / orientation-controlled segments |
| `np.linalg.solve(...)` | Solve the damped system without forming an explicit matrix inverse |
| `dq *= .5 / peak` when peak > .5 | Scale the entire velocity vector so maximum magnitude is .5 rad/s |
| `q_next = q + dq * .035` by default | Euler integration of planned joint velocity |
| Fractions .25, .5, .75, 1 | Extra configuration checks inside each integration interval |
| `reached()` | Position error < 6 mm; if orientation controlled, error < 3° |

`segment()` plans without changing the displayed robot. Execution happens later through `_execute()` or `move()`. FK feedback is feedback from the simulated mathematical model, not measured joint encoders or camera pixels.

**Singularity answer:** “At a singularity the Jacobian loses rank, so some instantaneous tool motions cannot be achieved. Near it, an undamped inverse can generate very large joint rates. Damping limits that amplification at the cost of accuracy. It does not remove the singularity.”

**Why pseudoinverse rather than inverse?** Position-only J is rectangular, and even a square J may be singular. The damped least-squares form handles both cases. The implemented damping is fixed; do not claim adaptive manipulability-based damping.

**Why hold rotation?** The carrying task needs a level tray. `hold_rotation` adds angular control. The fixed attachment alone makes the tray follow the tool; it does not keep the tray level if the tool tilts.

**Why not joint interpolation throughout?** A straight path in joint coordinates usually makes a curved tool path and changing orientation. Your code uses checked joint interpolation for empty-arm approaches and some returns; it uses RMRC for controlled Cartesian carrying. Both are intentional choices.

**Timing limitation:** Carry segments are sized using `CARRY_SPEED = .10 m/s`; planning uses controller dt .035 s. `_step()` divides the displayed timestep by `animation_speed`, configured as 3.0. Therefore the .5 rad/s cap is in planned controller time, not a certified displayed or physical speed limit. Cubic reference blending and velocity scaling do not establish acceleration/torque limits for the full robot.

## 7. Grasping, carrying and handout transforms

At grasp:

```text
T_tool_tray = inverse(T_world_tool_at_grasp) T_world_tray_at_grasp
```

At every carried sample:

```text
T_world_tray = T_world_tool(q) T_tool_tray
T_world_food = T_world_tray T_tray_food
```

In code: `_relative` is tool-to-tray; `_carried` identifies the attached tray; `payload` holds `(shape, tray_relative_pose)` pairs. `update_payload()` updates them for automatic, manual and recovery movement.

**Why not assign the tray to the tool pose directly?** The grasp is at the rim and the mesh origin is offset. Direct assignment would snap the tray origin onto the tool and lose the actual grasp geometry. The relative transform preserves position and orientation.

**Is it a physical grasp?** No: it is a rigid kinematic attachment after a checked approach. There is no simulated friction, force closure or food dynamics. Rotation control and geometric guards enforce the intended carrying behaviour.

`deliver()` executes approach → descend → attach → lift/carry/lower → release → retreat. Releasing clears `_carried`; the tray keeps its achieved pose. Placement is checked against **15 mm and 3°** release tolerances, distinct from the controller's 6 mm tool-target tolerance.

For handout, `_handout()` defines high alignment, low alignment and inside waypoints, and requests a **−90° world-z change in tray orientation**. The route includes a separately planned turning phase. `serving_gap_validator()` transforms the loaded tray's corners and checks the counter/opening. Robot link boxes remain behind the **Y = 4.64 m** safety plane, while the tray can extend through the aperture. `collect_served_meal()` animates the recipient pulling the released tray through.

In the integrated entry point, `_handout()` temporarily parks served meshes below the scene. `MealTrayTracker` remembers their last visible poses, `put_back()` restores them, and `run_orders()` puts the tray plus its riders in the `meals` queue. `prisoners_job()` waits on that queue, positions the meal in the prisoner's hands and walks away with it. This explains why both handout code and prisoner code touch a served tray.

## 8. Conveyor and integration

> “After Maryam releases the tray, conveyor movement is a scripted geometric animation. I calculate the number of samples from distance, belt speed and timestep. The tray moves along station waypoints, and each food item or towel follows through a tray-relative transform.”

The active conveyor is `integrated_meal.carry_tray`, not just the older `ConveyorBelt` class in the tray module. `BELT_SPEED = .45 m/s`, default dt .05 s. It preserves tray rotation and moves the mesh origin; `IntegratedMeal.serve()` converts geometric-centre waypoints to mesh-origin targets using the rotated, scaled centre offset.

Sequence: Maryam deposit → right end/other lane → Yara stop at X=13.2 → left end/near lane → ReBot stop at X=12.5 → Maryam pickup position → handout. The scene code supplies the lane and end coordinates. Yara's used food is identified by comparing inventory names before and after `run_task()`. The towel joins `riders` after placement.

`carry_tray()` prechecks sampled future tray poses against the supplied clearance predicate. In the integrated call this checks the tray against Maryam at her current q. Do not claim it is a physical conveyor simulation or that this callback performs every possible scene collision check.

**Order completion:** `run_orders()` finishes a meal before scanning again. It calls `mark_on_conveyor()` after successful pickup/deposit and `mark_delivered()` after successful serving. `TrayStacks` tracks actual remaining stack meshes; dispatcher status tracks logical colour-order state. `dispatcher.reset(colour)` makes that logical colour selectable again; it does not create a new physical tray.

## 9. Collision detection and avoidance: know the boundaries

> “I predict link and payload volumes at proposed joint configurations before assigning those joints. I use conservative boxes derived from the mesh bounds, plus task constraints such as joint limits, waiting trays and the serving opening. Detection rejects an intersecting configuration; a separate planner searches for another route.”

### Arm geometry

`stl_bounds()` reads binary-STL vertices and gets their minima and maxima. `RobotController.__init__()` pads each link box, converts it to the link's local frame at home, and caches it. `frames(q)` multiplies proposed DH transforms. `body_poses(q)` places each box at that proposed q without moving the live robot.

For a box obstacle, `boxes_overlap()` tests oriented-box intersection. It first performs a cheap world-axis rejection, then applies the Separating Axis Theorem using three axes from each box and nine cross-product axes. If any usable axis separates their projected intervals, they do not overlap. Parallel-axis cross products near zero are skipped.

For a sphere, transform its centre into the link box frame, clamp it to the box extents, and compare nearest-point distance with radius plus clearance. The serving-plane check projects box half-extents onto world Y; checking only the tool point would miss other links crossing the bars.

`valid()` checks shape, finite values, joint bounds, external collision and `extra_valid`. Bounds are the intersection of model limits with a 5° mechanical margin and `safe_qlim`. Limits alone do not prove collision clearance.

### Payload geometry

`configuration_clear()` composes arm, tray, waiting-tray and configured payload predicates. `tray_clears_robot()` includes the fixed housing as well as moving arm volumes, with task-specific grasp clearance. `tray_clears_obstacles()` uses a tray/meal envelope: loaded height .07 m versus .036 m when empty. `recovery_clear()` adds an 8° level constraint for a carried tray and any active serving-aperture validator.

### The cube demonstration

`TrayCollisionScenario.prepare()` creates a separate demonstration task and preserves the paused task. `set_cube()` updates visible and collision geometry together and invalidates an old plan. `plan()` densely samples the original phases, identifies the first rejected sample, and attempts a detour to the same phase goal. Carried phases try constrained Cartesian RMRC first, then a joint-space alternative if necessary. All proposed routes still use the scenario validity predicate.

`plan_joint_route()` tries direct joint interpolation, small local diversions and doglegs, then bounded bidirectional RRT-Connect. One tree starts at the beginning and one at the goal; checked extensions attempt to join them. Edge samples limit maximum per-joint change to .01 rad. Shortcutting is permitted only through checked edges. Search has time/iteration limits; it is neither optimal nor a proof of completeness.

`execute()` rejects changed cube revisions, a manually changed start pose or a newly blocked route, then checks each next sample during animation. The cube remains in place. `finish()` removes demonstration props and restores the paused task through checked movement. “Original route clear” is a valid result if the cube is outside the sweep. “Blocked” is the correct result if no verified detour exists, especially if the goal itself is occupied.

### Critical distinction in this exact version

`workcell_actions.install_collision_checks()` currently contains its documentation/example but **does not register any predicates**. `WorkcellRuntime.configuration_clear()` therefore returns True if the named robot has no registered checks. The registry wiring in `main_all.py` is infrastructure, not evidence of an implemented universal collision checker.

Maryam's local controller/delivery checks are implemented independently. `WorkcellRecovery.clear()` also checks selected scene solids and nearby robot volumes and is assigned to **manual bridge validators**; the collision scenario uses it too. Do not infer that every automatic ReBot/Yara/Hannah sample receives those same recovery checks merely because they receive `configuration_guard` hooks. ReBot's automatic guard currently reaches the shared registry, whose installation hook is empty.

Limits to explain: sampled conservative geometry, possible false rejection, no exact continuous triangle-mesh detection, no established complete arm self-collision coverage, selected scene obstacles rather than every decorative mesh, and no physical safety certification. Smaller sample steps reduce missed-motion risk but do not turn sampled checks into a continuous guarantee.

## 10. GUI: two concrete traces you should rehearse

### Automatic Start

```text
button.clicked -> MainWindow.start_sequence()
  -> runtime.submit(runtime.start)
  -> runtime.checkpoint() takes callback from deque
  -> runtime.start() checks permitted state/hazards, restores task poses if needed
  -> state RUNNING -> automatic task proceeds
```

Qt owns the widgets and event loop. The worker builds and runs the scene. `submit()` queues scene operations under a `threading.Condition`; `checkpoint()` services them even while automation is paused. This keeps long task work out of GUI button handlers and avoids uncoordinated scene writes from Qt callbacks. The GUI periodically reads runtime status and activity via `poll_runtime()`.

### Manual joint slider

```text
open Manual -> ManualMode.activate() requests pause
slider -> jog() records latest pending value
flush_jogs() -> queue(...) -> runtime.submit(...)
RobotBridge.set_joint() converts degrees to radians
RobotBridge.move_to() builds sampled joint path
_valid() checks limits, robot validator and configured guards
execute samples -> refresh model meshes and explicit payload attachments -> draw
```

The bridge holds references to the existing robots; it does not launch a new copy. `begin_manual()` saves the pre-manual joint/base pose. `start()` restores these saved poses before continuing preplanned automatic motion, because continuing from an arbitrary jogged q would invalidate the old trajectory.

Manual XYZ buttons use `RobotBridge.translate()`: finite-difference pose Jacobian, damped numerical pose solve, then `move_to()` for checked joint interpolation. This is **not the same motion generator as Maryam's automatic RMRC `segment()`**. Manual motion samples at about 1° maximum per joint; invalid paths are rejected.

Purple styling is in `theme.py`, with `apply_theme(app)` called by `main()`. Widgets/pages live in `main_gui.py`; `SystemManager` exposes Qt state/event signals and delegates to the runtime. Runtime is authoritative for the integrated simulation; standalone prototype state paths are not the main application.

### Thread coordination follow-up

`LockStep` wraps `env.step`. Background jobs such as Hannah and prisoners signal that they reached a step and wait for the next turn. The scene owner draws the shared frame and gives jobs turns using `go`/`back` events. An `RLock` serialises scene turns; missed deadlines request a stop. An idle ticker lets background jobs advance while kitchen code waits for camera input. This is cooperative scheduling, not a real-time operating system or collision avoidance by itself.

## 11. Camera: distinguish sensing from viewing

### Webcam colour detection

> “I use OpenCV to classify a colour card in the centre third of the webcam image. I convert that region from BGR to HSV, threshold green/blue/yellow masks, compare pixel coverage and require stable consecutive classifications before dispatching a task.”

`ColourScanner.classify()` picks the largest mask only if its ROI coverage is at least **30%**. Green H=35–85, blue 90–135, yellow 18–34; saturation/value thresholds reject weak/dark pixels. OpenCV hue spans 0–179. HSV separates hue from brightness more conveniently than raw RGB thresholds, but lighting, reflections and camera settings can still affect results.

The active integrated UI uses `CameraScanner.tick()` on a **40 ms Qt timer**, with **12 identical classifications** in its vote deque. This is roughly .48 s nominal observation time, not a measured timing guarantee. `show_identified()` adds a one-second acknowledgement display before returning the result. BGR frames are converted to RGB for `QImage`, copied and scaled for preview. Repeated frame-read failures produce an error.

Worker `runtime.scan()` sets `scan_requested` and waits on `scan_finished`; the Qt dialog opens/reads the camera and calls `runtime.finish_scan(result)`. This bridges task code and GUI without opening an OpenCV window on the worker thread. In `main_all`, `run_orders` normally receives the **runtime** as scanner, not a standalone `ColourScanner.scan()` loop.

`MealDispatcher.handle()` maps green/blue to an order and yellow to an alert with no tray. A yellow-held gate requires six non-yellow observations before allowing another yellow trigger. This is colour-based task selection. It does not identify people, measure tray pose, estimate camera extrinsics, close a visual-servo loop or automatically turn the yellow card into a physical emergency stop.

### Swift view camera

`select_camera_robot()` and `update_camera()` calculate a viewpoint from the selected robot and queue `env.set_camera_pose(position, look_at)`. It changes the rendered view, not joint q. It is separate from the physical laptop webcam. The general project description mentions a robot-mounted RGB camera, but the active implementation here is a laptop-webcam card scanner; describe the implementation you can show.

## 12. ReBot simulation: the active implementation

> “The integrated ReBot is a custom mesh robot, not the separate `Rebot_DH.py` class. Each joint has a fixed origin transform, a rotation axis and limits. I multiply these transforms to get the links and gripper, solve numerical IK for the measured finger-contact point, then animate joint trajectories while keeping the towel attached relative to the tool.”

`Rebot_class.ReBot` stores six `(xyz, rpy, axis, lower, upper)` entries. `origin_transform()` explicitly multiplies translation, Rx, Ry and Rz **in that code order**. `axis_rotation()` uses angle-axis rotation. `get_link_transforms(q)` accumulates `T = T * T_origin * T_rotation`. These are local-to-base transforms; world tool pose is `robot.base * gripper_transform`. Do not describe this as standard-DH FK merely because a separate ReBot DH file exists.

`get_gripper_transforms()` adds the fixed `T_gripper` and opposite local-Y finger translations. Mesh updates use motion relative to the assembled home transforms. The integrated task sets ReBot's world base to `(12.5, 3.6, .66)` with +90° yaw and widens the simulated maximum finger displacement to .10 m. That is a simulation configuration, not a claim about physical jaw travel.

The live task is `place_towel()`:

1. Define targets above the towel, at the towel, lifted above it, above the tray and at the tray.
2. `solve_contact()` uses bounded `least_squares` from several seeds. Its residual combines contact position with a weighted downward tool-z direction.
3. `contact_pose()` includes `FINGER_CONTACT = SE3(.09, -.0081, -.026)` because the actual finger contact is offset from the wrist/tool origin. This prevents a visually plausible wrist pose from missing the towel.
4. Move above, open, descend and verify contact position within 2 mm.
5. Project towel dimensions onto the actual jaw axis to determine whether the towel fits and the required opening.
6. Save `attachment = inverse(tool_pose) * towel_pose`; close, lift, transfer and lower with the towel following the tool.
7. Reject placement if towel-origin error exceeds 15 mm; open, clear `recovery_payloads`, retreat and return to `q_home`.

`animate_joints()` uses 80 samples and `3u²−2u³` joint blending, with dt .03 s. It calls any configured guard before assigning q and updates the attached towel and scene. Endpoint IK plus smooth joint interpolation does **not** guarantee a straight Cartesian tool path. It does not continuously use `RobotController.segment()`.

`solve_contact()` constrains position and downward direction, leaving yaw freedom; it does not constrain all three independent orientation components. The older `solve_joints()` builds a finite-difference Jacobian, and `run_rebot_towels()` is a standalone two-towel demonstration. These are not the functions called by `IntegratedMeal.serve()`; its call is **`place_towel()`**. The older `solve_joints` parameter `R_goal` is not used in its active residual, so do not claim it enforces that full rotation.

**Hardware question:** These modules animate Swift. Actual communication, calibration, taught hardware motion and PLC stop evidence must be explained from the separately demonstrated hardware work. This simulation does not prove hardware execution.

## 13. Stop, Reset, Home, Resume and Rebuild

> “Stop latches a shared simulation stop state. Reset acknowledges it after hazards are cleared and leaves everything stationary. Optional Home is a separate checked recovery move. Start/Resume restores any interrupted task poses before continuing. Rebuild closes the old simulation and creates fresh task state.”

| Action | Active path | Result |
| --- | --- | --- |
| Pause | `request_pause` | Park automation for manual intervention |
| STOP | `stop_all` → `SystemManager.e_stop` → `request_stop` | Set ESTOP and prevent continued normal stepping at checkpoints |
| Reset | `SystemManager.reset` → queued `runtime.reset` | Check hazards/latch generation; READY or PAUSED, no movement |
| Optional Home | `runtime.home('All')` → `WorkcellRecovery.home_robot` | Preflight all requested arm routes, then checked recovery |
| Start / Resume | queued `runtime.start` | Restore interrupted/manual poses, then RUNNING |
| Rebuild simulation | `runtime.restart` → unwind → `prepare_reload` | Close old jobs/scene, reset session, build again, wait at READY |

Home uses the joint poses saved at scene startup, not necessarily an all-zero pose. Recovery keeps mobile bases parked. Explicit attachments keep carried items with the appropriate tool, and held-item tilt and obstacle guards can reject a loaded Home. Reset must not automatically start motion.

The integrated runtime takes precedence over historical comments in `E_stop_test.py`. `wire_estop()` redirects the shared ESTOP object to it. The current GUI PLC page is explicitly disabled; `workcell_actions.on_stop()` reports physical integration pending, and `on_reset()` returns True for simulation acknowledgement. Do not claim those GUI buttons operate a verified PLC safety circuit.

## 14. Examiner drills: answer before reading the prompt's hint

| Question | A strong short answer must mention |
| --- | --- |
| Show me where you created your robot. | `MaryamBot.__init__`, six RevoluteDH links, mesh superclass, base separately |
| Explain joint 2 without reading a script. | a=.355 m, alpha=0, d=0, theta=q2+90°, radians, bounds |
| Does zero q mean theta is zero? | No, offsets still apply |
| What changes if I move the base? | World pose/reach relative to room changes; intrinsic link dimensions do not |
| Why can the mesh move incorrectly even when FK is consistent? | CAD frame/pivot/scale mismatch; internal mathematical consistency is not physical calibration |
| Point to the FK call used for IK. | Inside `_plan_pick_and_carry.residual` |
| Where is the inverse in your IK? | No closed-form inverse; bounded least-squares search using repeated FK |
| Why multiple seeds? | Local minima/branches; first solution may have a bad wrist or blocked carry |
| Is a successful IK solver enough? | No, residual, bounds, collisions and entire route must pass |
| What does a Jacobian column represent? | Instantaneous tool linear/angular velocity from unit rate of that joint at this q |
| How large is J? | 6×6 full twist, 3×6 position-only for Maryam |
| What frame is the velocity in? | Spatial/base-reference convention used by `jacob0`; match FK errors and reference coordinates, including mounted base transform |
| What happens near a singularity? | Rank loss/poor conditioning; damping reduces large rates but sacrifices tracking |
| Does damping avoid collisions? | No, geometric checks and route search are separate |
| Why scale all joint rates together? | Keep their direction/ratios while enforcing the peak cap |
| Where does the robot actually move? | q assignment in execution followed by payload/mesh update and scene step |
| How does the tray stay level? | Orientation feedback, fixed attachment and task validators |
| Why is tool position different from tray centre? | Rim grasp plus mesh-centre offset and rotation |
| What is the difference between attach and release? | Set/clear `_carried`, retain the achieved world pose on release |
| What makes the handout different from deposit? | Loaded envelope, planned yaw turn, aperture waypoints and robot barrier |
| Is your conveyor controlled by motors? | Scripted pose animation based on speed and dt in this simulation |
| What happens to food when the conveyor turns? | Rider pose = tray pose × saved relative pose |
| What stops a second meal from interfering? | Serial meal loop, stack/inventory state and geometric waiting-tray checks |
| What do collision boxes approximate? | Transformed padded link STL bounds and payload envelope |
| How do rotated boxes intersect? | SAT over face normals and cross-product axes |
| Can you guarantee no self-collision? | No complete self-collision coverage established |
| Why check during execution after planning? | Scene may change; cached plans may become invalid |
| What if the cube blocks the destination? | Reject execution; no invented route through occupied goal |
| Is RRT optimal? | No; bounded feasible-route search with checked shortcuts |
| Why is the GUI responsive? | Qt event loop/timers separated from task worker; scene commands queued |
| How does a slider reach the correct robot? | Robot name → shared bridge reference → checked q path |
| Why save and restore the manual pose? | Automatic trajectories expect their original starting state |
| What is the difference between the two cameras? | Webcam colour input versus Swift rendered viewpoint |
| How do you avoid reacting to one noisy frame? | Coverage threshold and 12 consecutive identical classifications |
| Is this visual servoing? | No image-error feedback into joint control; vision selects the task |
| Does yellow dispatch a yellow tray? | No, active meal dispatcher raises an alert |
| Is ReBot's FK standard DH here? | No, fixed origin/axis transform chain in active mesh model |
| What is ReBot's actual IK target? | Measured finger-contact point, downward direction, free yaw |
| Does ReBot use Maryam's RMRC? | No; numerical endpoint contact IK plus joint interpolation in active task |
| Does Reset move anything? | No; separate Home and Start |
| Are all automatic team collisions covered by the registry? | No; installation hook currently registers none |
| What would you improve? | Register team predicates, expand/validate collision coverage, independently calibrate model, verify hardware/sensor integration |

## 15. A practical rehearsal plan

**First pass, 20 minutes:** Memorise the system trace and opening answer. Open the main entry point, integrated meal, robot class, controller, tray module, GUI bridge, scanner and ReBot towel task. Find each function from the map.

**Second pass, 30 minutes:** On paper draw six joint frames, write one standard-DH matrix product, then explain FK → IK residual → Jacobian → damped q-dot → integration. Trace one blue tray from scan to prisoner, naming the object/transform being updated at every stage.

**Third pass, 20 minutes:** Explain a slider, a webcam result, a cube detour and an interrupted loaded recovery. For each, say who requests it, who executes it, which checks run and what happens on failure.

**Final pass:** Have someone choose ten drill questions in random order. Give a 20–40 second answer, then show the function. Any answer you cannot explain without reading becomes your revision priority.

Demonstration sequence: READY → Start → colour card → Maryam deposit → Yara food → ReBot towel → Maryam handout; pause and show one checked manual command; show cube Detect/Plan/Execute with cube retained; stop, clear hazard, Reset and demonstrate no movement; optional checked Home; separate Start. Use the existing `MARYAM_DEMO_GUIDE.md` for the detailed demonstration procedure.

## 16. Evidence and what the checks actually establish

`Maryam_Work/verify_model.py` compares an explicit standard-DH matrix product with Toolbox FK, compares finite-difference and Toolbox Jacobians, solves numerical IK for FK-generated targets, and checks a local RMRC segment. Run from the assignment root:

```powershell
& '../.venv/Scripts/python.exe' -B -m Maryam_Work.verify_model
```

It writes `documents/MARYAM_MODEL_CHECKS.md`, JSON results and a DH-frame image. These are **internal consistency checks**. They do not independently verify CAD alignment, manufacturer agreement, the entire integrated cycle, physical collision safety or hardware execution. Explain this distinction if asked how you tested the model.

Before the viva, have your actual frame sketch, CAD/mesh source and editing history, selection/approval evidence and any real-robot demonstration evidence ready. If you are unsure of a detail, open the function and trace its inputs, calculation, checks, state change and return value rather than guessing.
