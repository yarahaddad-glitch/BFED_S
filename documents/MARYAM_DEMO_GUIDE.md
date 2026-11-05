# Maryam: in-person demo and collision explanation

## Viva preparation: why these techniques?

Opening explanation:

> My six-joint Kawasaki RS007N model picks the selected tray, deposits it on the
> conveyor, then collects the completed meal and presents it through the serving
> gap. I use numerical IK to choose a feasible grasp, RMRC for controlled Cartesian
> carrying, collision checks for the arm and tray, and a separate planner when the
> reference route is obstructed. The GUI supports automatic operation, manual
> control and deliberate recovery from a simulated emergency stop.

| Tutor question | Answer tied to your code |
| --- | --- |
| What defines your model? | `MaryamBot.__init__` defines six standard-DH revolute links using `a`, `d`, `alpha`, joint offsets and limits. STL meshes visualise the chain; meshes alone do not define its kinematics. Six joints allow position and orientation control in suitable nonsingular configurations, not access to every pose. |
| What do the DH parameters mean? | `a`: length along x; `alpha`: twist about x; `d`: displacement along z; `theta`: rotation about z. For these revolute joints, theta is the joint variable plus its configured offset. Explain using your chosen standard-DH frames. |
| What is FK? | Multiply the link transforms, including base/tool transforms, to obtain the tool pose from joint angles. Your code calls `robot.fkine(q)`. |
| Where is IK? | `TrayDelivery._plan_pick_and_carry` uses bounded `least_squares` on FK position error, adding orientation error for selected candidates. Multiple seeds and rim grips help find a usable solution. This is numerical IK even though the function is not named `ikine`. |
| Why check more than the grasp? | An IK solution can reach the tray but have a colliding approach, poor wrist configuration or blocked carry. You verify the transfer before committing to the grasp. |
| Why RMRC? | Approach, lift, carry and lower are Cartesian tasks. RMRC converts desired tool velocity into joint velocities using the Jacobian. A reachable endpoint alone does not define suitable motion between endpoints. |
| How does yours work? | `RobotController.segment` computes a smooth Cartesian reference, adds proportional position-error correction and optional orientation correction, applies damped least squares, then integrates joint velocities. Planning does not move the displayed arm. |
| What does the Jacobian do? | It maps joint velocity to instantaneous tool linear/angular velocity. Your `jacob0(q)` is expressed in the base/world frame used by the controller. Position-only control uses three rows; orientation-controlled motion uses six. |
| Why damping? | Near singularities, an ordinary inverse can demand very large joint velocities. Damping reduces that amplification at the cost of tracking accuracy. It does not remove singularities or guarantee success. |
| How do you keep the tray level? | Carry segments supply `hold_rotation`. Orientation feedback plus a fixed tool-to-tray attachment maintains the tray attitude. The serving rotation is separately planned rather than uncontrolled wrist movement. |
| Why not use joint interpolation everywhere? | Interpolating joints generally gives a curved tool path and can tilt the tray. Your code still uses joint interpolation for checked empty-arm reconfiguration where appropriate. |
| What limits motion? | Mechanical/software joint bounds, configuration checks and a planned joint-velocity cap of 0.5 rad/s. The entire velocity vector is scaled when needed. These are kinematic simulation controls, not torque or acceleration certification. |
| Does RMRC avoid collisions? | RMRC generates local motion. Collision predicates reject blocked configurations; Cartesian detours and a separate joint-space planner search for another route. |
| What is RRT-Connect used for? | After simpler diversions fail, two joint-space trees grow from start and goal and attempt to connect through checked edges. Search is bounded: failure is not proof that no possible route exists. |
| Why does the tray follow the gripper? | At grasp, save the tool-to-tray transform. Each update multiplies world-to-tool by tool-to-tray. Food follows through tray-relative transforms. This is a rigid simulated attachment, not physics-based grasping. |
| Pause versus e-stop? | Pause suspends automatic work for manual control. E-stop latches a stop requiring deliberate Reset. Reset does not move anything; Start is separate. Home is optional checked motion after Reset. |
| Is colour detection visual servoing? | No. It selects the task. Camera feature error does not continuously correct your arm motion. |

Recognise the RMRC relationship:

`q_dot = J.T @ inverse(J @ J.T + lambda^2 * I) @ desired_tool_velocity`

The implementation uses a linear solve instead of explicitly forming the
inverse, then `q_next = q + q_dot * dt`. Desired velocity includes reference
feedforward and simulated FK error feedback. This is not measured hardware or
camera feedback. Joint constraints and collision predicates are checked separately.

### In-person evidence sequence

1. Run the integrated task. Identify your pickup, deposit and handout roles and
   how the conveyor links them to Yara and ReBot.
2. Open `Maryam_robotclass.py`: explain one DH link, its offset and limits.
3. Show `_plan_pick_and_carry` in `maryam_tray_delivery.py` for numerical IK and
   transfer validation. Show `segment` in `maryamrobot_controller.py` for the
   reference, error, Jacobian, damping and integration.
4. Show a level carry and the intentional serving rotation.
5. Show the cube scenario: predicted collision, verified detour, execution with
   the cube still present. A blocked destination should remain blocked.
6. Stop during motion, show the latched state, clear the hazard, Reset and wait
   visibly without movement. Demonstrate optional checked Home and separate Start.
   Explain restoration of the interrupted task poses before continuation.
7. Present hardware-stop evidence separately; explain your own real-robot
   contribution accurately. Simulation alone does not establish that contribution.

Do not claim exact mesh collision checks, complete self-collision coverage,
optimal paths, guaranteed avoidance, physical safety certification or visual
servoing. Defend the actual choices and their limits.

## Run and watch

Run `Collective_Folder/EnviroParts_S/main_all.py` using the subject `.venv`, or choose
**Run and Debug -> MealBot: integrated workcell (.venv)** in VS Code.
Wait for **STATE = READY**. Use **Camera -> Maryam** to watch automatic motion.
Opening **Manual** deliberately pauses the automatic jobs.

For the normal meal task, use **Automatic -> Start / Resume** and show a blue or
green card. Maryam picks up the selected tray, the conveyor takes it to Yara and
ReBot, and Maryam collects the completed meal and hands it through the serving gap.

## E-stop, Home and Restart: the exact order

**STOP -> clear the hazard -> Reset -> optional Home -> separate Restart / Resume.**

Reset acknowledges the stop and leaves all robots stationary. Home is movement,
so this application requires the stop to be reset before a checked recovery move.
Home is optional; it is not part of acknowledging every stop.

The Emergency / Recovery page contains:

1. **Clear safety test actor**, when that actor caused the intrusion.
2. **1. Reset stop (no movement)**.
3. **2. Home all four arms (optional)**.
4. **3. Restart / Resume task**.

Home all covers **MaryamBot, YaraBot, HannahBot and ReBot**. It returns their arms
to the joint configurations saved at scene startup. Mobile bases stay at their
current positions; this does not command a TurtleBot to drive back through the room.
The controller prechecks every requested route before starting any Home motion.
It refreshes each model's own link/gripper meshes and moves explicitly attached
trays, food or towels with the tool. It does not guess which nearby object to grab.

Home records the interrupted task poses. Restart / Resume first checks and
restores those poses, then allows the original task to continue. During recovery,
the badge shows **RECOVERING**. A new stop interrupts it without clearing the latch.
A route that hits a modelled obstacle, violates limits, or tilts a held item too
far is rejected. Do not force Home while carrying a tray if there is no clear,
level route to that joint configuration.

**Rebuild simulation**, on the Automatic page, is different: it discards the old
simulation session and builds fresh task state. It is not the e-stop Reset.

## Your dedicated tray-and-cube collision scenario

1. Start from READY or pause between tasks. If Maryam is holding a real meal,
   finish that meal first; the separate scenario will not take it from her.
2. Open **Collision tests -> Open Maryam tray + cube scenario**.
   The application pauses, selects Maryam in Manual and opens the controls.
3. Choose **Deposit blue tray on conveyor** or **Hand out a demonstration meal**.
4. Press **1. Prepare task + random cube**. The arm stays still while it plans a
   collision-free reference task. A separate tray borrows a real stack location;
   no real tray or food inventory is consumed.
5. The orange cube is deliberately placed in a predicted sweep, clear of the
   current pose and task endpoints where possible. Deposit placement favours
   carrying motion. The default handout placement favours the approach to the
   loaded tray, where there is room for an alternate route.
6. Edit **Cube X, Y, Z** in WORLD metres and its side length if wanted. Press
   **2. Apply edited cube position**. Applying changes invalidates the old plan.
   **New random position** chooses another position in the reference task sweep.
7. Press **3. Detect collision + plan around cube**. Read the DETECTED message:
   it identifies a task phase, sample and, for an arm/cube hit, the affected link.
   The actual arm remains stationary during this calculation.
8. A successful result says **VERIFIED DETOUR** and shows green tool-path markers.
   Press **4. Execute checked tray task**. The cube remains in place. Only Maryam
   executes the scenario; the other automatic jobs remain parked.
9. Press **5. Finish / restore paused automatic task**. This removes the scenario
   props, checks the empty-arm return, restores the real tray and original pose,
   and leaves automation paused. Use Automatic Start / Resume separately.

The handout scenario stages a demonstration meal on the receiving conveyor; it
is not an extra automatic food/towel order. It uses the same arm, measured tray
geometry, grasp planner and serving-gap constraints as the real task.

If a cube blocks the destination or seals the serving aperture, a detour may not
exist. The result is **BLOCKED** and Execute stays unavailable. Move the cube and
check again. This is the correct response; the program must not invent a route
through an occupied goal. A cube outside the sweep should report that the original
route is clear rather than falsely claiming an avoidance manoeuvre.

Jogging Maryam after planning invalidates the planned starting configuration;
Execute rejects it and asks for another check. Unsaved coordinate edits also
prevent execution. STOP works during planning and motion. After an interrupted
attempt, clear/reset any stop and use Finish / Restore before another scenario.

## What to say to your tutor

> I first calculate a reference tray task. The obstacle test checks the robot's
> link volumes and the carried-tray volume along that future motion. Here it
> predicts a collision with the cube before the robot touches it. I then search
> for another route to the same task destination and validate it before moving.
> The cube stays in the scene. If the target or the available passage is blocked,
> I keep the arm stationary rather than executing an unsafe route.

> My collision geometry uses conservative oriented bounding boxes, so it is an
> approximation of the meshes. I check joint limits, intermediate configurations,
> tray clearance, queued trays and the configured scene geometry. During execution
> I check each next configuration again. Detection and avoidance are different:
> detection identifies the blocked motion; the planner finds a feasible replacement.

> The tray is attached with a fixed tool-to-tray transform. Its food follows with
> tray-to-food transforms. Recovery uses explicit attachments and the robot's own
> mesh update method so the arm, gripper and payload do not separate visually.

## Functions and files: how, why and where

| Purpose | File and functions | How to explain it |
| --- | --- | --- |
| Parameter-based robot model | `Maryam_Work/Maryam_robotclass.py`: `MaryamBot.__init__` | Six standard-DH joints, offsets, dimensions and limits; graphical STL links in `Meshes_RS007N` follow the kinematic chain. |
| Robot link collision volumes | `Maryam_Work/maryamrobot_controller.py`: `stl_bounds`, `frames`, `body_poses` | Fit bounds to each link STL and transform them using the proposed joints, without moving the displayed robot. |
| Detect arm/obstacle contact | Same file: `colliding`, `valid` | Check limits, serving safety plane and obstacle volumes. A cube uses oriented-box overlap; a sphere uses nearest-point distance. The controller records which link was blocked. |
| Oriented-box intersection | `Maryam_Work/collision_geometry.py`: `boxes_overlap` | Separating Axis Theorem: test the three axes of each box and nine cross-product axes. A separating axis means the boxes do not overlap. |
| Tray and meal collision checks | `Maryam_Work/maryam_tray_delivery.py`: `configuration_clear`, `tray_clears_robot`, `queued_trays_clear`, `tray_clears_obstacles`, `recovery_clear` | Check the attached tray/meal envelope against the arm, fixed housing, waiting trays and cube. Include serving-aperture and level-tray constraints during detours. |
| Physical attachment in simulation | Same file: `update_payload` | `T_world_tray = T_world_tool * T_tool_tray`; food uses another relative transform. Use this for motion, not independent visual shifts of the tray. |
| Stage the realistic task | `Collective_Folder/maryam_collision_demo.py`: `TrayCollisionScenario.prepare` | Build a demonstration tray, prepare the actual pickup/deposit or handout legs, and retain the original task and inventory for restoration. |
| Edit the real obstacle | Same class: `place_random_cube`, `set_cube` | Place a visible cube and update its collision box together. Changing coordinates invalidates the old route. |
| Detect and replan | Same class: `plan`, `_cartesian_detour`, `_payload_valid` | Check the dense reference route. For a level carried tray, try constrained Cartesian RMRC; otherwise use a checked joint-space detour. Preserve the original phase destination. |
| Joint-space alternative route | `Maryam_Work/joint_planner.py`: `plan_joint_route`, `samples` | Try local diversions and doglegs, then bounded bidirectional RRT-Connect. Check edges at at most 0.01 rad change per joint; shortcut only across clear edges. |
| Controlled execution and cleanup | `TrayCollisionScenario.execute`, `finish` | Recheck the route before moving and each next sample during motion. Restore the paused task after removing only demonstration props. |
| All-four-arm Home | `Collective_Folder/maryam_recovery.py`: `WorkcellRecovery.register_all`, `home_robot`, `_recover`, `clear`, `restore` | Save Home joints, preflight all requested arms, execute while automatic jobs remain parked, and restore interrupted task joints before resuming. |
| Model-specific recovery geometry | Same class: `body_boxes`, `payloads`, `update_payloads` | Use Maryam's link boxes, ReBot's own transforms, Hannah's measured mesh transforms and Yara's link volumes. Keep mobile bases fixed. |
| Single stop state | `Collective_Folder/workcell_runtime.py`: `request_stop`, `reset`, `home`, `start`, `checkpoint` | Stop latches immediately; reset acknowledges only; queued scene commands remain serviceable; start is separate. |
| GUI controls | `Collective_Folder/MealBot_GUI/MealBot_GUI/MealBot_GUI/main_gui.py`: `CollisionScenarioDialog`, `open_collision_scenario` | The GUI queues work to the scene thread instead of editing geometry from the Qt thread. |

The small `recovery_payloads` hooks in `rebot_towels.py`, shared
`yara_main_testing.py` and Hannah's `Feed_Iso.py` identify the item and its actual
tool-relative attachment at grasp/release. Their task algorithms are retained.

## Limits to explain honestly

- These are sampled checks using conservative boxes/link volumes, not exact
  triangle-mesh continuous collision detection. Narrow geometry may be rejected
  conservatively. Full nonadjacent-link self-collision coverage is not established.
- The scenario and recovery checks include selected solid counters/walls and
  nearby robot volumes. They are not a claim that every decorative mesh and every
  possible robot interaction in the whole workcell has been modelled.
- Mobile bases stay parked during arm Home. A blocked loaded Home is rejected.
- The camera performs colour-based task selection; it is not visual servoing.
- Demonstrate your DH frames, FK/IK, RMRC and your real-robot contribution in your
  own words. Keep the team's existing physical PLC-stop footage as separate evidence.

For tomorrow, keep only the main entry point, robot class, controller, tray
module, collision scenario module and this guide open. Show the working system
first, then use the function table to answer code questions.
