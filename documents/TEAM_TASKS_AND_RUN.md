# Team tasks and running the shared workcell

Use this as the single integration guide. Everyone edits their own modules and
connects them to the existing scene; do not launch a second Swift environment.

## Run and demonstrate

From the repository root, with the subject Python environment activated:

```powershell
python Collective_Folder/EnviroParts_S/main_all.py
```

On Maryam's current machine the environment can also be selected explicitly:

```powershell
& '../.venv/Scripts/python.exe' Collective_Folder/EnviroParts_S/main_all.py
```

Wait for **Swift scene: LOADED / READY**, then press **Start / Resume**. This
opens the colour scanner and starts the scheduled prisoner/Hannah jobs. Show a
blue/green card; the dashboard reports each robot's task. Opening **Manual**
pauses automation. Select an arm, jog joints or world X/Y/Z, and restore its saved
pose. Camera views use the same scene. Resetting a saved robot pose is different
from acknowledging an emergency stop.

## Maryam - GUI, MaryamBot and shared integration

Files:

- `Collective_Folder/MealBot_GUI/MealBot_GUI/MealBot_GUI/`: GUI pages, live robot
  bridge, camera scanner and camera perspectives.
- `Collective_Folder/EnviroParts_S/main_all.py:run_workcell`: constructs the scene,
  registers robots/checks, then waits for Start. `run_orders` coordinates meals.
- `Maryam_Work/maryam_movements.py:_handout`, `return_home`: handout and return to
  the startup joint pose after collection. This automatic return is independent
  of the GUI Home hook, which still needs the team's recovery implementation.
- `Maryam_Work/maryam_tray_delivery.py`: plans the end-effector-driven 90-degree
  tray turn, follows the attached meal, and checks the arm, tray and waiting trays.
- `Collective_Folder/maryam_collision_demo.py`: inserted-object demonstration.

Remaining tasks: visually verify both tray colours/heights in Swift, add broader
environment/other-robot and nonadjacent-link self-collision coverage, and explain
the FK/IK, coordinate frames, RMRC, joint trajectories and collision approximations.
Before allowing manual jogging while carrying anything, connect payload following
and proposed payload clearance to the manual bridge, or inhibit that operation.
Keep model approval, dimension/mesh sources and validation evidence in
`documents/Robots/Robot3_Maryam`. GUI wiring alone cannot guarantee an individual
mark; demonstrate the model and be able to explain your code.

## Hannah - safety, prisoners and HannahBot

1. Finish/verify the detectors in `Collective_Folder/Light_curtain.py` and
   `Collective_Folder/Prison_break.py`. `Collective_Folder/safety_tests.py` already
   connects their intrusion signals to the shared stop latch and warning popup.
2. Put prisoner animation in `main_all.py:prisoner1_job`; add further jobs at the
   `HANNAH` comment after the first `runtime.checkpoint()` in `run_workcell`.
   Call `env.step(DT)` per frame so Stop and other robots can be serviced.
3. Put arm/payload collision geometry in your own module under `Hannah_Work/`.
   Register its predicate as shown below. `Hannah_Work/Feed_Iso.py:arm_to` now
   calls the shared guard before assigning each proposed joint pose.
4. Add predicted mobile-base/whole-arm clearance to `drive` and `face` in
   `Feed_Iso.py` BEFORE changing base poses. The arm-only callback does not cover
   a moving base. Check carried trays and walls/people/other robots too.
5. Demonstrate **Safety tests -> choose intrusion -> Remove prisoner -> Reset ->
   Start / Resume**. Closing the warning or removing the prisoner must not resume.

## Yara - emergency stop, recovery and YaraBot

Put shared button actions in `Collective_Folder/workcell_actions.py`:

| Function | Code to add |
| --- | --- |
| `on_stop(context)` | Send the actual robot/actuator stop command; never move Home on Stop. Current hook only reports simulation stopping. |
| `on_reset(context)` | Return False while any physical stop, sensor fault, stale connection or unsafe state remains. Return True only to acknowledge; do not start motion. |
| `on_home(context, robot_name)` | Choose each robot's known home pose and execute a checked recovery trajectory. `context.bridge.move_to(robot_name, q_home)` provides sampled joint checks; recovery hooks can use `context.motion_step(dt)` while paused. |
| `on_restart(context)` | Restore objects, inventories, attachments, prisoner jobs and robot states together. Also implement task-runner restart in `main_all.py`; returning True alone cannot restart a completed Python call stack. Leave the system paused for a separate Start. |

Add a physical input adapter, for example `Collective_Folder/plc_interface.py`.
An asynchronous asserted input calls `context.request_stop('Physical e-stop')`.
Release must not call Reset or Start. Validate real stopping and input timing
under the laboratory's approved controls; the Python latch does not stop physical
hardware, and the educational PLC is not a certified safety controller.

Your arm code is `Collective_Folder/yara_main_testing.py`. Put/extend predicted
collision checks there or in your own module; register the shared predicate below.
`configuration_safe` uses it during planning and `execute_path` calls the shared
guard before each joint assignment. Add equivalent proposed-base checks to your
mobile-base moves, including held food geometry.

## ReBot owner - towel arm

Put arm/payload collision prediction in your own module and register `ReBot` below.
`Collective_Folder/rebot_towels.py:animate_joints` calls the shared guard before
each joint assignment. Include the held towel, counter and nearby arms. Keep using
the shared `place_towel` operation in `Collective_Folder/rebot_towels.py`, called
by `Collective_Folder/integrated_meal.py`.

## Everyone - connect collision checks here

Implement a **prediction**, taking proposed joint angles in radians and returning
True if clear, False or a short reason string if blocked. Use FK at `proposed_q`;
checking only `robot.q` misses the collision about to happen. Use world-frame
geometry consistently. Do not assign joints, mutate the scene, call `env.step`,
open a window or start another simulation inside a predicate.
Do not call `controller.valid`, `yara.configuration_safe` or a bridge validator
from a registered predicate: those wrappers already call the registry and would
recurse. Call your underlying geometry routines instead.

Example interface in your own module (replace the body with actual geometry):

```python
def hannah_clear(context, proposed_q, payload_pose):
    robot = context.robots['HannahBot']
    # Calculate link/payload poses at proposed_q, then check against
    # context.objects, other robots, keep-out volumes and self-collision pairs.
    # payload_pose is an optional proposed tray SE3 for Maryam; otherwise None.
    return True  # Replace this placeholder; True alone supplies no protection.
```

Import and register it inside
`Collective_Folder/workcell_actions.py:install_collision_checks(context)`:

```python
from Hannah_Work.my_collision_checks import hannah_clear
context.register_collision_check('HannahBot', hannah_clear)
```

Robot keys are **MaryamBot**, **HannahBot**, **YaraBot**, **ReBot**. Multiple
predicates may be registered per robot; all must pass. Registration happens after
all robot instances exist. `context.env`, `context.objects` and `context.robots`
are the shared instances. No teammate predicates are supplied by the registration
stub: each owner still has to implement and test their geometry.

Automatic arm executors check before assigning joints; a blocked shared guard
latches Stop and waits at the existing checkpoint. Clear the hazard, then Reset,
then Start. Manual controls precheck the sampled path and recheck each sample;
blocked manual commands are rejected with status feedback. Existing Maryam/Yara
local checks remain in use. Base motion and custom routines outside these
executors must call their own proposed-pose guard before updating geometry.

GUI callbacks must queue scene work with `runtime.submit(callback)`; they must not
write robot joints directly. Sensor threads may call `runtime.request_stop(reason)`.
Long calculations/I/O must not prevent prompt checkpoint/stop handling. Stop and
fault handling must also interrupt or time out any physical device commands.

## Collision demonstration - separate from person safety tests

1. **Collision tests -> Insert obstacle on next tray move**.
2. **Start / Resume**, then show a colour if requested. A red object is placed in
   a future arm-link volume that clears the present robot/tray.
3. Show the stop before contact. Use **Remove obstacle -> Reset -> Start / Resume**
   to continue the interrupted path. Releasing/removing the obstacle alone must
   not resume motion.

This demonstrates predictive stopping. Active avoidance needs an actual verified
alternative path and is still further work. Approximate boxes and sampled paths
do not constitute complete mesh collision or certified safety protection.

## Shared hardware and assessment tasks

- Record real robot operation tied to the meal task, physical mushroom e-stop,
  release without restart, deliberate Reset, then separate Start.
- Complete the signed group Risk Assessment/SWMS and tutor approvals, model
  eligibility/source checks, attributable commits and individual demonstrations.
- If claiming PLC bonus, implement an observable physical sequence, an additional
  provided sensor, meaningful interlock/sequence changes, handshake/timeouts,
  I/O/wiring/logic evidence and approved risk documentation. Replace the GUI PLC
  placeholder with actual state/fault feedback in `main_gui.py:make_plc_page`.
- Prepare the required exact 60-second promotion, final video, real-hardware
  evidence and each person's code-viva explanations.

## Run checks before merging your changes

```powershell
python -m unittest check_meal_collision_guards check_workcell_runtime check_workcell_gui_qt
python check_meal_handout.py mixed
python check_integrated_meal.py blue green
```

Add a test that your predicate rejects an actual proposed collision without moving
the robot, accepts a clear pose, and permits recovery only after removal, Reset
and Start. Run the live Swift scene too; headless tests do not prove rendered
clearance, physical response or hardware safety. Cold trajectory planning can
take time; verified routes are cached locally in ignored `.mealbot_routes/`.
