# Maryam viva: quick revision sheet

Read the [full preparation guide](MARYAM_VIVA_PREPARATION.md) for explanations, exact function locations and examiner questions.

## Say this first

“I built MaryamBot's six-joint DH/mesh model, tray pickup and handout controller, attachment transforms and collision-aware route checks. I built and connected the purple GUI, webcam colour scanner, camera views and simulation recovery workflow. I worked on ReBot's simulated towel task and integrated the robots through the shared entry point.”

## Remember the flow

`main_all.main` → worker `_run_workcell_once` → `build_scene` → `IntegratedMeal` → READY → Start → `run_orders` → camera result → dispatcher → `IntegratedMeal.serve` → Maryam pickup → conveyor → Yara food → conveyor → ReBot towel → conveyor → Maryam handout → prisoner queue → next scan.

## Ten files to know

| File | Your answer |
| --- | --- |
| `Collective_Folder/EnviroParts_S/main_all.py` | Entry point, shared objects, GUI wiring, order loop, LockStep jobs |

| `Maryam_Work/Maryam_robotclass.py` | Six standard-DH revolute joints, mesh names, offsets, joint limits |

| `Maryam_Work/maryamrobot_controller.py` | `segment`: damped RMRC; `valid`/`colliding`: arm constraints |

| `Maryam_Work/maryam_tray_delivery.py` | `prepare`, `_plan_pick_and_carry`, `deliver`, `_execute`, `update_payload` |

| `Maryam_Work/maryam_movements.py` | Scene/controller setup, colour callbacks, `_handout`, aperture validator |


| `Collective_Folder/integrated_meal.py` | Active `serve`, conveyor routes, Yara/ReBot integration |

| `Collective_Folder/rebot_towels.py` | Active `place_towel`, `solve_contact`, `animate_joints` |

| `Collective_Folder/Rebot_class.py` | Active ReBot origin/axis FK, meshes and fingers |


| `Collective_Folder/workcell_runtime.py` | Queue, checkpoints, state, scan handshake, stop/reset/start |


| `Collective_Folder/maryam_collision_demo.py` | Prepare reference, detect collision, plan detour, execute, restore |

GUI directory: `Collective_Folder/MealBot_GUI/MealBot_GUI/MealBot_GUI/`.

`main_gui.py` = widgets/buttons/view camera; `manual_mode.py` = sliders and requests; `robot_bridge.py` = checked live-robot commands; `camera_scanner.py` = Qt webcam dialog; `theme.py` = purple stylesheet. All-arm Home lives in `Collective_Folder/maryam_recovery.py`.

## Four equations

```text
FK:        T_world_tool = T_world_base × A1 × ... × A6 × T_tool
DH link:   Ai = Rz(qi + offseti) × Tz(di) × Tx(ai) × Rx(alphai)
RMRC:      qdot = Jᵀ solve(JJᵀ + λ²I, desired_twist); qnext = q + qdot dt
Attachment:T_world_tray = T_world_tool × T_tool_tray
```

Food/towel riding the tray: `T_world_item = T_world_tray × T_tray_item`.

## Your DH numbers

| Joint | d (m) | a (m) | alpha | Offset |
| --- | --- | --- | --- | --- |
| 1 | .360 | 0 | +90° | −90° |
| 2 | 0 | .355 | 0° | +90° |
| 3 | 0 | 0 | +90° | +90° |
| 4 | .375 | 0 | +90° | +180° |
| 5 | 0 | 0 | −90° | 0° |
| 6 | .078 | 0 | 0° | +90° |

## Questions you must answer smoothly

- **Why six joints?** Position and orientation control for pickup, level carry and turning, subject to workspace and singularities.
- **FK?** Angles to tool pose by multiplying transforms.
- **IK?** Desired pose to angles using bounded numerical least squares on FK error; try seeds and validate the entire transfer.
- **RMRC?** Desired Cartesian velocity to joint velocity through a Jacobian; add simulated pose-error feedback, damp, integrate and check.
- **Damping?** Reduces huge joint rates near singularities, with a tracking-accuracy tradeoff.
- **Level tray?** Hold tool rotation plus fixed grasp attachment; validate tray attitude and aperture clearance.
- **Collision?** Proposed link boxes from STL bounds, payload/waiting-tray checks, SAT for rotated boxes, sphere distance and serving-plane checks.
- **Avoidance?** Reject blocked reference samples, then try checked Cartesian detours or bounded joint-space RRT-Connect.
- **GUI connection?** Button/slider → queued runtime callback → checkpoint → bridge/controller → same live robot → payload/mesh update → scene step.
- **Webcam?** Centre ROI → HSV masks → at least 30% coverage → 12 stable classifications → dispatch green/blue; yellow alerts without a tray.
- **ReBot?** Custom origin/axis FK → finger-contact numerical IK → smooth joint interpolation → tool-relative towel attachment.
- **Recovery?** STOP → clear hazard → Reset without movement → optional checked Home → separate Start/Resume, restoring interrupted poses first.

## Numbers worth recognising

Maryam RMRC dt .035 s; damping .015 position-only / .03 full pose; peak planned joint rate .5 rad/s; tool convergence 6 mm / 3°; tray release 15 mm / 3°; conveyor .45 m/s; display animation speed factor 3; handout yaw −90°; robot serving plane Y=4.64 m. Camera timer 40 ms. ReBot uses 80-sample cubic joint moves with dt .03 s.

## Claims to keep accurate

- The active meal coordinator overrides `MaryamMovements.serve`; read `IntegratedMeal.serve`.
- Maryam automatic carry uses RMRC. ReBot's active towel task uses endpoint IK and joint interpolation.
- Webcam task selection is separate from Swift camera viewing. Neither is image-based visual servoing.
- Conveyor and grasping are kinematic animations, not motor/contact physics.
- Conservative sampled collision checks do not guarantee continuous whole-scene or complete self-collision safety.
- The shared `install_collision_checks` hook currently registers no team predicates. Local Maryam checks, scenario checks and manual/recovery validators still exist.
- The GUI PLC controls are disabled and physical stop integration is pending in the adapter. Simulation behaviour is separate evidence from hardware work.
- Reset, Home, Resume and Rebuild are different actions.

## Verification you can explain

The existing `verify_model.py` passed four FK/Jacobian/IK cases and a 141-sample RMRC check with **0.206 mm** final position error. These test mathematical consistency, not independent CAD or physical calibration. See [the numerical report](MARYAM_MODEL_CHECKS.md) and [the calculated DH frames](MARYAM_DH_FRAMES.png).

Practise each answer in this order: **purpose → function → calculation → checks → update/return → limitation**.
