# ReBot contribution audit for the viva

Based on the available local Git history (`--all`), source diffs and current working files. A commit records who submitted a change; it does not establish who first conceived or typed every line, or prove a physical test succeeded. Use this with your own recollection and team records.

## Hannah

| Evidence | Recorded work |
| --- | --- |
| `1580fb7`, 9 October 2026 | Added ReBot mesh assets, `Rebot_DH.py` and the initial Swift file. |
| `2efc92e`, 12 October 2026 | Added the original `Rebot_class.py`, class/movement tests and mesh changes. |
| `a7c5c38`, 20 October 2026 | Corrected joint 1's lower limit in the separate DH model from +2.8 to -2.8. |
| `ee4451d`, 21 October 2026 | Substantially revised `rebot_bottle_demo.py`. Despite its filename, this version already contains a two-towel task and helpers later reused in `rebot_towels.py`. |

The earlier helpers include `tool_pose`, `animate_joints`, `solve_joints` and `run_rebot_towels`. Credit that existing model/helper foundation; do not claim these were all newly invented in the later towel file.

## Yara

| Evidence | Recorded work |
| --- | --- |
| `b732969`, 15 October 2026 | Added `self.base = SE3()` to `Rebot_class.py` and applied that world-base transform when positioning the base, moving-link and gripper meshes. |
| Same commit, `main_testing.py` | Instantiated ReBot in the shared environment, positioned it at `(12.5, 3.6, .66)` with +90-degree yaw, updated meshes and staged a bottle. |
| `c74ecea`, 22 October 2026 | Added shared ESTOP integration in `main_all.py`, including the historical `wire_estop` adapter, registration of loaded robots (including ReBot), task attachment tracking and startup/job coordination changes. |

Yara's world-base change is substantive: the assembled ReBot can be translated/rotated in the workcell while its local link kinematics stay the same. Its original source class is still Hannah's.

Yara's later commit `e3dd565` touches the compiled ReBot `.pyc`, not a new ReBot class source algorithm. Compiled cache changes are not evidence of designing a new solver. Her stop work affects ReBot as part of the shared system; it is not authorship of the towel IK.

No Yara-authored source commit to `rebot_towels.py` or `integrated_meal.py` was found in the available history. This does not rule out collaboration outside Git.

## Maryam

| Evidence | Recorded work |
| --- | --- |
| `0c78e38`, 14 October 2026 | Added an earlier bottle-demo script. |
| `2ec9138`, 15 October 2026 | Modified that demo and added `rebot_live_view.py` and `rebot_teach.py`, plus integration changes. |
| `de2dcb2`, 16 October 2026 | Added `rebot_remote_client.py`. |
| `e5ac98c`, 21 October 2026 | Added `rebot_towels.py` and `integrated_meal.py`; corrected the ReBot finger-opening signs in `Rebot_class.py`. |
| `0ae314e`, 22 October 2026 | Added a pre-assignment `configuration_guard` hook to towel joint animation, as part of runtime/GUI integration. |

The remote client, teaching and live-view scripts are additional recorded contributions, but they are not called by the current final meal simulation. Their existence does not establish successful physical execution; describe tests you actually performed and can evidence.

### Specific final-simulation work to present

1. `Rebot_class.get_gripper_transforms`: corrected right finger displacement to local +Y and left to local -Y, matching the meshes.
2. `rebot_towels.contact_pose`: added a measured finger-contact offset to the tool frame.
3. `rebot_towels.solve_contact`: added bounded least-squares IK for contact position and downward direction, with multiple initial guesses. Yaw is not fully fixed.
4. `rebot_towels.animate_gripper`: added stepped finger-opening animation.
5. `rebot_towels.place_towel`: added the single-meal pickup/place sequence, target planning, contact check, projected towel-width/gripper-fit calculation, rigid attachment, achieved-placement check, release and retreat.
6. `integrated_meal`: placed the towel inventory and coordinated conveyor arrival, one towel per meal, loaded tray riders and return to Maryam for handout. This reused the existing ReBot model and task helpers; it was not the first-ever placement of ReBot in a shared scene.
7. The later guard hook connects automatic joint proposals to shared runtime predicates; it does not by itself implement complete collision geometry. The current team registry installation hook registers no predicates.

The world-tool helper and cubic interpolation remain relevant theory you should understand, even when they build on existing team code. Explain what you adapted and what you reused.

## Current local changes

There are uncommitted changes in the class, towel module, integrated coordinator and final main file. ReBot class differences inspected were explanatory comments; towel differences also include explicit `recovery_payloads` registration/clearing. These local additions support held-item recovery, but are not evidenced by the historical commit IDs above. Recent viva comments/search labels are study aids, not historical contribution evidence.

The current stop/recovery implementation differs from Yara's historical adapter. Describe the current behaviour accurately, and credit the shared history instead of claiming sole authorship of all emergency-stop code.

## Accurate viva statement

> Hannah supplied the original ReBot model and existing simulation helpers. Yara added world-base positioning and shared-scene/stop integration. I developed the contact-point IK and single-meal towel pickup/placement workflow, corrected the finger-opening directions, and connected towel delivery into the conveyor meal sequence. I reused and adapted the team's model and helpers rather than creating every ReBot component from scratch.

## Evidence commands

```powershell
git show b732969 -- Collective_Folder/Rebot_class.py Collective_Folder/main_testing.py
git show ee4451d -- Collective_Folder/rebot_bottle_demo.py
git show e5ac98c -- Collective_Folder/Rebot_class.py Collective_Folder/rebot_towels.py Collective_Folder/integrated_meal.py
git show 0ae314e -- Collective_Folder/rebot_towels.py
git show c74ecea -- Collective_Folder/EnviroParts_S/main_all.py
git diff -- Collective_Folder/Rebot_class.py Collective_Folder/rebot_towels.py
```
