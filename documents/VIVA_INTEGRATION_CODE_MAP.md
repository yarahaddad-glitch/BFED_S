# Focused viva search map

Search the full label with Ctrl+F in the named file. At most five key labels per file.

These are study pointers for the contributions identified in this session, not proof of sole authorship.
Describe shared calls as integration. In Rebot_class.py, the retained label points to the finger-direction correction recorded in MaryamQu commit e5ac98c.

Other function explanations remain as ordinary comments. Executable code is unchanged.

## main_gui.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_GUI` | 328 | Build pages, bridge, state manager, navigation, STOP button, shortcuts and refresh timers. |
| `SHOW_VIEW_CAMERA` | 721 | Select which robot the rendered Swift camera follows; does not command joints. |
| `SHOW_STOP` | 822 | Request the shared simulation stop through SystemManager. |
| `SHOW_START` | 843 | Queue runtime.start() when READY or PAUSED; runtime performs restoration and hazard checks. |
| `SHOW_STATUS_SYNC` | 858 | Refresh status/activity/safety widgets and attach to each new simulation session. |

## manual_mode.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_MANUAL_GUI` | 10 | Build robot selector, degree sliders, XYZ jogs and timers; connect Qt signals to handlers. |
| `SHOW_SLIDER` | 131 | Record the latest requested joint angle; defer execution to the batching timer. |
| `SHOW_COMMAND_QUEUE` | 147 | Submit scene work to the runtime and record completion/rejection for the GUI. |
| `SHOW_RESTORE` | 173 | Queue a checked return to the saved pre-manual joint configuration. |
| `SHOW_CARTESIAN` | 179 | Queue a small world-axis movement; the bridge performs numerical pose solving. |

## robot_bridge.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_CONNECTION` | 24 | Store live environment/models, validators and drawing callback; expose this bridge to runtime. |
| `SHOW_COLLISION` | 77 | Check numerical values, limits, configured robot validator and registered team predicates. |
| `SHOW_JOINT` | 118 | Convert one slider angle from degrees to radians and delegate to checked joint motion. |
| `SHOW_IK` | 127 | Solve a small XYZ target while retaining orientation using a finite-difference Jacobian and damping. |
| `SHOW_RESTORE` | 160 | Check the base has not changed and return to the saved pre-manual pose. |

## camera_scanner.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_SCAN_LOOP` | 75 | While RUNNING and requested: read webcam, classify HSV, update preview and accept stable votes. |
| `SHOW_RESULT` | 91 | Publish the accepted result and wake the waiting meal-task worker. |
| `SHOW_PREVIEW` | 112 | Convert OpenCV BGR to RGB and copy the frame into the Qt preview. |
| `SHOW_STABILITY` | 125 | Require 12 consecutive identical classifications by default. |

## theme.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_THEME` | 45 | Apply PURPLE_THEME to the QApplication created by main_all.main(). |

## main_all.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_ORDERS` | 373 | Wait for a camera result, dispatch it, finish one integrated meal, then mark delivery and scan again. |
| `SHOW_MEAL_INTEGRATION` | 466 | Connect the meal coordinator and Maryam actions to this shared scene. |
| `SHOW_GUI_CONNECTION` | 503 | Link GUI commands to the same live robot models and configured validators. |
| `SHOW_GUI_STARTUP` | 594 | Connect the GUI and webcam dialog to the shared runtime before starting the worker. |

## integrated_meal.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_CONVEYOR` | 19 | Slide the tray along one straight belt leg; this is pose animation, not RMRC. |
| `SHOW_PICKUP` | 101 | Inherited Maryam pickup uses TrayDelivery's grasp IK and RMRC carrying. |
| `SHOW_YARA` | 129 | Integration: call Yara's food-placement task when the tray reaches her station. |
| `SHOW_REBOT` | 141 | Simulated towel grasp/place uses contact-point IK and joint animation. |
| `SHOW_HANDOUT` | 150 | Inherited Maryam action turns/presents the meal through the serving gap. |

## workcell_runtime.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_QUEUE` | 106 | Queue a callback under the condition lock for execution at a simulation checkpoint. |
| `SHOW_STOP` | 112 | Latch ESTOP, increment stop generation and wake waiting threads; do not move Home. |
| `SHOW_START` | 136 | Check state/hazards, restore recovery/manual poses, then allow normal automation. |
| `SHOW_RESET` | 161 | Check stop inputs/hazards and acknowledge the latch without starting motion. |
| `SHOW_CHECKPOINT` | 235 | Service queued commands; wait on the condition while paused/stopped; unwind close/rebuild requests. |

## maryam_recovery.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_ATTACHMENTS` | 154 | Return explicitly held objects and their tool-relative transforms; do not guess from proximity. |
| `SHOW_COLLISION` | 170 | Check limits, model guards, held-item tilt and selected scene/nearby-arm volumes. |
| `SHOW_HOME` | 218 | Select one or all saved startup arm configurations and request checked recovery. |
| `SHOW_RECOVERY` | 226 | Plan all requested routes before moving; recheck each execution sample and stop on changed hazards. |
| `SHOW_RESUME_POSE` | 279 | Restore interrupted task joints at unchanged bases before runtime allows automatic continuation. |

## maryam_collision_demo.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_PREPARE` | 169 | Preserve the paused real task and build a separate demonstration tray/reference transfer. |
| `SHOW_OBSTACLE` | 323 | Update visible cube/collision geometry together; invalidate the previous route revision. |
| `SHOW_DETECTION` | 399 | Densely check reference phases; for a blocked phase try Cartesian then joint-space detours. |
| `SHOW_EXECUTION` | 454 | Reject stale/changed plans; precheck then execute samples with the cube retained and other jobs parked. |
| `SHOW_RESTORE` | 503 | Remove only demonstration props and check the return to the saved real-task configuration. |

## Rebot_class.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_GRIPPER` | 273 | Finger-direction correction: right opens along local +Y, left along -Y. |

## rebot_towels.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_CONTACT` | 94 | Finger-contact offset added for contact-point targeting. |
| `SHOW_GRIPPER` | 119 | Stepped finger-opening/closing animation. |
| `SHOW_IK` | 104 | Use bounded least squares and multiple seeds for contact position/downward direction; yaw remains free. |
| `SHOW_SEQUENCE` | 133 | Plan approach/grasp/lift/drop targets; check contact/fit, attach towel, transfer, release and retreat. |
| `SHOW_ATTACHMENT` | 162 | Save tool-to-towel pose at grasp so the towel follows without snapping. |

## maryamrobot_controller.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_COLLISION` | 109 | Check link boxes against the serving plane and configured obstacles. |
| `SHOW_RMRC` | 191 | Plan tool motion using FK feedback, a damped Jacobian and joint integration. |
| `SHOW_FK` | 219 | Calculate the current planned tool pose from joint angles using the DH model. |
| `SHOW_DAMPING` | 255 | Calculate joint speeds while reducing excessive rates near singularities. |
| `SHOW_ORIENTATION` | 259 | Use all six rows to control position and maintain tool attitude. |

## maryam_tray_delivery.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_FK` | 480 | Forward kinematics: use joint angles to calculate the tool pose. |
| `SHOW_IK` | 488 | Inverse kinematics: find joint angles that reach the grasp target. |
| `SHOW_RMRC` | 549 | Call the RMRC controller to plan this carrying leg's joint samples. |

## maryam_movements.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_MAIN_CONNECTION` | 2 | This file IS used by the final main_all.py simulation: |
| `SHOW_APERTURE` | 220 | Create a checker for the loaded tray's clearance above the bench/through bars. |
| `SHOW_SETUP` | 270 | IntegratedMeal calls this through super().__init__(): wire the shared components. |
| `SHOW_PICKUP` | 294 | Blue stack to conveyor: TrayDelivery plans IK/RMRC and executes the transfer. |
| `SHOW_HANDOUT` | 310 | Collect a loaded tray, turn/align it and present it at the serving opening. |

## maryam_meal_dispatch.py

| Search | Line | Explain |
| --- | --- | --- |
| `SHOW_HSV` | 25 | Threshold hue, saturation and brightness to build a mask for each colour. |
| `SHOW_CLASSIFY` | 78 | Classify one image; return colour, coverage and detection-box coordinates. |
| `SHOW_CAMERA` | 98 | Open/reuse the laptop webcam; report an error if it is unavailable. |
| `SHOW_ALERT` | 274 | Record/print the warning and call any registered listeners; no tray is sent. |
| `SHOW_DISPATCH` | 339 | Decide CANCELLED, ALERT, REJECTED or SEND_TRAY from a completed scan. |
