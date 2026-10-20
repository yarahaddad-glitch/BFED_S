SMART MEALBOT — COMPLETE COMPACT PURPLE GUI (V3)

INSTALL:
1. Back up your old Collective_Folder/MealBot_GUI if you need to keep modifications.
2. Delete or rename the old MealBot_GUI folder (NOT Collective_Folder or main_testing.py).
3. Extract this ZIP into Collective_Folder. It contains ONE MealBot_GUI folder.
4. Verify these files sit together:
   main_gui.py, manual_mode.py, theme.py, robot_bridge.py, system_manager.py

RUN (from Assignment 2 directory using your existing robotics Python environment):
  python Collective_Folder/MealBot_GUI/main_gui.py

If necessary: python -m pip install PySide6

ROBOT SOURCE CODE AND MESHES:
Maryam_Work/Maryam_robotclass.py
Hannah_Work/Class_HannahBot.py and Hannah Meshes/
Yara_Work/YaraBot_DH.py and EC66_new_Mesh/
These remain in their existing directories alongside Collective_Folder.

CURRENT LIMITATIONS:
- Standalone Swift simulation; does NOT connect to main_testing.py yet.
- Manual joint setpoints; floor/obstacle collisions are NOT checked.
- Emergency stop is a software simulation latch only.
- PLC is not connected. No hardware is controlled.
- Yara joint ranges are provisional ±180 degrees; confirm actual limits.
- Simulation-only recovery demo password: demo123.
