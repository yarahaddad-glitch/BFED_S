SMART MEALBOT — ISOLATED MANUAL-JOGGING PROTOTYPE

Location required:
Assignment 2/Collective_Folder/MealBot_GUI/

Existing sibling folders (unchanged):
Assignment 2/Maryam_Work/Maryam_robotclass.py + RS007N mesh files
Assignment 2/Hannah_Work/Class_HannahBot.py + Hannah Meshes/
Assignment 2/Yara_Work/YaraBot_DH.py + EC66_new_Mesh/

Setup with your existing UTS virtual environment:
python -m pip install PySide6
python Collective_Folder/MealBot_GUI/main_gui.py

Click Manual Mode > select robot > Load selected robot in Swift.
Use sliders or +/- buttons for individual joints.
Press the GUI emergency stop to disable jogging. Demo unlock password: demo123.

IMPORTANT LIMITATIONS:
- No file outside MealBot_GUI is edited.
- GUI is simulated; PLC is not connected and nothing controls physical robots.
- Manual position updates are direct joint setpoints, not collision-checked trajectories.
- Home snaps to configuration instantly in simulation only.
- YaraBot currently provides NO joint limits; provisional +/-180 degree limits are NOT certified.
- HannahBot requires update_meshes() after changing robot.q.
- PLC/safety modes are placeholders to be connected after receiving teammate code.
- Do not use this interface to control a real physical robot.
