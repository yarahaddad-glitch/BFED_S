
import numpy as np
import swift
import time
import traceback

from spatialmath import SE3
from spatialgeometry import Sphere
from roboticstoolbox import jtraj

from Class_HannahBot import HannahBot


# ============================================================
# 1. START SWIFT
# ============================================================

env = swift.Swift()
env.launch(realtime=True)

time.sleep(2)

try:

    # ========================================================
    # 2. CREATE HANNAHBOT
    # ========================================================

    robot = HannahBot()

    q_start = np.zeros(6)

    robot.q = q_start.copy()
    robot.base = SE3(0, 0, 0)

    robot.add_to_env(env)

    env.step(0.05)

    print("\nHannahBot loaded successfully.")

    # ========================================================
    # 3. CREATE END-EFFECTOR MARKER
    # ========================================================

    # Red ball follows the actual end effector

    ee_marker = Sphere(
        radius=0.025,
        pose=SE3(),
        color=[255, 0, 0, 255]
    )

    env.add(ee_marker)

    # ========================================================
    # 4. INITIAL END-EFFECTOR POSITION
    # ========================================================

    T_start = robot.fkine(robot.q)

    ee_marker.T = SE3(*T_start.t).A

    print("\nInitial end-effector position:")
    print(T_start.t)

    # ========================================================
    # 5. DEFINE KNOWN REACHABLE TARGET
    # ========================================================

    q_known = np.deg2rad([
        10,
        20,
        -15,
        30,
        10,
        -20
    ])

    T_target = robot.fkine(q_known)

    print("\nTarget end-effector position:")
    print(T_target.t)

    # ========================================================
    # 6. CREATE GREEN TARGET MARKER
    # ========================================================

    target_marker = Sphere(
        radius=0.025,
        pose=SE3(*T_target.t),
        color=[0, 255, 0, 255]
    )

    env.add(target_marker)

    env.step(0.05)

    # ========================================================
    # 7. INVERSE KINEMATICS
    # ========================================================

    print("\nCalculating inverse kinematics...")

    q_guess = q_known + np.deg2rad([
        2, -2, 2, -2, 2, -2
    ])

    solution = robot.ikine_LM(
        T_target,
        q0=q_guess,
        joint_limits=True,
        ilimit=1000,
        slimit=100
    )

    if not solution.success:
        raise RuntimeError(
            f"IK failed: {solution.reason}"
        )

    q_target = solution.q

    print("\nIK successful!")

    print("\nTarget joint angles (degrees):")
    print(np.rad2deg(q_target))

    # ========================================================
    # 8. FUNCTION TO ANIMATE A TRAJECTORY
    # ========================================================

    def move_robot(q_from, q_to, steps=100):

        trajectory = jtraj(
            q_from,
            q_to,
            steps
        )

        for q in trajectory.q:

            robot.q = q

            # Update the meshes
            robot.update_meshes()

            # Update the red end-effector marker
            T_current = robot.fkine(robot.q)

            ee_marker.T = SE3(*T_current.t).A

            env.step(0.03)

    # ========================================================
    # 9. MOVE FORWARD TO TARGET
    # ========================================================

    print("\nMoving HannahBot to target...")

    move_robot(
        q_start,
        q_target,
        steps=100
    )

    # ========================================================
    # 10. CHECK TARGET ACCURACY
    # ========================================================

    T_final = robot.fkine(robot.q)

    position_error = np.linalg.norm(
        T_final.t - T_target.t
    )

    rotation_error = np.linalg.norm(
        T_final.R - T_target.R
    )

    print("\n" + "=" * 55)
    print("TARGET POSITION RESULTS")
    print("=" * 55)

    print("\nTarget position:")
    print(T_target.t)

    print("\nActual end-effector position:")
    print(T_final.t)

    print(f"\nPosition error: {position_error:.9f} m")
    print(f"Rotation matrix error: {rotation_error:.9f}")

    if position_error < 0.005 and rotation_error < 0.01:
        print("\nSUCCESS: HannahBot reached the target!")
    else:
        print("\nWARNING: Target accuracy outside tolerance.")

    # ========================================================
    # 11. PAUSE AT TARGET
    # ========================================================

    print("\nPausing at target for 2 seconds...")

    time.sleep(2)

    # ========================================================
    # 12. RETURN TO STARTING POSITION
    # ========================================================

    print("\nReturning HannahBot to starting position...")

    move_robot(
        q_target,
        q_start,
        steps=100
    )

    # ========================================================
    # 13. CHECK RETURN ACCURACY
    # ========================================================

    T_return = robot.fkine(robot.q)

    return_error = np.linalg.norm(
        T_return.t - T_start.t
    )

    print("\n" + "=" * 55)
    print("RETURN POSITION RESULTS")
    print("=" * 55)

    print("\nOriginal position:")
    print(T_start.t)

    print("\nReturned position:")
    print(T_return.t)

    print(f"\nReturn position error: {return_error:.9f} m")

    if return_error < 0.005:
        print("\nSUCCESS: HannahBot returned to start!")
    else:
        print("\nWARNING: HannahBot did not return accurately.")

    print("=" * 55)

except Exception:

    print("\nAn error occurred:")
    traceback.print_exc()

finally:

    input("\nPress Enter to close the simulation...")

