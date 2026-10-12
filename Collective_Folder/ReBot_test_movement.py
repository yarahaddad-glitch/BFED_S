
from pathlib import Path
import time
import numpy as np
import swift

from spatialgeometry import Mesh
from spatialmath import SE3


# ==========================================================
# 1. SETTINGS
# ==========================================================

mesh_folder = Path(__file__).parent / "ReBot_Meshes"

STEPS = 60
STEP_TIME = 0.03
TEST_ANGLE = np.deg2rad(20)

q_home = np.zeros(6)

USE_LOCAL_URDF_MESHES = False

# Gripper settings
# 0.00 = closed
# 0.05 = open (50 mm travel per finger)

q_gripper = 0.0

GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.05


# ==========================================================
# 2. START SWIFT
# ==========================================================

env = swift.Swift()
env.launch(realtime=True)


# ==========================================================
# 3. LOAD STL MESHES
# ==========================================================

mesh_names = [
    "IR_ReBot_base.STL",
    "IR_ReBot_link1.STL",
    "IR_ReBot_link2.STL",
    "IR_ReBot_link3.STL",
    "IR_ReBot_link4.STL",
    "IR_ReBot_link5.STL",
    "IR_ReBot_link6.STL",
    "IR_ReBot_gripper_base.STL",
    "IR_ReBot_right_finger_changed.STL",
    "IR_ReBot_left_finger.STL"
]

meshes = []

for name in mesh_names:

    path = mesh_folder / name

    if not path.exists():
        raise FileNotFoundError(
            f"Mesh not found: {path}"
        )

    mesh = Mesh(str(path), pose=SE3())

    meshes.append(mesh)
    env.add(mesh)


base_mesh = meshes[0]
link_meshes = meshes[1:7]

gripper_base = meshes[7]
right_finger = meshes[8]
left_finger = meshes[9]


# ==========================================================
# 4. REBOT DM URDF JOINT DEFINITIONS
# ==========================================================

# Format:
# (xyz, rpy, axis, lower_limit, upper_limit)

joint_data = [

    # Joint 1
    (
        [-0.00008416, 0, 0.08465],
        [0, 0, 0],
        [0, 0, 1],
        -2.8, 2.8
    ),

    # Joint 2
    (
        [0.020084, 0.031625, 0.05555],
        [-1.5708, 0, 0],
        [0, 0, -1],
        -3.14, 0
    ),

    # Joint 3
    (
        [-0.264, 0, 0],
        [0, 0, 0],
        [0, 0, 1],
        -3.14, 0
    ),

    # Joint 4
    (
        [0.2426, -0.054, -0.001625],
        [0, 0, 0],
        [0, 0, 1],
        -1.87, 1.57
    ),

    # Joint 5
    (
        [0.078308, -0.0375, -0.03],
        [-1.5708, 0, 0],
        [0, 0, 1],
        -1.57, 1.57
    ),

    # Joint 6
    (
        [0.028008, 0, 0.04],
        [0, 1.5708, 0],
        [0, 0, 1],
        -3.14, 3.14
    )
]


# ==========================================================
# 5. TRANSFORMATION FUNCTIONS
# ==========================================================

def origin_transform(xyz, rpy):

    x, y, z = xyz
    roll, pitch, yaw = rpy

    return (
        SE3(x, y, z)
        * SE3.Rx(roll)
        * SE3.Ry(pitch)
        * SE3.Rz(yaw)
    )


def axis_rotation(axis, angle):

    axis = np.array(axis, dtype=float)

    axis = axis / np.linalg.norm(axis)

    return SE3.AngleAxis(angle, axis)


# ==========================================================
# 6. CALCULATE LINK TRANSFORMS
# ==========================================================

def get_link_transforms(q):

    T = SE3()
    transforms = []

    for i in range(6):

        xyz, rpy, axis, lower, upper = joint_data[i]

        T_origin = origin_transform(xyz, rpy)
        T_rotation = axis_rotation(axis, q[i])

        T = T * T_origin * T_rotation

        transforms.append(T)

    return transforms


# ==========================================================
# 7. GRIPPER TRANSFORMS
# ==========================================================

# Fixed transformation between Link 6 and gripper

T_gripper = origin_transform(
    [0, 0, 0.15539],
    [0, -1.5708, 3.1415]
)


def get_gripper_transforms(link_transforms, opening=0.0):

    # Final robot link
    T6 = link_transforms[5]

    # Gripper base follows Link 6
    T_end = T6 * T_gripper

    # Right finger moves along negative local Y
    T_right = T_end * SE3(0, -opening, 0)

    # Left finger moves along positive local Y
    T_left = T_end * SE3(0, opening, 0)

    return [
        T_end,
        T_right,
        T_left
    ]


# ==========================================================
# 8. SAVE REFERENCE CONFIGURATION
# ==========================================================

q = q_home.copy()

# Record original assembled STL positions
original_mesh_poses = [
    SE3(mesh.T) for mesh in meshes
]

home_transforms = get_link_transforms(q_home)

home_gripper_transforms = get_gripper_transforms(
    home_transforms,
    opening=0.0
)


# ==========================================================
# 9. UPDATE ALL MESH POSITIONS
# ==========================================================

def update_robot_meshes():

    current_transforms = get_link_transforms(q)

    current_gripper_transforms = get_gripper_transforms(
        current_transforms,
        opening=q_gripper
    )

    # Fixed base
    base_mesh.T = original_mesh_poses[0].A

    # ----------------------------------------------
    # Six robot arm links
    # ----------------------------------------------

    for i in range(6):

        if USE_LOCAL_URDF_MESHES:

            new_pose = current_transforms[i]

        else:

            movement = (
                current_transforms[i]
                * home_transforms[i].inv()
            )

            new_pose = (
                movement
                * original_mesh_poses[i + 1]
            )

        link_meshes[i].T = new_pose.A

    # ----------------------------------------------
    # Gripper base, right finger, left finger
    # ----------------------------------------------

    gripper_parts = [
        gripper_base,
        right_finger,
        left_finger
    ]

    for i, mesh in enumerate(gripper_parts):

        if USE_LOCAL_URDF_MESHES:

            new_pose = current_gripper_transforms[i]

        else:

            # Relative transformation preserves
            # original mesh assembly.
            movement = (
                current_gripper_transforms[i]
                * home_gripper_transforms[i].inv()
            )

            new_pose = (
                movement
                * original_mesh_poses[i + 7]
            )

        mesh.T = new_pose.A


# ==========================================================
# 10. MOVE ONE ARM JOINT
# ==========================================================

def move_joint(joint_index, target_angle):

    global q

    q_start = q.copy()
    q_end = q.copy()

    q_end[joint_index] = target_angle

    for s in np.linspace(0, 1, STEPS):

        blend = 3*s**2 - 2*s**3

        q = (
            q_start
            + blend * (q_end - q_start)
        )

        update_robot_meshes()
        env.step(STEP_TIME)


# ==========================================================
# 11. TEST ALL SIX ARM JOINTS
# ==========================================================

def test_all_joints():

    print("\n================================")
    print(" REBOT URDF JOINT TEST")
    print("================================\n")

    update_robot_meshes()
    env.step(0.1)

    time.sleep(2)

    for i in range(6):

        joint_number = i + 1

        xyz, rpy, axis, lower, upper = joint_data[i]

        home_angle = q_home[i]

        positive_target = min(
            home_angle + TEST_ANGLE,
            upper
        )

        negative_target = max(
            home_angle - TEST_ANGLE,
            lower
        )

        print(f"\nTesting Joint {joint_number}")

        # Positive rotation
        print("Moving positive")
        move_joint(i, positive_target)

        time.sleep(0.5)

        # Negative rotation
        print("Moving negative")
        move_joint(i, negative_target)

        time.sleep(0.5)

        # Return home
        print("Returning home")
        move_joint(i, home_angle)

        time.sleep(1)

        print(f"Joint {joint_number} complete")

    print("\nAll six arm joints tested!")


# ==========================================================
# 12. MOVE GRIPPER FINGERS
# ==========================================================

def move_gripper(target_opening, steps=60):

    global q_gripper

    if not GRIPPER_MIN <= target_opening <= GRIPPER_MAX:
        raise ValueError(
            "Gripper opening outside joint limits"
        )

    start_opening = q_gripper

    for s in np.linspace(0, 1, steps):

        # Smooth interpolation
        blend = 3*s**2 - 2*s**3

        q_gripper = (
            start_opening
            + blend * (target_opening - start_opening)
        )

        update_robot_meshes()
        env.step(STEP_TIME)


# ==========================================================
# 13. TEST BOTH GRIPPER FINGERS
# ==========================================================

def test_gripper():

    print("\n================================")
    print(" REBOT GRIPPER TEST")
    print("================================\n")

    time.sleep(1)

    # Fully open
    print("Opening gripper")

    move_gripper(0.05)

    time.sleep(1)

    # Fully close
    print("Closing gripper")

    move_gripper(0.0)

    time.sleep(1)

    # Open halfway
    print("Opening gripper halfway")

    move_gripper(0.025)

    time.sleep(1)

    # Close again
    print("Closing gripper")

    move_gripper(0.0)

    print("\nGripper test complete!")


# ==========================================================
# 14. RUN SIMULATION
# ==========================================================

update_robot_meshes()
env.step(0.1)

# Test all six arm joints
test_all_joints()

# Test left and right gripper fingers
test_gripper()

# Keep Swift running
env.hold()
