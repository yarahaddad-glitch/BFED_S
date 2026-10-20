
import numpy as np
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH

np.set_printoptions(precision=6, suppress=True)

# ============================================================
# 1. MEASURED JOINT GEOMETRY FROM BLENDER
# ============================================================

pivots = np.array([
    [0.0,      0.0,       0.0],
    [-0.07002, 0.000345,  0.1473],
    [-0.14540, -0.000163, 0.5129],
    [-0.02014, -0.000163, 0.9431],
    [-0.07864, -0.000163, 0.9928],
    [-0.12830, -0.000163, 1.0510]
], dtype=float)

# Rotation axes in the zero configuration
axes = np.array([
    [0, 0, 1],  # Joint 1
    [1, 0, 0],  # Joint 2
    [0, 0, 1],  # Joint 3
    [0, 0, 1],  # Joint 4
    [1, 0, 0],  # Joint 5
    [0, 0, 1]   # Joint 6
], dtype=float)

# Measured end-effector attachment position
tool_position = np.array([
    -0.23330,
     0.000497,
     1.1150
], dtype=float)


# ============================================================
# 2. UTILITY FUNCTIONS
# ============================================================

def make_frame(origin, x_axis, z_axis):
    """Create an SE3 coordinate frame."""

    z = np.asarray(z_axis, dtype=float)
    z = z / np.linalg.norm(z)

    x = np.asarray(x_axis, dtype=float)
    x = x - np.dot(x, z) * z
    x = x / np.linalg.norm(x)

    y = np.cross(z, x)

    T = np.eye(4)
    T[:3, :3] = np.column_stack((x, y, z))
    T[:3, 3] = origin

    return SE3(T)


def closest_points(p1, z1, p2, z2):
    """Find the closest points on two infinite joint axes."""

    A = np.column_stack((z1, -z2))
    b = p2 - p1

    parameters = np.linalg.lstsq(A, b, rcond=None)[0]

    c1 = p1 + parameters[0] * z1
    c2 = p2 + parameters[1] * z2

    return c1, c2


def dh_transform(theta, d, a, alpha):
    """Standard DH transformation."""

    return (
        SE3.Rz(theta)
        * SE3(0, 0, d)
        * SE3(a, 0, 0)
        * SE3.Rx(alpha)
    )


# ============================================================
# 3. CALCULATE STANDARD DH COORDINATE FRAMES
# ============================================================

# Frame 0 is chosen with Z along Joint 1.
# Frame i-1 has Z along Joint i.

# For standard DH, the X direction of each subsequent frame
# is the common normal between its Z axis and the next axis.

origins = [np.zeros(3)]
x_directions = []
z_directions = []

# Calculate common normals between adjacent joint axes.
for i in range(5):

    c1, c2 = closest_points(
        pivots[i], axes[i],
        pivots[i + 1], axes[i + 1]
    )

    displacement = c2 - c1
    cross = np.cross(axes[i], axes[i + 1])

    if np.linalg.norm(displacement) > 1e-9:
        normal = displacement / np.linalg.norm(displacement)

    elif np.linalg.norm(cross) > 1e-9:
        normal = cross / np.linalg.norm(cross)

    else:
        raise ValueError(
            f"Cannot uniquely assign common normal "
            f"between joints {i + 1} and {i + 2}"
        )

    x_directions.append(normal)

# Construct frame 0 with a convenient X direction.
frames = [
    make_frame(
        origins[0],
        [1, 0, 0],
        axes[0]
    )
]

# Construct intermediate frames.
for i in range(1, 6):

    c_previous, c_current = closest_points(
        pivots[i - 1], axes[i - 1],
        pivots[i], axes[i]
    )

    frames.append(
        make_frame(
            c_current,
            x_directions[i - 1],
            axes[i]
        )
    )

# End frame uses Joint 6's Z axis.
frames.append(
    make_frame(
        pivots[5],
        frames[5].R[:, 0],
        axes[5]
    )
)


# ============================================================
# 4. EXTRACT DH PARAMETERS
# ============================================================

dh_parameters = []

print("\nHANNAHBOT STANDARD DH PARAMETERS")
print("=" * 80)

print(
    f"{'Joint':<8}"
    f"{'a (m)':<14}"
    f"{'d (m)':<14}"
    f"{'alpha (deg)':<18}"
    f"{'offset (deg)':<18}"
)

for i in range(6):

    T_relative = frames[i].inv() * frames[i + 1]

    R = T_relative.R
    t = T_relative.t

    theta_offset = np.arctan2(R[1, 0], R[0, 0])

    alpha = np.arctan2(R[2, 1], R[2, 2])

    a = (
        t[0] * np.cos(theta_offset)
        + t[1] * np.sin(theta_offset)
    )

    d = t[2]

    T_reconstructed = dh_transform(
        theta_offset, d, a, alpha
    )

    reconstruction_error = np.linalg.norm(
        T_relative.A - T_reconstructed.A
    )

    if reconstruction_error > 1e-6:
        raise RuntimeError(
            f"Joint {i + 1}: Invalid DH frame assignment. "
            f"Reconstruction error = {reconstruction_error:.9f}"
        )

    dh_parameters.append(
        (a, d, alpha, theta_offset)
    )

    print(
        f"{i + 1:<8}"
        f"{a:<14.6f}"
        f"{d:<14.6f}"
        f"{np.rad2deg(alpha):<18.3f}"
        f"{np.rad2deg(theta_offset):<18.3f}"
    )


# ============================================================
# 5. CREATE HANNAHBOT DH ROBOT
# ============================================================

links = []

for a, d, alpha, offset in dh_parameters:

    links.append(
        RevoluteDH(
            a=float(a),
            d=float(d),
            alpha=float(alpha),
            offset=float(offset),
            qlim=[-np.pi, np.pi]
        )
    )

robot = DHRobot(links, name="HannahBot_DH")

robot.base = frames[0]

# Align the DH tool frame with the measured gripper point.
robot.tool = frames[6].inv() * SE3(*tool_position)


# ============================================================
# 6. REFERENCE FK USING MEASURED JOINT AXES
# ============================================================

def measured_fkine(q):
    """
    Forward kinematics using the measured joint axes
    and pivots from Blender.
    """

    T = SE3()

    for i in range(6):

        p = pivots[i]
        angle = float(q[i])

        if np.allclose(axes[i], [1, 0, 0]):
            R = SE3.Rx(angle)
        else:
            R = SE3.Rz(angle)

        T_joint = (
            SE3(*p)
            * R
            * SE3(*(-p))
        )

        T = T * T_joint

    return T * SE3(*tool_position)


# ============================================================
# 7. VALIDATE DH FK AGAINST MEASURED FK
# ============================================================

test_configs = [
    [0, 0, 0, 0, 0, 0],
    [0, 30, 0, 0, 0, 0],
    [0, 0, 30, 0, 0, 0],
    [0, 0, 0, 30, 0, 0],
    [0, 0, 0, 0, 30, 0],
    [0, 0, 0, 0, 0, 30],
    [10, 20, -15, 30, 10, -20]
]

print("\nFORWARD KINEMATICS VALIDATION")
print("=" * 80)

all_passed = True

for angles in test_configs:

    q = np.deg2rad(angles)

    T_DH = robot.fkine(q)
    T_measured = measured_fkine(q)

    position_error = np.linalg.norm(
        T_DH.t - T_measured.t
    )

    rotation_error = np.linalg.norm(
        T_DH.R - T_measured.R
    )

    print(f"\nJoint angles (deg): {angles}")
    print(f"DH position:       {T_DH.t}")
    print(f"Measured position: {T_measured.t}")
    print(f"Position error:    {position_error:.9f} m")
    print(f"Rotation error:    {rotation_error:.9f}")

    if position_error > 1e-6 or rotation_error > 1e-6:
        all_passed = False


# ============================================================
# 8. FINAL RESULT
# ============================================================

print("\n" + "=" * 80)

if all_passed:
    print("SUCCESS: All FK validation tests passed!")
    print("The DH model matches the measured joint geometry.")
else:
    print("VALIDATION FAILED.")
    print("The DH frame assignments need further correction.")

print("=" * 80)
