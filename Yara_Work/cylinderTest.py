import swift
import numpy as np
from math import pi

from roboticstoolbox import DHRobot, RevoluteDH
from ir_support import CylindricalDHRobotPlot

# ------------------------------------------
# 1. EC66 DH PARAMETERS
# ------------------------------------------

links = [
    RevoluteDH(d=0.096, a=0, alpha=-pi/2),
    RevoluteDH(d=0, a=0.418, alpha=0),
    RevoluteDH(d=0, a=0.398, alpha=0),
    RevoluteDH(d=0.122, a=0, alpha=-pi/2),
    RevoluteDH(d=0.098, a=0, alpha=-pi/2),
    RevoluteDH(d=0.089, a=0, alpha=0)
]

robot = DHRobot(links, name="EC66_DH_Test")

# ------------------------------------------
# 2. VISUALISE USING CYLINDERS
# ------------------------------------------

cyl_viz = CylindricalDHRobotPlot(
    robot,
    cylinder_radius=0.025,
    color=[
        "#0e2144",
        "#9e0d0d",
        "#0e2144",
        "#9e0d0d",
        "#0e2144",
        "#9e0d0d"
    ]
)

robot = cyl_viz.create_cylinders()

# ------------------------------------------
# 3. START SWIFT
# ------------------------------------------

env = swift.Swift()
env.launch(realtime=True, browser=None)

env.add(robot)

robot.q = np.array([0, -pi/2, 0, 0, 0, 0])
env.step()

print("\nEC66 CYLINDER TEST")
print("Initial joint configuration:", robot.q)

input("\nInspect HOME position. Press Enter to begin...")

# ------------------------------------------
# 4. TEST EACH JOINT INDIVIDUALLY
# ------------------------------------------

for joint in range(6):

    robot.q = np.zeros(6)
    env.step()

    print(f"\nTesting Joint {joint + 1}")

    # Print joint axis and position
    T = robot.base.A.copy()

    for i in range(joint):
        T = T @ robot.links[i].A(0).A

    print("Axis:", np.round(T[:3, 2], 4))
    print("Position:", np.round(T[:3, 3], 4))

    input(f"Press Enter to rotate Joint {joint + 1}...")

    # Rotate selected joint by 30 degrees
    for angle in np.linspace(0, pi/6, 40):

        q = np.zeros(6)
        q[joint] = angle

        robot.q = q
        env.step(0.05)

    input("Inspect movement. Press Enter to reset...")

    robot.q = np.zeros(6)
    env.step()

print("\nAll joint tests completed!")

env.hold()