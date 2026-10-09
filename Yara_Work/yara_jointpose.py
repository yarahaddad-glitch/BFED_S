import numpy as np
from YaraBot_DH import YaraBot

robot = YaraBot()

# Use the corrected home configuration
q = np.array([0, -np.pi/2, 0, 0, 0, 0])

T = robot.base.A.copy()

print("Joint positions at HOME:\n")

for i in range(robot.n):
    print(f"Joint {i+1}:")
    print("Position:", np.round(T[:3, 3], 4))
    print("Axis:", np.round(T[:3, 2], 4))
    print()

    T = T @ robot.links[i].A(q[i]).A