import numpy as np
from spatialmath import SE3
 
WALK_SPEED = 0.6    # m/s
DT = 0.05           # time per step (s)
 
# Each prisoner's path: first point = where it starts, then where it walks to.
# Placeholder values - change them to suit the scene.
PATHS = {
    "prisoner1": [(12.5, 3.6), (12.5, 5.5)],            # straight through the curtain
    "prisoner2": [(11.0, 6.0), (11.0, 5.2), (12.0, 5.2)],
    "prisoner3": [(14.0, 6.0), (14.0, 5.0)],
}
 
 
def walk_prisoner(env, mesh, path, speed=WALK_SPEED):
    """Put the prisoner at the first point, then walk it to each next point in turn."""
    z = np.asarray(mesh.T, float)[2, 3]
    (x, y), heading = path[0], 0.0
    mesh.T = SE3(x, y, z).A
    env.step(DT)
    for x1, y1 in path[1:]:
        heading = np.arctan2(y1 - y, x1 - x)          # face the way it's walking
        steps = max(int(np.hypot(x1 - x, y1 - y) / speed / DT), 1)
        for s in np.linspace(0, 1, steps + 1)[1:]:
            mesh.T = (SE3(x + s * (x1 - x), y + s * (y1 - y), z) * SE3.Rz(heading)).A
            env.step(DT)
        x, y = x1, y1
 
 
def walk_all_prisoners(env, scene, paths=PATHS):
    """Walk each prisoner in PATHS, one after another."""
    for name, path in paths.items():
        if name not in scene:
            print(f"'{name}' is not in the scene - skipped.")
            continue
        print(f"{name} walking")
        walk_prisoner(env, scene[name], path)
 