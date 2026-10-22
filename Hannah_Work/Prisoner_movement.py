import numpy as np
 
DT = 0.05
 
 
def walk_one_prisoner(env, scene, distance=3.8
                      , steps=60):
    mesh = scene["prisoner1"]
    T = np.array(mesh.T, float)
    start = T[:3, 3].copy()
    for s in np.linspace(0, 1, steps):
        T[:3, 3] = start + s * distance * np.array([0, -1, 0])   # -y
        mesh.T = T
        env.step(DT)
 
def walk_all_prisoners(env, scene, forward=2.0, side=3.5, pause=1.0, steps=60):
    """One prisoner at a time: walk forward (-y), stop for `pause` seconds, then walk to the side (+x).
    Each prisoner walks 1 m further forward and 1 m further across than the one before."""
    for i, name in enumerate(["prisoner1", "prisoner2", "prisoner3"]):
        mesh = scene[name]
        T = np.array(mesh.T, float)
 
        for direction, distance in (([0, -1, 0], forward + i), ([1, 0, 0], side - i)):
            start = T[:3, 3].copy()
            for s in np.linspace(0, 1, steps):
                T[:3, 3] = start + s * distance * np.array(direction)
                mesh.T = T
                env.step(DT)
            if direction == [0, -1, 0]:          # stop for a second after walking forward
                for _ in range(int(pause / DT)):
                    env.step(DT)
 