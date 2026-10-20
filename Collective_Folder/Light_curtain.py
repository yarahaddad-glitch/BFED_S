import numpy as np
from spatialmath import SE3
from spatialgeometry import Cylinder, Cuboid
 
# Curtain along the front bars: runs along x, at y = CURTAIN_Y
CURTAIN_X = (10.0, 15.4)    # from post to post
CURTAIN_Y = 4.7
CURTAIN_Z = (0.15, 1.80)    # lowest to highest beam
PERSON_RADIUS = 0.25        # prisoner thickness (m)
ROBOT_RADIUS = 0.06         # robot link thickness (m)
 
 
def robot_points(robot):
    """Points along a robot's links, every 5 cm."""
    T = np.asarray(robot.base.A, float)
    joints = [T[:3, 3]]
    for link, q in zip(robot.links, robot.q):
        A = link.A(float(q))
        T = T @ np.asarray(getattr(A, "A", A), float)
        joints.append(T[:3, 3])
    return np.vstack([a + np.linspace(0, 1, 20)[:, None] * (b - a)
                      for a, b in zip(joints[:-1], joints[1:])])
 
 
class SafetyLightCurtain:
    def __init__(self, env):
        self.env = env
        self.prisoners = []
        self.robots = []
        self.muted = set()      # already reported, until they move away
 
        (x0, x1), (z0, z1) = CURTAIN_X, CURTAIN_Z
        for z in np.linspace(z0, z1, 8):                       # red beams
            env.add(Cylinder(radius=0.008, length=x1 - x0, color=[1, 0.1, 0.1, 0.6],
                             pose=SE3((x0 + x1) / 2, CURTAIN_Y, z) * SE3.Ry(np.pi / 2)))
        for x in (x0, x1):                                     # yellow posts
            env.add(Cuboid(scale=[0.06, 0.06, z1], color=[0.9, 0.8, 0.1, 1],
                           pose=SE3(x, CURTAIN_Y, z1 / 2)))
 
    def watch_prisoners(self, meshes):
        self.prisoners += list(meshes)
 
    def watch_robots(self, robots):
        self.robots += list(robots)
 
    def touching(self, points, r):
        """True if any point is within r of the curtain."""
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        return bool(np.any((x > CURTAIN_X[0] - r) & (x < CURTAIN_X[1] + r) &
                           (abs(y - CURTAIN_Y) < r) &
                           (z > CURTAIN_Z[0] - r) & (z < CURTAIN_Z[1] + r)))
 
    def arm(self):
        """Check the curtain every time env.step() is called."""
        step = self.env.step
 
        def guarded_step(*args, **kwargs):
            result = step(*args, **kwargs)
            self.check()
            return result
 
        self.env.step = guarded_step
 
    def check(self):
        inside = set()
        for i, mesh in enumerate(self.prisoners):
            x, y, z = np.asarray(mesh.T, float)[:3, 3]
            body = np.c_[np.full(10, x), np.full(10, y), z + np.linspace(0, 1.8, 10)]
            if self.touching(body, PERSON_RADIUS):
                inside.add(f"prisoner {i + 1}")
        for robot in self.robots:
            try:
                if self.touching(robot_points(robot), ROBOT_RADIUS):
                    inside.add(type(robot).__name__)
            except Exception:
                pass            # robot whose joints can't be read
        self.muted &= inside
        new = inside - self.muted
        if new:
            print("!!! SAFETY LIGHT CURTAIN BROKEN by:", ", ".join(new))
            input(">>> EMERGENCY STOP - press Enter to reset... ")
            self.muted |= inside
 