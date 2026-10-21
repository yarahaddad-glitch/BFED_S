import numpy as np
from spatialmath import SE3
from spatialgeometry import Cylinder, Cuboid
try:
    import E_STOP
except ImportError:
    from Collective_Folder import E_STOP

# Each curtain is a straight line between two posts: ((x, y) post 1, (x, y) post 2)
CURTAINS = [
    ((9.5, 4.7), (16.0, 4.7)),    # along the front bars (x direction)
    ((16.0, 4.7), (16.0, 2.0)),   # along x = 16 (y direction)
]
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
    def __init__(self, env, on_trip=None):
        self.env = env
        self.prisoners = []
        self.robots = []
        self.muted = set()      # already reported, until they move away
        self.on_trip = on_trip or E_STOP.stop

        z0, z1 = CURTAIN_Z
        for (xa, ya), (xb, yb) in CURTAINS:
            length = np.hypot(xb - xa, yb - ya)
            angle = np.arctan2(yb - ya, xb - xa)
            for z in np.linspace(z0, z1, 8):                   # red beams
                env.add(Cylinder(radius=0.008, length=length, color=[1, 0.1, 0.1, 0.6],
                                 pose=SE3((xa + xb) / 2, (ya + yb) / 2, z)
                                 * SE3.Rz(angle) * SE3.Ry(np.pi / 2)))
            for x, y in ((xa, ya), (xb, yb)):                  # yellow posts
                env.add(Cuboid(scale=[0.06, 0.06, z1], color=[0.9, 0.8, 0.1, 1],
                               pose=SE3(x, y, z1 / 2)))

    def watch_prisoners(self, meshes):
        self.prisoners += list(meshes)

    def watch_robots(self, robots):
        self.robots += list(robots)

    def touching(self, points, r):
        """True if any point is within r of any curtain."""
        in_height = (points[:, 2] > CURTAIN_Z[0] - r) & (points[:, 2] < CURTAIN_Z[1] + r)
        p = points[:, :2]
        for a, b in CURTAINS:
            a, b = np.array(a, float), np.array(b, float)
            t = np.clip((p - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
            near = np.linalg.norm(p - (a + t[:, None] * (b - a)), axis=1) < r
            if np.any(near & in_height):
                return True
        return False

    def arm(self):
        """Check the curtains every time env.step() is called."""
        step = self.env.step

        def guarded_step(*args, **kwargs):
            result = step(*args, **kwargs)
            self.check()
            return result

        self.env.step = guarded_step

    def intrusions(self):
        """Current beam interruptions, even while the stop latch is active."""
        inside = set()
        for i, mesh in enumerate(self.prisoners):
            x, y, z = np.asarray(mesh.T, float)[:3, 3]
            if z < -0.5:  # Parked/hidden scene actors are not people in the cell.
                continue
            body = np.c_[np.full(10, x), np.full(10, y), z + np.linspace(0, 1.8, 10)]
            if self.touching(body, PERSON_RADIUS):
                inside.add(f"prisoner {i + 1}")
        for robot in self.robots:
            try:
                if self.touching(robot_points(robot), ROBOT_RADIUS):
                    inside.add(type(robot).__name__)
            except Exception:
                pass            # robot whose joints can't be read
        return inside

    def check(self):
        inside = self.intrusions()
        self.muted &= inside
        new = inside - self.muted
        if new:
            self.on_trip("SAFETY LIGHT CURTAIN BROKEN by: " + ", ".join(sorted(new)))
            self.muted |= inside
        return inside
