import numpy as np
import xml.etree.ElementTree as ET
from spatialmath import SE3
from spatialgeometry import Cuboid
 
ISO_PARTS = ["ISO_cell1", "ISO_cell2", "ISO_cell3",
             "backWall", "backWall2", "LeftWall", "rightWall", "HannahCounter"]
MIN_HEIGHT = 0.2            # anything lower than this doesn't count
ROBOT_RADIUS = 0.06         # how thick HannahBot's links are (m)
SHOW_BOXES = True           # draw the collision boxes see-through, to check them
 
# Tray corners relative to its origin (same as Feed_Iso.py)
TRAY_LO = [0.0, -0.198, 0.0]
TRAY_HI = [0.3, 0.0002, 0.018]
 
 
def dae_object_boxes(path, T_world=None):
    """One axis-aligned world box per object in a COLLADA (.dae) file.
 
    Returns a list of (name, lo, hi). T_world is the 4x4 pose the mesh was
    added with (identity if it was added at the origin).
    """
    T_world = np.eye(4) if T_world is None else np.asarray(getattr(T_world, "A", T_world), float)
    root = ET.parse(str(path)).getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""
    T = lambda name: ns + name
 
    unit = 1.0
    u = root.find(f".//{T('unit')}")
    if u is not None and u.get("meter"):
        unit = float(u.get("meter"))
    up = (root.findtext(f".//{T('up_axis')}") or "Z_UP").strip()
 
    geoms = {}
    for g in root.iter(T("geometry")):
        mesh = g.find(T("mesh"))
        if mesh is None:
            continue
        sources = {s.get("id"): s for s in mesh.findall(T("source"))}
        pos_id = None
        v = mesh.find(T("vertices"))
        if v is not None:
            for inp in v.findall(T("input")):
                if inp.get("semantic") == "POSITION":
                    pos_id = inp.get("source").lstrip("#")
        if pos_id in sources:
            arr = sources[pos_id].find(T("float_array"))
            geoms[g.get("id")] = np.array(arr.text.split(), float).reshape(-1, 3)
 
    def node_matrix(node):
        M = np.eye(4)
        for ch in node:
            tag = ch.tag.replace(ns, "")
            vals = np.array(ch.text.split(), float) if ch.text else None
            if tag == "matrix":
                M = M @ vals.reshape(4, 4)
            elif tag == "translate":
                A = np.eye(4); A[:3, 3] = vals; M = M @ A
            elif tag == "scale":
                M = M @ np.diag([*vals, 1.0])
            elif tag == "rotate":
                ax, ang = vals[:3], np.deg2rad(vals[3])
                ax = ax / np.linalg.norm(ax)
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                A = np.eye(4)
                A[:3, :3] = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
                M = M @ A
        return M
 
    def to_world(P):
        P = P * unit
        if up == "Y_UP":
            P = np.c_[P[:, 0], -P[:, 2], P[:, 1]]
        return (np.c_[P, np.ones(len(P))] @ T_world.T)[:, :3]
 
    boxes = []
 
    def walk(node, parent, prefix):
        M = parent @ node_matrix(node)
        name = node.get("name") or node.get("id") or prefix
        for ig in node.findall(T("instance_geometry")):
            gid = ig.get("url").lstrip("#")
            if gid in geoms:
                P = (np.c_[geoms[gid], np.ones(len(geoms[gid]))] @ M.T)[:, :3]
                W = to_world(P)
                boxes.append((name, W.min(axis=0), W.max(axis=0)))
        for child in node.findall(T("node")):
            walk(child, M, name)
 
    scene = root.find(f".//{T('visual_scene')}")
    if scene is not None:
        for n in scene.findall(T("node")):
            walk(n, np.eye(4), "object")
    if not boxes:
        for gid, P in geoms.items():
            W = to_world(P)
            boxes.append((gid, W.min(axis=0), W.max(axis=0)))
    return boxes
 
 
class WallCollision:
    def __init__(self, env, scene):
        self.env = env
        self.robots, self.trays, self.boxes = [], [], []
        self.balls = []          # (centre, radius) - the arm steers around these
        self.test_ball = None    # set by test_ball_in_path()
        self.muted = set()
        for name in ISO_PARTS:
            mesh = scene.get(name)
            if mesh is None:
                continue
            T = np.array(mesh.T, float) @ np.diag([*np.resize(np.ravel(mesh.scale), 3), 1.0])
            for part, lo, hi in dae_object_boxes(mesh.filename, T):
                lo[2] = max(lo[2], MIN_HEIGHT)
                if hi[2] > lo[2]:
                    self.boxes.append((f"{name}:{part}", lo, hi))
                    if SHOW_BOXES:
                        env.add(Cuboid(scale=(hi - lo).tolist(), pose=SE3(*((lo + hi) / 2)),
                                       color=[1.0, 0.4, 0.0, 0.2]))
        print(f"Wall collision: {len(self.boxes)} boxes in the isolation area.")
 
    # ---------------- test: a ball in the arm's path ----------------
    def test_ball_in_path(self, radius=0.15):
        """Ask feed_iso to drop a ball in the hallway, halfway along the drive
        after it picks up the first tray. The TurtleBot drives round it."""
        self.test_ball = radius
 
    def add_ball(self, centre, radius=0.05):
        """A ball obstacle: the arm steers around it, and touching it is a collision."""
        from spatialgeometry import Sphere
        centre = np.asarray(centre, float)
        self.env.add(Sphere(radius=radius, pose=SE3(*centre), color=[1.0, 0.0, 0.0, 1.0]))
        self.balls.append((centre, radius))
        print(f"TEST: ball placed at {np.round(centre, 3).tolist()}")
 
    def watch_robot(self, robot, name="HannahBot"):
        self.robots.append((name, robot))
 
    def watch_trays(self, trays):
        self.trays += list(trays)
 
    def points(self):
        """(name, points, radius) for everything being watched."""
        out = []
        for name, robot in self.robots:
            T = np.asarray(robot.base.A, float)
            joints = [T[:3, 3]]
            for link, q in zip(robot.links, robot.q):
                A = link.A(float(q))
                T = T @ np.asarray(getattr(A, "A", A), float)
                joints.append(T[:3, 3])
            pts = np.vstack([a + np.linspace(0, 1, 20)[:, None] * (b - a)
                             for a, b in zip(joints[:-1], joints[1:])])
            out.append((name, pts, ROBOT_RADIUS))
        for i, tray in enumerate(self.trays):
            s = np.resize(np.ravel(tray.scale), 3)
            # a grid of points about 4 cm apart across the tray, top and bottom
            grid = np.array([[x, y, z]
                             for x in np.linspace(TRAY_LO[0], TRAY_HI[0], 9)
                             for y in np.linspace(TRAY_LO[1], TRAY_HI[1], 6)
                             for z in (TRAY_LO[2], TRAY_HI[2])]) * s
            T = np.asarray(tray.T, float)
            out.append((f"tray {i + 1}", grid @ T[:3, :3].T + T[:3, 3], 0.0))
        return out
 
    def arm(self):
        """Check for collisions every time env.step() is called."""
        step = self.env.step
 
        def guarded_step(*args, **kwargs):
            result = step(*args, **kwargs)
            self.check()
            return result
 
        self.env.step = guarded_step
 
    def check(self):
        hits = set()
        for name, pts, r in self.points():
            for wall, lo, hi in self.boxes:
                if np.any(np.all((pts > lo - r) & (pts < hi + r), axis=1)):
                    hits.add((name, wall))
            for k, (c, br) in enumerate(self.balls):
                if np.min(np.linalg.norm(pts - c, axis=1)) < br + r:
                    hits.add((name, f"ball {k + 1}"))
        self.muted &= hits
        new = hits - self.muted
        if new:
            for name, wall in new:
                print(f"!!! COLLISION: {name} hit {wall}")
            input(">>> EMERGENCY STOP - press Enter to reset... ")
            self.muted |= hits
 