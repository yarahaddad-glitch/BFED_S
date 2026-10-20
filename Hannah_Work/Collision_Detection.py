import numpy as np
import xml.etree.ElementTree as ET
from spatialmath import SE3
from spatialgeometry import Cylinder, Sphere, Cuboid
 
# Each entry: (first DH frame, last DH frame, radius in metres). Frame 0 is the
# arm base on the TurtleBot deck, frame 6 is the flange. Zero-length segments
# are fine (they become spheres). These radii are trial values - tune them.
DEFAULT_BODIES = [
    (0, 1, 0.065),   # base / shoulder
    (1, 2, 0.055),   # upper arm
    (2, 3, 0.050),   # elbow
    (3, 4, 0.045),   # forearm
    (4, 5, 0.040),   # wrist 1
    (5, 6, 0.040),   # wrist 2
]
GRIPPER_RADIUS = 0.040      # capsule from the flange to the tool point
DEFAULT_SAFETY_GAP = 0.030  # extra clearance kept from every obstacle
CULL_RADIUS = 1.6           # only check boxes this close to the arm base (m)
SAMPLE_STEP = 0.01          # spacing of check points along each capsule (m)
 
 
# ---------------- geometry helpers ----------------
def segment_transform(a, b):
    """Pose with a cylinder's local Z axis along the segment a -> b."""
    direction = np.asarray(b, float) - np.asarray(a, float)
    length = float(np.linalg.norm(direction))
    z_axis = direction / length if length > 1e-9 else np.array([0.0, 0.0, 1.0])
    helper = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z_axis, helper)) > 0.95:
        helper = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(helper, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    centre = (np.asarray(a, float) + np.asarray(b, float)) / 2.0
    return SE3.Rt(np.column_stack([x_axis, y_axis, z_axis]), centre), max(length, 0.001)
 
 
def box_signed_distance(p, lo, hi):
    """Distance from point p to an axis-aligned box (negative = inside)."""
    q = np.maximum(lo - p, p - hi)
    outside = np.linalg.norm(np.maximum(q, 0.0))
    inside = min(float(np.max(q)), 0.0)
    return float(outside + inside)
 
 
def boxes_signed_distance(P, LO, HI):
    """(N, M) signed distances from N points to M boxes."""
    q = np.maximum(LO[None] - P[:, None], P[:, None] - HI[None])
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=2)
    inside = np.minimum(q.max(axis=2), 0.0)
    return outside + inside
 
 
def points_segments_distance(P, A, B):
    """(N, K) distances from N points to K segments A[k] -> B[k]."""
    AB = B - A
    L2 = np.maximum(np.einsum("kj,kj->k", AB, AB), 1e-12)
    t = np.clip(np.einsum("nkj,kj->nk", P[:, None] - A[None], AB) / L2, 0.0, 1.0)
    closest = A[None] + t[..., None] * AB[None]
    return np.linalg.norm(P[:, None] - closest, axis=2)
 
 
def sample_segment(a, b, step=SAMPLE_STEP):
    n = max(2, int(np.linalg.norm(b - a) / step) + 1)
    return a + np.linspace(0, 1, n)[:, None] * (b - a)
 
 
def robot_frames(robot, q):
    """World transforms of the base and every DH frame of a robot."""
    try:
        T = np.asarray(robot.base.A, float).copy()
        out = [T.copy()]
        for link, angle in zip(robot.links, q):
            A = link.A(float(angle))
            T = T @ np.asarray(getattr(A, "A", A), float)
            out.append(T.copy())
        return out
    except Exception:
        return [np.asarray(t.A, float) for t in robot.fkine_all(q)]
 
 
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
 
 
# ---------------- checker ----------------
class CollisionChecker:
    """Approximate collision checks for HannahBot.
 
    robot      : the HannahBot object
    tool_fn    : function q -> tool XYZ (the gripper point the controller uses)
    safety_gap : extra clearance the arm must keep from obstacles (m)
    """
 
    def __init__(self, robot, tool_fn, bodies=None, gripper_radius=GRIPPER_RADIUS,
                 safety_gap=DEFAULT_SAFETY_GAP):
        self.robot = robot
        self.tool_fn = tool_fn
        self.bodies = list(bodies or DEFAULT_BODIES)
        self.gripper_radius = gripper_radius
        self.safety_gap = safety_gap
        self.spheres = []      # dicts: name, centre, radius, visual
        self.boxes = []        # dicts: name, lo, hi, soft, visual
        self.robots = []       # dicts: name, robot, radius
        self._arrays = None
        self._guides = []
        self._robot_cache = {}
 
    # ---------------- adding / removing obstacles ----------------
    def add_sphere(self, centre, radius, env=None, name=None, color=(1.0, 0.0, 0.0, 1.0)):
        centre = np.asarray(centre, float)
        name = name or f"ball{len(self.spheres) + 1}"
        vis = None
        if env is not None:
            vis = Sphere(radius=float(radius), pose=SE3(*centre.tolist()), color=color)
            env.add(vis)
        self.spheres.append({"name": name, "centre": centre, "radius": float(radius),
                             "visual": vis})
        print(f"Obstacle '{name}': sphere at {np.round(centre, 3).tolist()}, r = {radius:.3f} m")
 
    def add_box(self, lo, hi, env=None, name=None, color=(1.0, 0.3, 0.3, 0.25), quiet=False):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
        name = name or f"box{len(self.boxes) + 1}"
        vis = None
        if env is not None:
            size = np.maximum(hi - lo, 0.002)
            vis = Cuboid(scale=size.tolist(), pose=SE3(*((lo + hi) / 2).tolist()), color=color)
            env.add(vis)
        self.boxes.append({"name": name, "lo": lo, "hi": hi, "soft": False, "visual": vis})
        self._arrays = None
        if not quiet:
            print(f"Obstacle '{name}': box x {np.round([lo[0], hi[0]], 3).tolist()} "
                  f"y {np.round([lo[1], hi[1]], 3).tolist()} "
                  f"z {np.round([lo[2], hi[2]], 3).tolist()}")
 
    def add_mesh_boxes(self, path, T_world=None, env=None, floor_z=0.01, min_size=0.01):
        """Add one box per object in a scene .dae file (walls, tables, ...).
        Flat floor pieces (top below floor_z) and tiny objects are skipped."""
        added = 0
        stem = str(path).replace("\\", "/").split("/")[-1].rsplit(".", 1)[0]
        if not str(path).lower().endswith(".dae"):
            print(f"Scene '{stem}': not a .dae file - skipped (add a 'box' for it if needed).")
            return 0
        try:
            boxes = dae_object_boxes(path, T_world)
        except Exception as e:
            print(f"Scene '{stem}': could not read it ({e}) - skipped.")
            return 0
        for name, lo, hi in boxes:
            if hi[2] <= floor_z or np.max(hi - lo) < min_size:
                continue
            self.add_box(lo, hi, env, name=f"{stem}:{name}", quiet=True)
            added += 1
        print(f"Scene '{stem}': {added} obstacle box(es) added.")
        return added
 
    def add_robot(self, robot, radius=0.07, name=None):
        """Treat another robot's links as capsules (they follow the robot)."""
        name = name or type(robot).__name__
        self.robots.append({"name": name, "robot": robot, "radius": float(radius)})
        print(f"Obstacle '{name}': robot links, r = {radius:.3f} m")
 
    def mark_soft_table(self, tray_lo, tray_hi, below=0.03):
        """The table the tray sits on: the gripper may come close to it (it has
        to, to reach the tray) but not into it. The arm links still avoid it."""
        for b in self.boxes:
            top_ok = tray_lo[2] - below <= b["hi"][2] <= tray_lo[2] + 0.005
            overlap = (b["lo"][0] < tray_hi[0] and b["hi"][0] > tray_lo[0]
                       and b["lo"][1] < tray_hi[1] and b["hi"][1] > tray_lo[1])
            if top_ok and overlap:
                b["soft"] = True
                print(f"'{b['name']}' is the tray's table: the gripper may reach over it.")
        self._arrays = None
 
    def remove(self, name):
        """Remove obstacles whose name contains `name` (or an index from 'list')."""
        everything = self.spheres + self.boxes + self.robots
        if name.isdigit() and 1 <= int(name) <= len(everything):
            targets = [everything[int(name) - 1]]
        else:
            targets = [o for o in everything if name.lower() in o["name"].lower()]
        for o in targets:
            if o.get("visual") is not None:
                o["visual"].T = SE3(0, 0, -100).A
            for lst in (self.spheres, self.boxes, self.robots):
                if o in lst:
                    lst.remove(o)
            print(f"Removed '{o['name']}'.")
        if not targets:
            print(f"No obstacle matching '{name}'.")
        self._arrays = None
 
    def clear(self, keep_scene=True):
        """Remove hand-added obstacles (and the scene too if keep_scene=False)."""
        for o in list(self.spheres):
            self.remove(o["name"])
        for b in list(self.boxes):
            if not keep_scene or ":" not in b["name"]:
                self.remove(b["name"])
 
    def list(self):
        everything = self.spheres + self.boxes + self.robots
        for i, o in enumerate(everything, 1):
            kind = "sphere" if o in self.spheres else "robot" if o in self.robots else \
                ("table (soft)" if o.get("soft") else "box")
            print(f"  {i:3d}. {o['name']}  [{kind}]")
        if not everything:
            print("  (no obstacles)")
 
    # ---------------- internal ----------------
    def _box_arrays(self):
        if self._arrays is None:
            if self.boxes:
                LO = np.array([b["lo"] for b in self.boxes])
                HI = np.array([b["hi"] for b in self.boxes])
                soft = np.array([b["soft"] for b in self.boxes])
            else:
                LO = HI = np.zeros((0, 3))
                soft = np.zeros(0, bool)
            self._arrays = (LO, HI, soft)
        return self._arrays
 
    def _robot_segments(self):
        """(A, B, R) capsules of all other robots, cached while they don't move."""
        A, B, R = [], [], []
        for o in list(self.robots):
            rb = o["robot"]
            try:
                q = np.asarray(rb.q, float)
                key = (id(rb), q.tobytes(), np.asarray(rb.base.A, float).tobytes())
                if key not in self._robot_cache:
                    fs = robot_frames(rb, q)
                    self._robot_cache[key] = [(fs[i][:3, 3], fs[i + 1][:3, 3])
                                              for i in range(len(fs) - 1)]
            except Exception as e:
                print(f"Can't read the joints of '{o['name']}' ({e}) - "
                      f"it will not be avoided.")
                self.robots.remove(o)
                continue
            for a, b in self._robot_cache[key]:
                A.append(a); B.append(b); R.append(o["radius"])
        if len(self._robot_cache) > 50:
            self._robot_cache.clear()
        if not A:
            return None
        return np.array(A), np.array(B), np.array(R)
 
    def _clearances(self, P, radius, centre_hint):
        """Clearance of points P (inflated by radius) to every obstacle type.
        Returns (hard, soft): soft is the distance to the tray's table only."""
        hard, soft = np.inf, np.inf
        for s in self.spheres:
            d = np.linalg.norm(P - s["centre"], axis=1).min() - s["radius"] - radius
            hard = min(hard, d)
        LO, HI, softmask = self._box_arrays()
        if len(LO):
            near = boxes_signed_distance(centre_hint[None], LO, HI)[0] < CULL_RADIUS
            if near.any():
                D = boxes_signed_distance(P, LO[near], HI[near]).min(axis=0)
                sm = softmask[near]
                if (~sm).any():
                    hard = min(hard, D[~sm].min() - radius)
                if sm.any():
                    soft = min(soft, D[sm].min())
        rs = self._robot_segments()
        if rs is not None:
            A, B, R = rs
            D = points_segments_distance(P, A, B).min(axis=0) - R - radius
            hard = min(hard, D.min())
        return hard, soft
 
    # ---------------- geometry of HannahBot ----------------
    def frames(self, q):
        return robot_frames(self.robot, q)
 
    def segments(self, q):
        """List of (label, a, b, radius) capsules for the arm + gripper."""
        fs = self.frames(q)
        segs = []
        for i, j, r in self.bodies:
            if j < len(fs):
                segs.append((f"link {i}->{j}", fs[i][:3, 3], fs[j][:3, 3], r))
        segs.append(("gripper", fs[-1][:3, 3], np.asarray(self.tool_fn(q), float),
                     self.gripper_radius))
        return segs
 
    def has_obstacles(self):
        return bool(self.spheres or self.boxes or self.robots)
 
    # ---------------- checks ----------------
    def safe(self, q, attached=None, extra=None):
        """True if pose q is clear.
 
        attached : (N, 3) tray points relative to the tool point, or None
        extra    : (M, 3) world points (e.g. TurtleBot body), or None
        Tray and TurtleBot points may touch an obstacle surface (the tray sits
        on the table) but not go inside it.
        """
        if not self.has_obstacles():
            return True
        segs = self.segments(q)
        hint = segs[0][1]
        for label, a, b, r in segs:
            hard, soft = self._clearances(sample_segment(a, b), r, hint)
            if hard <= self.safety_gap:
                return False
            if label == "gripper":
                if soft < -1e-4:
                    return False
            elif soft - r <= self.safety_gap:
                return False
        pts = []
        if attached is not None:
            pts.append(np.asarray(self.tool_fn(q), float) + np.asarray(attached, float))
        if extra is not None:
            pts.append(np.asarray(extra, float))
        if pts:
            hard, soft = self._clearances(np.vstack(pts), 0.0, hint)
            if min(hard, soft) < -1e-4:
                return False
        return True
 
    def safe_step(self, qa, qb, attached=None, extra=None, samples=5):
        """Check poses between qa and qb, not only the ends."""
        return all(self.safe((1 - t) * qa + t * qb, attached, extra)
                   for t in np.linspace(0, 1, samples))
 
    def remove_blocking(self, q, attached=None, extra=None):
        """Remove scene boxes that already collide with the robot in pose q
        (e.g. a room-sized box around it). Returns the names removed."""
        removed = []
        for b in list(self.boxes):
            if self.safe(q, attached, extra):
                break
            only = self.boxes, self.spheres, self.robots
            self.boxes, self.spheres, self.robots = [b], [], []
            self._arrays = None
            blocking = not self.safe(q, attached, extra)
            self.boxes, self.spheres, self.robots = only
            self._arrays = None
            if blocking:
                self.remove(b["name"])
                removed.append(b["name"])
        return removed
 
    def report(self, q, attached=None, extra=None, top=3):
        """Print clearances (negative = colliding) and the nearest obstacles."""
        print("Clearances (mm, after the safety gap):")
        segs = self.segments(q)
        hint = segs[0][1]
        for label, a, b, r in segs:
            hard, soft = self._clearances(sample_segment(a, b), r, hint)
            line = f"  {label:<12}: {(hard - self.safety_gap) * 1000:8.1f}"
            if np.isfinite(soft):
                s = soft * 1000 if label == "gripper" else (soft - r - self.safety_gap) * 1000
                line += f"   (tray table: {s:.1f})"
            print(line)
        if attached is not None:
            P = np.asarray(self.tool_fn(q), float) + np.asarray(attached, float)
            print(f"  {'held tray':<12}: {min(self._clearances(P, 0, hint)) * 1000:8.1f}")
        if extra is not None:
            print(f"  {'turtlebot':<12}: "
                  f"{min(self._clearances(np.asarray(extra), 0, hint)) * 1000:8.1f}")
 
        # Which obstacles are closest to any part of the robot?
        P = np.vstack([sample_segment(a, b) for _, a, b, _ in segs]
                      + ([np.asarray(extra)] if extra is not None else []))
        named = []
        for s in self.spheres:
            named.append((np.linalg.norm(P - s["centre"], axis=1).min() - s["radius"], s["name"]))
        for bx in self.boxes:
            named.append((boxes_signed_distance(P, bx["lo"][None], bx["hi"][None]).min(),
                          bx["name"]))
        for o in self.robots:
            fs = robot_frames(o["robot"], np.asarray(o["robot"].q, float))
            A = np.array([f[:3, 3] for f in fs[:-1]])
            B = np.array([f[:3, 3] for f in fs[1:]])
            named.append((points_segments_distance(P, A, B).min() - o["radius"], o["name"]))
        if named:
            print("Nearest obstacles (surface distance, mm):")
            for d, n in sorted(named)[:top]:
                print(f"  {d * 1000:8.1f}  {n}")
 
    # ---------------- see-through capsules in Swift ----------------
    def show_bodies(self, env, q, alpha=0.15):
        """Draw the collision capsules so you can tune their size."""
        tint = (0.15, 0.95, 0.95, alpha)
        for label, a, b, r in self.segments(q):
            pose, length = segment_transform(a, b)
            body = Cylinder(radius=float(r), length=float(length), pose=pose, color=tint)
            end_a = Sphere(radius=float(r), pose=SE3(*a.tolist()), color=tint)
            end_b = Sphere(radius=float(r), pose=SE3(*b.tolist()), color=tint)
            for shape in (body, end_a, end_b):
                env.add(shape)
            self._guides.append((body, end_a, end_b))
 
    def update_bodies(self, q):
        if not self._guides:
            return
        for (body, end_a, end_b), (_, a, b, _) in zip(self._guides, self.segments(q)):
            pose, _ = segment_transform(a, b)
            body.T = pose.A
            end_a.T = SE3(*a.tolist()).A
            end_b.T = SE3(*b.tolist()).A
 