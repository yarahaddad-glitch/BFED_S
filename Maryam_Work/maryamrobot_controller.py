"""RS007N RMRC controller with joint-limit checks and approximate external obstacle avoidance."""
from pathlib import Path
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation


def stl_bounds(filename, unit_scale=1.0):
    data = Path(filename).read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL: {filename}")
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {filename}")
    dtype = np.dtype([("normal", "<f4", (3,)), ("xyz", "<f4", (3, 3)), ("attr", "<u2")])
    xyz = np.frombuffer(data, dtype=dtype, count=count, offset=84)["xyz"].reshape(-1, 3)
    xyz = xyz.astype(float) * float(unit_scale)
    return xyz.min(axis=0), xyz.max(axis=0)


class RobotController:
    def __init__(self, robot, stl_files, obstacles=None, *, stl_scale=1.0,
                 box_padding=0.008, clearance=0.015, dt=0.035,
                 barrier_y=None, base_stl=None):
        self.robot = robot
        self.extra_valid = None
        self.obstacles = list(obstacles or [])
        if not self.obstacles and barrier_y is None:
            print("[RMRC] No external obstacles configured; obstacle avoidance is inactive.")
        elif barrier_y is not None:
            print(f"[RMRC] Robot link bounds must stay behind Y={barrier_y:.3f}.")
        self.dt = dt
        self.barrier_y = barrier_y
        self.clearance = clearance
        self.joint_margin = np.deg2rad(5)

        # Cache mechanical limits once. Re-reading robot.qlim on every
        # RMRC sample is costly during multi-waypoint detour planning.
        mechanical_qlim = np.asarray(robot.qlim, dtype=float)
        if mechanical_qlim.shape != (2, robot.n):
            raise ValueError("Robot joint limits must have shape (2, n)")
        self.mechanical_min = mechanical_qlim[0].copy() + self.joint_margin
        self.mechanical_max = mechanical_qlim[1].copy() - self.joint_margin

        # Apply software limits from MaryamBot, if present.
        self.q_min = self.mechanical_min.copy()
        self.q_max = self.mechanical_max.copy()
        if hasattr(robot, "safe_qlim"):
            software_qlim = np.asarray(robot.safe_qlim, dtype=float)
            if software_qlim.shape != (2, robot.n):
                raise ValueError("safe_qlim must have shape (2, n)")
            self.q_min = np.maximum(self.q_min, software_qlim[0])
            self.q_max = np.minimum(self.q_max, software_qlim[1])
        if np.any(self.q_min >= self.q_max):
            raise ValueError("Mechanical and software joint limits do not overlap")
        self.boxes = []
        self.base_box = None
        if base_stl is not None:
            lo, hi = stl_bounds(base_stl, stl_scale)
            self.base_box = (hi - lo + 2 * box_padding, SE3(*((lo + hi) / 2)))
        home_frames = self.frames(np.zeros(robot.n))
        if len(stl_files) != robot.n:
            raise ValueError("Supply one moving-link STL for each robot joint")
        for i, filename in enumerate(stl_files):
            lo, hi = stl_bounds(filename, stl_scale)
            size = hi - lo + 2 * box_padding

            # STL vertices are assumed to be in the zero-configuration base frame.
            center_local = SE3(*((lo + hi) / 2).tolist())
            world_home = self.robot.base * center_local
            link_local_box = home_frames[i + 1].inv() * world_home
            self.boxes.append((size, link_local_box))

    def frames(self, q):
        T = SE3(self.robot.base.A)
        result = [T]
        for link, angle in zip(self.robot.links, q):
            T = T * SE3(link.A(float(angle)).A)
            result.append(T)
        return result

    def body_poses(self, q):
        frames = self.frames(q)
        return [(frames[i + 1] * local, size) for i, (size, local) in enumerate(self.boxes)]

    def tray_body_poses(self, q):
        """Tray clearance includes the fixed housing as well as moving links."""
        bodies = self.body_poses(q)
        if self.base_box is not None:
            size, local = self.base_box
            bodies = [(self.robot.base * local, size)] + bodies
        return bodies

    def colliding(self, q):
        """Approximate external obstacle check; not a self-collision checker."""
        if not self.obstacles and self.barrier_y is None:
            return False
        for body_pose, size in self.body_poses(q):
            if self.barrier_y is not None:
                max_y = body_pose.t[1] + np.abs(body_pose.R[1]) @ (size / 2)
                if max_y >= self.barrier_y:
                    return True
            for obs in self.obstacles:
                if obs["type"] == "sphere":
                    center = np.asarray(obs["center"], dtype=float)
                    local = body_pose.R.T @ (center - body_pose.t)
                    nearest = np.clip(local, -size / 2, size / 2)
                    if np.linalg.norm(local - nearest) <= float(obs["radius"]) + self.clearance:
                        return True
                elif obs["type"] == "box":
                    obstacle_pose = obs["pose"]
                    if not isinstance(obstacle_pose, SE3):
                        obstacle_pose = SE3(obstacle_pose)
                    radius = np.linalg.norm(np.asarray(obs["size"], dtype=float)) / 2
                    local = body_pose.R.T @ (obstacle_pose.t - body_pose.t)
                    nearest = np.clip(local, -size / 2, size / 2)
                    if np.linalg.norm(local - nearest) <= radius + self.clearance:
                        return True
                else:
                    raise ValueError(f"Unsupported obstacle: {obs['type']}")
        return False


    def valid(self, q):
        """
        Validate a joint configuration.

        Checks:
        1. Mechanical joint limits with 5-degree margin
        2. Conservative software joint limits
        3. External obstacle clearance

        Self-collision bounding-box checks are disabled.
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (self.robot.n,):
            return False

        if not np.all(np.isfinite(q)):
            return False

        # Compare against cached mechanical + software joint limits.
        if np.any(q < self.q_min) or np.any(q > self.q_max):
            return False

        # Check external obstacles
        if self.colliding(q):
            return False

        if self.extra_valid is not None and not self.extra_valid(q):
            return False

        return True


    @staticmethod
    def rotation_error(target_R, current_R):
        """World-frame rotation needed to move current orientation toward target."""
        return Rotation.from_matrix(target_R @ current_R.T).as_rotvec()

    def reached(self, q, target, hold_rotation):
        pose = self.robot.fkine(q)
        if np.linalg.norm(pose.t - target) >= 0.006:
            return False
        if hold_rotation is None:
            return True
        return np.linalg.norm(self.rotation_error(hold_rotation, pose.R)) < np.deg2rad(3)

    def segment(self, q0, target, *, steps=140, hold_rotation=None):
        """Plan RMRC in joint space without moving the displayed robot."""
        q = np.asarray(q0, dtype=float).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)) or not self.valid(q):
            return None
        if hold_rotation is not None:
            hold_rotation = np.asarray(hold_rotation, dtype=float)
            if (hold_rotation.shape != (3, 3)
                or not np.all(np.isfinite(hold_rotation))
                or not np.allclose(hold_rotation.T @ hold_rotation, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(hold_rotation), 1.0, atol=1e-5)):
                raise ValueError("hold_rotation must be a valid 3x3 rotation matrix")

        start = self.robot.fkine(q).t.copy()
        path = [q.copy()]
        for k in range(steps + 180):
            pose = self.robot.fkine(q)
            actual = pose.t
            if k >= steps and self.reached(q, target, hold_rotation):
                return path

            if k < steps:
                u = k / steps
                next_u = (k + 1) / steps
                blend = u*u*(3 - 2*u)
                next_blend = next_u*next_u*(3 - 2*next_u)
                desired = start + blend*(target - start)
                feedforward = ((next_blend - blend) / self.dt) * (target - start)
            else:
                desired = target
                feedforward = np.zeros(3)

            linear_velocity = feedforward + 2.0 * (desired - actual)
            if hold_rotation is None:
                J = self.robot.jacob0(q)[:3, :]
                velocity = linear_velocity
                lam = 0.015
                dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), velocity)
            else:
                J = self.robot.jacob0(q)
                angular_velocity = 2.0 * self.rotation_error(hold_rotation, pose.R)
                velocity = np.r_[linear_velocity, angular_velocity]
                lam = 0.03
                dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), velocity)

            if not np.all(np.isfinite(dq)):
                return None
            peak = np.max(np.abs(dq))
            if peak > 0.5:
                dq *= 0.5 / peak
            q_next = q + dq * self.dt
            for fraction in (0.25, 0.5, 0.75, 1.0):
                if not self.valid(q + fraction * (q_next - q)):
                    return None
            q = q_next
            path.append(q.copy())
        return path if self.reached(q, target, hold_rotation) else None

    def diagnose(self):
        """Report limits and external obstacle status for current pose."""
        q = np.asarray(self.robot.q, dtype=float)
        limits_ok = bool(np.all(np.isfinite(q))
                         and np.all(q >= self.mechanical_min)
                         and np.all(q <= self.mechanical_max))
        software_ok = self.robot.joints_safe(q) if hasattr(self.robot, "joints_safe") else "NOT CONFIGURED"
        print(f"Mechanical joint limits: {'OK' if limits_ok else 'BLOCKED'}")
        print(f"Software joint limits: {software_ok} (enforced if configured)")
        print(f"External obstacle collision: {'YES' if self.colliding(q) else 'NO'}")
        print(f"Controller configuration valid: {self.valid(q)}")
        print("Current EE XYZ:", np.round(self.robot.fkine(q).t, 4))

    def plan(self, target, hold_rotation=None, *, q_start=None):
        """Plan from the current or supplied joints without moving the robot."""
        q0 = np.asarray(self.robot.q if q_start is None else q_start, dtype=float).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Target must be a finite XYZ vector")
        if not self.valid(q0):
            return None, "START POSE BLOCKED (joint limits or external obstacle)"
        direct = self.segment(q0, target, hold_rotation=hold_rotation)
        if direct is not None:
            return direct, "DIRECT"

        start = self.robot.fkine(q0).t
        midpoint = (start + target) / 2
        for dz in (0.12, 0.20, 0.30):
            for dx, dy in ((0, 0), (0.14, 0), (-0.14, 0), (0, 0.14), (0, -0.14)):
                waypoint = midpoint + [dx, dy, dz]
                first = self.segment(q0, waypoint, hold_rotation=hold_rotation)
                if first is None:
                    continue
                second = self.segment(first[-1], target, hold_rotation=hold_rotation)
                if second is not None:
                    return first + second[1:], "DETOUR"

        rng = np.random.default_rng(17)
        for _ in range(45):
            fraction = rng.uniform(0.25, 0.75)
            waypoint = start + fraction * (target - start)
            waypoint += [rng.uniform(-0.35, 0.35),
                         rng.uniform(-0.35, 0.35), rng.uniform(0.12, 0.45)]
            first = self.segment(q0, waypoint, steps=110, hold_rotation=hold_rotation)
            if first is None:
                continue
            second = self.segment(first[-1], target, hold_rotation=hold_rotation)
            if second is not None:
                return first + second[1:], "ALTERNATIVE DETOUR"
        return None, "NO VERIFIED ROUTE (try a safe intermediate pose)"

    def move(self, target, step_callback=None, hold_rotation=None):
        path, method = self.plan(target, hold_rotation=hold_rotation)
        if path is None:
            print(method)
            self.diagnose()
            return False
        print(method)
        for q in path:
            if not self.valid(q):
                print("Blocked during execution; movement held")
                return False
            self.robot.q = q.copy()
            if step_callback:
                step_callback()
        return True
