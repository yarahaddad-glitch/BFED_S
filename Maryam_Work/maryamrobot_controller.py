"""Reusable position-RMRC controller with STL-based link boxes and obstacle checks."""
from pathlib import Path
import numpy as np
from spatialmath import SE3


def stl_bounds(filename, unit_scale=1.0):
    data = Path(filename).read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL: {filename}")
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {filename}")
    dtype = np.dtype([("normal", "<f4", (3,)), ("xyz", "<f4", (3, 3)), ("attr", "<u2")])
    xyz = np.frombuffer(data, dtype=dtype, count=count, offset=84)["xyz"].reshape(-1, 3)
    xyz = xyz.astype(float) * float(unit_scale)  # Set unit_scale=0.001 for millimetre STLs.
    return xyz.min(axis=0), xyz.max(axis=0)


class RobotController:
    def __init__(self, robot, stl_files, obstacles=None, *, stl_scale=1.0,
                 box_padding=0.008, clearance=0.015, dt=0.035):
        self.robot = robot
        self.obstacles = list(obstacles or [])  # Sphere: {center, radius}; box: {pose, size}.
        self.dt = dt
        self.clearance = clearance
        self.joint_margin = np.deg2rad(5)
        self.boxes = []
        home_frames = self.frames(np.zeros(robot.n))
        if len(stl_files) != robot.n:
            raise ValueError("Supply one moving-link STL for each robot joint")
        for i, filename in enumerate(stl_files):
            lo, hi = stl_bounds(filename, stl_scale)
            size = hi - lo + 2 * box_padding
            world_home = SE3(*((lo + hi) / 2).tolist())
            self.boxes.append((size, home_frames[i + 1].inv() * world_home))

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

    def colliding(self, q):
        for body_pose, size in self.body_poses(q):
            for obs in self.obstacles:
                if obs["type"] == "sphere":
                    center = np.asarray(obs["center"], dtype=float)
                    local = body_pose.R.T @ (center - body_pose.t)
                    nearest = np.clip(local, -size / 2, size / 2)
                    if np.linalg.norm(local - nearest) <= float(obs["radius"]) + self.clearance:
                        return True
                elif obs["type"] == "box":
                    # Conservative bounding-sphere test around registered box (may over-reject).
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
        return (np.all(q >= self.robot.qlim[0] + self.joint_margin)
                and np.all(q <= self.robot.qlim[1] - self.joint_margin)
                and not self.colliding(q))

    def segment(self, q0, target, *, steps=140):
        """Plan an RMRC trajectory without changing the displayed robot."""
        q = np.asarray(q0, dtype=float).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)) or not self.valid(q):
            return None
        start = self.robot.fkine(q).t.copy()
        path = [q.copy()]
        duration = steps * self.dt
        for k in range(steps + 180):
            actual = self.robot.fkine(q).t
            if k >= steps and np.linalg.norm(actual - target) < 0.006:
                return path
            if k < steps:
                u = k / steps
                next_u = (k + 1) / steps
                blend = u*u*(3 - 2*u)
                next_blend = next_u*next_u*(3 - 2*next_u)
                desired = start + blend*(target - start)
                feedforward = ((next_blend-blend) / self.dt) * (target-start)
            else:
                desired = target
                feedforward = np.zeros(3)
            J = self.robot.jacob0(q)[:3, :]
            lam = 0.015
            velocity = feedforward + 2.0*(desired-actual)
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2*np.eye(3), velocity)
            peak = np.max(np.abs(dq))
            if peak > 0.5:
                dq *= 0.5 / peak
            q_next = q + dq*self.dt
            for fraction in (0.25, 0.5, 0.75, 1.0):
                if not self.valid(q + fraction*(q_next-q)):
                    return None
            q = q_next
            path.append(q.copy())
        return path if np.linalg.norm(self.robot.fkine(q).t-target) < 0.006 else None

    def diagnose(self):
        q = np.asarray(self.robot.q, dtype=float)
        limits = np.all(q >= self.robot.qlim[0] + self.joint_margin) and np.all(q <= self.robot.qlim[1] - self.joint_margin)
        print(f"Start joint limits: {'OK' if limits else 'BLOCKED'}")
        print(f"Start collision: {'YES' if self.colliding(q) else 'NO'}")
        print("Current EE XYZ:", np.round(self.robot.fkine(q).t, 4))

    def plan(self, target):
        q0 = np.asarray(self.robot.q, dtype=float).copy()
        target = np.asarray(target, dtype=float)
        if not self.valid(q0):
            return None, "START POSE BLOCKED (joint limits or obstacle)"
        direct = self.segment(q0, target)
        if direct is not None:
            return direct, "DIRECT"
        start = self.robot.fkine(q0).t
        midpoint = (start + target) / 2
        # Small, bounded detour search; no route is guaranteed.
        for dz in (0.12, 0.20, 0.30):
            for dx, dy in ((0,0), (0.14,0), (-0.14,0), (0,0.14), (0,-0.14)):
                waypoint = midpoint + [dx, dy, dz]
                first = self.segment(q0, waypoint)
                if first is None:
                    continue
                second = self.segment(first[-1], target)
                if second is not None:
                    return first + second[1:], "DETOUR"
        return None, "NO VERIFIED ROUTE"

    def move(self, target, step_callback=None):
        path, method = self.plan(target)
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
