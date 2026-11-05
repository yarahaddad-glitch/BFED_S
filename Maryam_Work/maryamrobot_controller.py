from pathlib import Path
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation
from Maryam_Work.collision_geometry import boxes_overlap


# Read the STL vertices and return the mesh's scaled minimum and maximum XYZ bounds.
# Read STL bounds used to build approximate link collision boxes.
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
    # Set up joint limits, obstacles and padded collision boxes for each robot link.
    def __init__(self, robot, stl_files, obstacles=None, *, stl_scale=1.0,
                 box_padding=0.008, clearance=0.015, dt=0.035,
                 barrier_y=None, base_stl=None):
        self.robot = robot
        # Hooks supplied by the integrated application: extra configuration checks
        # and a runtime checkpoint that can service GUI commands or wait on a stop.
        self.extra_valid = None
        self.checkpoint = None
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
        # Combine model limits with a 5-degree margin and smaller software limits.
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
        # Build geometry once at zero joints; frames(q) will move it mathematically later.
        home_frames = self.frames(np.zeros(robot.n))
        if len(stl_files) != robot.n:
            raise ValueError("Supply one moving-link STL for each robot joint")
        for i, filename in enumerate(stl_files):
            lo, hi = stl_bounds(filename, stl_scale)
            size = hi - lo + 2 * box_padding

            # STL vertices are assumed to be in the zero-configuration base frame.
            center_local = SE3(*((lo + hi) / 2).tolist())
            world_home = self.robot.base * center_local
            # Save the box relative to its link, rather than at a fixed world position.
            link_local_box = home_frames[i + 1].inv() * world_home
            self.boxes.append((size, link_local_box))

    # Multiply the DH transforms to calculate every link frame in world coordinates.
    # Multiply each link's DH transform to find the individual link frames.
    def frames(self, q):
        T = SE3(self.robot.base.A)
        result = [T]
        for link, angle in zip(self.robot.links, q):
            # link.A(angle) includes that link's DH dimensions and joint offset.
            T = T * SE3(link.A(float(angle)).A)
            result.append(T)
        return result

    # Calculate the moving link collision-box poses at the proposed joint angles.
    def body_poses(self, q):
        frames = self.frames(q)
        # World link pose * link-relative box pose = world collision-box pose.
        return [(frames[i + 1] * local, size) for i, (size, local) in enumerate(self.boxes)]

    # Include the fixed housing and moving links when checking tray clearance.
    def tray_body_poses(self, q):
        bodies = self.body_poses(q)
        if self.base_box is not None:
            size, local = self.base_box
            bodies = [(self.robot.base * local, size)] + bodies
        return bodies

    # Check link boxes against the serving plane and sphere/box obstacles.
    # SHOW_COLLISION - Check link boxes against the serving plane and configured obstacles.
    # This checks approximate external collisions, not complete arm self-collision.
    def colliding(self, q):
        self.last_collision = ''
        if not self.obstacles and self.barrier_y is None:
            return False
        for index, (body_pose, size) in enumerate(self.body_poses(q), 1):
            if self.barrier_y is not None:
                # Project the rotated box's half-extents onto world Y, so the whole
                # link box must stay behind the plane, not just its centre or the tool.
                max_y = body_pose.t[1] + np.abs(body_pose.R[1]) @ (size / 2)
                if max_y >= self.barrier_y:
                    self.last_collision = f'link {index} crosses the serving safety plane'
                    return True
            for obs in self.obstacles:
                if obs["type"] == "sphere":
                    center = np.asarray(obs["center"], dtype=float)
                    # Express the sphere centre in box coordinates, find the nearest
                    # point on/in the box, then compare distance with radius + clearance.
                    local = body_pose.R.T @ (center - body_pose.t)
                    nearest = np.clip(local, -size / 2, size / 2)
                    if np.linalg.norm(local - nearest) <= float(obs["radius"]) + self.clearance:
                        self.last_collision = f'link {index} intersects sphere'
                        return True
                elif obs["type"] == "box":
                    obstacle_pose = obs["pose"]
                    if not isinstance(obstacle_pose, SE3):
                        obstacle_pose = SE3(obstacle_pose)
                    # collision_geometry.boxes_overlap uses the Separating Axis Theorem.
                    if boxes_overlap(body_pose, size, obstacle_pose, obs['size'],
                                     margin=self.clearance):
                        self.last_collision = f'link {index} intersects cube/box'
                        return True
                else:
                    raise ValueError(f"Unsupported obstacle: {obs['type']}")
        return False


    # Reject invalid joint values, exceeded limits, collisions or failed extra checks.
    # Reject invalid joints, exceeded limits, collisions or failed extra checks.
    def valid(self, q):
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

        # For example, the tray module adds waiting-tray clearance through this hook.
        if self.extra_valid is not None and not self.extra_valid(q):
            return False

        return True


    # Calculate the rotation vector needed to turn the current orientation to the target.
    @staticmethod
    def rotation_error(target_R, current_R):
        # A rotation vector's direction is the correction axis; its length is radians.
        return Rotation.from_matrix(target_R @ current_R.T).as_rotvec()

    # Check tool position within 6 mm and, if requested, orientation within 3 degrees.
    # Accept tool error below 6 mm and, when requested, 3 degrees.
    def reached(self, q, target, hold_rotation):
        pose = self.robot.fkine(q)
        if np.linalg.norm(pose.t - target) >= 0.006:
            return False
        if hold_rotation is None:
            return True
        return np.linalg.norm(self.rotation_error(hold_rotation, pose.R)) < np.deg2rad(3)

    # Plan a Cartesian motion using damped Jacobian RMRC and checked joint samples.
    # Return the path, or None if planning fails; the displayed robot does not move here.
    # SHOW_RMRC - Plan tool motion using FK feedback, a damped Jacobian and joint integration.
    # Returns joint samples; TrayDelivery._execute() runs the final tray task's samples.
    def segment(self, q0, target, *, steps=140, hold_rotation=None):

        # INPUTS: q0 = starting joint angles in radians; target = tool XYZ in metres.
        q = np.asarray(q0, dtype=float).copy()
        target = np.asarray(target, dtype=float)
        # Reject malformed/non-finite XYZ or a blocked starting configuration.
        if target.shape != (3,) or not np.all(np.isfinite(target)) or not self.valid(q):
            return None
        if hold_rotation is not None:
            hold_rotation = np.asarray(hold_rotation, dtype=float)
            # A proper rotation has orthonormal columns and determinant +1.
            if (hold_rotation.shape != (3, 3)
                or not np.all(np.isfinite(hold_rotation))
                or not np.allclose(hold_rotation.T @ hold_rotation, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(hold_rotation), 1.0, atol=1e-5)):
                raise ValueError("hold_rotation must be a valid 3x3 rotation matrix")

        # RMRC STEP 1 - FORWARD KINEMATICS: calculate the starting tool position.
        start = self.robot.fkine(q).t.copy()
        path = [q.copy()]
        # Allow up to 180 extra iterations after the reference ends to settle the error.
        for k in range(steps + 180):
            if self.checkpoint and k % 16 == 0:
                self.checkpoint()
            # FK provides simulated feedback at the current PLANNED joints.
            # This is model feedback, not webcam or physical encoder feedback.
            # SHOW_FK - Calculate the current planned tool pose from joint angles using the DH model.
            pose = self.robot.fkine(q)
            actual = pose.t
            if k >= steps and self.reached(q, target, hold_rotation):
                return path

            if k < steps:
                # RMRC STEP 2 - REFERENCE: cubic smoothstep moves from start to target.
                # Its zero endpoint slopes soften the reference's start and finish.
                u = k / steps
                next_u = (k + 1) / steps
                # Smooth the reference from the starting point to the target.
                blend = u*u*(3 - 2*u)
                next_blend = next_u*next_u*(3 - 2*next_u)
                desired = start + blend*(target - start)
                # Feedforward velocity follows the reference's next position increment.
                feedforward = ((next_blend - blend) / self.dt) * (target - start)
            else:
                # Reference finished: hold the target and use error correction only.
                desired = target
                feedforward = np.zeros(3)

            # RMRC STEP 3 - TOOL VELOCITY: reference velocity + proportional correction.
            # desired - actual is position error; 2.0 is the proportional gain.
            # Follow the reference and correct simulated position error.
            linear_velocity = feedforward + 2.0 * (desired - actual)
            if hold_rotation is None:

                # RMRC STEP 4 - JACOBIAN: Control XYZ position only; tool orientation is free to change.
                # These three rows control XYZ motion; orientation is free.
                J = self.robot.jacob0(q)[:3, :]
                velocity = linear_velocity
                lam = 0.015

              # Calculate how fast each joint should move to produce the requested tool velocity.
              # Damping reduces very large joint speeds near singularities. dq stores joint speeds.
                # SHOW_DAMPING - Calculate joint speeds while reducing excessive rates near singularities.
                dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), velocity)
            else:
                 # Control position and orientation to keep the attached tray level during carrying
                # SHOW_ORIENTATION - Use all six rows to control position and maintain tool attitude.
                J = self.robot.jacob0(q)
                angular_velocity = 2.0 * self.rotation_error(hold_rotation, pose.R)


                # Stack XYZ velocity and angular velocity into a six-component twist.
                velocity = np.r_[linear_velocity, angular_velocity]
                lam = 0.03
                dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), velocity)

            if not np.all(np.isfinite(dq)):
                return None
            # Scale all joint speeds together if any exceeds 0.5 rad/s in controller time.
            peak = np.max(np.abs(dq))
            if peak > 0.5:
                # Scale all rates together to preserve their ratios; cap at 0.5 rad/s in controller time.
                dq *= 0.5 / peak

            # RMRC STEP 5 - INTEGRATE: joint angle + joint rate * dt (Euler integration)
            # Use joint speeds and timestep to calculate the next joint angles.
            q_next = q + dq * self.dt

            # RMRC STEP 6 - VALIDATE: reject a blocked step before adding it to the path.
            # Check intermediate configurations before accepting the next sample.
            for fraction in (0.25, 0.5, 0.75, 1.0):
                if not self.valid(q + fraction * (q_next - q)):
                    return None
            q = q_next
            path.append(q.copy())
        # OUTPUT: a list of joint configurations, or None if convergence/checks failed.
        return path if self.reached(q, target, hold_rotation) else None

    # Print current limits, collision status and tool position to help explain failures.
    def diagnose(self):
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

    # Try a direct RMRC route, then waypoint detours, without moving the robot.
    # Return the planned joint path and a description of the result.
    # Try direct RMRC, then Cartesian waypoints if it fails; this is not RRT.
    def plan(self, target, hold_rotation=None, *, q_start=None):
        # Start from live joints unless another planning start is explicitly supplied.
        q0 = np.asarray(self.robot.q if q_start is None else q_start, dtype=float).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Target must be a finite XYZ vector")
        if not self.valid(q0):
            return None, "START POSE BLOCKED (joint limits or external obstacle)"
        direct = self.segment(q0, target, hold_rotation=hold_rotation)
        if direct is not None:
            return direct, "DIRECT"

        # A failed segment may be blocked or fail to converge. Try raised/side waypoints;
        # each candidate needs two successful segments: start -> waypoint -> target.
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
                    # Skip the repeated waypoint joint sample when joining the two lists.
                    return first + second[1:], "DETOUR"

        # Reproducible extra waypoint guesses, not the RRT planner in joint_planner.py.
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

    # Execute the planned joints, recheck each sample and call the scene-update callback.
    # Plan, recheck and assign live joints; call the supplied scene/payload update.
    def move(self, target, step_callback=None, hold_rotation=None):
        # Unlike segment()/plan(), this function changes the live robot's joints.
        path, method = self.plan(target, hold_rotation=hold_rotation)
        if path is None:
            print(method)
            self.diagnose()
            return False
        print(method)
        for q in path:
            # Recheck because configured obstacles/constraints may change after planning.
            if not self.valid(q):
                print("Blocked during execution; movement held")
                return False
            self.robot.q = q.copy()
            if step_callback:
                # Caller updates carried items and advances Swift; this file does not
                # itself attach trays or call env.step(). TrayDelivery also has its own
                # _execute() route for executing preplanned multi-leg transfers.
                step_callback()
        return True
