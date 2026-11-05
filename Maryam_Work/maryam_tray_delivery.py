# Tray pickup, carrying and placement for MaryamBot.
from __future__ import annotations

from dataclasses import dataclass
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from spatialmath import SE3
from Maryam_Work.collision_geometry import boxes_overlap


# Tray centre relative to the tray mesh origin, BEFORE scaling. Must match the
# centre_x / centre_y numbers inside the active Full_sim.add_tray_stacks().
TRAY_CENTRE_UNSCALED = np.array([0.15, -0.0989120895, 0.0])
TRAY_HALF_LENGTH_UNSCALED = 0.15
EDGE_GRIP_INSET = 0.0        # grip the rim; green needs the full reach


GRASP_HEIGHT = 0.10           # wrist above the rim, leaving room for corner gripping
APPROACH_HEIGHT = 0.28        # lifts tray above the 187 mm fixed housing before transfer
BELT_APPROACH_HEIGHT = 0.18   # hover height above the belt before lowering
RETREAT_HEIGHT = 0.18         # straight-up retreat after letting go

# Set-down tolerance
RELEASE_POS_TOL = 0.015       # metres
RELEASE_ROT_TOL_DEG = 3.0     # degrees

# Average speed used to size the carried RMRC segments
CARRY_SPEED = 0.10            # metres per second

# Gap between trays queued on the belt the trays are
# about 0.6 m long, so a second tray stops this far behind the first.
BELT_TRAY_SPACING = 0.65

STACK_COLOURS = ("green", "blue")



# TRAY STACKS
class TrayStacks:
    # Track the available tray meshes by colour, taking the top tray first.

    # Group tray names by colour and sort their numbered levels from top to bottom.
    def __init__(self, trays: dict):
        self.trays = trays
        self.available = {}
        for colour in STACK_COLOURS:
            keys = [k for k in trays if k.startswith(f"{colour}_tray_")]
            self.available[colour] = sorted(
                keys, key=lambda k: int(k.rsplit("_", 1)[1]), reverse=True)

    # Return the next tray's name and mesh, or None if no trays of that colour remain.
    def next_tray(self, colour: str):
        keys = self.available.get(colour.lower(), [])
        if not keys:
            return None
        return keys[0], self.trays[keys[0]]

    # Remove a successfully collected tray from available inventory; do not delete its mesh.
    def mark_used(self, key: str):
        for keys in self.available.values():
            if key in keys:
                keys.remove(key)

    # Count the trays still available for a colour.
    def remaining(self, colour: str) -> int:
        return len(self.available.get(colour.lower(), []))



# BELT LAYOUT + ANIMATION
@dataclass
class BeltLayout:
    # Store world positions of the tray's geometric centre and the belt speed.
    place_centre: np.ndarray     # where the robot sets the tray down
    middle_centre: np.ndarray    # where the belt carries it (middle, near the food area)
    speed: float = 0.15          # belt speed, metres per second


class ConveyorBelt:
    # Simple conveyor helper; the final meal route uses integrated_meal.carry_tray instead.

    # Store the scene, layout, mesh-centre offset, timestep and waiting-tray list.
    def __init__(self, env, layout: BeltLayout, tray_scale: float, dt: float):
        self.env = env
        self.layout = layout
        self.centre_local = TRAY_CENTRE_UNSCALED * tray_scale
        self.dt = dt
        self.on_belt = []             # trays currently queued on the belt

    # Clear this helper's queue after trays have been taken off the belt.
    def clear(self):
        self.on_belt.clear()

    # Slide a tray toward the middle, backing off for trays already in this helper's queue.
    def carry_to_middle(self, tray_mesh):
        pose = SE3(tray_mesh.T)
        start = pose.t.copy()

        # A tray already waiting at the middle? Stop one tray-length behind it.
        lane = self.layout.middle_centre - self.layout.place_centre
        lane_length = float(np.linalg.norm(lane))
        direction = lane / lane_length if lane_length > 1e-9 else np.zeros(3)
        back_off = BELT_TRAY_SPACING * len(self.on_belt)
        if back_off > lane_length - BELT_TRAY_SPACING:
            print("[BELT] Warning: belt is nearly full - call belt.clear() when "
                  "trays are taken off. This tray may overlap another.")
        stop_centre = self.layout.middle_centre - direction * back_off
        end = stop_centre - pose.R @ self.centre_local                 # mesh origin target
        self.on_belt.append(tray_mesh)
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(np.ceil(distance / (self.layout.speed * self.dt))))
        print(f"[BELT] Carrying tray {distance:.2f} m at {self.layout.speed} m/s",
              flush=True)
        for k in range(1, steps + 1):
            tray_mesh.T = SE3.Rt(pose.R, start + (end - start) * k / steps).A
            self.env.step(self.dt)
        print("[BELT] Tray is at the middle of the belt", flush=True)



# RMRC PICK AND PLACE
@dataclass
class DeliveryResult:
    # Return both the placement outcome and a readable explanation to the task coordinator.
    status: str      # PLACED / PLACED_OFF_TARGET / NOT_PICKED
    message: str

    # A tray may have been released even if its final pose is outside tolerance.
    @property
    def on_belt(self) -> bool:
        return self.status in ("PLACED", "PLACED_OFF_TARGET")


class TrayDelivery:
    # Plan a full transfer before moving, then approach, attach, carry, release and retreat.

    # Store tray geometry, the shared robot/controller, attachment state and optional hooks.
    def __init__(self, env, robot, controller, layout: BeltLayout,
                 tray_scale: float, return_home: bool = False,
                 animation_speed: float = 1.0, cache_dir=None):
        if not np.isfinite(animation_speed) or animation_speed <= 0:
            raise ValueError("animation_speed must be positive")
        self.env = env
        self.robot = robot
        self.controller = controller
        self.layout = layout
        self.centre_local = TRAY_CENTRE_UNSCALED * tray_scale
        self.return_home = return_home
        self.animation_speed = animation_speed
        self.half_width = 0.0989120895 * tray_scale
        self.edge_grip_offset = max(
            0.0, TRAY_HALF_LENGTH_UNSCALED * tray_scale - EDGE_GRIP_INSET)
        self.home_xyz = robot.fkine(robot.q).t.copy()
        self.hold_R = None            # None = free orientation
        self._carried = None          # tray mesh currently attached to the tool
        self._relative = None         # tray pose relative to the tool
        self.payload = []             # (shape, pose relative to tray) for meal props
        self.before_path = None
        self.wait_for_clearance = None
        self.remaining_path = None
        self.stationary_trays = []
        self.payload_check = None
        self.configuration_guard = None
        self.cache_dir = Path(cache_dir) if cache_dir else None
        self.cache_hits = 0
        self.cache_misses = 0
        self._grip_seeds = []
        # Old routes may be invalid after edits, but their grip joints are useful
        # numerical starting guesses. Every new route still gets full checks.
        if self.cache_dir is not None and self.cache_dir.exists():
            for path in self.cache_dir.glob("*.npz"):
                try:
                    with np.load(path, allow_pickle=False) as saved:
                        seed = saved["leg_0"][0].copy()
                    if seed.shape == (self.robot.n,) and np.all(np.isfinite(seed)):
                        self._grip_seeds.append(seed)
                except (OSError, ValueError, KeyError):
                    pass

    # Make a cache filename from source files and task settings, so changed inputs get a new key.
    def _cache_path(self, tray_pose, destination, waypoints, destination_rotation=None):
        if self.cache_dir is None:
            return None
        root = Path(__file__).resolve().parents[1]
        digest = hashlib.sha256()
        for filename in (
                "Maryam_Work/maryam_tray_delivery.py",
                "Maryam_Work/maryamrobot_controller.py",
                "Maryam_Work/Maryam_robotclass.py",
                "Maryam_Work/maryam_movements.py",
                "Collective_Folder/EnviroParts_S/Full_sim.py"):
            digest.update((root / filename).read_bytes())
        data = dict(tray=tray_pose.A.tolist(), destination=destination.tolist(),
                    destination_rotation=None if destination_rotation is None
                    else np.asarray(destination_rotation).tolist(),
                    waypoints=None if waypoints is None else np.asarray(waypoints).tolist(),
                    base=self.robot.base.A.tolist(), tool=self.robot.tool.A.tolist(),
                    qmin=self.controller.q_min.tolist(), qmax=self.controller.q_max.tolist(),
                    dt=self.controller.dt, barrier=self.controller.barrier_y,
                    obstacles=repr(self.controller.obstacles),
                    boxes=[(size.tolist(), pose.A.tolist())
                           for size, pose in self.controller.boxes])
        digest.update(json.dumps(data, sort_keys=True).encode())
        return self.cache_dir / f"{digest.hexdigest()}.npz"

    # Load saved carrying legs and build/check an empty-arm approach from the current joints.
    # prepare() performs further tray checks; execution also checks current constraints.
    def _cached_plan(self, path):
        if path is None or not path.is_file():
            return None
        try:
            with np.load(path, allow_pickle=False) as saved:
                legs = [(str(name), saved[f"leg_{i}"].copy())
                        for i, name in enumerate(saved["names"])]
                relative, hold_R = SE3(saved["relative"]), saved["hold_R"].copy()
            descent = list(reversed(legs[0][1]))
            q_start, q_above = self.robot.q.copy(), descent[0]
            count = max(2, int(np.ceil(
                np.max(np.abs(q_above - q_start)) / (0.35 * self.controller.dt))))
            approach = np.linspace(q_start, q_above, count + 1)
            if not all(self.controller.valid(q) for q in approach):
                return None
            self.cache_hits += 1
            print("[CACHE] Reusing saved route; no IK search.", flush=True)
            return approach, descent, legs, relative, hold_R
        except (OSError, ValueError, KeyError):
            return None

    # Save a successful plan and remember its grasp joints as a future IK starting guess.
    def _save_plan(self, path, plan):
        if path is None or plan is None:
            return
        _, _, legs, relative, hold_R = plan
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {f"leg_{i}": np.asarray(points) for i, (_, points) in enumerate(legs)}
        np.savez_compressed(path, names=np.array([name for name, _ in legs]),
                            relative=relative.A, hold_R=hold_R, **data)
        self._grip_seeds.append(np.asarray(legs[0][1][0]).copy())

    # ---- helpers ----
    # After an executed joint sample, update carried objects and advance the shared scene.
    def _step(self):
        self.update_payload()
        self.env.step(self.controller.dt / self.animation_speed)

    # Keep the tray fixed relative to the tool, and food/towels fixed relative to the tray.
    # Automatic, manual and recovery movement can all reuse this attachment update.
    def update_payload(self):
        if self._carried is not None:

            # FK gives the current world tool pose; _relative preserves the original rim grasp.
            self._carried.T = (self.robot.fkine(self.robot.q) * self._relative).A
            for shape, relative in self.payload:
                # Each item's saved tray-relative pose makes it follow without sliding visually.
                shape.T = (SE3(self._carried.T) * relative).A

    # Convenience route: ask controller.move() to plan/execute and call our scene update.
    def _move(self, xyz, label) -> bool:
        print(label, flush=True)
        ok = self.controller.move(np.asarray(xyz, dtype=float),
                                  step_callback=self._step,
                                  hold_rotation=self.hold_R)
        print(label, "OK" if ok else "FAILED", flush=True)
        return ok

    # Execute a preplanned joint path, checking the next configuration before assigning it.
    def _execute(self, path):
        callback = getattr(self, 'before_path', None)
        if callback:
            callback(path)
        # A newly inserted object must be checked against the whole pending leg,
        # including cached trajectories, before any of that leg is animated.
        if getattr(getattr(self, 'controller', None), 'obstacles', []):
            while not all(self.configuration_clear(q) for q in path[1:]):
                replan = getattr(self, 'replan_path', None)
                alternative = replan(path) if replan else None
                if alternative is not None:
                    path = alternative
                    break
                self._hold_for_clearance()
        for index, q in enumerate(path[1:], 1):
            self.remaining_path = path[index:]
            guard = getattr(self, 'configuration_guard', None)
            if guard:
                watched = getattr(self, '_watched_tray', None)
                pose = (None if watched is None else self.robot.fkine(q)*self._relative
                        if self._carried is not None else SE3(watched.T))
                guard(q, pose)
            while not self.configuration_clear(q):
                replan = getattr(self, 'replan_path', None)
                alternative = replan(path[index - 1:]) if replan else None
                if alternative is not None:
                    return self._execute(alternative)
                self._hold_for_clearance()
            # THIS is where a planned sample becomes the live robot configuration.
            self.robot.q = q.copy()
            self._step()
        self.remaining_path = None

    # Recovery/detours must pass ordinary checks and keep a carried tray within 8 degrees of level.
    def recovery_clear(self, q):
        if not self.configuration_clear(q):
            return False
        if self._carried is not None:
            pose = self.robot.fkine(q) * self._relative
            # Food has no simulated retaining fixture: keep the tray level.
            if pose.R[2, 2] < np.cos(np.deg2rad(8)):
                return False
            validator = getattr(self, '_carry_validator', None)
            if validator and not validator(pose):
                return False
        return True

    # Use the integrated wait/stop handler if supplied; otherwise report a blocked movement.
    def _hold_for_clearance(self):
        callback = getattr(self, 'wait_for_clearance', None)
        if callback is None:
            raise RuntimeError('Tray or arm would collide; stopped before contact')
        callback()

    # Combine arm, active-tray, waiting-tray and configured payload/obstacle checks.
    def configuration_clear(self, q):
        watched = getattr(self, '_watched_tray', None)
        if watched is not None:
            pose = (self.robot.fkine(q) * self._relative
                    if self._carried is not None else SE3(watched.T))
            if not self.tray_clears_robot(q, pose):
                return False
            if not self.queued_trays_clear(q, pose, watched):
                return False
            check = getattr(self, 'payload_check', None)
            if check is not None and not check(q, pose):
                return False
            if getattr(getattr(self, 'controller', None), 'obstacles', []):
                if not self.tray_clears_obstacles(pose):
                    return False
        controller = getattr(self, 'controller', None)
        return controller.valid(q) if controller is not None else True

    # Size the RMRC segment using travel distance, desired carry speed and controller timestep.
    def _steps_for(self, a, b) -> int:
        distance = float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
        return int(np.clip(distance / CARRY_SPEED / self.controller.dt, 140, 700))

    # Check a conservative tray/meal box against the arm and configured fixed housing box.
    def tray_clears_robot(self, q, tray_pose):
        height = .07 if getattr(self, 'payload', []) else .036
        centre = tray_pose.t + tray_pose.R @ (self.centre_local + [0, 0, height/2])
        half = np.array([self.edge_grip_offset, self.half_width, height/2])
        tray_box = SE3.Rt(tray_pose.R, centre)
        body_poses = getattr(self.controller, 'tray_body_poses', self.controller.body_poses)
        for index, (body, size) in enumerate(body_poses(q)):
            if boxes_overlap(tray_box, half * 2, body, size):
                self.last_tray_collision = ('fixed base' if index == 0 and
                    getattr(self.controller, 'base_box', None) is not None else f'body {index}')
                return False
        return True

    # Reject arm or carried-tray overlap with other visible trays waiting on the bench.
    def queued_trays_clear(self, q, carried_pose=None, active_tray=None):
        if not getattr(self, 'stationary_trays', []):
            return True
        size = np.array([2*self.edge_grip_offset, 2*self.half_width, .036])
        centre_offset = self.centre_local + [0, 0, .018]
        bodies = None
        for other in getattr(self, 'stationary_trays', []):
            if other is active_tray or np.asarray(other.T)[2, 3] < -10:
                continue
            pose = SE3(other.T)
            box = pose * SE3(*centre_offset)
            if carried_pose is not None:
                height = .07 if getattr(self, 'payload', []) else .036
                moving = carried_pose * SE3(*(self.centre_local + [0, 0, height/2]))
                moving_size = np.array([size[0], size[1], height])
                if boxes_overlap(moving, moving_size, box, size):
                    return False
            if bodies is None:
                bodies = self.controller.body_poses(q)
            if any(boxes_overlap(body, body_size, box, size) for body, body_size in bodies):
                return False
        return True

    # Plan the serving turn using many small pose targets and numerical IK, not segment() RMRC.
    def rotate_attached_tray(self, q_start, relative, angle, destination_centre=None):
        q = np.asarray(q_start, float).copy()
        initial = self.robot.fkine(q) * relative
        centre = initial.t + initial.R @ self.centre_local
        initial_tool = self.robot.fkine(q)
        final_R = SE3.Rz(float(angle)).R @ initial.R
        final_centre = centre if destination_centre is None else np.asarray(destination_centre)
        final_tray = SE3.Rt(final_R, final_centre-final_R@self.centre_local)
        final_tool = final_tray * relative.inv()
        path = [q.copy()]
        for fraction in np.linspace(0, 1, 91)[1:]:
            yaw = angle*fraction
            rotation = SE3.Rz(float(yaw)).R @ initial.R
            if destination_centre is None:
                tray = SE3.Rt(rotation, centre-rotation@self.centre_local)
                target = tray * relative.inv()
            else:
                tool_xyz = (1-fraction)*initial_tool.t + fraction*final_tool.t
                # Bow toward the belt while crossing the robot's centreline.
                tool_xyz[1] -= .15*np.sin(np.pi*fraction)
                target = SE3.Rt(rotation@relative.R.T, tool_xyz)

            # Measure tool position/orientation error at a candidate set of joint angles.
            def residual(candidate):
                actual = self.robot.fkine(candidate)
                return np.r_[actual.t-target.t,
                    .3*self.controller.rotation_error(target.R, actual.R)]

            # Start from the previous solution to encourage nearby joint solutions along the turn.
            solution = least_squares(residual, q,
                bounds=(self.controller.q_min, self.controller.q_max), max_nfev=70,
                ftol=1e-9, xtol=1e-9, gtol=1e-9)
            next_q = solution.x
            if np.linalg.norm(residual(next_q)) > .002:
                return None
            count = max(1, int(np.ceil(np.max(np.abs(next_q-q)) / .015)))
            for sample in np.linspace(q, next_q, count+1)[1:]:
                actual_tray = self.robot.fkine(sample) * relative
                if (not self.controller.valid(sample)
                        or not self.tray_clears_robot(sample, actual_tray)
                        or not self.queued_trays_clear(sample, actual_tray, self._planning_tray)):
                    return None
                path.append(sample.copy())
            q = next_q
        return path

    # Test the tray/meal envelope against configured boxes or spheres, including clearance.
    def tray_clears_obstacles(self, tray_pose):
        height = 0.07 if self.payload else 0.036
        centre = tray_pose.t + tray_pose.R @ (self.centre_local + [0, 0, height / 2])
        half = np.array([self.edge_grip_offset, self.half_width, height / 2])
        for obstacle in self.controller.obstacles:
            if obstacle['type'] == 'box':
                pose = obstacle['pose']
                pose = pose if isinstance(pose, SE3) else SE3(pose)
                if boxes_overlap(SE3.Rt(tray_pose.R, centre), half * 2,
                                 pose, obstacle['size'], margin=self.controller.clearance):
                    return False
                continue
            if obstacle['type'] == 'sphere':
                point = np.asarray(obstacle['center'], float)
                radius = float(obstacle['radius'])
            else:
                pose = obstacle['pose']
                point = pose.t if hasattr(pose, 't') else np.asarray(pose)[:3, 3]
                radius = np.linalg.norm(obstacle['size']) / 2
            local = tray_pose.R.T @ (point - centre)
            nearest = np.clip(local, -half, half)
            if np.linalg.norm(local - nearest) <= radius + self.controller.clearance:
                return False
        return True

    # Find usable grasp joints, then verify the carrying legs and approach before returning a plan.
    def _plan_pick_and_carry(self, tray_pose, grasp, above, place_origin,
                             waypoints=None, carry_validator=None, destination_rotation=None):
        controller = self.controller
        rejected = Counter()
        q_start = self.robot.q.copy()
        rng = np.random.default_rng(24)
        nearby = sorted(self._grip_seeds,
                        key=lambda q: np.linalg.norm(self.robot.fkine(q).t - grasp))
        # IK can converge to different elbow/wrist configurations: try several starting guesses.
        seeds = nearby[:4] + [q_start, np.zeros(self.robot.n)]
        seeds.extend(rng.uniform(controller.q_min, controller.q_max, (60, self.robot.n)))
        candidates = [(seed, None) for seed in seeds]
        # Explicit downward grips avoid arbitrary wrist poses from position-only IK.
        for yaw in (0, np.pi / 2, np.pi, -np.pi / 2):
            rotation = (SE3.Rz(yaw) * SE3.Rx(np.pi)).R
            candidates.extend((seed, rotation) for seed in seeds[:12])
        for attempt, (seed, rotation) in enumerate(candidates, 1):

            # SHOW_FK - Forward kinematics: use joint angles to calculate the tool pose.
            def residual(q):
                pose = self.robot.fkine(q)
                error = pose.t - grasp
                if rotation is not None:
                    # Some candidates also request a downward orientation; 0.3 weights its error.
                    error = np.r_[error, 0.3 * controller.rotation_error(rotation, pose.R)]
                return error

            # SHOW_IK - Inverse kinematics: find joint angles that reach the grasp target.
            # This is numerical inverse kinematics even though the function is not named ikine().
            solution = least_squares(
                residual,
                np.clip(seed, controller.q_min + 1e-6, controller.q_max - 1e-6),
                bounds=(controller.q_min, controller.q_max),
                max_nfev=180, gtol=1e-9,
            )
            # IK STEP 3: a solver result is only a candidate; check limits, clearance and accuracy.
            q_grasp = solution.x
            if not controller.valid(q_grasp) or not controller.reached(q_grasp, grasp, None):
                rejected["grip unreachable"] += 1
                continue
            ee = self.robot.fkine(q_grasp)
            hold_R = ee.R.copy()
            # Save where the tray sits relative to the tool at this grasp; do not snap it to the wrist.
            relative = ee.inv() * tray_pose
            if not self.tray_clears_robot(q_grasp, tray_pose):
                rejected["grip intersects tray"] += 1
                continue
            # Convert desired tray placement into tool targets using the grasp's rotated offset.
            ee_offset = hold_R @ relative.t
            final_R = tray_pose.R if destination_rotation is None else destination_rotation
            final_tool_R = final_R @ relative.R.T
            final_offset = final_tool_R @ relative.t
            targets = [
                ("lift", above),
                ("clear robot beside belt", place_origin + [
                    0.55 if tray_pose.t[0] + self.centre_local[0] > self.robot.base.t[0]
                    else -0.55, 0, BELT_APPROACH_HEIGHT] - ee_offset),
                ("over belt", place_origin + [0, 0, BELT_APPROACH_HEIGHT] - ee_offset),
                ("lower onto belt", place_origin - ee_offset),
            ]
            if waypoints is not None:
                targets = [("lift", above)]
                if destination_rotation is not None:
                    turn_centre = self.layout.place_centre + [0, -.30, .23]
                    targets.extend([
                        ('move to turning clearance', turn_centre - tray_pose.R @ self.centre_local - ee_offset),
                        ('rotate tray 90 degrees', None)])
                targets.extend(
                    (f"handout waypoint {i + 1}",
                     np.asarray(centre) - final_R @ self.centre_local - final_offset)
                    for i, centre in enumerate(waypoints))
                targets.append(("slide through serving gap", place_origin - final_offset))
            legs, q = [], q_grasp
            current_R = hold_R
            for name, target in targets:
                if name == 'rotate tray 90 degrees':
                    delta = final_R @ tray_pose.R.T
                    angle = np.arctan2(delta[1, 0], delta[0, 0])
                    path = self.rotate_attached_tray(q, relative, angle, waypoints[0])
                    if path is None:
                        rejected[name] += 1
                        break
                    legs.append((name, path))
                    q = path[-1]
                    current_R = final_tool_R
                    continue


                # SHOW_RMRC - Call the RMRC controller to plan this carrying leg's joint samples.
                # Holding tool orientation preserves the tray attitude through the fixed grasp.
                path = controller.segment(
                    q, target, steps=self._steps_for(self.robot.fkine(q).t, target),
                    hold_rotation=current_R)
                if path is None and name == "over belt":
                    midpoint = (self.robot.fkine(q).t + target) / 2
                    for offset in ([0, -0.30, 0.15], [0, 0.30, 0.15],
                                   [0, -0.45, 0.0], [0, 0, 0.30]):
                        waypoint = midpoint + offset
                        first = controller.segment(q, waypoint, steps=250,
                                                   hold_rotation=hold_R)
                        if first is None:
                            continue
                        second = controller.segment(first[-1], target, steps=250,
                                                    hold_rotation=hold_R)
                        if second is not None:
                            path = first + second[1:]
                            break
                if path is None:
                    rejected[name] += 1
                    break
                legs.append((name, path))
                q = path[-1]
            if len(legs) != len(targets):
                continue
            # A reachable tool route is not enough: predict the attached tray along every carry leg.
            bad_leg = next((name for name, path in legs for q in path
                            if not self.tray_clears_robot(q, self.robot.fkine(q) * relative)), None)
            if bad_leg is not None:
                rejected[f"tray intersects {self.last_tray_collision} during {bad_leg}"] += 1
                continue
            if not all(self.queued_trays_clear(q, self.robot.fkine(q)*relative, self._planning_tray)
                       for _, path in legs for q in path):
                rejected['carried tray intersects waiting trays'] += 1
                continue
            if carry_validator is not None and not all(
                    carry_validator(self.robot.fkine(q) * relative)
                    for _, path in legs for q in path):
                rejected["loaded tray would clip serving gap"] += 1
                continue
            placed = self.robot.fkine(q) * relative
            if (np.linalg.norm(placed.t - place_origin) > RELEASE_POS_TOL
                    or np.linalg.norm(controller.rotation_error(final_R, placed.R))
                    > np.deg2rad(RELEASE_ROT_TOL_DEG)):
                continue
            # Reverse the verified lift to descend with the chosen wrist pose.
            descent = list(reversed(legs[0][1]))
            q_above = descent[0]
            # Reconfigure while empty, checking interpolated joints and obstacles.
            count = max(2, int(np.ceil(
                np.max(np.abs(q_above - q_start)) / (0.35 * controller.dt))))
            approach = np.linspace(q_start, q_above, count + 1)
            if not all(controller.valid(q) for q in approach):
                continue
            if not all(self.tray_clears_robot(q, tray_pose)
                       for q in list(approach) + list(descent)):
                rejected["approach intersects tray"] += 1
                continue
            retreat_legs = legs[-2:] if waypoints is not None else legs[-1:]
            if not all(self.tray_clears_robot(q, placed)
                       for _, path in retreat_legs for q in path):
                rejected["retreat intersects tray"] += 1
                continue
            print(f"[PLAN] Complete pickup and carry verified (grip {attempt}).", flush=True)
            # No live joints moved: return the empty approach, descent, carry legs and grasp data.
            return approach, descent, legs, relative, hold_R
        print("[PLAN] Rejected routes:", dict(rejected), flush=True)
        return None

    # ---- main entry point ----
    # PLAN ONLY: choose rim targets, reuse/check a cached plan or search for a new transfer.
    def prepare(self, tray_mesh, destination=None, waypoints=None,
                carry_validator=None, destination_rotation=None):
        tray_pose = SE3(tray_mesh.T)
        self._planning_tray = tray_mesh
        # The mesh origin is offset from its visible centre; rotate that offset into the world.
        centre = tray_pose.t + tray_pose.R @ self.centre_local
        # Pick from the corner closest to the robot, not the middle of a rim.
        robot_local = tray_pose.R.T @ (self.robot.base.t - centre)
        rims = [np.array([x, y, 0.0])
                for x in (-self.edge_grip_offset, self.edge_grip_offset)
                for y in (-self.half_width, self.half_width)]
        rims.sort(key=lambda rim: np.linalg.norm((robot_local - rim)[:2]))
        rims = rims[:1]
        if centre[0] < self.robot.base.t[0]:
            # Preserve the working green-stack grip; the corner correction is
            # for the blue stack on the other side of the robot.
            rims = [np.array([self.edge_grip_offset, 0.0, 0.0])]
        if waypoints is not None:
            # Standard serving grip: trailing rim keeps the wrist inside the bars.
            rims = [np.array([x, -self.half_width, 0.0])
                    for x in (self.edge_grip_offset, 0.0, -self.edge_grip_offset)]
            if destination_rotation is not None:
                rims = [np.array([self.edge_grip_offset, y, 0.0])
                        for y in (self.half_width, 0.0, -self.half_width)]
        tray_at_grasp = tray_pose
        destination = (self.layout.place_centre if destination is None
                       else np.asarray(destination, dtype=float))
        final_R = tray_pose.R if destination_rotation is None else destination_rotation
        # The destination describes the tray centre; calculate where its mesh origin must go.
        place_origin = destination - final_R @ self.centre_local
        cache_path = self._cache_path(tray_pose, destination, waypoints, destination_rotation)
        cached = self._cached_plan(cache_path)
        if cached is not None:
            approach, descent, legs, relative, _ = cached
            placed = self.robot.fkine(legs[-1][1][-1]) * relative
            retreat = legs[-2:] if waypoints is not None else legs[-1:]
            if (not all(self.tray_clears_robot(q, tray_pose)
                        for q in list(approach) + list(descent))
                    or not all(self.tray_clears_robot(q, self.robot.fkine(q) * relative)
                               for _, path in legs for q in path)
                    or not all(self.tray_clears_robot(q, placed)
                               for _, path in retreat for q in path)
                    or not all(self.queued_trays_clear(q, self.robot.fkine(q)*relative, tray_mesh)
                               for _, path in legs for q in path)):
                cached = None
        if cached is not None:
            return (self.robot.q.copy(), tray_pose, place_origin, cached)
        self.cache_misses += 1
        print("[PLAN] Searching grip poses for the complete transfer...", flush=True)
        plan = None
        for edge_local in rims:
            print(f"[GRIP] Trying rim offset {np.round(edge_local, 3)}", flush=True)
            pickup = centre + tray_pose.R @ edge_local
            above = pickup + np.array([0.0, 0.0, APPROACH_HEIGHT])
            grasp = pickup + np.array([0.0, 0.0, GRASP_HEIGHT])
            # Fast necessary reach check for this DH chain, before costly IK.
            # The first joint's origin is fixed; the remaining links cannot
            # reach beyond the sum of their lengths in any configuration.
            shoulder = self.robot.base * self.robot.links[0].A(0.0)
            reach = sum(np.hypot(link.a, link.d) for link in self.robot.links[1:])
            reach += np.linalg.norm(self.robot.tool.t)
            final_tool = destination + final_R @ edge_local + [0, 0, GRASP_HEIGHT]
            if np.linalg.norm(final_tool - shoulder.t) > reach + RELEASE_POS_TOL:
                print("[GRIP] This rim puts the final tool position out of reach.",
                      flush=True)
                continue
            plan = self._plan_pick_and_carry(
                tray_pose, grasp, above, place_origin, waypoints=waypoints,
                carry_validator=carry_validator, destination_rotation=destination_rotation)
            if plan is not None:
                break
        self._save_plan(cache_path, plan)
        return (self.robot.q.copy(), tray_pose, place_origin, plan)

    # EXECUTE TRANSFER: prepare -> approach -> descend -> attach -> carry -> release -> retreat.
    def deliver(self, tray_mesh, label: str = "tray", destination=None,
                waypoints=None, prepared=None, carry_validator=None,
                destination_rotation=None) -> DeliveryResult:
        self.hold_R = None
        self._carry_validator = carry_validator
        self._carried = None
        if prepared is None:
            prepared = self.prepare(tray_mesh, destination, waypoints, carry_validator,
                                    destination_rotation)
        q_start, tray_at_grasp, place_origin, plan = prepared

        # A prepared trajectory is invalid if the robot or tray has moved since planning.
        if (not np.allclose(self.robot.q, q_start, atol=1e-8)
                or not np.allclose(tray_mesh.T, tray_at_grasp.A, atol=1e-8)):
            return DeliveryResult("NOT_PICKED", "Prepared route is stale; scene changed")
        if plan is None:
            return DeliveryResult("NOT_PICKED", "No verified pickup and carry configuration")
        approach, descent, legs, relative, hold_R = plan
        self._watched_tray = tray_mesh
        print("[PICK] Approach", flush=True)
        self._execute(approach)
        print("[PICK] Descend to edge", flush=True)
        self._execute(descent)

        # Attach only after reaching the planned grasp
        self._carried, self._relative, self.hold_R = tray_mesh, relative, hold_R
        self._step()
        for name, path in legs:
            print(f"[CARRY] {name}", flush=True)
            self._execute(path)

        # Use the achieved FK pose to measure placement accuracy, not the planned target
        delivered = self.robot.fkine(self.robot.q) * relative
        position_error = float(np.linalg.norm(delivered.t - place_origin))
        rotation_error = float(np.linalg.norm(
            self.controller.rotation_error(tray_at_grasp.R if destination_rotation is None
                                           else destination_rotation, delivered.R)))
        print(f"[PLACE] Position error {position_error:.4f} m, "
              f"rotation error {np.rad2deg(rotation_error):.2f} deg")
        tray_mesh.T = delivered.A
        self._carried = None                      # released: tray stays here
        on_target = (position_error <= RELEASE_POS_TOL
                     and rotation_error <= np.deg2rad(RELEASE_ROT_TOL_DEG))
        print(f"[PLACE] {label} released at destination", flush=True)

        # Reuse the final leg(s) in reverse for the empty-arm retreat after releasing the tray.
        if waypoints is not None:
            # Withdraw through the opening before raising the empty gripper.
            self._execute(list(reversed(legs[-1][1])))
            self._execute(list(reversed(legs[-2][1])))
        else:
            self._execute(list(reversed(legs[-1][1])))
        print("[RETREAT] Reused the verified approach path.", flush=True)
        self.hold_R = None
        if self.return_home:
            self._move(self.home_xyz, "Return home")

        if on_target:
            return DeliveryResult("PLACED", "Tray placed at destination")
        return DeliveryResult("PLACED_OFF_TARGET",
                              "Tray released but outside the placement tolerance")
