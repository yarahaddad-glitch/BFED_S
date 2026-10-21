"""Tray delivery for the Kawasaki RS007N (MaryamBot).

Pick one tray from its stack with RMRC, set it on the conveyor belt, then
animate the belt carrying it to the middle (towards the food area).

Classes
  TrayStacks     which trays are still left in each coloured stack
  BeltLayout     where the tray is set down and where the belt carries it
  TrayDelivery   RMRC pick-up of one tray and set-down on the belt
  ConveyorBelt   animates a tray along the belt (visual only)

Put this file in Maryam_Work next to maryamrobot_controller.py.
Everything you are likely to tune is in the CONFIG block below.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from spatialmath import SE3

# ============================================================
# CONFIG
# ============================================================
# Tray centre relative to the tray mesh origin, BEFORE scaling. Must match the
# centre_x / centre_y numbers inside add_tray_stacks() in swift_simulation.py.
TRAY_CENTRE_UNSCALED = np.array([0.15, -0.0989120895, 0.0])
TRAY_HALF_LENGTH_UNSCALED = 0.15
EDGE_GRIP_INSET = 0.0        # grip the rim; green needs the full reach

# Heights in metres (same idea as your working green-tray pickup).
GRASP_HEIGHT = 0.10           # wrist above the rim, leaving room for corner gripping
APPROACH_HEIGHT = 0.18        # hover height above the tray
BELT_APPROACH_HEIGHT = 0.18   # hover height above the belt before lowering
RETREAT_HEIGHT = 0.18         # straight-up retreat after letting go

# Set-down tolerance (same as before).
RELEASE_POS_TOL = 0.015       # metres
RELEASE_ROT_TOL_DEG = 3.0     # degrees

# Average speed used to size the carried RMRC segments (slower = more steps).
CARRY_SPEED = 0.10            # metres per second

# Gap between trays queued on the belt (centre to centre, metres). Trays are
# about 0.6 m long, so a second tray stops this far behind the first.
BELT_TRAY_SPACING = 0.65

STACK_COLOURS = ("green", "blue")


# ============================================================
# TRAY STACKS
# ============================================================
class TrayStacks:
    """Tracks which trays are left in each stack (top tray is taken first).

    trays: the dict returned by add_tray_stacks(), e.g. {"green_tray_1": mesh, ...}
    """

    def __init__(self, trays: dict):
        self.trays = trays
        self.available = {}
        for colour in STACK_COLOURS:
            keys = [k for k in trays if k.startswith(f"{colour}_tray_")]
            self.available[colour] = sorted(
                keys, key=lambda k: int(k.rsplit("_", 1)[1]), reverse=True)

    def next_tray(self, colour: str):
        """Return (key, mesh) of the top tray, or None if the stack is empty."""
        keys = self.available.get(colour.lower(), [])
        if not keys:
            return None
        return keys[0], self.trays[keys[0]]

    def mark_used(self, key: str):
        for keys in self.available.values():
            if key in keys:
                keys.remove(key)

    def remaining(self, colour: str) -> int:
        return len(self.available.get(colour.lower(), []))


# ============================================================
# BELT LAYOUT + ANIMATION
# ============================================================
@dataclass
class BeltLayout:
    """World positions of the TRAY CENTRE (bottom of the tray, on the belt)."""
    place_centre: np.ndarray     # where the robot sets the tray down
    middle_centre: np.ndarray    # where the belt carries it (middle, near the food area)
    speed: float = 0.15          # belt speed, metres per second


class ConveyorBelt:
    """Visual belt: slides the tray from the set-down point to the middle."""

    def __init__(self, env, layout: BeltLayout, tray_scale: float, dt: float):
        self.env = env
        self.layout = layout
        self.centre_local = TRAY_CENTRE_UNSCALED * tray_scale
        self.dt = dt
        self.on_belt = []             # trays currently queued on the belt

    def clear(self):
        """Call when the trays have been taken off the belt (e.g. by Yara's robot)."""
        self.on_belt.clear()

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


# ============================================================
# RMRC PICK AND PLACE
# ============================================================
@dataclass
class DeliveryResult:
    status: str      # PLACED / PLACED_OFF_TARGET / NOT_PICKED
    message: str

    @property
    def on_belt(self) -> bool:
        return self.status in ("PLACED", "PLACED_OFF_TARGET")


class TrayDelivery:
    """Uses your RobotController (RMRC) to move one tray onto the belt.

    Sequence
      1. Search grip configurations and plan approach, descent and every carry leg.
         If no complete route is found, nothing moves.
      2. Approach empty, then descend with the verified grip orientation.
      3. Attach the tray, carry it, check the set-down pose, release
      4. Retreat straight up (orientation still held)
    """

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

    def _cache_path(self, tray_pose, destination, waypoints):
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
                    waypoints=None if waypoints is None else np.asarray(waypoints).tolist(),
                    base=self.robot.base.A.tolist(), tool=self.robot.tool.A.tolist(),
                    qmin=self.controller.q_min.tolist(), qmax=self.controller.q_max.tolist(),
                    dt=self.controller.dt, barrier=self.controller.barrier_y,
                    obstacles=repr(self.controller.obstacles),
                    boxes=[(size.tolist(), pose.A.tolist())
                           for size, pose in self.controller.boxes])
        digest.update(json.dumps(data, sort_keys=True).encode())
        return self.cache_dir / f"{digest.hexdigest()}.npz"

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
    def _step(self):
        """Called every RMRC sample: keeps a carried tray attached to the tool."""
        if self._carried is not None:
            self._carried.T = (self.robot.fkine(self.robot.q) * self._relative).A
            for shape, relative in self.payload:
                shape.T = (SE3(self._carried.T) * relative).A
        self.env.step(self.controller.dt / self.animation_speed)

    def _move(self, xyz, label) -> bool:
        print(label, flush=True)
        ok = self.controller.move(np.asarray(xyz, dtype=float),
                                  step_callback=self._step,
                                  hold_rotation=self.hold_R)
        print(label, "OK" if ok else "FAILED", flush=True)
        return ok

    def _execute(self, path):
        for q in path[1:]:
            self.robot.q = q.copy()
            self._step()

    def _steps_for(self, a, b) -> int:
        distance = float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
        return int(np.clip(distance / CARRY_SPEED / self.controller.dt, 140, 700))

    def tray_clears_robot(self, q, tray_pose):
        """Separating-axis check of tray volume against every moving link box."""
        centre = tray_pose.t + tray_pose.R @ (self.centre_local + [0, 0, 0.015])
        half = np.array([self.edge_grip_offset, self.half_width, 0.015])
        for body, size in self.controller.body_poses(q):
            axes = [*tray_pose.R.T, *body.R.T]
            axes.extend(np.cross(a, b) for a in tray_pose.R.T for b in body.R.T)
            delta = body.t - centre
            separated = False
            for axis in axes:
                length = np.linalg.norm(axis)
                if length < 1e-8:
                    continue
                axis = axis / length
                radius = (np.abs(axis @ tray_pose.R) @ half
                          + np.abs(axis @ body.R) @ (size / 2))
                if abs(axis @ delta) > radius + 0.002:
                    separated = True
                    break
            if not separated:
                return False
        return True

    def _plan_pick_and_carry(self, tray_pose, grasp, above, place_origin,
                             waypoints=None, carry_validator=None):
        """Search grip configurations and verify the whole route before moving."""
        controller = self.controller
        rejected = Counter()
        q_start = self.robot.q.copy()
        rng = np.random.default_rng(24)
        nearby = sorted(self._grip_seeds,
                        key=lambda q: np.linalg.norm(self.robot.fkine(q).t - grasp))
        seeds = nearby[:4] + [q_start, np.zeros(self.robot.n)]
        seeds.extend(rng.uniform(controller.q_min, controller.q_max, (60, self.robot.n)))
        candidates = [(seed, None) for seed in seeds]
        # Explicit downward grips avoid arbitrary wrist poses from position-only IK.
        for yaw in (0, np.pi / 2, np.pi, -np.pi / 2):
            rotation = (SE3.Rz(yaw) * SE3.Rx(np.pi)).R
            candidates.extend((seed, rotation) for seed in seeds[:12])
        for attempt, (seed, rotation) in enumerate(candidates, 1):
            def residual(q):
                pose = self.robot.fkine(q)
                error = pose.t - grasp
                if rotation is not None:
                    error = np.r_[error, 0.3 * controller.rotation_error(rotation, pose.R)]
                return error
            solution = least_squares(
                residual,
                np.clip(seed, controller.q_min + 1e-6, controller.q_max - 1e-6),
                bounds=(controller.q_min, controller.q_max),
                max_nfev=180, gtol=1e-9,
            )
            q_grasp = solution.x
            if not controller.valid(q_grasp) or not controller.reached(q_grasp, grasp, None):
                rejected["grip unreachable"] += 1
                continue
            ee = self.robot.fkine(q_grasp)
            hold_R = ee.R.copy()
            relative = ee.inv() * tray_pose
            if waypoints is None and not self.tray_clears_robot(q_grasp, tray_pose):
                rejected["grip intersects tray"] += 1
                continue
            ee_offset = hold_R @ relative.t
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
                targets.extend(
                    (f"handout waypoint {i + 1}",
                     np.asarray(centre) - tray_pose.R @ self.centre_local - ee_offset)
                    for i, centre in enumerate(waypoints))
                targets.append(("slide through serving gap", place_origin - ee_offset))
            legs, q = [], q_grasp
            for name, target in targets:
                path = controller.segment(
                    q, target, steps=self._steps_for(self.robot.fkine(q).t, target),
                    hold_rotation=hold_R)
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
            if waypoints is None and not all(self.tray_clears_robot(q, self.robot.fkine(q) * relative)
                       for _, path in legs for q in path):
                rejected["carried tray intersects robot"] += 1
                continue
            if carry_validator is not None and not all(
                    carry_validator(self.robot.fkine(q) * relative)
                    for _, path in legs for q in path):
                rejected["loaded tray would clip serving gap"] += 1
                continue
            placed = self.robot.fkine(q) * relative
            if (np.linalg.norm(placed.t - place_origin) > RELEASE_POS_TOL
                    or np.linalg.norm(controller.rotation_error(tray_pose.R, placed.R))
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
            if waypoints is None and not all(self.tray_clears_robot(q, tray_pose)
                       for q in list(approach) + list(descent)):
                rejected["approach intersects tray"] += 1
                continue
            print(f"[PLAN] Complete pickup and carry verified (grip {attempt}).", flush=True)
            return approach, descent, legs, relative, hold_R
        print("[PLAN] Rejected routes:", dict(rejected), flush=True)
        return None

    # ---- main entry point ----
    def prepare(self, tray_mesh, destination=None, waypoints=None,
                carry_validator=None):
        """Plan without moving; callers can prepare both colours before scanning."""
        tray_pose = SE3(tray_mesh.T)
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
            rims = [np.array([0.0, -self.half_width, 0.0])]
        tray_at_grasp = tray_pose
        destination = (self.layout.place_centre if destination is None
                       else np.asarray(destination, dtype=float))
        place_origin = destination - tray_at_grasp.R @ self.centre_local
        cache_path = self._cache_path(tray_pose, destination, waypoints)
        cached = self._cached_plan(cache_path)
        if waypoints is None and cached is not None and not all(
                self.tray_clears_robot(q, tray_pose) for q in cached[0]):
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
            final_tool = destination + (grasp - centre)
            if np.linalg.norm(final_tool - shoulder.t) > reach + RELEASE_POS_TOL:
                print("[GRIP] This rim puts the final tool position out of reach.",
                      flush=True)
                continue
            plan = self._plan_pick_and_carry(
                tray_pose, grasp, above, place_origin, waypoints=waypoints,
                carry_validator=carry_validator)
            if plan is not None:
                break
        self._save_plan(cache_path, plan)
        return (self.robot.q.copy(), tray_pose, place_origin, plan)

    def deliver(self, tray_mesh, label: str = "tray", destination=None,
                waypoints=None, prepared=None, carry_validator=None) -> DeliveryResult:
        self.hold_R = None
        self._carried = None
        if prepared is None:
            prepared = self.prepare(tray_mesh, destination, waypoints, carry_validator)
        q_start, tray_at_grasp, place_origin, plan = prepared
        if (not np.allclose(self.robot.q, q_start, atol=1e-8)
                or not np.allclose(tray_mesh.T, tray_at_grasp.A, atol=1e-8)):
            return DeliveryResult("NOT_PICKED", "Prepared route is stale; scene changed")
        if plan is None:
            return DeliveryResult("NOT_PICKED", "No verified pickup and carry configuration")
        approach, descent, legs, relative, hold_R = plan
        print("[PICK] Approach", flush=True)
        self._execute(approach)
        print("[PICK] Descend to edge", flush=True)
        self._execute(descent)

        # 3. Commit: attach, carry, release
        self._carried, self._relative, self.hold_R = tray_mesh, relative, hold_R
        self._step()
        for name, path in legs:
            print(f"[CARRY] {name}", flush=True)
            self._execute(path)

        delivered = self.robot.fkine(self.robot.q) * relative
        position_error = float(np.linalg.norm(delivered.t - place_origin))
        rotation_error = float(np.linalg.norm(
            self.controller.rotation_error(tray_at_grasp.R, delivered.R)))
        print(f"[PLACE] Position error {position_error:.4f} m, "
              f"rotation error {np.rad2deg(rotation_error):.2f} deg")
        tray_mesh.T = delivered.A
        self._carried = None                      # released: tray stays here
        on_target = (position_error <= RELEASE_POS_TOL
                     and rotation_error <= np.deg2rad(RELEASE_ROT_TOL_DEG))
        print(f"[PLACE] {label} released at destination", flush=True)

        # 4. Retreat straight up, still holding the same orientation
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
