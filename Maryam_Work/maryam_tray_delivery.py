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
EDGE_GRIP_INSET = 0.0        # grip the nearest X rim; green needs the full reach

# Heights in metres (same idea as your working green-tray pickup).
GRASP_HEIGHT = 0.05           # tool point sits this far above the tray bottom when "gripping"
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
                 tray_scale: float, return_home: bool = False):
        self.env = env
        self.robot = robot
        self.controller = controller
        self.layout = layout
        self.centre_local = TRAY_CENTRE_UNSCALED * tray_scale
        self.return_home = return_home
        self.edge_grip_offset = max(
            0.0, TRAY_HALF_LENGTH_UNSCALED * tray_scale - EDGE_GRIP_INSET)
        self.home_xyz = robot.fkine(robot.q).t.copy()
        self.hold_R = None            # None = free orientation
        self._carried = None          # tray mesh currently attached to the tool
        self._relative = None         # tray pose relative to the tool

    # ---- helpers ----
    def _step(self):
        """Called every RMRC sample: keeps a carried tray attached to the tool."""
        if self._carried is not None:
            self._carried.T = (self.robot.fkine(self.robot.q) * self._relative).A
        self.env.step(self.controller.dt)

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

    def _plan_pick_and_carry(self, tray_pose, grasp, above, place_origin):
        """Search grip configurations and verify the whole route before moving."""
        controller = self.controller
        q_start = self.robot.q.copy()
        rng = np.random.default_rng(24)
        seeds = [q_start, np.zeros(self.robot.n)]
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
                continue
            ee = self.robot.fkine(q_grasp)
            hold_R = ee.R.copy()
            relative = ee.inv() * tray_pose
            ee_offset = hold_R @ relative.t
            targets = [
                ("lift", above),
                ("over belt", place_origin + [0, 0, BELT_APPROACH_HEIGHT] - ee_offset),
                ("lower onto belt", place_origin - ee_offset),
            ]
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
                    break
                legs.append((name, path))
                q = path[-1]
            if len(legs) != len(targets):
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
            print(f"[PLAN] Complete pickup and carry verified (grip {attempt}).", flush=True)
            return approach, descent, legs, relative, hold_R
        return None

    # ---- main entry point ----
    def deliver(self, tray_mesh, label: str = "tray") -> DeliveryResult:
        self.hold_R = None
        self._carried = None

        tray_pose = SE3(tray_mesh.T)
        centre = tray_pose.t + tray_pose.R @ self.centre_local
        # Choose the X rim nearest the robot in the tray's local frame.
        robot_local = tray_pose.R.T @ (self.robot.base.t - centre)
        edge_local = np.array([
            np.sign(robot_local[0]) * self.edge_grip_offset, 0.0, 0.0])
        pickup = centre + tray_pose.R @ edge_local
        above = pickup + np.array([0.0, 0.0, APPROACH_HEIGHT])
        grasp = pickup + np.array([0.0, 0.0, GRASP_HEIGHT])
        print(f"\n[PICK] {label}: tray centre {np.round(centre, 3)}")
        print(f"[PICK] Near-edge grip {np.round(grasp, 3)}")

        tray_at_grasp = tray_pose
        place_origin = self.layout.place_centre - tray_at_grasp.R @ self.centre_local
        print("[PLAN] Searching grip poses for the complete transfer...", flush=True)
        plan = self._plan_pick_and_carry(tray_pose, grasp, above, place_origin)
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
        print("[PLACE] Tray released on the conveyor belt", flush=True)

        # 4. Retreat straight up, still holding the same orientation
        retreat = self.robot.fkine(self.robot.q).t.copy()
        retreat[2] += RETREAT_HEIGHT
        self._move(retreat, "Retreat vertically")
        self.hold_R = None
        if self.return_home:
            self._move(self.home_xyz, "Return home")

        if on_target:
            return DeliveryResult("PLACED", "Tray placed on the belt")
        return DeliveryResult("PLACED_OFF_TARGET",
                              "Tray released but outside the placement tolerance")
