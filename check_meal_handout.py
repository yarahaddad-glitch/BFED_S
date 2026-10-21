"""Headless regression for the actual MealBot pickup and serving routes."""
import sys
import time
from types import SimpleNamespace

import numpy as np
import swift
from spatialmath import SE3

from Maryam_Work import maryam_movements as main
from Collective_Folder.EnviroParts_S.Full_sim import add_maryam_robot, add_tray_stacks
from Maryam_Work.maryam_tray_delivery import TrayDelivery


class HeadlessScene(swift.Swift):
    def __init__(self):
        super().__init__()
        self.timesteps = []

    def add(self, shape, **kwargs):
        return 0

    def step(self, dt=0):
        self.timesteps.append(dt)
        if hasattr(self, "observe"):
            self.observe()


def check(colour):
    env = HeadlessScene()
    objects = {"botCounter": SimpleNamespace(T=SE3(12.75, 4.4, -0.006).A)}
    robot = add_maryam_robot(env, objects)
    trays = add_tray_stacks(env, robot)
    layout = main.build_belt_layout(objects, robot)
    controller = main.build_controller(robot)
    delivery = TrayDelivery(env, robot, controller, layout, tray_scale=main.TRAY_SCALE,
                            animation_speed=main.ROBOT_ANIMATION_SPEED,
                            cache_dir=main.ROUTE_CACHE_DIR)
    gap = np.array([13.4, 4.7, .5]) + main.HANDOUT_GAP_LOCAL
    inside = gap + [0, -main.HANDOUT_SIDE_CLEARANCE, 0]
    outside = gap + [0, main.HANDOUT_INSERTION, 0]
    align = inside.copy()
    align[1] = layout.place_centre[1]
    align[2] = main.BELT_SURFACE_Z + .18
    low = align.copy()
    low[2] = inside[2]
    initial = {key: mesh.T.copy() for key, mesh in trays.items()}
    def observe():
        assert controller.valid(robot.q), "Robot crossed bar guard during playback"
    env.observe = observe
    sequence = ([("green", 2), ("blue", 2), ("green", 1), ("blue", 1)]
                if colour == "mixed" else [(colour, 2), (colour, 1)])
    for repeat in (False, True):
        if repeat:
            # A warm run must not enter the expensive search at all.
            def unexpected_search(*args, **kwargs):
                raise AssertionError("Warm route unexpectedly invoked IK search")
            delivery._plan_pick_and_carry = unexpected_search
        started = time.perf_counter()
        for selected, level in sequence:
            key = f"{selected}_tray_{level}"
            tray = trays[key]
            tray.T = initial[key]
            env.timesteps.clear()
            assert delivery.deliver(tray).on_belt
            assert np.allclose(env.timesteps, controller.dt / main.ROBOT_ANIMATION_SPEED)
            food = SimpleNamespace(T=SE3(tray.T).A)
            delivery.payload = [(food, SE3(0, 0, .055))]
            result = delivery.deliver(
                tray, destination=outside, waypoints=[align, low, inside],
                carry_validator=main.serving_gap_validator(gap))
            assert result.status == "PLACED", result
            centre = SE3(tray.T).t + SE3(tray.T).R @ delivery.centre_local
            assert np.linalg.norm(centre - outside) < .015
            assert centre[2] > .6267448 + .008
            assert np.allclose(food.T, (SE3(tray.T) * SE3(0, 0, .055)).A)
            assert robot.fkine(robot.q).t[1] < gap[1]
            before = robot.q.copy()
            main.collect_served_meal(env, tray, delivery.payload, gap, controller.dt)
            assert np.array_equal(before, robot.q)
            delivery.payload = []
            print(f"PASS: {key}, {'warm' if repeat else 'initial'} cycle, "
                  "bench/bar clearance, recipient collection", flush=True)
        print(f"[TEST] {'Warm' if repeat else 'Initial'} {len(sequence)}-tray run: "
              f"{time.perf_counter() - started:.2f}s (headless)", flush=True)


if __name__ == "__main__":
    check(sys.argv[1] if len(sys.argv) > 1 else "blue")
