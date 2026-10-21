"""Headless regression for the actual MealBot pickup and serving routes."""
import sys
import time
from types import SimpleNamespace

import numpy as np
import swift
from spatialmath import SE3

from Maryam_Work import maryam_movements as main
from Collective_Folder.EnviroParts_S.Full_sim import add_maryam_robot, add_tray_stacks


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
    movements = main.MaryamMovements(env, objects, robot, trays)
    delivery, controller = movements.delivery, movements.controller
    initial = {key: mesh.T.copy() for key, mesh in trays.items()}
    attached_frames = []
    def observe():
        assert controller.valid(robot.q), "Robot crossed bar guard during playback"
        if getattr(delivery, '_watched_tray', None) is not None:
            assert delivery.configuration_clear(robot.q), 'Tray/arm clearance failed during playback'
        if delivery._carried is not None:
            np.testing.assert_allclose(delivery._carried.T,
                (robot.fkine(robot.q)*delivery._relative).A, atol=1e-8)
            attached_frames.append(SE3(delivery._carried.T).R.copy())
            for food, relative in delivery.payload:
                np.testing.assert_allclose(food.T,
                    (SE3(delivery._carried.T)*relative).A, atol=1e-8)
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
        # Restore the whole inventory between runs, preserving real stack heights.
        for key, tray in trays.items():
            tray.T = initial[key]
        for selected, level in sequence:
            key = f"{selected}_tray_{level}"
            tray = trays[key]
            tray.T = initial[key]
            env.timesteps.clear()
            assert delivery.deliver(tray).on_belt
            assert np.allclose(env.timesteps, controller.dt / main.ROBOT_ANIMATION_SPEED)
            food = SimpleNamespace(T=SE3(tray.T).A)
            delivery.payload = [(food, SE3(0, 0, .055))]
            start_R = SE3(tray.T).R.copy()
            attached_frames.clear()
            result = movements.handouts[selected](tray, key)
            assert result.status == "PLACED", result
            expected_R = SE3.Rz(-np.pi/2).R@start_R
            np.testing.assert_allclose(movements.last_handout_pose.R, expected_R, atol=.003)
            assert any(np.allclose(R, expected_R, atol=.003) for R in attached_frames)
            np.testing.assert_allclose(robot.q, movements.home_q, atol=1e-8)
            assert delivery.payload == []
            assert tray.T[2, 3] < -10
            print(f"PASS: {key}, {'warm' if repeat else 'initial'} cycle, "
                  "90-degree tool turn, waiting-tray clearance, recipient collection, home", flush=True)
        print(f"[TEST] {'Warm' if repeat else 'Initial'} {len(sequence)}-tray run: "
              f"{time.perf_counter() - started:.2f}s (headless)", flush=True)


if __name__ == "__main__":
    check(sys.argv[1] if len(sys.argv) > 1 else "blue")
