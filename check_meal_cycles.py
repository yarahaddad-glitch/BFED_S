"""Exercise real Swift.step through consecutive meals without a browser/camera."""
from types import SimpleNamespace
import sys
import numpy as np
import swift
from spatialmath import SE3
from Maryam_Work.maryam_movements import (
    MaryamMovements, add_maryam_robot, add_tray_stacks, food_geometry)


def check():
    env = swift.Swift()
    env.headless = True
    objects = {"botCounter": SimpleNamespace(T=SE3(12.75, 4.4, -.006).A)}
    robot = add_maryam_robot(env, objects)
    trays = add_tray_stacks(env, robot)
    movements = MaryamMovements(env, objects, robot, trays)
    if "--cached" in sys.argv:
        def no_search(*args, **kwargs):
            raise AssertionError("Saved cycle unexpectedly restarted IK search")
        movements.delivery._plan_pick_and_carry = no_search
    for colour in ("blue", "green", "green", "blue"):
        on_belt = []
        assert movements.serve(colour, lambda: on_belt.append(True))
        assert on_belt == [True]
        assert all(obj is not None for obj in env.swift_objects)
        env.step(.035)  # Previously crashed after removing the first meal.
        assert movements.delivery.payload == []
        print(f"PASS: {colour} complete cycle and Swift step after handout", flush=True)
    assert movements.stacks.remaining("blue") == 0
    assert movements.stacks.remaining("green") == 0
    for colour, name in (("blue", "cookies.dae"), ("green", "lettuce.dae")):
        path, scale, centre = food_geometry(colour)
        assert path.name == name and scale > 0 and np.all(np.isfinite(centre))


if __name__ == "__main__":
    check()
