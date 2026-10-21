"""Exercise the shared scene and real robot kinematics without camera/browser."""
import sys
import numpy as np
import swift
from Collective_Folder.EnviroParts_S.Full_sim import create_environment, add_maryam_robot, add_tray_stacks
from Collective_Folder.integrated_meal import IntegratedMeal, yara
from spatialmath import SE3


class HeadlessSwift(swift.Swift):
    def _send_socket(self, code, data=None, *args, **kwargs):
        # Do not hide the live-update bug that caused browser startup to hang.
        assert code != "shape_update", "Unexpected live shape update"
        return 0


def check():
    env = HeadlessSwift()
    env.headless = True
    objects = create_environment(env)
    objects.pop("bluetray").T = SE3(0, 0, -100).A
    robot = add_maryam_robot(env, objects)
    trays = add_tray_stacks(env, robot)
    objects.update(trays)
    movements = IntegratedMeal(env, objects, robot, trays)
    yara.DEBUG = False
    for colour in (sys.argv[1:] or ["blue", "green", "green", "blue"]):
        called = []
        food_count = len(yara.FOOD_NAMES[colour])
        towel_count = len(movements.towels)
        assert movements.serve(colour, lambda: called.append(True))
        assert called == [True]
        assert movements.delivery.payload == []
        np.testing.assert_allclose(robot.q, movements.home_q, atol=1e-8)
        assert len(yara.FOOD_NAMES[colour]) == food_count - 1
        assert len(movements.towels) == towel_count - 1
        assert np.allclose(movements.rebot.base.t, [12.5, 3.6, .66])
        env.step(.03)
        print(f"PASS: {colour} Maryam/Yara/ReBot/full handout", flush=True)


if __name__ == "__main__":
    check()
