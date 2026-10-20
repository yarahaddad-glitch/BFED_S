"""Headless regression check using the real robot kinematics and RMRC."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from spatialmath import SE3
from Maryam_Work.Maryam_robotclass import MaryamBot
from Maryam_Work.maryamrobot_controller import RobotController
from Maryam_Work.maryam_tray_delivery import BeltLayout, TrayDelivery, ConveyorBelt
from Collective_Folder.EnviroParts_S.main_meal_bot import build_belt_layout


def main():
    root = Path(__file__).resolve().parent
    robot = MaryamBot(base=SE3(14.5, 4.25, 0.6287448) * SE3.Rz(np.pi))
    controller = RobotController(
        robot, [root / "Maryam_Work" / f"RS007N-J{i}.STL" for i in range(1, 7)])
    layout = build_belt_layout({}, robot)
    env = SimpleNamespace(step=lambda dt: None)
    delivery = TrayDelivery(env, robot, controller, layout, tray_scale=2)
    for colour, x, level in (("green", 13.45, 1), ("blue", 15.15, 1),
                             ("green", 13.45, 0), ("blue", 15.15, 0)):
        tray = SimpleNamespace(T=SE3(x - 0.3, 4.3 + 0.197824179,
                                    0.6287448 + 0.002 + level * 0.041087036).A)
        result = delivery.deliver(tray, colour)
        print(colour, result, flush=True)
        assert result.status == "PLACED", result
        centre = SE3(tray.T).t + SE3(tray.T).R @ delivery.centre_local
        assert np.linalg.norm(centre - layout.place_centre) <= 0.015
        belt = ConveyorBelt(env, layout, tray_scale=2, dt=controller.dt)
        belt.carry_to_middle(tray)
        centre = SE3(tray.T).t + SE3(tray.T).R @ delivery.centre_local
        assert np.allclose(centre, layout.middle_centre)
    print("PASS: both edge pickups, placements and belt transfers.")


if __name__ == "__main__":
    main()
