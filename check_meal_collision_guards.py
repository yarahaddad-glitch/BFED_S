"""Regression checks for tray geometry and stop-before-motion guards."""
import unittest
from types import SimpleNamespace
import numpy as np
from spatialmath import SE3
from Maryam_Work.maryam_tray_delivery import TrayDelivery
from Collective_Folder.integrated_meal import carry_tray


class CollisionChecks(unittest.TestCase):
    def test_tray_link_intersection_and_clearance(self):
        delivery = TrayDelivery.__new__(TrayDelivery)
        delivery.centre_local = np.zeros(3)
        delivery.edge_grip_offset = .3
        delivery.half_width = .2
        delivery.controller = SimpleNamespace(body_poses=lambda q: [
            (SE3(*q), np.array([.1, .1, .1]))])
        self.assertFalse(delivery.tray_clears_robot([0, 0, 0], SE3()))
        self.assertTrue(delivery.tray_clears_robot([.5, 0, 0], SE3()))
        self.assertFalse(delivery.tray_clears_robot([0, 0, 0], SE3.Rz(.7)))

    def test_arm_stops_before_assigning_unsafe_joints(self):
        delivery = TrayDelivery.__new__(TrayDelivery)
        delivery.robot = SimpleNamespace(q=np.zeros(6))
        delivery._watched_tray = SimpleNamespace(T=SE3().A)
        delivery._carried = None
        delivery.tray_clears_robot = lambda q, pose: False
        delivery._step = lambda: self.fail("Unsafe pose was animated")
        with self.assertRaisesRegex(RuntimeError, "collide"):
            delivery._execute([np.zeros(6), np.ones(6)])
        np.testing.assert_array_equal(delivery.robot.q, np.zeros(6))

    def test_belt_validates_entire_leg_before_moving(self):
        tray = SimpleNamespace(T=SE3().A)
        env = SimpleNamespace(step=lambda dt: self.fail("Unsafe belt leg was animated"))
        with self.assertRaisesRegex(RuntimeError, "belt stopped"):
            carry_tray(env, tray, [1, 0, 0], clearance=lambda pose: pose.t[0] < .5)
        np.testing.assert_array_equal(tray.T, SE3().A)


if __name__ == "__main__":
    unittest.main()
