"""Regression checks for tray geometry and stop-before-motion guards."""
import unittest
import threading
from types import SimpleNamespace
import numpy as np
from spatialmath import SE3
from Maryam_Work.maryam_tray_delivery import TrayDelivery
from Collective_Folder.integrated_meal import carry_tray
from Maryam_Work.maryam_movements import collect_served_meal
from unittest.mock import patch
from Collective_Folder.workcell_runtime import WorkcellRuntime, WorkcellClosed
from Collective_Folder.maryam_collision_demo import MaryamCollisionDemo


class CollisionChecks(unittest.TestCase):
    def test_quarter_turn_clears_waiting_green_tray(self):
        delivery = TrayDelivery.__new__(TrayDelivery)
        delivery.centre_local = np.array([.3, -.197824179, 0])
        delivery.edge_grip_offset = .3
        delivery.half_width = .197824179
        delivery.controller = SimpleNamespace(body_poses=lambda q: [])
        green_centre = np.array([13.75, 4.35, .645])
        green = SimpleNamespace(T=SE3(*(green_centre-delivery.centre_local)).A)
        delivery.stationary_trays = [green]
        centre = np.array([14.28485, 4.58, .645])
        original = SE3(*(centre-delivery.centre_local))
        rotated = SE3.Rt(SE3.Rz(-np.pi/2).R,
                         centre-SE3.Rz(-np.pi/2).R@delivery.centre_local)
        self.assertFalse(delivery.queued_trays_clear(np.zeros(6), original))
        self.assertTrue(delivery.queued_trays_clear(np.zeros(6), rotated))

    def test_fixed_base_is_included_in_tray_clearance(self):
        from Maryam_Work.Maryam_robotclass import MaryamBot
        from Maryam_Work.maryam_movements import build_controller
        robot = MaryamBot(base=SE3(14.5, 4.35, .6267448) * SE3.Rz(np.pi))
        controller = build_controller(robot)
        delivery = TrayDelivery.__new__(TrayDelivery)
        delivery.centre_local = np.array([.3, -.197824179, 0])
        delivery.edge_grip_offset = .3
        delivery.half_width = .197824179
        delivery.controller = controller
        tray = SE3(*(np.array([14.22485, 4.58, .645]) - delivery.centre_local))
        self.assertFalse(delivery.tray_clears_robot(robot.q, tray))
        robot.base = SE3(14.66, 4.35, .6267448) * SE3.Rz(np.pi)
        self.assertTrue(delivery.tray_clears_robot(robot.q, tray))

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

    def test_recipient_cannot_pull_tray_through_robot(self):
        tray = SimpleNamespace(T=SE3().A)
        env = SimpleNamespace(step=lambda dt:self.fail('Unsafe recipient motion animated'))
        with patch('Maryam_Work.maryam_movements.serving_gap_validator', return_value=lambda pose:True):
            with self.assertRaisesRegex(RuntimeError, 'hit MaryamBot'):
                collect_served_meal(env, tray, [], [0, 1, 0], .05,
                                    clearance=lambda pose:False)
        np.testing.assert_array_equal(tray.T, SE3().A)

    def test_inserted_obstacle_is_checked_before_cached_path_is_played(self):
        delivery = TrayDelivery.__new__(TrayDelivery)
        delivery.robot = SimpleNamespace(q=np.zeros(6))
        delivery._watched_tray = None
        delivery.controller = SimpleNamespace(obstacles=[{'type':'sphere'}],
                                              valid=lambda q:q[0] < .5)
        delivery._step = lambda:self.fail('Blocked preplanned leg was animated')
        with self.assertRaisesRegex(RuntimeError, 'before contact'):
            delivery._execute([np.zeros(6), np.ones(6)])
        np.testing.assert_array_equal(delivery.robot.q, np.zeros(6))

    def test_interrupted_path_keeps_its_remaining_joint_targets(self):
        delivery = TrayDelivery.__new__(TrayDelivery)
        delivery.robot = SimpleNamespace(q=np.zeros(6))
        delivery._watched_tray = None
        obstacles = [object()]
        delivery.controller = SimpleNamespace(obstacles=obstacles,
                                              valid=lambda q:not obstacles or q[0] < .5)
        observations = []
        delivery._step = lambda:observations.append(delivery.robot.q.copy())

        def remove_after_stop():
            np.testing.assert_array_equal(delivery.robot.q, np.zeros(6))
            obstacles.clear()

        delivery.wait_for_clearance = remove_after_stop
        delivery._execute([np.zeros(6), np.ones(6)*.25, np.ones(6)])
        np.testing.assert_array_equal(delivery.robot.q, np.ones(6))
        self.assertEqual(len(observations), 2)

    def test_real_arm_obstacle_stop_reset_and_resume_retains_task(self):
        from Maryam_Work.Maryam_robotclass import MaryamBot
        from Maryam_Work.maryam_movements import build_controller
        from Maryam_Work.maryam_tray_delivery import BeltLayout
        robot = MaryamBot(base=SE3(14.8, 4.35, .6267448) * SE3.Rz(np.pi))
        controller = build_controller(robot)
        q0 = robot.q.copy()
        candidates = []
        for delta in (-.6, .6):
            target = q0.copy(); target[1] += delta
            path = np.linspace(q0, target, 50)
            if all(controller.valid(q) for q in path):
                candidates.append(path)
        self.assertTrue(candidates)
        path = candidates[0]
        env = SimpleNamespace(add=lambda mesh:None, step=lambda dt:None)
        delivery = TrayDelivery(env, robot, controller,
            BeltLayout(np.zeros(3), np.ones(3)), tray_scale=2)
        runtime = WorkcellRuntime()
        runtime.env = env
        runtime.bridge = SimpleNamespace(draw=env.step, restore_all=lambda:None)
        runtime.set_state('READY', '')
        demo = MaryamCollisionDemo(runtime, SimpleNamespace(delivery=delivery, controller=controller))
        runtime.collision = demo
        complete = threading.Event()
        errors = []

        def worker():
            try:
                runtime.checkpoint()
                delivery._execute(path)
                runtime.set_state('COMPLETE', '')
                complete.set()
                runtime.checkpoint()
            except WorkcellClosed:
                pass
            except Exception as exc:
                errors.append(exc)
                complete.set()

        stopped = threading.Event()
        original_stop = runtime.request_stop

        def stop(reason):
            original_stop(reason)
            stopped.set()

        runtime.request_stop = stop
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        try:
            runtime.submit(demo.arm_test)
            runtime.submit(runtime.start)
            self.assertTrue(stopped.wait(3))
            self.assertFalse(complete.is_set())
            np.testing.assert_array_equal(robot.q, q0)
            recovered = threading.Event()
            runtime.submit(demo.remove_obstacle)
            runtime.submit(runtime.reset)
            runtime.submit(recovered.set)
            self.assertTrue(recovered.wait(3))
            self.assertEqual(runtime.state, 'PAUSED')
            np.testing.assert_array_equal(robot.q, q0)
            runtime.submit(runtime.start)
            self.assertTrue(complete.wait(3))
            self.assertEqual(errors, [])
            np.testing.assert_allclose(robot.q, path[-1])
        finally:
            runtime.close()
            thread.join(2)


if __name__ == "__main__":
    unittest.main()
