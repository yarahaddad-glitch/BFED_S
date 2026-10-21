"""Actual Qt + four robot models, headless Swift; never connects real hardware.

Run with ../.venv/Scripts/python.exe -m unittest -v check_workcell_gui_qt
"""
import importlib.util
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
AVAILABLE = all(importlib.util.find_spec(name) for name in
                ('PySide6', 'swift', 'roboticstoolbox', 'ir_support'))


@unittest.skipUnless(AVAILABLE, 'Use the subject .venv with Qt and robotics packages')
class WorkcellGuiChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(Path.cwd() / 'Collective_Folder/MealBot_GUI/MealBot_GUI/MealBot_GUI'))
        from PySide6.QtWidgets import QApplication
        from PySide6.QtGui import QFontDatabase
        from main_gui import MainWindow
        from theme import apply_theme
        from Collective_Folder.workcell_runtime import WorkcellRuntime
        from Collective_Folder.EnviroParts_S.main_all import run_workcell
        from Collective_Folder.EnviroParts_S.Full_sim import (
            create_environment, add_maryam_robot, add_tray_stacks)
        from check_integrated_meal import HeadlessSwift
        from spatialmath import SE3

        cls.app = QApplication.instance() or QApplication([])
        # Qt's offscreen Windows backend needs an explicit font for readable QA images.
        for font in ('trebuc.ttf', 'trebucbd.ttf'):
            path = Path('C:/Windows/Fonts') / font
            if path.exists():
                QFontDatabase.addApplicationFont(str(path))
        apply_theme(cls.app)
        cls.runtime = WorkcellRuntime()
        cls.window = MainWindow(runtime=cls.runtime)
        cls.window.show()

        def build():
            env = HeadlessSwift()
            env.headless = True
            objects = create_environment(env)
            objects.pop('bluetray').T = SE3(0, 0, -100).A
            robot = add_maryam_robot(env, objects)
            trays = add_tray_stacks(env, robot)
            objects.update(trays)
            return env, objects, robot, trays

        cls.build_patch = patch('Collective_Folder.EnviroParts_S.main_all.build_scene', build)
        cls.build_patch.start()
        cls.worker = threading.Thread(target=run_workcell,
                                      args=(cls.runtime, cls.window.bridge), daemon=True)
        cls.worker.start()
        cls.pump_until(lambda: cls.runtime.state != 'LOADING', timeout=15)
        if cls.runtime.state != 'READY':
            cls.tearDownClass()
            raise AssertionError(cls.runtime.status())

    @classmethod
    def pump_until(cls, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.app.processEvents()
            if predicate():
                return
            # Release the GIL so the scene worker can service GUI requests.
            time.sleep(.01)
        raise AssertionError('Timed out: ' + str(cls.runtime.status()))

    @classmethod
    def tearDownClass(cls):
        cls.window.close()
        cls.runtime.close()
        cls.worker.join(2)
        cls.build_patch.stop()

    def test_joint_controls_move_all_four_existing_robots_and_restore(self):
        import numpy as np
        from PySide6.QtCore import Qt, QPointF
        from PySide6.QtTest import QTest
        window = self.window
        window.navigate(2)
        page = window.manual_page
        for name in ('MaryamBot', 'YaraBot', 'HannahBot', 'ReBot'):
            with self.subTest(robot=name):
                page.selector.setCurrentText(name)
                self.pump_until(lambda: page.name == name and not self.runtime.manual_pending)
                start = window.bridge.angles_deg(name).copy()
                plus = page.grid.itemAtPosition(0, 3).widget()
                point = plus.mapTo(window.scaled_panel.panel, plus.rect().center())
                viewport_point = window.scaled_panel.mapFromScene(QPointF(point))
                for _ in range(2):
                    QTest.mouseClick(window.scaled_panel.viewport(), Qt.LeftButton,
                                     Qt.NoModifier, viewport_point)
                page.flush_jogs()
                self.pump_until(lambda: not self.runtime.manual_pending)
                actual = window.bridge.angles_deg(name)
                self.assertAlmostEqual(actual[0], round(start[0]) + 2, places=5,
                                       msg=page.status.text() + ' / ' + self.runtime.manual_message)
                self.assertEqual(window.camera_selected, name.replace('Bot', '') if name != 'ReBot' else name)
                page.restore_pose()
                self.pump_until(lambda: not self.runtime.manual_pending)
                np.testing.assert_allclose(window.bridge.angles_deg(name), start, atol=1e-7)

    def test_panel_scales_uniformly_and_has_small_default_size(self):
        self.window.resize(350, 400)
        self.app.processEvents()
        view = self.window.scaled_panel
        self.assertEqual((view.panel.width(), view.panel.height()), (490, 560))
        self.assertAlmostEqual(view.transform().m11(), view.transform().m22())
        self.assertLess(view.transform().m11(), .75)
        self.window.resize(420, 480)
        self.app.processEvents()
        self.assertAlmostEqual(view.transform().m11(), view.transform().m22())
        self.window.resize(350, 400)


    def test_entering_manual_pauses_running_sequence_and_selects_robot(self):
        from main_gui import MainWindow
        from Collective_Folder.workcell_runtime import WorkcellRuntime
        runtime = WorkcellRuntime()
        runtime.set_state('RUNNING', '')
        window = MainWindow(runtime=runtime)
        try:
            window.bridge.attach(self.runtime.env, self.runtime.robots, runtime, lambda dt: None)
            window.navigate(2)
            self.assertEqual(runtime.state, 'PAUSED')
            self.assertEqual(window.manual_page.name, 'MaryamBot')
            self.assertTrue(window.manual_page.sliders[0].isEnabled())
        finally:
            window.close()

    def test_collision_mode_inserts_real_obstacle_in_future_path(self):
        import numpy as np
        runtime, window = self.runtime, self.window
        window.navigate(3)
        demo = runtime.collision
        q = demo.robot.q.copy()
        paths = []
        initial_bodies = demo.controller.body_poses(q)
        for joint in (1, 2, 0):
            for delta in (-.6, .6):
                target = q.copy()
                target[joint] += delta
                candidate = np.linspace(q, target, 50)
                if all(demo.controller.valid(point) for point in candidate):
                    distance = max(np.linalg.norm(a.t-b.t) for (a,_),(b,_) in
                                   zip(initial_bodies, demo.controller.body_poses(target)))
                    paths.append((distance, candidate))
        path = max(paths, key=lambda item:item[0])[1]
        self.assertTrue(all(demo.controller.valid(point) for point in path))
        window.arm_collision_test()
        self.pump_until(lambda:demo.armed)
        runtime.submit(lambda:demo.before_path(path))
        self.pump_until(lambda:runtime.state == 'ESTOP')
        settled = threading.Event()
        runtime.submit(settled.set)
        self.pump_until(settled.is_set)
        self.assertIsNotNone(demo.obstacle)
        self.assertTrue(demo.delivery.configuration_clear(q))
        self.assertTrue(any(not demo.delivery.configuration_clear(point) for point in path))
        np.testing.assert_array_equal(demo.robot.q, q)
        with self.assertRaisesRegex(ValueError, 'Remove the collision'):
            runtime.reset()
        window.remove_collision_obstacle()
        self.pump_until(lambda:demo.obstacle is None)
        runtime.submit(runtime.reset)
        self.pump_until(lambda:runtime.state == 'READY')
        self.assertTrue(all(demo.delivery.configuration_clear(point) for point in path))

    def test_refresh_does_not_pull_slider_back_during_drag(self):
        self.window.navigate(2)
        page = self.window.manual_page
        self.pump_until(lambda: not self.runtime.manual_pending)
        slider = page.sliders[0]
        original = slider.value()
        try:
            slider.setSliderDown(True)
            slider.setValue(original + 10)
            page.refresh_angles()
            self.assertEqual(slider.value(), original + 10)
        finally:
            page.pending_jogs.clear()
            slider.setSliderDown(False)
            page.refresh_angles()

    def test_camera_views_stay_on_the_clear_side_and_above_hannah(self):
        from main_gui import _camera_pose_for_robot
        yara = self.runtime.robots['YaraBot']
        position, target = _camera_pose_for_robot(yara, 'Yara')
        self.assertGreater(position[1], 1.8)  # In front of wall and kitchen counter.
        hannah = self.runtime.robots['HannahBot']
        position, target = _camera_pose_for_robot(hannah, 'Hannah')
        self.assertGreater(position[2] - hannah.base.t[2], 2.5)
        self.assertAlmostEqual(target[0], hannah.base.t[0])
        self.assertAlmostEqual(target[1], hannah.base.t[1])
        maryam = self.runtime.robots['MaryamBot']
        position, target = _camera_pose_for_robot(maryam, 'Maryam', near_robot=yara)
        self.assertAlmostEqual(position[0], yara.base.t[0])
        self.assertGreater(position[1], yara.base.t[1])
        self.assertLess(position[1], maryam.base.t[1])

    def test_safety_intrusions_latch_stop_block_reset_and_clear_without_restart(self):
        import numpy as np
        window, runtime = self.window, self.runtime
        window.navigate(7)
        original = np.array(runtime.objects['prisoner1'].T, float).copy()
        joints = {name: robot.q.copy() for name, robot in runtime.robots.items()}
        for kind in ('curtain', 'kitchen', 'isolation'):
            with self.subTest(test=kind):
                window.run_safety_test(kind)
                self.pump_until(lambda: runtime.state == 'ESTOP')
                # Queue a barrier so the test animation and stop hook have finished.
                settled = threading.Event()
                runtime.submit(settled.set)
                self.pump_until(settled.is_set)
                window.poll_runtime()
                self.assertTrue(window.safety_warning.isVisible())
                self.assertEqual(window.safety_warning.text(), 'Prisoner in area')
                window.safety_warning.accept()
                self.assertTrue(runtime.safety.active_hazards())
                self.assertIn('LIGHT CURTAIN' if kind == 'curtain' else kind,
                              runtime.safety.last_event)
                runtime.submit(runtime.reset)
                rejected = threading.Event()
                runtime.submit(rejected.set)
                self.pump_until(rejected.is_set)
                self.assertEqual(runtime.state, 'ESTOP')
                self.assertIn('Clear the unsafe zone', runtime.message)
                window.clear_safety_actor()
                self.pump_until(lambda: runtime.safety.saved_pose is None
                                and not runtime.safety.active_hazards())
                self.assertEqual(runtime.state, 'ESTOP')
                runtime.submit(runtime.reset)
                self.pump_until(lambda: runtime.state == 'READY')
                self.assertFalse(runtime.started)
                np.testing.assert_allclose(runtime.objects['prisoner1'].T, original)
                for name, q in joints.items():
                    np.testing.assert_allclose(runtime.robots[name].q, q)

    def test_sidebar_order_and_activity_terminal(self):
        self.assertEqual([button.text() for button in self.window.nav_buttons],
                         ['Dashboard', 'Automatic', 'Manual', 'Camera', 'Collision tests',
                          'Safety tests', 'PLC', 'Emergency stop'])
        self.window.navigate(6)
        self.assertEqual(self.window.nav_buttons[3].property('active'), 'true')
        self.assertEqual(self.window.nav_buttons[6].property('active'), 'false')
        self.runtime.log('Green detected')
        self.runtime.log('MaryamBot: picking up green tray')
        self.window.poll_runtime()
        self.assertTrue(self.window.activity_terminal.toPlainText().endswith(
            'Green detected\nMaryamBot: picking up green tray'))

    def test_scanner_shows_identified_colour_before_returning_result(self):
        from camera_scanner import CameraScanner
        from Collective_Folder.workcell_runtime import WorkcellRuntime
        from Maryam_Work.maryam_meal_dispatch import DetectionResult
        runtime = WorkcellRuntime()
        runtime.set_state('RUNNING', '')
        runtime.scan_requested.set()
        scanner = CameraScanner(runtime, self.window)
        scanner.timer.stop()  # Deterministic ticks; never open a webcam.
        try:
            scanner.show_identified(DetectionResult('GREEN', .9))
            self.assertEqual(scanner.identified.text(), 'Colour identified - Green')
            scanner.tick()
            self.assertFalse(runtime.scan_finished.is_set())
            scanner.result_ready_at = 0
            runtime.set_state('PAUSED', '')
            scanner.tick()
            self.assertFalse(runtime.scan_finished.is_set())
            runtime.set_state('RUNNING', '')
            scanner.tick()
            self.assertTrue(runtime.scan_finished.is_set())
            self.assertEqual(runtime.scan_result.colour, 'GREEN')
        finally:
            scanner.close()


if __name__ == '__main__':
    unittest.main()
