"""Headless checks for GUI/worker coordination; no robot hardware is contacted."""
import ast
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace
import types
import unittest
from unittest.mock import patch

from Collective_Folder.workcell_runtime import WorkcellRuntime, WorkcellClosed
from Collective_Folder import E_STOP


class RuntimeChecks(unittest.TestCase):
    def start_checkpoint(self, runtime):
        finished = threading.Event()

        def run():
            try:
                runtime.checkpoint()
            except WorkcellClosed:
                pass
            finally:
                finished.set()

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        self.addCleanup(runtime.close)
        self.addCleanup(lambda: thread.join(1))
        return finished

    def test_ready_services_commands_but_waits_for_start(self):
        runtime = WorkcellRuntime()
        runtime.set_state('READY', 'Loaded')
        serviced = threading.Event()
        finished = self.start_checkpoint(runtime)
        runtime.submit(serviced.set)
        self.assertTrue(serviced.wait(1))
        self.assertFalse(finished.is_set())
        self.assertFalse(runtime.started)
        runtime.submit(runtime.start)
        self.assertTrue(finished.wait(1))
        self.assertEqual(runtime.state, 'RUNNING')

    def test_stop_acknowledge_and_start_are_separate(self):
        runtime = WorkcellRuntime()
        runtime.set_state('RUNNING', '')
        runtime.started = True
        runtime.request_stop('Test input')
        finished = self.start_checkpoint(runtime)
        acknowledged = threading.Event()
        runtime.submit(lambda: (runtime.reset(), acknowledged.set()))
        self.assertTrue(acknowledged.wait(1))
        self.assertEqual(runtime.state, 'PAUSED')
        self.assertFalse(finished.is_set())
        runtime.submit(runtime.start)
        self.assertTrue(finished.wait(1))

    def test_sensor_can_reject_reset(self):
        runtime = WorkcellRuntime()
        runtime.request_stop()
        with patch('Collective_Folder.workcell_runtime.actions.on_reset', return_value=False):
            with self.assertRaisesRegex(ValueError, 'still active'):
                runtime.reset()
        self.assertEqual(runtime.state, 'ESTOP')

    def test_registered_collision_holds_until_clear_reset_and_separate_start(self):
        import numpy as np
        runtime = WorkcellRuntime()
        runtime.set_state('RUNNING', '')
        runtime.started = True
        blocked = threading.Event()
        blocked.set()
        entered = threading.Event()
        runtime.register_collision_check('TestArm', lambda ctx, q, payload:
            (entered.set(), 'Object in path' if blocked.is_set() else True)[1])
        finished = threading.Event()

        def run():
            try:
                runtime.guard_configuration('TestArm', np.zeros(6))
                finished.set()
            except WorkcellClosed:
                pass

        thread = threading.Thread(target=run, daemon=True)
        self.addCleanup(lambda: thread.join(1))
        self.addCleanup(runtime.close)
        thread.start()
        self.assertTrue(entered.wait(1))
        # Service reset on the scene thread, after its stop was latched.
        attempted = threading.Event()
        result = []

        def reset_while_blocked():
            try:
                runtime.reset()
            except ValueError as exc:
                result.append(str(exc))
            attempted.set()

        runtime.submit(reset_while_blocked)
        self.assertTrue(attempted.wait(1))
        self.assertIn('collision', result[0])
        self.assertEqual(runtime.state, 'ESTOP')
        blocked.clear()
        acknowledged = threading.Event()
        runtime.submit(lambda: (runtime.reset(), acknowledged.set()))
        self.assertTrue(acknowledged.wait(1))
        self.assertEqual(runtime.state, 'PAUSED')
        self.assertFalse(finished.is_set())
        runtime.submit(runtime.start)
        self.assertTrue(finished.wait(1))

    def test_resume_restores_manual_poses_before_running(self):
        runtime = WorkcellRuntime()
        runtime.set_state('PAUSED', '')
        observed = []
        runtime.bridge = SimpleNamespace(restore_all=lambda: observed.append(runtime.state))
        runtime.start()
        self.assertEqual(observed, ['PAUSED'])
        self.assertEqual(runtime.state, 'RUNNING')

    def test_failed_restore_does_not_resume(self):
        runtime = WorkcellRuntime()
        runtime.set_state('PAUSED', '')

        def blocked():
            raise ValueError('Blocked restore route')

        runtime.bridge = SimpleNamespace(restore_all=blocked)
        with self.assertRaisesRegex(ValueError, 'Blocked'):
            runtime.start()
        self.assertEqual(runtime.state, 'PAUSED')

    def test_stop_arriving_during_restore_is_not_overwritten_by_start(self):
        runtime = WorkcellRuntime()
        runtime.set_state('PAUSED', '')
        runtime.bridge = SimpleNamespace(restore_all=lambda: runtime.request_stop('New input'))
        with self.assertRaisesRegex(ValueError, 'interrupted'):
            runtime.start()
        self.assertEqual(runtime.state, 'ESTOP')

    def test_new_stop_during_reset_requires_another_acknowledgement(self):
        runtime = WorkcellRuntime()
        runtime.request_stop('First input')

        def new_input(context):
            context.request_stop('New input')
            return True

        with patch('Collective_Folder.workcell_runtime.actions.on_reset', side_effect=new_input):
            with self.assertRaisesRegex(ValueError, 'new stop'):
                runtime.reset()
        self.assertEqual(runtime.state, 'ESTOP')

    def test_shutdown_wakes_paused_worker(self):
        runtime = WorkcellRuntime()
        runtime.set_state('PAUSED', '')
        finished = self.start_checkpoint(runtime)
        runtime.close()
        self.assertTrue(finished.wait(1))

    def test_camera_request_and_result_handshake(self):
        runtime = WorkcellRuntime()
        result = []
        thread = threading.Thread(target=lambda: result.append(runtime.scan()), daemon=True)
        thread.start()
        self.assertTrue(runtime.scan_requested.wait(1))
        runtime.finish_scan('GREEN')
        thread.join(1)
        self.assertEqual(result, ['GREEN'])
        self.assertFalse(runtime.scan_requested.is_set())

    def test_shutdown_cancels_pending_camera_request(self):
        runtime = WorkcellRuntime()
        cancelled = threading.Event()

        def scan():
            try:
                runtime.scan()
            except WorkcellClosed:
                cancelled.set()

        thread = threading.Thread(target=scan, daemon=True)
        thread.start()
        self.assertTrue(runtime.scan_requested.wait(1))
        runtime.close()
        thread.join(1)
        self.assertTrue(cancelled.is_set())

    def test_fault_cannot_be_reset_into_dead_task_runner(self):
        runtime = WorkcellRuntime()
        runtime.fail('Task failed')
        runtime.request_stop()
        with self.assertRaisesRegex(ValueError, 'runner failed'):
            runtime.reset()

    def test_legacy_collision_stop_delegates_without_console_input(self):
        calls = []
        E_STOP.set_stop_callback(calls.append)
        self.addCleanup(lambda: E_STOP.set_stop_callback(None))
        with patch('builtins.input', side_effect=AssertionError('Blocking console input')):
            E_STOP.stop('Obstacle')
        self.assertEqual(calls, ['Obstacle'])

    def test_hannah_prepare_does_not_call_the_animation(self):
        # Check the actual prepare/run branch independently of the graphical imports.
        tree = ast.parse(Path('Hannah_Work/Feed_Iso.py').read_text(encoding='utf-8-sig'))
        feed = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == 'feed_iso')
        branch = next(node for node in feed.body if isinstance(node, ast.If)
                      and 'prepare_only' in ast.unparse(node.test))
        scope = {'unused': {'prepare_only': True}, 'hannahbot': 'arm', 'turtle': 'base',
                 'run_sequence': lambda: self.fail('Animation ran before Start')}
        wrapper = ast.FunctionDef(name='prepare', args=ast.arguments(
            posonlyargs=[], args=[], kwonlyargs=[], kw_defaults=[], defaults=[]),
            body=[branch, feed.body[-1]], decorator_list=[])
        module = ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[]))
        exec(compile(module, '<prepare check>', 'exec'), scope)
        arm, base, runner = scope['prepare']()
        self.assertEqual((arm, base), ('arm', 'base'))
        self.assertTrue(callable(runner))


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'NumPy unavailable')
class SharedRobotBridgeChecks(unittest.TestCase):
    """Use a small kinematic arm to exercise commands without Swift/hardware."""
    def setUp(self):
        import numpy as np
        self.np = np

        class Pose:
            def __init__(self, value=None):
                self.A = np.eye(4) if value is None else value.A.copy()

            @property
            def t(self):
                return self.A[:3, 3]

            @property
            def R(self):
                return self.A[:3, :3]

        class Arm:
            n = 6
            q = np.zeros(6)
            base = Pose()
            qlim = np.array([[-1.] * 6, [1.] * 6])

            def fkine(self, q):
                pose = Pose()
                pose.A[:3, 3] = q[:3]
                return pose

        module_path = Path('Collective_Folder/MealBot_GUI/MealBot_GUI/MealBot_GUI/robot_bridge.py')
        spec = importlib.util.spec_from_file_location('shared_bridge_under_test', module_path)
        module = importlib.util.module_from_spec(spec)
        fake_spatialmath = types.ModuleType('spatialmath')
        fake_spatialmath.SE3 = Pose
        with patch.dict('sys.modules', {'spatialmath': fake_spatialmath}):
            spec.loader.exec_module(module)
        self.runtime = WorkcellRuntime()
        self.runtime.set_state('PAUSED', '')
        self.arm = Arm()
        self.frames = []
        self.bridge = module.RobotBridge()
        self.bridge.attach(None, {'TestArm': self.arm}, self.runtime,
                           lambda dt: self.frames.append(self.arm.q.copy()))

    def test_jog_uses_existing_instance_and_restore_returns_saved_pose(self):
        self.bridge.set_joint('TestArm', 0, 5)
        self.assertAlmostEqual(self.arm.q[0], self.np.deg2rad(5))
        self.assertIs(self.bridge.load('TestArm'), self.arm)
        self.bridge.restore('TestArm')
        self.np.testing.assert_allclose(self.arm.q, self.np.zeros(6))
        self.assertNotIn('TestArm', self.bridge.saved)

    def test_collision_rejection_does_not_animate_any_part_of_route(self):
        self.bridge.validators['TestArm'] = lambda q: q[0] < .02
        with self.assertRaisesRegex(ValueError, 'guard rejected'):
            self.bridge.move_to('TestArm', self.np.array([.05, 0, 0, 0, 0, 0]))
        self.assertEqual(self.frames, [])
        self.np.testing.assert_allclose(self.arm.q, self.np.zeros(6))

    def test_registered_team_check_rejects_manual_path_before_movement(self):
        self.runtime.register_collision_check('TestArm',
            lambda context, q, payload: q[0] < .02)
        with self.assertRaisesRegex(ValueError, 'Registered team collision'):
            self.bridge.move_to('TestArm', self.np.array([.05, 0, 0, 0, 0, 0]))
        self.assertEqual(self.frames, [])
        self.np.testing.assert_allclose(self.arm.q, self.np.zeros(6))

    def test_stop_interrupts_a_manual_trajectory(self):
        self.bridge.draw = lambda dt: self.runtime.request_stop()
        with self.assertRaisesRegex(ValueError, 'interrupted'):
            self.bridge.move_to('TestArm', self.np.ones(6)*.2)
        self.assertLess(self.arm.q[0], .2)

    def test_running_automatic_sequence_rejects_manual_commands(self):
        self.runtime.set_state('RUNNING', '')
        with self.assertRaisesRegex(ValueError, 'Pause'):
            self.bridge.set_joint('TestArm', 0, 5)
        self.assertEqual(self.frames, [])

    def test_cartesian_translation_reaches_requested_world_axis(self):
        self.bridge.translate('TestArm', 1, .01)
        self.assertAlmostEqual(self.arm.q[1], .01, delta=.001)
        self.assertAlmostEqual(self.arm.q[0], 0.)


if __name__ == '__main__':
    unittest.main()
