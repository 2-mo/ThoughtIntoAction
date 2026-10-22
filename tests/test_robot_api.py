"""The simple Robot API owns leases, not hardware; all transports are fake here."""
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from embodied import Robot
from embodied.applications.operations import robot as robot_api


class RobotApiTests(unittest.TestCase):
    def setUp(self):
        self.state = {
            'model_id': 'example-model', 'q': [0.] * 6,
            'pose': [.3, 0., .25, 0., 0., 0., 1.], 'gripper_width': .03,
        }
        self.plan = {
            'plan_id': 'example-plan', 'kind': 'movej', 'times': [0., .8],
            'limits': {'speed': .5, 'acceleration': 2., 'jerk': 10.},
        }
        self.operation = {'operation_id': 'example-operation', 'status': 'running'}
        self.completed = {'operation_id': 'example-operation', 'status': 'completed'}
        self.client = MagicMock()
        self.client.directory = Path('/fake/run')
        self.client.identity = {'runtime_id': 'example-runtime', 'backend': 'sim'}
        self.client.status.return_value = self.state
        self.client.preview.return_value = self.plan
        self.client.describe.return_value = {
            'model_id': self.state['model_id'],
            'robot': {'gripper_max_width': .08},
            'motion_limits': {
                'defaults': {'speed': .15, 'acceleration': 2., 'jerk': 10.},
                'joint_velocity_limits_rad_s': [5., 5., 5.5, 5.5, 5., 5.],
            },
        }
        self.session = MagicMock()
        self.session.__enter__.return_value = self.session
        self.session.status.return_value = self.state
        self.session.preview.return_value = self.plan
        self.session.execute.return_value = self.operation
        self.session.wait.return_value = self.completed
        self.client_factory = self.enterContext(
            patch.object(robot_api, 'ControlClient', return_value=self.client))
        self.session_factory = self.enterContext(
            patch.object(robot_api, 'ControlSession', return_value=self.session))

    def test_reading_and_planning_do_not_acquire_and_vectors_are_copies(self):
        with Robot.connect('/fake/run') as arm:
            self.assertEqual(arm.state, self.state)
            arm.q[0] = 100.
            arm.pose[0] = 100.
            self.assertEqual(self.state['q'][0], 0.)
            self.assertEqual(self.state['pose'][0], .3)
            self.assertEqual(arm.describe(), self.client.describe.return_value)
            self.assertEqual(arm.plan_movej([.1, 0., 0., 0., 0., 0.]), self.plan)
        self.session_factory.assert_not_called()
        self.client.preview.assert_called_once()
        self.client.request.assert_not_called()

    def test_actions_share_one_session_and_wait_with_per_call_limits(self):
        with Robot('/fake/run', speed=.5, acceleration=2., jerk=10.) as arm:
            result = arm.movej([.1, 0., 0., 0., 0., 0.], speed=0., timeout=3.)
            self.assertEqual(result.result, self.completed)
            arm.movej([0.] * 6)
        self.session_factory.assert_called_once()
        self.session.__enter__.assert_called_once()
        self.assertEqual(self.session.execute.call_count, 2)
        self.assertEqual(self.session.wait.call_count, 2)
        self.assertEqual(self.session.wait.call_args_list[0].args[0], 'example-operation')
        self.assertEqual(self.session.wait.call_args_list[0].kwargs['timeout'], 3.)
        previews = self.client.preview.call_args_list + self.session.preview.call_args_list
        self.assertEqual(len(previews), 2)
        self.assertEqual(previews[0].kwargs['speed'], 0.)
        self.assertEqual(previews[1].kwargs['speed'], .5)
        for preview in previews:
            self.assertEqual(preview.kwargs['acceleration'], 2.)
            self.assertEqual(preview.kwargs['jerk'], 10.)

    def test_async_execute_returns_operation_without_waiting(self):
        with Robot.connect('/fake/run') as arm:
            motion = arm.execute(self.plan, wait=False)
            self.assertEqual(motion.operation_id, self.operation['operation_id'])
            self.assertIsNone(motion.result)
            self.session.wait.assert_not_called()
        self.session.execute.assert_called_once_with(self.plan)

    def test_wait_failure_and_interrupt_cancel_before_propagating(self):
        for failure in (TimeoutError('late feedback'), KeyboardInterrupt()):
            with self.subTest(failure=type(failure).__name__):
                self.session.reset_mock()
                self.session.wait.side_effect = failure
                with Robot.connect('/fake/run') as arm:
                    with self.assertRaises(type(failure)) as caught:
                        arm.execute(self.plan)
                    self.assertIs(caught.exception, failure)
                    self.assertTrue(any(call.args and call.args[0] == 'cancel'
                                        for call in self.session.command.call_args_list))

    def test_execution_error_cancels_before_propagating(self):
        failure = RuntimeError('command connection lost after submission')
        self.session.execute.side_effect = failure
        with Robot.connect('/fake/run') as arm:
            with self.assertRaises(RuntimeError) as caught:
                arm.execute(self.plan)
            self.assertIs(caught.exception, failure)
            self.session.command.assert_any_call('cancel')

    def test_movel_offsets_use_xyzw_orientation_and_preserve_rotation(self):
        # A 90-degree Z rotation maps tool +X to base +Y.
        self.state['pose'] = [.3, .1, .25, 0., 0., math.sqrt(.5), math.sqrt(.5)]
        with Robot.connect('/fake/run') as arm:
            for frame, target in (('base', [.32, .1, .25]), ('tool', [.3, .12, .25])):
                with self.subTest(frame=frame):
                    self.client.preview.reset_mock()
                    self.session.preview.reset_mock()
                    arm.movel(offset=[.02, 0., 0.], frame=frame)
                    previews = self.client.preview.call_args_list + self.session.preview.call_args_list
                    self.assertEqual(len(previews), 1)
                    self.assertEqual(previews[0].args[0], 'movel')
                    pose = previews[0].kwargs['pose']
                    for actual, expected in zip(pose[:3], target):
                        self.assertAlmostEqual(actual, expected)
                    self.assertEqual(pose[3:], self.state['pose'][3:])
            before = self.session.execute.call_count
            with self.assertRaises(ValueError):
                arm.movel(pose=self.state['pose'], frame='tool')
            self.assertEqual(self.session.execute.call_count, before)

    def test_teach_is_a_snapshot_and_foreign_model_return_cannot_move(self):
        self.state['feedback_monotonic_ns'] = 123456
        with Robot.connect('/fake/run') as arm:
            point = arm.teach()
            self.assertEqual(point['feedback_monotonic_ns'], 123456)
            self.assertEqual(point['q'], self.state['q'])
            self.assertEqual(point['model_id'], self.state['model_id'])
            point['q'][0] = .2
            self.assertEqual(self.state['q'][0], 0.)
            point['model_id'] = 'another-model'
            with self.assertRaises(ValueError):
                arm.return_to(point)
        self.session_factory.assert_not_called()
        self.client.preview.assert_not_called()

    def test_absolute_xyz_preserves_measured_orientation_and_does_not_mutate_feedback(self):
        self.state['pose'][3:] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
        with Robot.connect('/fake/run') as arm:
            xyz = arm.xyz
            xyz[2] += .02
            arm.movel(xyz=xyz)
        target = self.client.preview.call_args.kwargs['pose']
        self.assertEqual(target, xyz + self.state['pose'][3:])
        self.assertEqual(self.state['pose'][:3], [.3, 0., .25])
        self.session.execute.assert_called_once_with(self.plan)

    def test_ur_axis_angle_and_xyzw_reach_the_same_wire_pose(self):
        # A rotation around two axes distinguishes a rotation vector from Euler angles.
        vector = [math.pi / math.sqrt(2), math.pi / math.sqrt(2), 0.]
        expected = [math.sqrt(.5), math.sqrt(.5), 0., 0.]
        with Robot.connect('/fake/run') as arm:
            for orientation, quaternion in ((vector, expected), ([0., 0., 0.], [0., 0., 0., 1.]),
                                            ([0., 0., 1e-14], [0., 0., 5e-15, 1.]),
                                            ([0., 0., 0., 2.], [0., 0., 0., 1.])):
                with self.subTest(orientation=orientation):
                    arm.plan_movel([.3, .1, .25] + orientation)
                    target = self.client.preview.call_args.kwargs['pose']
                    self.assertEqual(target[:3], [.3, .1, .25])
                    for actual, wanted in zip(target[3:], quaternion):
                        self.assertAlmostEqual(actual, wanted, places=14)
        self.session_factory.assert_not_called()

    def test_invalid_cartesian_targets_never_submit_a_plan(self):
        with Robot.connect('/fake/run') as arm:
            for options in ({}, {'xyz': [0., 0., 0.], 'offset': [0., 0., .02]},
                            {'pose': self.state['pose'], 'xyz': [0., 0., 0.]},
                            {'xyz': [0., 0., 0.], 'frame': 'tool'},
                            {'xyz': [0., float('nan'), 0.]}, {'xyz': [0., 0.]},
                            {'pose': [0.] * 7}, {'pose': [0.] * 5},
                            {'pose': [0., 0., 0., float('inf'), 0., 0.]}):
                with self.subTest(options=options), self.assertRaises(ValueError):
                    arm.plan_movel(**options)
        self.client.preview.assert_not_called()
        self.session_factory.assert_not_called()

    def test_public_joint_api_uses_connected_state_dimension(self):
        for size in (4, 7):
            with self.subTest(size=size):
                self.state['q'] = [0.] * size
                with Robot.connect('/fake/run') as arm:
                    arm.plan_movej([.1] * size)
                    self.assertEqual(len(self.client.preview.call_args.kwargs['q']), size)
                    arm.jog(size - 1, .1)
                    self.assertEqual(self.session.preview.call_args.kwargs['q'], [0.] * (size - 1) + [.1])
                    with self.assertRaises(ValueError):
                        arm.plan_movej([0.] * (size + 1))
                    with self.assertRaises(ValueError):
                        arm.jog(size, .1)
        # Only the API contract is under test; the current X5 kernel is still six-axis.

    def test_movel_example_routes_xyz_and_ur_pose_without_implementing_motion(self):
        from examples.common import _parser
        from examples.control.movel import run
        robot = MagicMock()
        robot.xyz = [.3, .1, .25]
        args = _parser('movel', '').parse_args(['--xyz', '.3', '.1', '.27'])
        run(robot, xyz=args.xyz)
        self.assertEqual(robot.movel.call_args.kwargs['xyz'], [.3, .1, .27])
        pose = [.3, .1, .27, 0., 0., 0.]
        args = _parser('movel', '').parse_args(['--pose', *map(str, pose)])
        run(robot, pose=args.pose, preview_only=True)
        self.assertEqual(robot.plan_movel.call_args.args[0], pose)
        run(robot, frame='tool')
        self.assertEqual(robot.movel.call_args.kwargs['offset'], [0., 0., .02])
        self.assertIsNone(robot.movel.call_args.kwargs['xyz'])

    def test_record_propagates_incomplete_capture_without_acquiring(self):
        reader = MagicMock()
        with patch('embodied.contracts.runtime.RuntimeReader', return_value=reader), \
             patch('embodied.data.capture.capture.capture', return_value=(
                 Path('/fake/episode'), {'lifecycle': 'failed', 'error': 'observer gap'})):
            with Robot.connect('/fake/run') as arm:
                with self.assertRaises(RuntimeError):
                    arm.record(.2, output='/fake/output')
        self.session_factory.assert_not_called()

    def test_gripper_timeout_cancels_and_refreshes_its_command_ttl(self):
        with Robot.connect('/fake/run') as arm, \
             patch.object(robot_api.time, 'monotonic', side_effect=[0., 0., 0., .1]), \
             patch.object(robot_api.time, 'sleep'):
            with self.assertRaises(TimeoutError):
                arm.gripper.move(0., timeout=.05)
        self.session.command.assert_any_call('gripper', gripper_width=0., ttl_ms=200)
        self.session.command.assert_any_call('cancel')

    def test_managed_simulation_is_closed_without_starting_real_services(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder) / 'run'
            run.mkdir()
            (run / 'observer.sock').touch()
            for viewer in (False, True):
                with self.subTest(viewer=viewer):
                    context = MagicMock()
                    context.__enter__.return_value = run
                    with patch('embodied.applications.workstation.control.sim_kernel', return_value=context) as start, \
                         patch('embodied.core.project.PROJECT_ROOT', Path(folder)), \
                         patch.object(robot_api.tempfile, 'mkdtemp', return_value=folder), \
                         patch('subprocess.Popen', side_effect=AssertionError('unexpected service start')):
                        with Robot.sim(viewer=viewer) as arm:
                            self.assertEqual(arm.run, run)
                            self.assertEqual(arm.backend, 'sim')
                            self.assertEqual(arm.q, self.state['q'])
                        arm.close()
                    start.assert_called_once_with(Path(folder) / 'runtime.log', viewer=viewer)
                    context.__enter__.assert_called_once()
                    context.__exit__.assert_called_once()
        self.session_factory.assert_not_called()

    def test_typed_connections_reject_wrong_or_missing_backend_before_feedback_or_control(self):
        for expected, actual in (('real', 'sim'), ('sim', 'real'), ('real', None), ('sim', 'unknown')):
            with self.subTest(expected=expected, actual=actual):
                self.client.reset_mock()
                self.client.identity = {'runtime_id': 'example-runtime'}
                if actual is not None:
                    self.client.identity['backend'] = actual
                with patch('subprocess.Popen', side_effect=AssertionError('unexpected service start')):
                    with self.assertRaises(ValueError):
                        connection = Robot.real('/fake/run') if expected == 'real' else Robot.sim(run='/fake/run')
                        with connection:
                            self.fail('Wrong backend was accepted')
                self.client.status.assert_not_called()
                self.client.preview.assert_not_called()
                self.session_factory.assert_not_called()

    def test_real_connection_uses_shared_motion_api_without_owning_runtime(self):
        self.client.identity['backend'] = 'real'
        with patch('subprocess.Popen', side_effect=AssertionError('unexpected process start')), \
             patch('subprocess.run', side_effect=AssertionError('unexpected process command')), \
             patch('os.kill', side_effect=AssertionError('unexpected process signal')):
            arm = Robot.real('/fake/run', speed=.5)
            with arm:
                self.assertEqual(arm.backend, 'real')
                self.assertEqual(arm.movej([.1, 0., 0., 0., 0., 0.]).result, self.completed)
                self.assertEqual(arm.movel(xyz=[.3, 0., .27]).result, self.completed)
            arm.close()
        self.session_factory.assert_called_once()
        self.assertEqual(self.session.execute.call_count, 2)
        previews = self.client.preview.call_args_list + self.session.preview.call_args_list
        self.assertEqual([call.args[0] for call in previews], ['movej', 'movel'])
        self.assertTrue(all(call.kwargs['speed'] == .5 for call in previews))
        self.assertEqual(self.session.close.call_count + self.session.__exit__.call_count, 1)
        self.assertTrue(all(call.args[0] in ('cancel', 'release')
                            for call in self.session.command.call_args_list))

    def test_explicit_simulation_run_is_not_started_or_stopped(self):
        with patch('embodied.applications.workstation.control.sim_kernel',
                   side_effect=AssertionError('unexpected simulation start')), \
             patch('subprocess.Popen', side_effect=AssertionError('unexpected process start')), \
             patch('subprocess.run', side_effect=AssertionError('unexpected process command')), \
             patch('os.kill', side_effect=AssertionError('unexpected process signal')):
            arm = Robot.sim(run='/fake/run')
            with arm:
                self.assertEqual(arm.backend, 'sim')
                self.assertEqual(arm.run, Path('/fake/run'))
                arm.execute(self.plan)
            arm.close()
        self.assertEqual(self.session.close.call_count + self.session.__exit__.call_count, 1)
        self.assertTrue(all(call.args[0] in ('cancel', 'release')
                            for call in self.session.command.call_args_list))

    def test_invalid_connection_options_fail_without_io(self):
        constructors = (
            lambda: Robot.real(None),
            lambda: Robot(backend='real'),
            lambda: Robot.sim(run='/fake/run', viewer=True),
            lambda: Robot.connect('/fake/run', viewer=True),
        )
        with patch('subprocess.Popen', side_effect=AssertionError('unexpected process start')):
            for construct in constructors:
                with self.subTest(construct=construct), self.assertRaises(ValueError):
                    construct()
        self.client_factory.assert_not_called()
        self.session_factory.assert_not_called()

    def test_closing_an_existing_run_only_releases_its_session(self):
        # Connecting does not grant ownership of the already-running kernel.
        with patch('subprocess.Popen', side_effect=AssertionError('unexpected process start')), \
             patch('subprocess.run', side_effect=AssertionError('unexpected process command')), \
             patch('os.kill', side_effect=AssertionError('unexpected process signal')):
            arm = Robot.connect('/fake/run')
            with arm:
                arm.execute(self.plan)
            arm.close()  # Idempotent close cannot send a second action or stop the kernel.
        self.assertEqual(self.session.close.call_count + self.session.__exit__.call_count, 1)
        self.assertTrue(all(call.args[0] in ('cancel', 'release')
                            for call in self.session.command.call_args_list))


if __name__ == '__main__':
    unittest.main()
