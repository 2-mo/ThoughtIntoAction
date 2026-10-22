"""Motion CLI routes common operations through Robot; no devices are opened."""
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, call, patch

from embodied.applications.operations import cli


class MotionCliTests(unittest.TestCase):
    def setUp(self):
        self.plan = {'plan_id': 'test-plan', 'times': [0., 1.2],
                     'limits': {'speed': .5, 'acceleration': 2., 'jerk': 10.},
                     'metrics': {'duration_s': 1.2, 'peak_velocity_rad_s': .36}}
        self.robot = MagicMock()
        self.robot.q = [0.] * 6
        self.robot.describe.return_value = {'motion_limits': {
            'defaults': {'speed': .15, 'acceleration': 2., 'jerk': 10.},
            'joint_velocity_limits_rad_s': [5., 5., 5.5, 5.5, 5., 5.]}}
        self.robot.plan_movej.return_value = self.robot.plan_movel.return_value = self.plan
        self.motion = MagicMock(operation_id='test-operation')
        self.motion.wait.return_value = {'operation_id': 'test-operation', 'status': 'completed'}
        for method in ('execute', 'home', 'jog', 'return_to'):
            getattr(self.robot, method).return_value = self.motion
        self.context = MagicMock()
        self.context.__enter__.return_value = self.robot

    def invoke(self, arguments):
        with patch.object(cli.Robot, 'connect', return_value=self.context) as connect, \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            result = cli.main(['--run', '/fake/run', *arguments])
        connect.assert_called_once_with(Path('/fake/run'))
        return result, output.getvalue()

    def assert_no_motion(self):
        for name in ('plan_movej', 'plan_movel', 'execute', 'home', 'jog', 'return_to'):
            getattr(self.robot, name).assert_not_called()

    def test_every_planned_motion_forwards_explicit_limits_through_robot(self):
        with tempfile.TemporaryDirectory() as folder:
            point = Path(folder) / 'point.json'
            point.write_text(json.dumps({'q': [0.] * 6, 'model_id': 'test-model', 'gripper_width': .03}))
            for command, arguments, method in (
                    ('preview', ['--q', *(['0'] * 6)], 'plan_movej'),
                    ('movej', ['--q', *(['0'] * 6)], 'plan_movej'),
                    ('movel', ['--pose', '0', '0', '.2', '0', '0', '0', '1'], 'plan_movel'),
                    ('home', [], 'home'),
                    ('jog', ['--joint', '2', '--step', '.01'], 'jog'),
                    ('return', ['--point', str(point)], 'return_to')):
                with self.subTest(command=command):
                    self.robot.reset_mock()
                    self.motion.reset_mock()
                    result, output = self.invoke([command, *arguments, '--speed', '.5', '--acceleration', '2', '--jerk', '10'])
                    self.assertEqual(result, 0)
                    selected = getattr(self.robot, method)
                    self.assertEqual({key: selected.call_args.kwargs[key] for key in self.plan['limits']}, self.plan['limits'])
                    if command == 'preview':
                        self.robot.execute.assert_not_called()
                        self.motion.wait.assert_not_called()
                    else:
                        self.assertEqual([json.loads(line)['status'] for line in output.splitlines()], ['running', 'completed'])
                        self.motion.wait.assert_called_once_with()
                        if command in ('movej', 'movel'):
                            self.robot.execute.assert_called_once_with(self.plan, wait=False)
                        else:
                            self.assertIs(selected.call_args.kwargs['wait'], False)
                    if command == 'jog':
                        self.assertEqual(selected.call_args.args, (1, .01))

    def test_omitted_limits_stay_with_kernel_defaults_and_preview_summary_keeps_metrics(self):
        result, output = self.invoke(['preview', '--q', *(['0'] * 6)])
        self.assertEqual(result, 0)
        self.robot.describe.assert_not_called()
        self.robot.plan_movej.assert_called_once_with([0.] * 6)
        summary = json.loads(output)
        self.assertEqual(summary['limits'], self.plan['limits'])
        self.assertEqual(summary['metrics'], self.plan['metrics'])
        self.robot.execute.assert_not_called()

    def test_old_kernel_rejects_tuning_before_acquiring_or_executing(self):
        self.robot.describe.return_value = {'robot': {}}
        for option in ('--acceleration', '--jerk'):
            with self.subTest(option=option), patch('sys.stderr', new_callable=io.StringIO) as error:
                with self.assertRaises(SystemExit) as stopped:
                    self.invoke(['home', option, '1'])
                self.assertEqual(stopped.exception.code, 2)
                self.assertIn('当前内核不支持', error.getvalue())
                self.assert_no_motion()

    def test_sdk_speed_request_is_forwarded_including_zero(self):
        for speed in ('0', '20'):
            with self.subTest(speed=speed):
                result, _ = self.invoke(['home', '--speed', speed])
                self.assertEqual(result, 0)
                self.robot.home.assert_called_with(wait=False, speed=float(speed))
                self.robot.describe.assert_called()

    def test_old_or_malformed_sdk_capability_rejects_speed_before_acquiring(self):
        capabilities = [None, [1.] * 5, [1.] * 7, [float('nan')] * 6,
                        [float('inf')] * 6, [0.] * 6, [-1.] * 6, [True] * 6]
        for speed in ('0', '2'):
            for capability in capabilities:
                self.robot.describe.return_value = {'motion_limits': {
                    'maximum': self.plan['limits'], 'joint_velocity_limits_rad_s': capability}}
                with self.subTest(speed=speed, capability=capability), \
                     patch('sys.stderr', new_callable=io.StringIO) as error:
                    with self.assertRaises(SystemExit) as stopped:
                        self.invoke(['home', '--speed', speed])
                    self.assertEqual(stopped.exception.code, 2)
                    self.assertIn('逐轴速度能力', error.getvalue())
                    self.assert_no_motion()
        self.robot.q = [0.] * 7
        self.robot.describe.return_value = {'motion_limits': {'joint_velocity_limits_rad_s': [1.] * 7}}
        self.assertEqual(self.invoke(['home', '--speed', '0'])[0], 0)

    def test_cartesian_targets_share_robot_conventions_and_reject_ambiguous_arguments(self):
        for flag, values, frame in (
                ('--pose', [0, 0, .2, 0, 0, 0], 'base'),
                ('--pose', [0, 0, .2, 0, 0, 0, 1], 'base'),
                ('--xyz', [.1, 0, .3], 'base'),
                ('--offset', [0, 0, .02], 'tool')):
            with self.subTest(flag=flag, size=len(values)):
                self.robot.reset_mock()
                self.assertEqual(self.invoke(['preview', flag, *map(str, values), '--frame', frame])[0], 0)
                target = {'pose': None, 'xyz': None, 'offset': None, 'frame': frame}
                target[flag[2:]] = values
                self.robot.plan_movel.assert_called_once_with(**target)
        for arguments in (
                ['movel', '--xyz', '0', '0', '0', '--offset', '0', '0', '0'],
                ['movel', '--xyz', '0', '0', '0', '--frame', 'tool'],
                ['movej', '--pose', '0', '0', '0', '0', '0', '0'],
                ['movel', '--pose', '0', '0', '0', '0', '0'],
                ['jog', '--joint', '0', '--step', '.01']):
            with self.subTest(arguments=arguments), patch.object(cli.Robot, 'connect') as connect, \
                 patch('sys.stderr', new_callable=io.StringIO):
                with self.assertRaises(SystemExit):
                    cli.main(['--run', '/fake/run', *arguments])
                connect.assert_not_called()

    def test_preview_and_teach_are_read_only_and_never_overwrite_files(self):
        from embodied.applications.operations import robot as robot_api
        state = {'q': [0.] * 6, 'pose': [0., 0., .2, 0., 0., 0., 1.], 'model_id': 'test-model',
                 'gripper_width': .03, 'feedback_monotonic_ns': 12345}
        client = MagicMock()
        client.status.return_value = state
        client.preview.return_value = self.plan
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(robot_api, 'ControlClient', return_value=client), \
             patch.object(robot_api, 'ControlSession') as acquire, \
             patch('sys.stdout', new_callable=io.StringIO):
            for command, arguments, expected in (
                    ('teach', [], state), ('preview', ['--q', *(['0'] * 6)], self.plan)):
                path = Path(folder) / (command + '.json')
                argv = ['--run', '/fake/run', command, *arguments, '--output', str(path)]
                self.assertEqual(cli.main(argv), 0)
                self.assertEqual(json.loads(path.read_text()), expected)
                with self.assertRaises(FileExistsError):
                    cli.main(argv)
                self.assertEqual(json.loads(path.read_text()), expected)
            acquire.assert_not_called()

    def test_gripper_and_saved_plan_use_robot_operations(self):
        self.assertEqual(self.invoke(['gripper', '--width', '.03'])[0], 0)
        self.robot.gripper.move.assert_called_once_with(.03)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'plan.json'
            path.write_text(json.dumps(self.plan))
            result, output = self.invoke(['execute', '--plan', str(path)])
        self.assertEqual(result, 0)
        self.robot.execute.assert_called_once_with(self.plan, wait=False)
        self.assertEqual(len(output.splitlines()), 2)

    def test_stop_and_free_drive_keep_separate_administrative_sessions(self):
        with patch.object(cli.Robot, 'connect') as connect, \
             patch.object(cli, 'ControlClient') as client, \
             patch.object(cli, 'ControlSession') as context, \
             patch('sys.stdout', new_callable=io.StringIO):
            client.return_value.request.return_value = {'stopped': True}
            self.assertEqual(cli.main(['--run', '/fake/run', 'stop']), 0)
            client.return_value.request.assert_called_once_with('stop')
            context.assert_not_called()
            session = context.return_value.__enter__.return_value
            with patch.object(cli.time, 'monotonic', side_effect=[0., 0., 1.]), patch.object(cli.time, 'sleep'):
                self.assertEqual(cli.main(['--run', '/fake/run', 'free-drive', '--duration', '.1']), 0)
            self.assertEqual(session.command.call_args_list, [call('free-drive', ttl_ms=250), call('hold')])
            connect.assert_not_called()


if __name__ == '__main__':
    unittest.main()
