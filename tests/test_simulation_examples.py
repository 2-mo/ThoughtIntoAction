"""Simulation selection and lifetime checks with fake runtimes only."""
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from embodied.applications.workstation import control
from examples import common
from examples.simulation import control as simulation


class SimulationExamplesTests(unittest.TestCase):
    def test_viewer_changes_only_display_while_kernel_remains_simulation_idle(self):
        for viewer in (False, True):
            with self.subTest(viewer=viewer), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                runtime = MagicMock(pid=12345)
                runtime.poll.return_value = None
                for name, pid in (('owned', runtime.pid), ('foreign', 99999)):
                    directory = root / 'run' / name
                    directory.mkdir(parents=True)
                    (directory / 'runtime.json').write_text(json.dumps({'supervisor_pid': pid}))
                    (directory / 'runtime.ready').touch()
                with patch.object(control, 'CONTROL', root), \
                     patch.object(control.subprocess, 'Popen', return_value=runtime) as start, \
                     patch.object(control.time, 'sleep'), \
                     patch.dict(os.environ, {'DISPLAY': ':99'}):
                    with control.sim_kernel(root / 'runtime.log', viewer=viewer) as directory:
                        self.assertEqual(directory, root / 'run/owned')
                        runtime.send_signal.assert_not_called()
                    command = start.call_args.args[0]
                    self.assertEqual(command[command.index('--backend') + 1], 'sim')
                    self.assertEqual(command[command.index('--input') + 1], 'idle')
                    self.assertIn('--managed', command)
                    self.assertEqual('--headless' in command, not viewer)
                    runtime.send_signal.assert_called_once_with(signal.SIGINT)
                    runtime.wait.assert_called_once()

    def test_native_viewer_without_display_fails_before_process_start(self):
        with patch.object(control.sys, 'platform', 'linux'), \
             patch.dict(os.environ, {'DISPLAY': ''}), \
             patch.object(control.subprocess, 'Popen') as start:
            with self.assertRaisesRegex(ValueError, 'DISPLAY'):
                with control.sim_kernel('/unused/runtime.log', viewer=True):
                    self.fail('A desktop viewer was accepted without DISPLAY')
        start.assert_not_called()

    def test_common_cli_rejects_ambiguous_connections_and_noninteractive_wait_before_io(self):
        for args in (['--backend', 'real'], ['--viewer', '--run', '/fake/run'],
                     ['--viewer', '--backend', 'real', '--run', '/fake/run'], ['--keep-open']):
            with self.subTest(args=args), patch.object(common, 'Robot') as robot, \
                 patch.object(common.sys.stdin, 'isatty', return_value=False), \
                 patch('builtins.input', side_effect=AssertionError('unexpected prompt')), \
                 redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    common.run_example('read_state', lambda robot: robot.state, args)
                self.assertEqual(caught.exception.code, 2)
                self.assertEqual(robot.mock_calls, [])

    def test_common_cli_defaults_to_sim_and_requires_explicit_real_selection(self):
        for args, backend in (([], 'sim'), (['--run', '/fake/run'], 'sim'),
                              (['--backend', 'real', '--run', '/fake/run'], 'real')):
            with self.subTest(args=args), patch.object(common, 'Robot') as robot, \
                 redirect_stdout(io.StringIO()):
                connection = getattr(robot, backend).return_value
                demo = MagicMock(return_value={})
                self.assertEqual(common.run_example('read_state', demo, args), 0)
                demo.assert_called_once_with(connection.__enter__.return_value)
                getattr(robot, 'real' if backend == 'sim' else 'sim').assert_not_called()
                connection.__exit__.assert_called_once()

    def test_visual_simulation_requires_terminal_before_creating_kernel(self):
        with patch.object(simulation, 'Robot') as robot, \
             patch.object(simulation.sys.stdin, 'isatty', return_value=False), \
             redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as caught:
                simulation.main([])
        self.assertEqual(caught.exception.code, 2)
        self.assertEqual(robot.mock_calls, [])

    def test_headless_simulation_needs_no_terminal_or_keyboard_input(self):
        with patch.object(simulation, 'Robot') as robot, \
             patch.object(simulation, 'run', return_value={'q': [0.] * 6}) as run, \
             patch.object(simulation.sys.stdin, 'isatty', return_value=False), \
             patch('builtins.input', side_effect=AssertionError('unexpected prompt')), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(simulation.main(['--headless']), 0)
        robot.sim.assert_called_once_with(viewer=False, speed=.5)
        robot.real.assert_not_called()
        run.assert_called_once()
        robot.sim.return_value.__exit__.assert_called_once()


if __name__ == '__main__':
    unittest.main()
