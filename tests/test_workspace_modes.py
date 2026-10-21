"""Mode selection must stop owned control before starting a simulator."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/workspace'))
import dashboard
from embodied.monitor import mode, control
w = dashboard.workspace


class ModeTests(unittest.TestCase):
    def setUp(self):
        selection = patch.object(w, 'selected_mode', return_value='real')
        selection.start()
        self.addCleanup(selection.stop)

    def test_persistent_selection_defaults_to_existing_real_workflow(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(mode, 'mode_path', return_value=Path(folder) / 'mode'):
            self.assertEqual(mode.selected_mode(), 'real')
            mode.save_mode('simulation')
            self.assertEqual(mode.selected_mode(), 'simulation')
            with self.assertRaises(ValueError):
                mode.save_mode('invalid')
            self.assertEqual(mode.selected_mode(), 'simulation')

    def test_simulation_switch_orders_shutdown_before_viewer(self):
        events = []
        with patch.object(w, 'exists', return_value=True), \
             patch.object(w, 'stop_pane', side_effect=lambda role: events.append(role)), \
             patch.object(w, 'teleop_running', return_value=False), \
             patch.object(w, 'arm_info', return_value=None), \
             patch.object(w, 'save_mode', side_effect=lambda value: events.append(value)), \
             patch.object(control, 'command', side_effect=lambda *a, **k: events.append('viewer') or 'ok'):
            w.switch_mode('simulation')
        self.assertEqual(events, ['teleop', 'arm', 'simulation', 'viewer'])

    def test_external_teleop_blocks_switch_and_preserves_arm(self):
        with patch.object(w, 'exists', return_value=True), patch.object(w, 'stop_pane') as stop, \
             patch.object(w, 'teleop_running', return_value=True), patch.object(w, 'save_mode') as save, \
             patch.object(control, 'command') as viewer:
            with self.assertRaisesRegex(RuntimeError, '外部遥操'):
                w.switch_mode('simulation')
        stop.assert_called_once_with('teleop')
        save.assert_not_called()
        viewer.assert_not_called()

    def test_incomplete_stop_does_not_commit_selection(self):
        with patch.object(w, 'exists', return_value=True), \
             patch.object(w, 'stop_pane', side_effect=RuntimeError('still saving')), \
             patch.object(w, 'save_mode') as save, patch.object(control, 'command') as viewer:
            with self.assertRaisesRegex(RuntimeError, 'still saving'):
                w.switch_mode('simulation')
        save.assert_not_called()
        viewer.assert_not_called()

    def test_return_to_real_only_switches_viewer_without_enabling_arm(self):
        with patch.object(w, 'save_mode') as save, patch.object(control, 'command', return_value='ok') as viewer, \
             patch.object(w, 'start_role') as start, patch.object(w, 'tmux') as tmux:
            w.switch_mode('real')
        save.assert_called_once_with('real')
        viewer.assert_called_once_with('start', mode='real')
        start.assert_not_called()
        tmux.assert_not_called()

    def test_simulation_cannot_forward_arm_commands(self):
        with patch.object(w, 'selected_mode', return_value='simulation'), patch.object(w, 'tmux') as tmux, patch.object(control, 'command', return_value='仿真待机') as viewer:
            self.assertIn('仿真', w.start_role('arm'))
            self.assertIn('仿真', w.start_role('teleop'))
            with patch.object(dashboard, 'viewer_command', return_value='sim') as virtual:
                dashboard.arm_key('h')
                virtual.assert_called_once_with('sim-home')
            with self.assertRaisesRegex(RuntimeError, '真机阻尼'):
                dashboard.arm_key('d')
        tmux.assert_not_called()
        self.assertEqual(viewer.call_count, 2)

    def test_switch_keeps_settled_physical_service_connected(self):
        with patch.object(w, 'exists', return_value=True), patch.object(w, 'stop_pane') as stop, \
             patch.object(w, 'teleop_running', return_value=False), \
             patch.object(w, 'arm_info', return_value={'mode': 'hold', 'owner': None}), \
             patch.object(w, 'save_mode'), patch.object(control, 'command', return_value='ok'):
            w.switch_mode('simulation')
        stop.assert_called_once_with('teleop')

    def test_simulation_panes_do_not_prepare_hardware(self):
        with patch.dict('os.environ', {'TMUX_PANE': '%9'}), \
             patch.object(w, 'tmux', return_value=subprocess.CompletedProcess([], 0, stdout='1\n')), \
             patch.object(w, 'check_cancelled'), patch.object(w, 'selected_mode', return_value='simulation'), \
             patch.object(w, 'execute_details') as details, \
             patch.object(w, 'execute') as execute, patch.object(w, 'can_connected') as can:
            w.run_pane('arm')
            w.run_pane('teleop')
        can.assert_not_called()
        execute.assert_not_called()
        self.assertEqual([call.args for call in details.call_args_list], [('arm',), ('teleop',)])

    def test_viewer_replacement_reuses_port_and_explicit_mode(self):
        info = {'pid': 123, 'url': 'http://127.0.0.1:18081/', 'mode': 'real'}
        replacement = {**info, 'pid': 456, 'mode': 'simulation'}
        with tempfile.TemporaryDirectory() as folder, patch.object(control, 'DeviceLease'), \
             patch.object(control, 'load_config', return_value={'paths': {'log_dir': folder}, 'viewer_service': {'port': 18081}}), \
             patch.object(control, 'status', side_effect=[info, None, None, replacement]), \
             patch.object(control, 'request') as request, patch.object(control.time, 'sleep'), \
             patch.object(control.subprocess, 'Popen') as spawn:
            spawn.return_value.poll.return_value = None
            result = control.command('start', mode='simulation')
        self.assertIn('仿真', result)
        request.assert_called_once()
        self.assertEqual(request.call_args.args[1], 'stop')
        args = spawn.call_args.args[0]
        self.assertEqual(args[args.index('--mode') + 1], 'simulation')
        self.assertEqual(args[args.index('--port') + 1], '18081')
