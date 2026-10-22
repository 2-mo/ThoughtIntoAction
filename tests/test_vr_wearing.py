"""Wear controls keep the terminal responsive and never start robot control."""
from concurrent.futures import Future
from contextlib import nullcontext
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import call, patch

from embodied.applications.workstation import bench, bench_panels as panels


class VRWearingTests(unittest.TestCase):
    def test_unavailable_usb_never_sends_a_wear_command(self):
        for state in ('unauthorized', 'missing', 'offline', 'multiple'):
            with self.subTest(state=state), \
                 patch.object(panels, 'vr_transport_status', return_value={'usb_status': state}), \
                 patch.object(panels, 'transport_command') as command:
                result = panels.request_vr_wearing(True)
                self.assertEqual(result.level, 'error')
                self.assertIn('未发送', str(result))
                command.assert_not_called()

    def test_only_authorized_booted_headset_receives_request_not_claimed_state(self):
        state = {'usb_status': 'ready', 'boot_completed': False}
        with patch.object(panels, 'vr_transport_status', return_value=state), \
             patch.object(panels, 'transport_command', return_value=subprocess.CompletedProcess([], 0, '', '')) as command:
            self.assertIn('仍在开机', str(panels.request_vr_wearing(True)))
            command.assert_not_called()
            state['boot_completed'] = True
            for enabled, action in ((True, 'wear'), (False, 'unwear')):
                result = panels.request_vr_wearing(enabled)
                self.assertEqual(command.call_args.args[0][-3:], ['quest', 'wake', action])
                self.assertIn('实际佩戴状态未读回', str(result))
                self.assertNotEqual(result.level, 'ready')
            command.return_value = subprocess.CompletedProcess([], 1, '', 'device unauthorized')
            self.assertEqual(panels.request_vr_wearing(True).level, 'error')

    def test_wear_requests_run_in_background_and_cancel_can_follow_pending_wear(self):
        jobs = []
        def submit(fn, *args):
            future = Future()
            jobs.append((fn, args, future))
            return future
        with patch.object(panels, 'ThreadPoolExecutor') as executor:
            executor.return_value.submit.side_effect = submit
            vr = panels.VRConnection()
            vr.set_wearing(True)
            vr.snapshot()
            self.assertIs(jobs[0][0], panels.request_vr_wearing)
            self.assertTrue(jobs[0][1][0])
            vr.set_wearing(False)
            vr.snapshot()
            self.assertEqual(len(jobs), 1)  # Never block or overlap a pending ADB call.
            jobs[0][2].set_result(panels.Status('wear sent', 'waiting'))
            vr.snapshot()
            self.assertIs(jobs[1][0], panels.request_vr_wearing)
            self.assertFalse(jobs[1][1][0])
            jobs[1][2].set_result(panels.Status('unwear sent', 'waiting'))
            vr.snapshot()
            self.assertEqual(str(vr.wear_message), 'unwear sent')
            vr.close()

    def test_vr_window_w_and_u_are_available_without_a_robot(self):
        with patch.object(panels.sys.stdin, 'isatty', return_value=True), \
             patch.object(panels.sys.stdout, 'isatty', return_value=True), \
             patch('embodied.core.terminal_keys.terminal_keys', return_value=nullcontext()), \
             patch('embodied.core.terminal_keys.read_key', side_effect=['w', 'u', 'eof']), \
             patch('rich.live.Live'), patch.object(panels, 'VRConnection') as connection, \
             patch.object(panels, 'vr_rows', return_value=([], [])), \
             patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench, 'start_kernel') as start:
            self.assertEqual(panels.main(['vr', '--serve']), 0)
            self.assertEqual(connection.return_value.set_wearing.call_args_list, [call(True), call(False)])
            start.assert_not_called()

    def test_wear_and_unwear_script_do_not_launch_or_stop_apps(self):
        script = panels.ROOT / 'scripts/quest/simulate_wearing.sh'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / 'calls'
            adb = root / 'adb'
            adb.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$WEAR_TEST_LOG"\n')
            adb.chmod(0o755)
            env = {**os.environ, 'PATH': str(root) + os.pathsep + os.environ['PATH'], 'WEAR_TEST_LOG': str(log)}
            for action, expected in (
                ('wear', ['-d shell input keyevent KEYCODE_WAKEUP',
                          '-d shell am broadcast -a com.oculus.vrpowermanager.prox_close']),
                ('unwear', ['-d shell am broadcast -a com.oculus.vrpowermanager.prox_far']),
            ):
                log.write_text('')
                subprocess.run(['bash', str(script), action], env=env, check=True, timeout=2)
                self.assertEqual(log.read_text().splitlines(), expected)


if __name__ == '__main__':
    unittest.main()
