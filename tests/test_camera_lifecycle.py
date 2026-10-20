"""Shutdown identity and recording protections; no physical cameras or signals."""
from contextlib import ExitStack
import os
import signal
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from embodied.sensors import lifecycle


class CameraLifecycleTests(unittest.TestCase):
    def test_real_graceful_shutdown_of_dummy_socket_owner(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'camera.sock'
            config = Path(folder) / 'local.toml'
            config.write_text(f'[camera_service]\nsocket = "{path}"\n[runtime]\nsocket = "{folder}/no-teleop.sock"\n')
            code = '''import signal,sys,threading
from embodied.core.local_server import LocalServer
stop=threading.Event()
signal.signal(signal.SIGTERM,lambda *_:stop.set())
server=LocalServer(sys.argv[-1],lambda _: {})
print('ready',flush=True)
stop.wait()
server.close()
'''
            with patch.dict(os.environ, {'QUEST_TELEOP_CONFIG': str(config),
                                        'PYTHONPATH': str(Path(__file__).resolve().parents[1] / 'src')}):
                child = subprocess.Popen([sys.executable, '-B', '-c', code, 'embodied.sensors.service', str(path)],
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                try:
                    self.assertEqual(child.stdout.readline().strip(), 'ready')
                    lifecycle.stop_service()
                    self.assertEqual(child.wait(timeout=3), 0)
                    self.assertFalse(path.exists())
                finally:
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=3)
                    child.stdout.close()
                    child.stderr.close()

    def setup_mocks(self, stack, argv=b'python\0-m\0embodied.sensors.service\0'):
        stack.enter_context(patch.object(lifecycle, 'DeviceLease'))
        stack.enter_context(patch.object(lifecycle, 'recording_guard'))
        guard = stack.enter_context(patch.object(lifecycle, 'check_legacy_session'))
        sock = stack.enter_context(patch.object(lifecycle.socket, 'socket')).return_value.__enter__.return_value
        sock.getsockopt.return_value = struct.pack('3i', 12345, os.getuid(), 0)
        stack.enter_context(patch.object(lifecycle.os, 'pidfd_open', return_value=98, create=True))
        stack.enter_context(patch.object(lifecycle.os, 'close'))
        stack.enter_context(patch.object(lifecycle.Path, 'read_bytes', return_value=argv))
        send = stack.enter_context(patch.object(lifecycle.signal, 'pidfd_send_signal', create=True))
        wait = stack.enter_context(patch.object(lifecycle.select, 'select', return_value=([98], [], [])))
        return guard, send, wait

    def test_sigterm_only_after_recording_and_identity_checks(self):
        with ExitStack() as stack:
            guard, send, wait = self.setup_mocks(stack)
            lifecycle.stop_service()
            guard.assert_called_once()
            send.assert_called_once_with(98, signal.SIGTERM)
            wait.assert_called_once_with([98], [], [], 15)

    def test_recording_or_wrong_owner_prevents_signal(self):
        for recording in (True, False):
            with self.subTest(recording=recording), ExitStack() as stack:
                guard, send, _ = self.setup_mocks(stack, argv=b'python\0unrelated.py\0')
                if recording:
                    guard.side_effect = RuntimeError('正在录制')
                with self.assertRaises(RuntimeError):
                    lifecycle.stop_service()
                send.assert_not_called()

    def test_timeout_does_not_force_kill(self):
        with ExitStack() as stack:
            _, send, wait = self.setup_mocks(stack)
            wait.return_value = ([], [], [])
            with self.assertRaisesRegex(RuntimeError, '未强制'):
                lifecycle.stop_service()
            send.assert_called_once_with(98, signal.SIGTERM)


if __name__ == '__main__':
    unittest.main()
