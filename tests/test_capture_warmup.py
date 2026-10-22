"""Cold image imports precede recording; frame sockets belong to reader threads."""
import os
from pathlib import Path
import subprocess
import sys
import textwrap
import unittest


class CaptureWarmupTests(unittest.TestCase):
    def test_cold_dependencies_are_ready_before_capture_without_opening_devices(self):
        script = textwrap.dedent('''
            import sys
            import tempfile
            import threading
            import time
            from unittest.mock import patch
            from embodied.devices.sensors.client import SharedCamera

            assert 'numpy' not in sys.modules and 'zmq' not in sys.modules
            metadata = {'serial': 'test', 'width': 2, 'height': 2, 'fps': 30,
                        'stream_id': 'test-stream', 'zmq_frame_protocol': 1}
            state = {'camera': {'id': 'wrist_rgb', 'metadata': metadata}}
            with patch('embodied.devices.sensors.client.ensure_service'), \\
                 patch('embodied.devices.sensors.client.request', return_value=state):
                with SharedCamera('test', 2, 2) as camera:
                    assert 'numpy' in sys.modules and 'zmq' in sys.modules
                    assert camera.frames is None  # __enter__ creates no ZMQ socket.
                    from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
                    with tempfile.TemporaryDirectory() as folder:
                        writer = RuntimeEpisodeWriter(folder, {'runtime_id': 'test'}, 'warmup', metadata)
                        assert 'cv2' in sys.modules
                        constructed, received, errors = [], [], []
                        class Frames:
                            def __init__(self, path):
                                constructed.append(threading.get_ident())
                            def read(self, camera_id):
                                assert threading.get_ident() == constructed[0]
                                return {'host_received_monotonic_ns': time.monotonic_ns(),
                                        'frame_number': 1, 'stream_id': 'test-stream'}, bytes(12)
                            def close(self):
                                pass
                        def read():
                            try:
                                received.append(camera.read())
                            except BaseException as error:
                                errors.append(error)
                        with patch('embodied.devices.sensors.transport.FrameClient', Frames):
                            reader = threading.Thread(target=read)
                            reader.start()
                            reader.join(timeout=2)
                            assert not reader.is_alive() and not errors, errors
                        assert constructed == [reader.ident]
                        assert reader.ident != threading.get_ident()
                        assert received[0]['image'].shape == (2, 2, 3)
                        writer.add_frame(received[0])
                        writer.close('stream-error', 'offline test has no robot stream')
        ''')
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run([sys.executable, '-B', '-c', script], cwd=root,
                                env={**os.environ, 'PYTHONPATH': str(root / 'src')},
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
