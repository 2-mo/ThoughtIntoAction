"""Pair composition and receive-only display checks with isolated fake streams."""
import json
import os
from pathlib import Path
import socket
import tempfile
import time
from types import SimpleNamespace
import unittest

from embodied.applications.workstation.robot_pair import prepare
from embodied.applications.workstation.pair_state import active_pair
from embodied.applications.monitor.pair_view import PairView


class Reader:
    def __init__(self, run, mode, timeout):
        self.sock, self.sender = socket.socketpair()
        self.descriptor = {'backend': 'sim'}
        self.tick = {'terminal': False, 'error': None,
            'feedback': {'q': ([.1]*6 if run == 'left' else [-.2]*6), 'gripper_width': .07},
            'timing': {'feedback_received_ns': time.monotonic_ns()}}
        self.sender.send(b'1')
    def read(self):
        self.sock.recv(1)
        return self.tick
    def close(self):
        self.sock.close(); self.sender.close()


class PairTests(unittest.TestCase):
    def test_deployments_share_only_input_and_keep_hardware_explicit(self):
        with tempfile.TemporaryDirectory() as folder:
            prefix, path, configs = prepare(folder)
            text = path.read_text()
            self.assertEqual(text.count('[[channels]]'), 2)
            self.assertIn('left_controller', text)
            for side, path in configs.items():
                config = json.loads(path.read_text())
                self.assertEqual(config['robot'], 'realman.real')
                self.assertEqual(config['pair']['side'], side)
                vr = json.loads(Path(config['vr_config']).read_text())
                self.assertEqual((vr['controller'], vr['input_prefix']), (side, prefix))

    def test_manifest_requires_both_live_matching_runs_of_this_boot(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            state = dict(schema=1, runtime_id='pair-test', pid=os.getpid(), boot_id=boot, runs={})
            for side in ('left', 'right'):
                run = root/side; run.mkdir()
                state['runs'][side] = str(run)
                (run/'runtime.ready').touch()
                (run/'runtime.json').write_text(json.dumps({'boot_id': boot, 'supervisor_pid': os.getpid()}))
                (run/'workbench.json').write_text(json.dumps({'pair': {'id': 'pair-test', 'side': side}}))
            manifest = root/'pair.json'; manifest.write_text(json.dumps(state))
            self.assertEqual(active_pair(manifest), state)
            (root/'left/runtime.ready').unlink()
            self.assertIsNone(active_pair(manifest))

    def test_two_standard_streams_render_together_and_stale_side_is_withdrawn(self):
        state = {'runtime_id': 'pair-test', 'runs': {'left': 'left', 'right': 'right'}}
        packets = []
        model = SimpleNamespace(metadata=lambda: {})
        scene = SimpleNamespace(robot=None, _manifest={},
            set_model=lambda value: setattr(scene, 'robot', value), accept=packets.append,
            set_stream_error=lambda error: None)
        view = PairView(finder=lambda: state, reader_factory=Reader, model_factory=lambda hands: model)
        try:
            self.assertTrue(view.step(scene))
            self.assertEqual(packets[-1]['actual'], [.1]*6+[-.2]*6)
            self.assertEqual(packets[-1]['source']['kind'], 'virtual')
            self.assertIn('left', view.monitor()['runtime_runs'])
            view.ticks['left']['timing']['feedback_received_ns'] -= 1_000_000_000
            view.step(scene)
            self.assertIsNone(scene.robot)
            self.assertIsNone(view.view.latest)
        finally:
            view.close()


if __name__ == '__main__': unittest.main()
