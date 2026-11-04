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
        self.descriptor = {'backend': 'sim', 'parameters': {'effective_robot_config': {'gripper_max_width': .04}}}
        self.tick = {'terminal': False, 'error': None,
            'feedback': {'q': ([.1]*6 if run == 'left' else [-.2]*6),
                         'dq': ([.3]*6 if run == 'left' else [-.4]*6), 'gripper_width': .07},
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

    def test_two_standard_streams_keep_model_identity_across_a_feedback_gap(self):
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
            self.assertEqual(packets[-1]['actual_vel'], [.3]*6+[-.4]*6)
            self.assertIsNone(packets[-1]['target_vel'])
            self.assertEqual(packets[-1]['source']['kind'], 'virtual')
            self.assertEqual(packets[-1]['source']['input'], 'idle')
            for reader in view.readers.values():
                reader.descriptor['native'] = {'device': 'quest.shared'}
            view.step(scene)
            self.assertEqual(packets[-1]['source']['input'], 'quest')
            self.assertIn('left', view.monitor()['runtime_runs'])
            view.ticks['left']['timing']['feedback_received_ns'] -= 1_000_000_000
            view.step(scene)
            self.assertIs(scene.robot, model)
            self.assertIsNone(view.view.latest)
            for tick in view.ticks.values():
                tick['timing']['feedback_received_ns'] = time.monotonic_ns()
            view.step(scene)
            self.assertIs(scene.robot, model)
            self.assertIsNotNone(view.view.latest)
        finally:
            view.close()

    def test_pair_monitor_uses_each_hands_actual_input_and_expires_independently(self):
        from unittest.mock import patch
        view = PairView()
        view.identity = {'runtime_id': 'pair-test', 'runs': {'left': 'left', 'right': 'right'}}
        stamp = time.monotonic_ns()
        for side in ('left', 'right'):
            view.ticks[side] = {'input': {'sample_monotonic_ns': stamp,
                'device_timestamp_ns': 123, 'hands': {side: {'grip': .4, 'trigger': .2}}}}
        with patch('embodied.applications.workstation.pair_state.selected_run', return_value=None):
            payload = view.monitor()
            self.assertEqual(set(payload['input']['hands']), {'left', 'right'})
            self.assertIsNone(payload['input_error'])
            view.ticks['left']['input']['sample_monotonic_ns'] -= 1_000_000_000
            payload = view.monitor()
            self.assertIsNone(payload['input']['hands']['left'])
            self.assertEqual(payload['input']['hands']['right']['trigger'], .2)
            view.ticks['right']['input']['sample_monotonic_ns'] -= 1_000_000_000
            self.assertIsNone(view.monitor()['input'])

    def test_pair_gripper_feedback_reaches_scene_with_independent_widths(self):
        from embodied.applications.monitor.realman_view import display_model
        from embodied.applications.monitor.scene import Scene
        state = {'runtime_id': 'pair-gripper-test', 'runs': {'left': 'left', 'right': 'right'}}
        with tempfile.TemporaryDirectory() as folder:
            view = PairView(finder=lambda: state, reader_factory=Reader,
                            model_factory=lambda hands: display_model(hands, folder))
            scene = Scene(None)
            try:
                view.step(scene)
                for side, width in (('left', .01), ('right', .03)):
                    view.ticks[side]['feedback']['gripper_width'] = width
                    view.ticks[side]['command'] = {'gripper_width': .02}
                    view.ticks[side]['timing']['feedback_received_ns'] = time.monotonic_ns()
                view.sequence += 1
                view.step(scene)
                arms = {a['hand']: a for a in scene.snapshot()['command_details']['realman']}
                self.assertEqual(arms['left']['gripper_width_m'], .01)
                self.assertEqual(arms['right']['gripper_width_m'], .03)
                self.assertEqual(arms['right']['gripper_max_width_m'], .04)
                grips = {m['gripper']['side']: m for m in scene._manifest['meshes'] if 'gripper' in m}
                self.assertEqual(set(grips), {'left', 'right'})
                self.assertEqual(grips['left']['link'], 'left_tool_palm')
                self.assertEqual(scene.robot.model.nq, 12)  # Arm joint contract is unchanged.
                view.ticks['right']['feedback']['gripper_width'] = None
                for tick in view.ticks.values():
                    tick['timing']['feedback_received_ns'] = time.monotonic_ns()
                view.sequence += 1
                view.step(scene)
                arms = {a['hand']: a for a in scene.snapshot()['command_details']['realman']}
                self.assertIsNone(arms['right']['gripper_width_m'])
            finally:
                scene.close()
                view.close()


if __name__ == '__main__': unittest.main()
