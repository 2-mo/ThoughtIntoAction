"""Offline observer fixtures: no HTTP server, control gateway, robot or camera."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from embodied.applications.monitor.kernel_view import KernelMonitor, collection_snapshot, kernel_packet


def tick(runtime_id='one'):
    return {'runtime_id': runtime_id, 'sequence': 4, 'terminal': False, 'error': None, 'source': 'hold',
            'timing': {'feedback_received_ns': time.monotonic_ns()},
            'feedback': {'q': [.1] * 6, 'dq': [.2] * 6, 'gripper_width': .025},
            'command': {'q': [.3] * 6, 'dq': [.4] * 6, 'gripper_width': .03,
                        'kp': [20.] * 6, 'kd': [1.] * 6, 'feedforward': [9.] * 6},
            'sdk_output': {'q': [.31] * 6, 'dq': [.41] * 6, 'torque': [8.] * 6}}


class KernelMonitorTests(unittest.TestCase):
    def test_packet_preserves_physical_feedback_and_separates_sdk_output(self):
        robot = Mock()
        robot.metadata.return_value = {'eef_frame': 'tcp_link'}
        packet = kernel_packet(robot, tick(), {'backend': 'real', 'input': 'idle'})
        self.assertEqual(packet['source']['kind'], 'hardware')
        self.assertEqual(packet['actual'], [.1] * 6)
        self.assertEqual(packet['target'], [.3] * 6)
        self.assertEqual(packet['actual_vel'], [.2] * 6)
        self.assertEqual(packet['gripper']['actual'], .025)
        self.assertEqual(packet['gripper']['max'], .082)
        self.assertEqual(kernel_packet(robot, tick(), {'backend': 'real'}, {'gripper_max_width': .075})['gripper']['max'], .075)
        self.assertEqual(packet['command_details']['sdk_cmd_pos'], [.31] * 6)
        self.assertIsNone(packet['actual_torque'])  # SDK output/feedforward is not measured torque

    def test_observer_waits_then_follows_runs_and_closes_only_its_reader(self):
        identity = {'backend': 'real', 'runtime_id': 'one', 'boot_id': 'this-boot', 'input': 'idle'}
        second = {**identity, 'runtime_id': 'two'}
        finder = Mock(return_value=None)
        readers = [Mock(), Mock()]
        for reader in readers:
            reader.descriptor = {'parameters': {'effective_robot_config': {'gripper_max_width': .082}}}
        readers[0].read.return_value = tick('one')
        readers[1].read.return_value = tick('two')
        factory = Mock(side_effect=readers)
        scene = Mock()
        scene.robot.metadata.return_value = {'eef_frame': 'tcp_link'}
        monitor = KernelMonitor(reader_factory=factory, finder=finder, episode_roots=[])
        monitor.scene = scene
        monitor.step()
        factory.assert_not_called()
        self.assertEqual(monitor.monitor()['collection']['state'], 'OFFLINE')
        for directory, expected in [(Path('/fake/one'), identity), (Path('/fake/two'), second)]:
            finder.return_value = directory, expected
            monitor.discovery_at = 0
            monitor.step()
            self.assertEqual(monitor.monitor()['runtime_id'], expected['runtime_id'])
            self.assertEqual(scene.accept.call_args.args[0]['stream_id'], expected['runtime_id'])
        readers[0].close.assert_called_once()
        self.assertEqual(factory.call_args_list[0].args, (Path('/fake/one'), 'latest'))
        self.assertEqual(factory.call_args_list[0].kwargs, {'timeout': .5})
        rendered = scene.accept.call_count
        monitor.display_at = time.monotonic() + 1
        readers[1].read.return_value = {**tick('two'), 'sequence': 5}
        monitor.step()
        self.assertEqual(monitor.latest['sequence'], 5)  # reading continues between display frames
        self.assertEqual(scene.accept.call_count, rendered)
        readers[1].read.return_value = {**tick('two'), 'terminal': True}
        monitor.step()
        self.assertIsNone(monitor.monitor()['state_age_s'])
        self.assertIn('内核已退出', monitor.monitor()['monitor_message'])
        monitor.close()
        readers[1].close.assert_called_once()

    def test_collection_never_calls_uncommitted_metadata_a_live_or_saved_episode(self):
        identity = {'runtime_id': 'one', 'boot_id': 'this-boot'}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.assertEqual(collection_snapshot(identity, [root])['collection']['state'], '待机')
            episode = root / 'episode'
            episode.mkdir()
            path = episode / 'episode.json'
            manifest = {'schema_version': 2, 'runtime': identity, 'lifecycle': 'recording',
                        'task': 'test', 'counts': {'state': 0}, 'started_monotonic_ns': 1}
            path.write_text(json.dumps(manifest))
            pending = collection_snapshot(identity, [root])
            self.assertIsNone(pending['recording'])
            self.assertIsNone(pending['episode_metadata']['counts'])
            self.assertEqual(pending['collection_count'], 0)
            self.assertEqual(pending['collection']['state'], '录制状态待确认')
            path.write_text(json.dumps({**manifest, 'lifecycle': 'complete', 'counts': {'state': 123}}))
            saved = collection_snapshot(identity, [root])
            self.assertEqual(saved['collection_count'], 1)
            self.assertEqual(saved['episode_metadata']['counts']['state'], 123)
            self.assertIsNone(saved['recording'])
            self.assertEqual(collection_snapshot({**identity, 'boot_id': 'old-boot'}, [root])['collection_count'], 0)

    def test_kernel_cli_dispatch_does_not_construct_legacy_inputs_or_simulation(self):
        from embodied.applications.monitor.cli import main
        with patch('embodied.applications.monitor.kernel_view.serve') as serve, \
             patch('embodied.applications.monitor.cli.Registry') as registry, \
             patch('embodied.applications.monitor.cli.StateSubscriber') as subscriber:
            main(['--kernel', '--mode', 'real', '--no-camera', '--no-connections'])
            serve.assert_called_once()
            self.assertEqual(serve.call_args.args[0].host, '127.0.0.1')
            registry.assert_not_called()
            subscriber.assert_not_called()

    def test_scene_binds_only_the_requested_ipv4_and_reports_that_address(self):
        from embodied.applications.monitor.scene import Scene
        for host in ('127.0.0.1', '100.112.78.10'):
            with self.subTest(host=host), \
                 patch('embodied.applications.monitor.scene.ThreadingHTTPServer') as server, \
                 patch('embodied.applications.monitor.scene.threading.Thread'), \
                 patch.object(Scene, 'set_model'):
                server.return_value.server_port = 18081
                scene = Scene(Mock(urdf=Path('fixture.urdf')), port=18081, host=host)
                self.assertEqual(server.call_args.args[0], (host, 18081))
                self.assertEqual(scene.url, f'http://{host}:18081/')
                scene.close()
        with patch('embodied.applications.monitor.scene.ThreadingHTTPServer') as server:
            with self.assertRaises(ValueError):
                Scene(Mock(), host='0.0.0.0')
            server.assert_not_called()


if __name__ == '__main__':
    unittest.main()
