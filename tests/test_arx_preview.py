"""Simulation feedback provenance and separation from the physical session."""
import json
import unittest
from unittest.mock import Mock, patch
from urllib.request import ProxyHandler, Request, build_opener

from embodied.core.model import RobotModel
from embodied.monitor.arx_preview import Simulation, preview_packet
from embodied.monitor.scene import Scene


class ArxPreviewTests(unittest.TestCase):
    def test_feedback_is_not_command_or_feedforward(self):
        robot = Mock()
        robot.metadata.return_value = {'name': 'test'}
        tick = {
            'runtime_id': 'sim-only', 'sequence': 0,
            'timing': {'feedback_received_ns': 1234},
            'feedback': {'q': [.1] * 6, 'dq': [.2] * 6, 'gripper_width': .025},
            'command': {'q': [.3] * 6, 'dq': [.4] * 6, 'gripper_width': .06,
                        'feedforward': [9.] * 6, 'kp': [20.] * 6, 'kd': [1.] * 6},
        }
        packet = preview_packet(robot, tick)
        self.assertEqual(packet['source']['kind'], 'virtual')
        self.assertEqual(packet['sequence'], 1)
        self.assertEqual(packet['sample_monotonic_ns'], 1234)
        self.assertEqual(packet['actual'], [.1] * 6)
        self.assertEqual(packet['actual_vel'], [.2] * 6)
        self.assertEqual(packet['target'], [.3] * 6)
        self.assertEqual(packet['gripper']['actual'], .025)
        self.assertIsNone(packet['actual_torque'])
        self.assertIsNone(packet['gripper_actual_torque'])

    def test_simulation_commands_never_reach_physical_session(self):
        simulation = Simulation()
        scene = Scene(RobotModel(), simulation=simulation)
        self.addCleanup(scene.close)
        http = build_opener(ProxyHandler({}))
        with patch('embodied.core.session_protocol.request') as physical_request:
            with http.open(scene.url + 'api/replay', timeout=2) as response:
                self.assertEqual(json.load(response), {})
            request = Request(scene.url + 'api/replay', data=b'{"action":"load_last"}',
                              headers={'Content-Type': 'application/json'})
            with http.open(request, timeout=2) as response:
                self.assertIn('error', json.load(response))
            for action, expected in [('stop', False), ('restart', True)]:
                request = Request(scene.url + 'api/simulation',
                                  data=json.dumps({'action': action}).encode())
                with http.open(request, timeout=2) as response:
                    self.assertTrue(json.load(response)['ok'])
                self.assertEqual(simulation.active, expected)
            physical_request.assert_not_called()
        with self.assertRaises(ValueError):
            simulation.command('real')
