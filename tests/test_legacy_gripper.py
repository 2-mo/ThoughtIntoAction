"""Viewer compatibility reads must not acquire or command a running arm."""
import json
import time
import unittest
from unittest.mock import MagicMock, patch

from embodied.applications.monitor.connections import LegacyGripperFeedback


class LegacyGripperTests(unittest.TestCase):
    def setUp(self):
        self.reader = LegacyGripperFeedback('/unused.sock')
        self.packet = {'stream_id': 'one', 'robot': {'model': 'test'},
                       'source': {'controller': 'arm-service', 'backend': 'test', 'kind': 'hardware'}}
        self.info = {'robot': self.packet['robot'], 'backend': 'test', 'hardware': True,
                     'settings': {'gripper_width': .085},
                     'state': {'host_monotonic_ns': time.monotonic_ns(), 'gripper_actual_pos': .032}}
        self.socket = MagicMock()
        self.socket.__enter__.return_value = self.socket
        self.socket.recvmsg.side_effect = lambda *_: (json.dumps({'ok': True, 'result': self.info}).encode(), [], 0, None)
        self.patch = patch('embodied.applications.monitor.connections.socket.socket', return_value=self.socket)
        self.factory = self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_read_only_feedback_is_cached_and_target_stays_unknown(self):
        result = self.reader.read(self.packet)
        self.assertEqual(result, {'actual': .032, 'target': None, 'max': .085})
        self.assertEqual(self.reader.read(self.packet), result)
        self.factory.assert_called_once()
        request = json.loads(self.socket.sendall.call_args.args[0])
        self.assertEqual(request['op'], 'describe')
        self.assertEqual(set(request), {'op', 'sent_monotonic_ns'})

    def test_legacy_torque_is_measured_feedback_and_missing_is_empty(self):
        self.assertIsNone(self.reader.read_torque(self.packet))
        self.assertIsNone(self.reader.read_gripper_torque(self.packet))
        self.info['state']['gripper_actual_torque'] = .35
        self.reader.next_probe = 0
        self.assertEqual(self.reader.read_gripper_torque(self.packet), .35)
        self.info['state']['actual_torque'] = [1, -2, 0, .4, -.5, 3]
        self.reader.next_probe = 0
        self.assertEqual(self.reader.read_torque(self.packet), [1, -2, 0, .4, -.5, 3])

    def test_native_feedback_and_other_sources_do_not_probe(self):
        grip = {'actual': .02, 'target': .03, 'max': .085}
        self.assertEqual(self.reader.read({**self.packet, 'gripper': grip}), grip)
        self.assertIsNone(self.reader.read({**self.packet, 'source': {'kind': 'replay'}}))
        self.factory.assert_not_called()

    def test_stale_mismatched_and_unavailable_feedback_stays_empty(self):
        for changes in ({'robot': {}}, {'hardware': False}, {'backend': 'other'},
                        {'state': {**self.info['state'], 'host_monotonic_ns': time.monotonic_ns() - 1_000_000_000}}):
            original = self.info
            self.info = {**original, **changes}
            self.reader.next_probe = 0
            self.assertIsNone(self.reader.read(self.packet))
            self.info = original
        self.socket.connect.side_effect = OSError('disconnected')
        self.reader.next_probe = 0
        self.assertIsNone(self.reader.read(self.packet))

    def test_replaced_stream_does_not_reuse_previous_feedback(self):
        self.assertIsNotNone(self.reader.read(self.packet))
        self.socket.connect.side_effect = OSError('disconnected')
        self.assertIsNone(self.reader.read({**self.packet, 'stream_id': 'two'}))
