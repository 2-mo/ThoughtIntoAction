"""Separate state publishers and monitors must survive independent lifecycles."""

import copy
import socket
import time
import unittest

import numpy as np

from robot_core.model import RobotModel
from robot_core.telemetry import LocalPublisher, StateSubscriber
from robot_simulation.scene import Scene


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            self.address = f'tcp://127.0.0.1:{sock.getsockname()[1]}'
        self.robot = RobotModel()
        self.q = np.array([0., 1.5, 1.45, -1.52, 0., 0.])

    def publisher(self, kind='virtual'):
        obj = LocalPublisher(self.robot, source={'kind': kind, 'input': 'test'}, address=self.address)
        self.addCleanup(obj.close)
        obj.update(self.q)
        return obj

    def subscriber(self):
        obj = StateSubscriber(self.address)
        self.addCleanup(obj.close)
        return obj

    def packet(self, subscriber, stream_id=None):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            packet = subscriber.receive(100)
            if packet is not None and (stream_id is None or packet['stream_id'] == stream_id):
                return packet
        self.fail('No robot state received')

    def test_publisher_can_run_without_monitor_and_monitor_can_restart(self):
        publisher = self.publisher()
        for index in range(100):
            publisher.update(self.q + index * .0001)
        first = StateSubscriber(self.address)
        try:
            packet = self.packet(first)
            np.testing.assert_allclose(packet['actual'], self.q + .0099)
        finally:
            first.close()
        publisher.update(self.q + .02)
        second = self.subscriber()
        np.testing.assert_allclose(self.packet(second)['actual'], self.q + .02)
        self.assertTrue(publisher._thread.is_alive())

    def test_monitor_can_start_first_and_follow_a_replacement_publisher(self):
        subscriber = self.subscriber()
        first = self.publisher()
        one = self.packet(subscriber)
        first.close()
        second = self.publisher('replay')
        two = self.packet(subscriber, second._base['stream_id'])
        self.assertNotEqual(one['stream_id'], two['stream_id'])
        self.assertEqual(two['source']['kind'], 'replay')

    def test_another_publisher_cannot_mix_states_into_an_owned_channel(self):
        self.publisher()
        with self.assertRaisesRegex(RuntimeError, 'Cannot publish robot state'):
            LocalPublisher(self.robot, address=self.address)

    def test_scene_marks_old_samples_stale_even_with_fresh_heartbeats(self):
        publisher = self.publisher()
        packet = self.packet(self.subscriber())
        scene = Scene(self.robot)
        self.addCleanup(scene.close)
        scene.accept(packet)
        self.assertEqual(scene.snapshot()['stream']['status'], 'live')
        old = copy.deepcopy(packet)
        old['sample_monotonic_ns'] -= 2_000_000_000
        old['emitted_monotonic_ns'] = time.monotonic_ns()
        scene.accept(old)
        self.assertEqual(scene.snapshot()['stream']['status'], 'stale')
        publisher.update(self.q + .01)
        self.assertTrue(scene._thread.is_alive())

    def test_model_mismatch_rejected_and_new_source_clears_old_trajectory(self):
        publisher = self.publisher()
        publisher.path('old_reference', [[0, 0, 0], [1, 0, 0]], 0x33CC55)
        packet = self.packet(self.subscriber())
        scene = Scene(self.robot)
        self.addCleanup(scene.close)
        scene.accept(packet)
        invalid = copy.deepcopy(packet)
        invalid['robot']['eef_frame'] = 'wrong-frame'
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            scene.accept(invalid)
        np.testing.assert_allclose(scene.snapshot()['actual']['q'], self.q)
        new = {**packet, 'stream_id': 'new-controller', 'paths': {}, 'actual': (self.q + .01).tolist()}
        scene.accept(new)
        self.assertNotIn('old_reference', scene.snapshot()['paths'])
        self.assertEqual(len(scene.history), 1)

    def test_invalid_packet_does_not_change_last_good_state(self):
        packet = self.packet(self.subscriber_after_publisher())
        scene = Scene(self.robot)
        self.addCleanup(scene.close)
        scene.accept(packet)
        for changes in ({'actual': [float('nan')] * 6}, {'paths': []}, {'source': []}):
            with self.assertRaises(ValueError):
                scene.accept({**packet, **changes})
            np.testing.assert_allclose(scene.snapshot()['actual']['q'], self.q)

    def test_gripper_feedback_target_and_legacy_absence(self):
        publisher = self.publisher()
        subscriber = self.subscriber()
        grip = {"actual": .018, "target": .04, "max": .085}
        publisher.update(self.q, gripper=grip)
        packet = self.packet(subscriber)
        scene = Scene(self.robot)
        self.addCleanup(scene.close)
        scene.accept(packet)
        self.assertEqual(scene.snapshot()['gripper'], grip)
        for invalid in ({**grip, 'max': 0}, {**grip, 'actual': float('nan')},
                        {**grip, 'target': float('inf')}):
            with self.assertRaises(ValueError):
                scene.accept({**packet, 'gripper': invalid})
            self.assertEqual(scene.snapshot()['gripper'], grip)
        scene.accept({**packet, 'sequence': packet['sequence'] + 1,
                      'gripper': {**grip, 'target': None}})
        self.assertIsNone(scene.snapshot()['gripper']['target'])
        packet.pop('gripper')
        packet['sequence'] += 2
        scene.accept(packet)
        self.assertIsNone(scene.snapshot()['gripper'])

    def subscriber_after_publisher(self):
        self.publisher()
        return self.subscriber()
