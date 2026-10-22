"""Offline boundary checks for the small workbench adaptations."""
import ctypes
import json
from pathlib import Path
import sys
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]

import numpy as np
from embodied.control.ticks import RobotTickV3, TickPublisher, TickSubscriber, layout, to_dict
from embodied.control.settings import load_settings
from embodied.devices.robots.arx5.simulation import MujocoBackend
from embodied.contracts.control import ControlCommand, Mode


class IntegrationChecks(unittest.TestCase):
    def test_wire_layout_is_stable(self):
        expected = json.loads((ROOT / 'src/embodied/control/robot_tick_v3.json').read_text())
        self.assertEqual(layout(), expected)

    def test_observation_does_not_change_control_outputs(self):
        config = load_settings().robot
        plain, observed = MujocoBackend(config), MujocoBackend(config)
        prefix = 'arx5-test-' + uuid.uuid4().hex[:12]
        reader, publisher = TickSubscriber(prefix), TickPublisher(prefix, joint_count=len(config.park_q))
        try:
            pose = plain.read_state(0.).pose
            for i in range(300):
                tick = i / 250
                command = ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=pose.copy(), gripper_target=.04)
                state = observed.read_state(tick)
                plain.step(command, tick)
                observed.step(command, tick)
                publisher.publish(state, command, observed, tick, time.monotonic_ns(), 'control', True)
                np.testing.assert_array_equal(plain.data.qpos, observed.data.qpos)
                np.testing.assert_array_equal(plain.controller.q_target, observed.controller.q_target)
            publisher.finish()
            rows = []
            while (packet := reader.receive()) is not None:
                rows.append(to_dict(packet))
            self.assertEqual(len(rows), 301)
            self.assertTrue(rows[-1]['terminal'])
            self.assertEqual([x['sequence'] for x in rows], list(range(301)))
            self.assertEqual(publisher.dropped, 0)
            first = rows[0]
            self.assertGreater(first['timing']['command_submitted_ns'], first['timing']['command_started_ns'])
            self.assertEqual(first['command']['gripper_width'], .04)
        finally:
            publisher.close()
            reader.close()
            plain.close()
            observed.close()

    def test_slow_subscriber_cannot_block_or_hide_overflow(self):
        prefix = 'arx5-test-' + uuid.uuid4().hex[:12]
        reader, writer = TickSubscriber(prefix), TickPublisher(prefix, joint_count=7)
        try:
            began = time.monotonic()
            for _ in range(4500):
                writer.finish()
            self.assertLess(time.monotonic() - began, 5.)
            self.assertEqual(writer.dropped, 0)
            first = reader.receive()
            self.assertIsNotNone(first)
            self.assertGreater(first.sequence, 0)
            self.assertEqual(first.size, ctypes.sizeof(RobotTickV3))
        finally:
            writer.close()
            reader.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
