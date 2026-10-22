"""Real UDP reception; robot command ownership is replaced, never hardware."""
from dataclasses import replace
from pathlib import Path
import socket
import json
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from embodied.core.session_protocol import request
from embodied.devices.inputs.phone import PhoneController
from embodied.devices.inputs import phone_service
from embodied.devices.phone.protocol.golden import GOLDEN_POSE
from embodied.devices.phone.protocol.packet import encode_packet, decode_packet, EStopResetRequestPacket, EStopResetAckPacket, SourceType


class PhoneServiceTests(unittest.TestCase):
    def setUp(self):
        self.controller = PhoneController(port=0, ghost=True)
        self.controller.start()
        self.addCleanup(self.controller.stop)
        with patch.object(phone_service, 'addresses', return_value=['192.0.2.1']):
            self.service = phone_service.PhoneService(self.controller)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.sock.close)
        self.seq = 0

    def send(self, **fields):
        self.seq += 1
        packet = replace(GOLDEN_POSE, sequence=self.seq, session_id=1,
                         client_time_ns=time.monotonic_ns(), clutch_active=False,
                         recenter_requested=False)
        packet = replace(packet, **fields)
        self.sock.sendto(encode_packet(packet), self.controller.transport.local_address)
        self.service.step()
        return self.service.snapshot()

    def test_observation_needs_no_robot_and_stale_data_cannot_enable(self):
        with patch('embodied.devices.inputs.phone.ControlSession') as session:
            state = self.send()
            self.assertTrue(state['connected'])
            self.assertFalse(state['enabled'])
            self.assertIsNone(self.controller.client)
            session.assert_not_called()
            self.controller.last_received -= 1
            self.assertFalse(self.service.snapshot()['connected'])
            with self.assertRaisesRegex(ValueError, '正常追踪'):
                self.service.dispatch({'command': 'enable', 'runtime_id': 'test'})
            session.assert_not_called()

    def test_explicit_enable_checks_identity_and_rearms_clutch_then_disable_releases(self):
        client = Mock(identity={'runtime_id': 'test'})
        session = Mock()
        session.__enter__ = Mock(return_value=session)
        session.status.return_value = {'operations': [], 'pose': [.3, 0, .2, 0, 0, 0, 1],
            'q': [0] * 6, 'dq': [0] * 6, 'feedback_monotonic_ns': time.monotonic_ns(), 'gripper_width': .04}
        session.parameters = {'effective_robot_config': {'gripper_max_width': .082}}
        with patch.object(phone_service, 'live_runs', return_value=[Path('/test/runtime')]), \
             patch.object(phone_service, 'ControlClient', return_value=client), \
             patch('embodied.devices.inputs.phone.ControlSession', return_value=session) as factory:
            self.send(clutch_active=True)
            with self.assertRaisesRegex(ValueError, 'Clutch'):
                self.service.dispatch({'command': 'enable', 'runtime_id': 'test'})
            self.send()
            with self.assertRaisesRegex(ValueError, '实例已变化'):
                self.service.dispatch({'command': 'enable', 'runtime_id': 'old'})
            factory.assert_not_called()
            state = self.service.dispatch({'command': 'enable', 'runtime_id': 'test'})
            self.assertTrue(state['enabled'])
            session.command.assert_not_called()
            self.send(clutch_active=True)
            session.command.assert_not_called()  # must release again after acquiring
            self.send(gripper=None)
            self.send(clutch_active=True)
            self.assertEqual(session.command.call_args.args, ('cartesian',))
            preview = self.service.snapshot()['orientation_preview']
            self.assertEqual(preview['frame'], 'base_tcp')
            self.assertEqual(preview['target_quaternion'], self.controller.last_target['pose'][3:])
            self.assertEqual(preview['input_quaternion'], [0., 0., 0., 1.])
            self.controller.last_received -= 1
            self.assertIsNone(self.service.snapshot()['orientation_preview'])
            self.service.step()
            self.assertEqual(session.command.call_args.args, ('hold',))
            self.service.dispatch({'command': 'disable'})
            session.close.assert_called_once()
            self.assertTrue(self.send()['connected'])
            self.assertFalse(self.service.snapshot()['enabled'])

    def test_gateway_error_releases_but_receiver_remains(self):
        self.controller.ghost = False
        self.controller.client = Mock()
        self.controller.client.status.side_effect = RuntimeError('lease lost')
        client = self.controller.client
        state = self.send()
        self.assertFalse(state['enabled'])
        self.assertEqual(state['error'], 'lease lost')
        client.close.assert_called_once()
        self.assertTrue(self.send()['connected'])

    def test_phone_commands_are_session_bound_ordered_and_retries_do_not_reexecute(self):
        self.send()
        self.sock.settimeout(.05)
        def send(sequence, action, **fields):
            data = {'protocol': phone_service.CONTROL_PROTOCOL, 'id': sequence, 'session_id': 1,
                    'client_time_ns': self.controller.last_input.timestamp_ns, 'action': action,
                    'runtime_id': 'test', **fields}
            self.sock.sendto(json.dumps(data).encode(), self.controller.transport.local_address)
            self.service.step()
        with patch.object(phone_service, 'live_runs', return_value=[]), \
             patch.object(self.service, 'dispatch', wraps=self.service.dispatch) as dispatch:
            send(1, 'monitor')
            self.assertTrue(json.loads(self.sock.recv(2048))['ok'])
            dispatch.assert_not_called()
            send(2, 'disable')
            first = self.sock.recv(2048)
            dispatch.assert_called_once_with({'command': 'disable', 'runtime_id': 'test'})
            send(2, 'disable')
            self.assertEqual(self.sock.recv(2048), first)
            self.assertEqual(dispatch.call_count, 1)
            send(1, 'enable')
            with self.assertRaises(TimeoutError): self.sock.recv(2048)
            send(3, 'enable', session_id=2)
            with self.assertRaises(TimeoutError): self.sock.recv(2048)
            send(3, 'enable', client_time_ns=self.controller.last_input.timestamp_ns - 1_000_000_000)
            self.assertFalse(json.loads(self.sock.recv(2048))['ok'])
            self.assertEqual(dispatch.call_count, 1)

    def test_phone_recovery_clears_only_phone_stop_and_never_acquires_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            self.service.latch_path = Path(directory) / 'stop.json'
            self.send(estop_requested=True)
            self.assertTrue(json.loads(self.service.latch_path.read_text())['latched'])
            self.send(estop_requested=False, clutch_active=False)
            packet = EStopResetRequestPacket(SourceType.ARKIT, 1, 1, time.monotonic_ns())
            with patch('embodied.devices.inputs.phone.ControlSession') as acquire:
                self.sock.sendto(encode_packet(packet), self.controller.transport.local_address)
                self.service.step()
                self.sock.settimeout(.1)
                self.assertIsInstance(decode_packet(self.sock.recv(2048)), EStopResetAckPacket)
                acquire.assert_not_called()
                self.assertFalse(self.controller.estop)
                self.assertTrue(self.controller.ghost)
                self.assertFalse(json.loads(self.service.latch_path.read_text())['latched'])
                with patch.object(self.controller, 'disable') as disable:
                    self.assertTrue(self.service.reset_from_phone(packet))
                    disable.assert_not_called()
            self.send(estop_requested=True, clutch_active=True)
            self.assertFalse(self.service.reset_from_phone(packet))
            self.assertTrue(self.controller.estop)
            self.send(estop_requested=False, clutch_active=False)
            self.assertFalse(self.service.reset_from_phone(packet))
            self.assertTrue(self.controller.estop)  # An old RESET cannot clear a newer stop.

    def test_service_restart_preserves_stop_latch(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'stop.json'
            path.write_text('{"latched":true}')
            with patch.object(phone_service, 'addresses', return_value=[]):
                other = phone_service.PhoneService(self.controller, latch_path=path)
            self.assertTrue(other.controller.estop)

    def test_idle_udp_does_not_starve_local_monitor_or_shutdown(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(phone_service, 'SOCKET', Path(directory) / 'phone.sock'):
            thread = threading.Thread(target=self.service.run_forever)
            thread.start()
            try:
                deadline = time.monotonic() + 2
                while not phone_service.SOCKET.exists() and time.monotonic() < deadline:
                    time.sleep(.01)
                for _ in range(5):
                    state = request(phone_service.SOCKET, 'monitor', timeout=.3)
                    self.assertFalse(state['enabled'])
            finally:
                self.service.closed.set()
                thread.join(timeout=2)
            self.assertFalse(thread.is_alive())


if __name__ == '__main__':
    unittest.main()
