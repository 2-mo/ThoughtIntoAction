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
from embodied.devices.phone.core.models import TrackingQuality
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
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

    def enable_fake_robot(self, *, align=True):
        self.send()
        client = Mock(identity={'runtime_id': 'test', 'plugins': {'robot': 'arx.sim'}})
        client.status.return_value = {'operations': [], 'pose': [.3, 0, .2, 0, 0, 0, 1],
            'q': [0] * 6, 'dq': [0] * 6, 'feedback_monotonic_ns': time.monotonic_ns(), 'gripper_width': .02}
        client.parameters = {'effective_robot_config': {'gripper_max_width': .082, 'gripper_velocity_max_m_s': .04}}
        client.preview.return_value = {'plan_id': 'plan'}
        client.execute.return_value = {'operation_id': 'motion'}
        self.controller.client, self.controller.ghost = client, False
        self.service.run = 'test'
        if align:
            self.controller.request_alignment(client.status(), 'test')
            client.command.reset_mock()
        return client

    def test_raw_phone_pose_is_available_without_control_and_expires(self):
        state = self.send()
        self.assertEqual(state['phone_pose'], {'frame': 'phone_reference',
            'quaternion_xyzw': list(GOLDEN_POSE.pose.orientation.as_tuple())})
        self.assertIsNone(state['orientation_preview'])
        self.assertIsNone(self.controller.client)
        self.controller.last_received -= 1
        self.assertIsNone(self.service.snapshot()['phone_pose'])
        self.assertIsNone(self.send(tracking_quality=TrackingQuality.LIMITED)['phone_pose'])

    def test_one_press_alignment_is_immediate_and_survives_clutch_and_preset(self):
        client = self.enable_fake_robot()
        self.send()
        self.send(clutch_active=True)
        basis = self.controller.policy.rotation_alignment
        self.assertIsNotNone(basis)
        self.send()
        self.assertEqual(self.controller.policy.rotation_alignment, basis)
        before = client.command.call_count
        self.service.dispatch({'command': 'align', 'runtime_id': 'test'})
        self.assertTrue(self.service.snapshot()['aligned'])
        self.assertEqual([call.args[0] for call in client.command.call_args_list[before:]], ['source-metadata'])
        self.send()
        self.send(clutch_active=True)
        self.assertTrue(self.service.snapshot()['aligned'])
        self.send()
        self.service.dispatch({'command': 'pregrasp', 'runtime_id': 'test'})
        self.assertTrue(self.service.snapshot()['aligned'])
        with self.assertRaisesRegex(ValueError, '等待动作结束'):
            self.service.dispatch({'command': 'align', 'runtime_id': 'test'})

    def test_uncalibrated_clutch_never_commands_motion_then_one_press_releases_gate(self):
        client = self.enable_fake_robot(align=False)
        self.send()
        self.send(clutch_active=True)
        client.command.assert_not_called()
        self.send()
        self.service.dispatch({'command': 'align', 'runtime_id': 'test'})
        self.assertTrue(self.service.snapshot()['aligned'])
        self.send()
        self.send(clutch_active=True)
        self.assertEqual(client.command.call_args.args, ('cartesian',))
        self.assertEqual(client.command.call_args.kwargs['pose'], client.status.return_value['pose'])

    def test_alignment_in_monitor_mode_never_acquires_robot_and_tracking_loss_invalidates(self):
        client = self.enable_fake_robot(align=False)
        self.controller.disable()
        self.send()
        with patch.object(phone_service, 'live_runs', return_value=[Path('/test/runtime')]), \
             patch.object(phone_service, 'ControlClient', return_value=client), \
             patch('embodied.devices.inputs.phone.ControlSession') as lease:
            self.service.dispatch({'command': 'align', 'runtime_id': 'test'})
            lease.assert_not_called()
        client.command.assert_not_called()
        self.assertFalse(self.service.snapshot()['enabled'])
        self.assertTrue(self.send(tracking_quality=TrackingQuality.LIMITED)['aligned'])
        self.assertFalse(self.send(tracking_quality=TrackingQuality.UNAVAILABLE)['aligned'])

    def test_alignment_rejects_stale_robot_and_resets_on_new_phone_session(self):
        client = self.enable_fake_robot(align=False)
        client.status.return_value['feedback_monotonic_ns'] -= 1_000_000_000
        with self.assertRaisesRegex(ValueError, '反馈过期'):
            self.service.dispatch({'command': 'align', 'runtime_id': 'test'})
        self.assertFalse(self.service.snapshot()['aligned'])
        client.status.return_value['feedback_monotonic_ns'] = time.monotonic_ns()
        self.service.dispatch({'command': 'align', 'runtime_id': 'test'})
        self.assertFalse(self.send(session_id=2)['aligned'])

    def test_home_opens_gripper_before_planning_and_reports_completion(self):
        client = self.enable_fake_robot()
        state = self.service.dispatch({'command': 'home', 'runtime_id': 'test'})
        self.assertEqual(state['preset']['phase'], 'opening')
        client.preview.assert_not_called()
        self.send()
        client.command.assert_called_with('gripper', gripper_width=.082, ttl_ms=200)
        self.send()
        client.preview.assert_not_called()  # elapsed ticks are not evidence of opening
        client.status.return_value['gripper_width'] = .081
        self.send()
        client.preview.assert_called_once_with('home', gripper_width=.082)
        client.execute.assert_called_once_with({'plan_id': 'plan'})
        client.status.return_value['operations'] = [{'operation_id': 'motion', 'status': 'running'}]
        self.send(home_requested=True)  # no queued repeat while the preset is running
        client.execute.assert_called_once()
        client.status.return_value['operations'][0]['status'] = 'completed'
        self.assertEqual(self.send()['preset']['phase'], 'completed')
        self.send(clutch_active=True)
        self.assertNotEqual(client.command.call_args.args, ('cartesian',))  # rearm after completion
        self.send()
        self.send(clutch_active=True)
        self.assertEqual(client.command.call_args.args, ('cartesian',))

    def test_pregrasp_uses_shared_pose_and_preserves_gripper(self):
        client = self.enable_fake_robot()
        self.service.dispatch({'command': 'pregrasp', 'runtime_id': 'test'})
        self.send()
        client.preview.assert_called_once_with('movej', q=list(DEFAULT_GRASP_READY_JOINTS))
        client.command.assert_not_called()  # no gripper open/close, no new speed override

    def test_preset_rejects_wrong_run_held_clutch_and_concurrent_actions(self):
        client = self.enable_fake_robot()
        with self.assertRaisesRegex(ValueError, '实例已变化'):
            self.service.dispatch({'command': 'home', 'runtime_id': 'old'})
        self.send(clutch_active=True)
        with self.assertRaisesRegex(ValueError, '松开 Clutch'):
            self.service.dispatch({'command': 'home', 'runtime_id': 'test'})
        self.send()
        client.identity['plugins']['robot'] = 'other.real'
        with self.assertRaisesRegex(ValueError, '未配置预抓取'):
            self.service.dispatch({'command': 'pregrasp', 'runtime_id': 'test'})
        self.service.dispatch({'command': 'home', 'runtime_id': 'test'})
        with self.assertRaisesRegex(ValueError, '正在执行'):
            self.service.dispatch({'command': 'home', 'runtime_id': 'test'})
        client.execute.assert_not_called()

    def test_blocked_gripper_never_starts_home(self):
        client = self.enable_fake_robot()
        self.service.dispatch({'command': 'home', 'runtime_id': 'test'})
        self.controller.homing.open_deadline = time.monotonic() - 1
        state = self.send()
        self.assertIn('夹爪未能张开', state['error'])
        self.assertFalse(state['enabled'])
        client.execute.assert_not_called()
        client.close.assert_called_once()

    def test_stopping_opening_discards_arm_move(self):
        client = self.enable_fake_robot()
        self.service.dispatch({'command': 'home', 'runtime_id': 'test'})
        self.send()
        self.service.dispatch({'command': 'disable'})
        self.assertEqual(self.send()['preset']['phase'], 'cancelled')
        client.execute.assert_not_called()
        client.close.assert_called_once()

    def test_tracking_loss_cancels_running_preset_without_resuming(self):
        client = self.enable_fake_robot()
        self.service.dispatch({'command': 'pregrasp', 'runtime_id': 'test'})
        self.send()
        state = self.send(tracking_quality=TrackingQuality.LIMITED)
        self.assertEqual(state['preset']['phase'], 'cancelled')
        client.command.assert_called_with('hold')
        self.send()
        client.execute.assert_called_once()

    def test_input_loss_cancels_gripper_open_without_moving_arm(self):
        client = self.enable_fake_robot()
        self.service.dispatch({'command': 'home', 'runtime_id': 'test'})
        self.send()
        self.controller.last_received -= .2
        self.service.step()
        client.command.assert_called_with('hold')
        self.send()
        client.execute.assert_not_called()

    def test_preset_failure_is_visible_instead_of_reported_as_completed(self):
        client = self.enable_fake_robot()
        self.service.dispatch({'command': 'pregrasp', 'runtime_id': 'test'})
        self.send()
        client.status.return_value['operations'] = [{'operation_id': 'motion', 'status': 'failed', 'reason': '未到位'}]
        state = self.send()
        self.assertIn('预抓取未完成：未到位', state['error'])
        self.assertFalse(state['enabled'])

    def test_preset_command_retry_does_not_restart_sequence(self):
        client = self.enable_fake_robot()
        self.sock.settimeout(.1)
        payload = json.dumps({'protocol': phone_service.CONTROL_PROTOCOL, 'id': 1, 'session_id': 1,
            'client_time_ns': self.controller.last_input.timestamp_ns, 'action': 'home', 'runtime_id': 'test'}).encode()
        with patch.object(phone_service, 'live_runs', return_value=[]), \
             patch.object(self.controller, 'request_preset', wraps=self.controller.request_preset) as preset:
            for _ in range(2):
                self.sock.sendto(payload, self.controller.transport.local_address)
                self.service.step()
                reply = json.loads(self.sock.recv(2048))
                self.assertTrue(reply['ok'])
                self.assertEqual(reply['preset']['phase'], 'opening')
            preset.assert_called_once_with('home')
        client.execute.assert_not_called()

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
        session = Mock(identity={'runtime_id': 'test'})
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
            self.service.dispatch({'command': 'align', 'runtime_id': 'test'})
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
