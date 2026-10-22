"""Transport evidence and refusal tests; no adb calls or real connection changes."""
import importlib.util
import io
import json
from pathlib import Path
import unittest
import tempfile
from unittest.mock import patch, Mock

spec = importlib.util.spec_from_file_location("transport", Path(__file__).resolve().parents[1] / "scripts/quest/transport.py")
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


class TransportTests(unittest.TestCase):
    def json_status(self, *, devices='', reverse='', peers='', listening=True, adb_error=None):
        def read(*args):
            if args == ('ss', '-Hltn', 'sport = :63901'):
                return 'LISTEN 0 128 0.0.0.0:63901 0.0.0.0:*' if listening else ''
            if args == ('adb', 'devices', '-l'):
                if adb_error:
                    raise adb_error
                return 'List of devices attached\n' + devices
            if args == ('ss', '-Htn'):
                return peers
            if args[:2] == ('adb', '-s') and args[-2:] == ('reverse', '--list'):
                return reverse
            self.fail(f'Unexpected status command: {args}')
        with patch.object(transport, 'command', side_effect=read) as command, \
             patch.object(transport, 'ensure_xr_service', side_effect=AssertionError('status started service')), \
             patch.object(transport.subprocess, 'run', side_effect=AssertionError('unmocked subprocess')), \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            code = transport.main(['status', '--json'])
        return code, json.loads(output.getvalue()), [call.args for call in command.call_args_list]

    def inspect(self, peer, reverse=True):
        def command(*args):
            if args == ("adb", "devices", "-l"):
                return "List of devices attached\nQUEST device usb:4-8 model:Quest_Pro\n"
            if args[-1] == "--list":
                return "UsbFfs tcp:63901 tcp:63901" if reverse else ""
            return f"ESTAB 0 0 [::ffff:192.168.5.8]:63901 {peer}:12345\n" if peer else ""
        with patch.object(transport, "command", side_effect=command):
            return transport.inspect()

    def test_usb_requires_device_mapping_loopback_and_no_network_peer(self):
        self.assertTrue(self.inspect("[::ffff:127.0.0.1]")[0])
        self.assertFalse(self.inspect("127.0.0.1", reverse=False)[0])
        self.assertFalse(self.inspect("192.168.5.25")[0])
        self.assertFalse(self.inspect(None)[0])

    def test_wifi_adb_device_is_not_a_usb_device(self):
        with patch.object(transport, "command", return_value="192.168.5.25:5555 device model:Quest_Pro"):
            with self.assertRaises(RuntimeError):
                transport.usb_device()

    def test_local_sdk_socket_does_not_count_as_usb_xr_connection(self):
        self.assertEqual(transport.peers_from_ss("ESTAB 0 0 127.0.0.1:60061 127.0.0.1:40000"), [])

    def test_prepare_keeps_existing_mapping_and_does_not_claim_connection(self):
        state = (False, "等待 Quest 通过 USB 连接", "QUEST", True, [])
        with patch.object(transport, "usb_device", return_value="QUEST"), patch.object(transport, "command") as command, patch.object(transport, "inspect", return_value=state), patch.object(transport, "ensure_xr_service") as ensure, patch.object(transport, "xr_listening", return_value=True):
            self.assertEqual(transport.main(["prepare"]), 0)
            ensure.assert_called_once()
            command.assert_called_once_with("adb", "-s", "QUEST", "reverse", "tcp:63901", "tcp:63901")

    def test_existing_xr_receiver_is_reused(self):
        with patch.object(transport, 'xr_listening', return_value=True), patch.object(transport.subprocess, 'run') as run:
            transport.ensure_xr_service()
            run.assert_not_called()

    def test_missing_xr_receiver_is_started_before_waiting_for_headset(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(transport, 'LOG_DIR', Path(folder)), patch.object(
                transport, 'xr_listening', side_effect=[False, False, True]), patch.object(
                transport.subprocess, 'run', return_value=Mock(returncode=0)) as run:
            transport.ensure_xr_service()
            self.assertEqual(run.call_args.args[0], ['bash', str(transport.PROJECT_ROOT / 'scripts/quest/start_service.sh')])
            self.assertTrue(run.call_args.kwargs['start_new_session'])

    def test_json_status_reports_receiver_even_when_usb_is_absent(self):
        code, state, calls = self.json_status()
        self.assertEqual(code, 0)
        self.assertEqual(state, {
            'receiver_listening': True, 'usb_devices': [], 'usb_serial': None,
            'usb_status': 'missing', 'reverse_configured': False, 'peers': [],
            'usb_connected': False, 'error': None,
        })
        self.assertFalse(any('reverse' in call for call in calls))

    def test_json_status_preserves_wifi_connection_without_usb(self):
        code, state, _ = self.json_status(
            devices='192.168.5.25:5555 device model:Quest_Pro\n',
            peers='ESTAB 0 0 192.168.5.8:63901 192.168.5.25:12345\n')
        self.assertEqual(code, 0)
        self.assertEqual(state['peers'], ['192.168.5.25'])
        self.assertEqual(state['usb_status'], 'missing')
        self.assertFalse(state['usb_connected'])

    def test_unconfirmed_devices_are_never_reported_as_authorized_usb(self):
        for device, expected in [
                ('QUEST unauthorized transport_id:1\n', 'unauthorized'),
                ('QUEST offline transport_id:1\n', 'offline'),
                ('???????????? no permissions (missing udev rules)\n', 'unauthorized'),
                ('QUEST device transport_id:1\n', 'missing'),
                ('192.168.5.25:5555 unauthorized\n', 'missing'),
                ('adb-QUEST-token._adb-tls-connect._tcp offline\n', 'missing')]:
            with self.subTest(device=device):
                code, state, calls = self.json_status(devices=device)
                self.assertEqual(code, 0)
                self.assertEqual(state['usb_status'], expected)
                self.assertIsNone(state['usb_serial'])
                self.assertFalse(state['usb_connected'])
                self.assertFalse(any('reverse' in call for call in calls))

    def test_multiple_authorized_devices_are_not_selected_arbitrarily(self):
        _, state, calls = self.json_status(devices='ONE device usb:4-1\nTWO device usb:4-2\n')
        self.assertEqual(state['usb_status'], 'multiple')
        self.assertEqual(len(state['usb_devices']), 2)
        self.assertIsNone(state['usb_serial'])
        self.assertFalse(any('reverse' in call for call in calls))

    def test_json_usb_connection_requires_mapping_and_only_loopback_peers(self):
        arguments = {'devices': 'QUEST device usb:4-8 model:Quest_Pro\n',
                     'reverse': 'UsbFfs tcp:63901 tcp:63901\n',
                     'peers': 'ESTAB 0 0 [::ffff:127.0.0.1]:63901 [::ffff:127.0.0.1]:12345\n'}
        code, state, _ = self.json_status(**arguments)
        self.assertEqual(code, 0)
        self.assertEqual(state['usb_status'], 'ready')
        self.assertEqual(state['usb_serial'], 'QUEST')
        self.assertEqual(state['peers'], ['127.0.0.1'])
        self.assertTrue(state['usb_connected'])
        _, unmapped, _ = self.json_status(**{**arguments, 'reverse': ''})
        self.assertFalse(unmapped['usb_connected'])
        arguments['peers'] += 'ESTAB 0 0 192.168.5.8:63901 192.168.5.25:12345\n'
        _, mixed, _ = self.json_status(**arguments)
        self.assertFalse(mixed['usb_connected'])

    def test_failed_probe_still_emits_json_with_other_available_evidence(self):
        code, state, _ = self.json_status(
            adb_error=FileNotFoundError('adb unavailable'),
            peers='ESTAB 0 0 192.168.5.8:63901 192.168.5.25:12345\n')
        self.assertEqual(code, 1)
        self.assertTrue(state['receiver_listening'])
        self.assertEqual(state['peers'], ['192.168.5.25'])
        self.assertIn('adb unavailable', state['error'])
        self.assertFalse(state['usb_connected'])

    def test_prepare_ensures_receiver_before_reporting_missing_usb(self):
        events = []
        def missing_usb():
            events.append('usb')
            raise RuntimeError('未检测到 USB Quest')
        with patch.object(transport, 'ensure_xr_service', side_effect=lambda: events.append('receiver')), \
             patch.object(transport, 'usb_device', side_effect=missing_usb), \
             patch.object(transport, 'command') as command, \
             patch.object(transport.console, 'print') as output:
            self.assertEqual(transport.main(['prepare']), 1)
        self.assertEqual(events, ['receiver', 'usb'])
        command.assert_not_called()
        self.assertIn('未检测到 USB Quest', output.call_args.args[0].renderable.plain)
