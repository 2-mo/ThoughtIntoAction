"""Transport evidence and refusal tests; no adb calls or real connection changes."""
import importlib.util
from pathlib import Path
import unittest
import tempfile
from unittest.mock import patch, Mock

spec = importlib.util.spec_from_file_location("transport", Path(__file__).resolve().parents[1] / "scripts/quest/transport.py")
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


class TransportTests(unittest.TestCase):
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
