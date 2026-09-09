"""Power sequencing tests use fake subprocesses; never call adb or control a device."""
import importlib.util
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location("quest_console", Path(__file__).resolve().parents[1] / "scripts/quest/console.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.console = module.Console("/fake/run.sh", ["teleop", "--actuate"], Path(self.folder.name))
        self.addCleanup(self.console.log.close)
        self.console.transport = Mock(return_value=True)

    def test_sleep_stops_and_waits_before_requesting_power(self):
        child = Mock()
        child.poll.return_value = None
        events = []
        child.send_signal.side_effect = lambda sig: events.append(("signal", sig))
        child.wait.side_effect = lambda **kw: events.append(("wait", kw["timeout"]))
        self.console.child = child
        self.console.power = lambda command: events.append(("power", command)) or True
        self.console.sleep()
        self.assertEqual(events, [("signal", signal.SIGINT), ("wait", 10), ("power", "stop")])

    def test_stuck_child_blocks_sleep(self):
        self.console.child = Mock()
        self.console.child.poll.return_value = None
        self.console.child.wait.side_effect = subprocess.TimeoutExpired("teleop", 10)
        self.console.power = Mock()
        self.console.sleep()
        self.console.power.assert_not_called()

    def test_failed_wake_does_not_start_control_and_success_spawns_fresh_child(self):
        self.console.power = Mock(return_value=False)
        with patch.object(module.subprocess, "Popen") as spawn:
            self.console.wake()
            spawn.assert_not_called()
            self.console.power.return_value = True
            self.console.wake()
            self.assertEqual(spawn.call_args.args[0], ["/fake/run.sh", "teleop", "--actuate"])
            self.assertTrue(spawn.call_args.kwargs["start_new_session"])
            self.assertEqual(spawn.call_args.kwargs["stdin"], subprocess.DEVNULL)

    def test_quit_stops_control_without_power_change(self):
        self.console.wake = Mock()
        self.console.stop = Mock(return_value=True)
        self.console.power = Mock()
        with patch("builtins.input", return_value="q"):
            self.console.run()
        self.console.stop.assert_called()
        self.console.power.assert_not_called()

    def test_unwear_and_rewear_keep_same_child_and_order_pause_before_power(self):
        child = Mock()
        child.poll.return_value = None
        self.console.child = child
        events = []
        self.console.session = lambda action: events.append(("session", action)) or True
        self.console.power = lambda action: events.append(("power", action)) or True
        self.console.unwear()
        self.console.wake()
        self.assertIs(self.console.child, child)
        child.send_signal.assert_not_called()
        self.assertEqual(events, [("session", "pause"), ("power", "unwear"),
                                  ("session", "pause"), ("power", "start"), ("session", "resume")])

    def test_failed_pause_does_not_change_power(self):
        self.console.child = Mock()
        self.console.child.poll.return_value = None
        self.console.session = Mock(return_value=False)
        self.console.power = Mock()
        self.console.unwear()
        self.console.power.assert_not_called()

    def test_missing_usb_prevents_wake_and_start(self):
        self.console.transport.return_value = False
        self.console.power = Mock()
        with patch.object(module.subprocess, "Popen") as spawn:
            self.console.wake()
            self.console.power.assert_not_called()
            spawn.assert_not_called()

    def test_unconfirmed_usb_after_wake_does_not_start_control(self):
        self.console.transport.side_effect = [True, False]
        self.console.power = Mock(return_value=True)
        with patch.object(module.subprocess, "Popen") as spawn:
            self.console.wake()
            self.console.power.assert_called_once_with("start")
            spawn.assert_not_called()

    def test_exit_is_reported_without_waiting_for_status_key(self):
        self.console.log.write(b"ValueError: outside model limits\n")
        self.console.child = Mock()
        self.console.child.poll.return_value = 1
        stop = Mock()
        stop.wait.side_effect = [False, True]
        with patch.object(module.rich_console, "print") as display:
            self.console.watch_child(stop)
        self.assertTrue(any("outside model limits" in str(call) for call in display.call_args_list))
