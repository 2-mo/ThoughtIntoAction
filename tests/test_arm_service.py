"""Service ownership, device-independent lifecycle and failure handling over actual RPC."""

import json
import socket
import time
import unittest
from unittest.mock import patch, MagicMock

import numpy as np

from arm_service_fixture import ServiceFixture
from robot_control.client import ArmClient
from robot_control.service import rpc_listener


class ArmServiceTests(unittest.TestCase):
    def setUp(self):
        self.fixture = ServiceFixture()
        self.addCleanup(self.fixture.close)

    def client(self, **kwargs):
        client = ArmClient(self.fixture.robot, path=self.fixture.path, hardware=False, **kwargs)
        # Disconnected/revoked clients can fail release; this does not stop the service.
        def close():
            try:
                client.close()
            except RuntimeError:
                pass
        self.addCleanup(close)
        return client

    def test_command_timing_is_measured_and_not_refreshed_by_observer_reads(self):
        client = self.client(actuate=True)
        self.assertFalse(hasattr(client, "command_timing"))
        before = time.monotonic_ns()
        client.set_catch_pos(.04)
        timing = dict(client.command_timing)
        after = time.monotonic_ns()
        self.assertLessEqual(before, timing["sample_monotonic_ns"])
        self.assertLessEqual(timing["sample_monotonic_ns"], after)
        self.assertGreaterEqual(timing["duration_ms"], 0)
        self.assertLessEqual(timing["duration_ms"], (after - before) / 1e6)
        client.snapshot()
        self.assertEqual(client.command_timing, timing)

    def test_full_open_request_is_capped_without_losing_control(self):
        client = self.client(actuate=True)
        client.set_catch_pos(.082)
        time.sleep(.06)
        state = client.snapshot()
        self.assertAlmostEqual(state["service_command"]["gripper"], .077)
        self.assertEqual(client.settings["gripper_width"], .082)
        self.assertIsNotNone(self.fixture.service.owner)
        client.set_catch_pos(.04)
        time.sleep(.06)
        self.assertAlmostEqual(client.snapshot()["service_command"]["gripper"], .04)

    def test_idle_feedback_advances_without_any_controller(self):
        observer = self.client()
        initial = observer.snapshot()
        time.sleep(.06)
        state = observer.snapshot()
        self.assertGreater(state["host_monotonic_ns"], initial["host_monotonic_ns"])
        self.assertIsNone(self.fixture.service.owner)
        self.assertFalse(state["actuate"])
        self.fixture.publisher.update.assert_called()
        began = time.monotonic_ns()
        self.assertGreaterEqual(observer.snapshot()["host_monotonic_ns"], began)

    def test_model_or_backend_mismatch_never_acquires_control(self):
        with self.assertRaisesRegex(ValueError, "hardware/virtual"):
            ArmClient(self.fixture.robot, path=self.fixture.path, hardware=True)
        with patch.object(self.fixture.robot, "metadata", return_value={"wrong": True}):
            # A separate model avoids changing the server's own metadata.
            from robot_core.model import RobotModel
            with self.assertRaisesRegex(ValueError, "model/frame"):
                ArmClient(RobotModel(), path=self.fixture.path, hardware=False)
        self.assertIsNone(self.fixture.service.owner)

    def test_only_one_owner_and_observers_do_not_release_it(self):
        one, two, observer = self.client(actuate=True), self.client(actuate=True), self.client()
        one.start_control()
        with self.assertRaisesRegex(RuntimeError, "owned by"):
            two.start_control()
        observer.close()
        self.assertIsNotNone(self.fixture.service.owner)
        one.close()
        two.start_control()
        self.assertIsNotNone(self.fixture.service.owner)

    def test_explicit_speed_overrides_default_and_next_owner_without_speed_uses_default(self):
        default = self.fixture.service.settings["max_joint_speed_rad_s"]
        for speed in (default * 2, default / 2, None):
            with self.subTest(speed=speed):
                client = self.client(actuate=True, speed=speed)
                client.start_control()
                expected = default if speed is None else speed
                client.snapshot()
                self.assertEqual(client.info["effective_max_joint_speed_rad_s"], expected)
                self.assertAlmostEqual(self.fixture.arm._max_step, expected * self.fixture.arm._dt)
                self.assertEqual(self.fixture.service.settings["max_joint_speed_rad_s"], default)
                client.close()

    def test_motion_is_rate_limited_at_service_ticks_and_release_keeps_feedback(self):
        client = self.client(actuate=True, speed=.1)
        initial = client.get_joint_positions()
        target = initial[:6].copy()
        target[0] += .2
        client.start_control()
        start = time.monotonic()
        for _ in range(20):
            client.set_joint_positions(target)
            client.set_catch_pos(initial[6])
        time.sleep(.06)
        actual = client.get_joint_positions()
        self.assertGreater(actual[0], initial[0])
        self.assertLessEqual(actual[0] - initial[0], .1 * (time.monotonic() - start + .02))
        client.close()
        observer = self.client()
        time.sleep(.05)
        self.assertFalse(observer.snapshot()["actuate"])
        self.assertIsNone(self.fixture.service.owner)
        self.assertTrue(self.fixture.worker.is_alive())

    def test_watchdog_revokes_silent_owner_and_rejects_old_token(self):
        client = self.client(actuate=True)
        client.start_control()
        time.sleep(.6)
        observer = self.client()
        self.assertIsNone(observer.info["owner"])
        with self.assertRaisesRegex(RuntimeError, "expired"):
            client.set_catch_pos(.04)
        self.assertIsNone(self.fixture.service.owner)

    def test_delayed_repeated_and_invalid_commands_are_rejected(self):
        client = self.client(actuate=True)
        client.start_control()
        q = client.get_joint_positions()[:6].tolist()
        client._rpc("command", sequence=1, q=q, gripper=.04)
        for args in ({"sequence": 1, "q": q, "gripper": .04},
                     {"sequence": 2, "q": [99]*6, "gripper": .04},
                     {"sequence": 2, "q": q, "gripper": -.1},
                     {"sequence": 2, "q": q, "gripper": .04,
                      "sent_monotonic_ns": time.monotonic_ns() - 1_000_000_000}):
            with self.subTest(args=args), self.assertRaises(RuntimeError):
                client._rpc("command", **args)
        self.assertEqual(self.fixture.service.owner["sequence"], 1)

    def test_damping_client_cannot_move_arm(self):
        client = self.client(actuate=False)
        q = client.get_joint_positions()
        target = q[:6].copy()
        target[0] += .2
        client.set_joint_positions(target)
        client.set_catch_pos(.01)
        time.sleep(.05)
        np.testing.assert_array_equal(client.get_joint_positions(), q)

    def test_silent_and_malformed_rpc_clients_do_not_stop_sampling(self):
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as silent:
            silent.connect(str(self.fixture.path))
            observer = self.client()
            stamp = observer.snapshot()["host_monotonic_ns"]
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as malformed:
                malformed.settimeout(.3)
                malformed.connect(str(self.fixture.path))
                malformed.send(b"[]")
                self.assertFalse(json.loads(malformed.recv(65536))["ok"])
            time.sleep(.05)
            self.assertGreater(observer.snapshot()["host_monotonic_ns"], stamp)

    def test_listener_exclusivity_and_non_socket_file_preservation(self):
        with self.assertRaises(RuntimeError):
            with rpc_listener(self.fixture.path):
                self.fail("Second listener acquired socket")
        path = self.fixture.path.parent / "keep.txt"
        path.write_text("keep")
        with self.assertRaisesRegex(ValueError, "non-socket"):
            with rpc_listener(path):
                self.fail("Overwrote user file")
        self.assertEqual(path.read_text(), "keep")

    def test_restart_requires_new_client_no_command_replay(self):
        client = self.client(actuate=True)
        client.start_control()
        with patch.object(self.fixture.service, "service_id", "restarted"):
            with self.assertRaisesRegex(RuntimeError, "restarted"):
                client.set_catch_pos(.04)

    def test_native_control_enable_seeds_feedback_before_gains_and_damping_blocks_submit(self):
        import arx5_interface as arx5
        from robot_control.hardware import OfficialX5HardwareAdapter
        native = MagicMock()
        state = arx5.JointState(6)
        state.pos()[:] = [0.1, 1.1, 1.2, -1.0, .1, .2]
        state.gripper_pos = .035
        native.get_joint_state.return_value = state
        settings = self.fixture.service.settings
        with patch.object(arx5, "Arx5JointController", return_value=native):
            backend = OfficialX5HardwareAdapter(**settings, can_port="offline-service-enable", actuate=False)
            try:
                native.reset_mock()
                backend.set_control_enabled(True)
                calls = [entry[0] for entry in native.mock_calls]
                self.assertLess(calls.index("set_joint_cmd"), calls.index("set_gain"))
                command = native.set_joint_cmd.call_args.args[0]
                np.testing.assert_array_equal(command.pos(), state.pos())
                self.assertEqual(command.gripper_pos, state.gripper_pos)
                np.testing.assert_array_equal(command.torque(), np.zeros(6))
                backend.set_control_enabled(False)
                native.reset_mock()
                backend.set_catch_pos(.01)
                native.set_joint_cmd.assert_not_called()
                self.assertFalse(backend.actuate)
            finally:
                backend.close()
