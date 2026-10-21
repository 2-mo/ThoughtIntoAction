"""Frozen Quest frames must not keep issuing arm or gripper commands."""

import threading
import unittest
from unittest.mock import Mock, patch

from legacy.quest_teleop.inputs import InputFreshness
from legacy.quest_teleop.controller import X5TeleopController


class FreshnessTests(unittest.TestCase):
    def setUp(self):
        self.now, self.stamp = 0., 0
        self.guard = InputFreshness(lambda: self.stamp, clock=lambda: self.now)

    def test_initial_cached_frame_requires_timestamp_progress(self):
        self.assertFalse(self.guard.ready())
        self.now, self.stamp = .1, 123
        self.assertFalse(self.guard.ready())
        self.now = .2
        self.assertFalse(self.guard.ready())
        self.stamp += 1
        self.assertTrue(self.guard.ready())

    def test_frozen_positive_frame_is_rejected_and_timestamp_reset_requires_restart(self):
        self.stamp = 100
        self.guard.ready()
        self.now, self.stamp = .1, 101
        self.assertTrue(self.guard.ready())
        self.now = .7
        with self.assertRaisesRegex(RuntimeError, "stopped updating"):
            self.guard.ready()
        self.stamp = 1
        with self.assertRaisesRegex(RuntimeError, "timestamp reset"):
            self.guard.ready()

    def test_stationary_pose_is_valid_as_long_as_frames_advance(self):
        for index in range(100):
            self.now, self.stamp = index * .02, index + 1
            self.assertEqual(self.guard.ready(), index > 0)

    def test_startup_without_data_has_bounded_wait(self):
        self.now = 9.
        self.assertFalse(self.guard.ready())
        self.now = 11.
        with self.assertRaisesRegex(RuntimeError, "No Quest frames"):
            self.guard.ready()

    def test_frozen_input_clears_send_gate_and_stops_threads(self):
        controller = object.__new__(X5TeleopController)
        controller._state_lock = threading.RLock()
        controller._stop_event = threading.Event()
        controller._input_ready = True
        controller._input_freshness = Mock()
        controller._input_freshness.ready.side_effect = RuntimeError("frozen")
        controller._control_gate_ready = Mock(return_value=True)
        controller._update_robot_state = Mock()
        with self.assertRaisesRegex(RuntimeError, "frozen"):
            controller._ik_rate = Mock()
            controller._ik_thread(controller._stop_event)
        self.assertFalse(controller._input_ready)
        self.assertTrue(controller._stop_event.is_set())
        controller._update_robot_state.assert_not_called()

    def test_worker_exception_reaches_main_thread_instead_of_success_exit(self):
        from legacy.quest_teleop.base_controller import X5TeleopBase, HardwareTeleopController
        controller = object.__new__(X5TeleopController)
        controller._service_mode = False
        controller._thread_error = ValueError("outside model limits")
        controller.close = Mock()
        with patch.object(HardwareTeleopController, "run"):
            with self.assertRaisesRegex(RuntimeError, "outside model limits"):
                X5TeleopBase.run(controller)
        controller.close.assert_called_once()

    def test_input_processing_time_covers_work_without_using_device_clock(self):
        controller = object.__new__(X5TeleopController)
        controller._state_lock = threading.RLock()
        controller._stop_event = threading.Event()
        controller._input_freshness = None
        controller.dt = .02
        controller._control_gate_ready = Mock(return_value=True)
        controller._update_robot_state = Mock()
        controller._update_gripper_target = Mock()
        controller._pre_ik_update = Mock()
        controller._update_ik = Mock(side_effect=controller._stop_event.set)
        with patch("legacy.quest_teleop.base_controller.time.monotonic_ns", side_effect=[1_000_000, 4_500_000]):
            controller._ik_rate = Mock()
            controller._ik_thread(controller._stop_event)
        self.assertEqual(controller._input_timing, {"sample_monotonic_ns": 4_500_000, "duration_ms": 3.5})
