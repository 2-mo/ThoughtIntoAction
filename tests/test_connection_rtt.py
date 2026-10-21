"""Real local round trips; fake USB echo process avoids device operations."""
import subprocess
import time
import unittest
from unittest.mock import patch
from arm_service_fixture import ServiceFixture
from embodied.monitor.connections import UsbEcho, arm_rtt, ConnectionProbe


class ConnectionTests(unittest.TestCase):
    def test_arm_probe_is_read_only_and_does_not_acquire_control(self):
        fixture = ServiceFixture()
        try:
            self.assertGreaterEqual(arm_rtt(fixture.path), 0)
            self.assertIsNone(fixture.service.owner)
        finally:
            fixture.close()

    def test_echo_times_round_trip_and_closes_process(self):
        real_popen = subprocess.Popen
        def spawn(*args, **kwargs):
            return real_popen(["sh", "-c", 'while IFS= read -r line; do printf "%s\\n" "$line"; done'], **kwargs)
        with patch("embodied.monitor.connections.subprocess.Popen", side_effect=spawn):
            echo = UsbEcho("fake")
        try:
            self.assertGreaterEqual(echo.measure(), 0)
        finally:
            echo.close()
        self.assertIsNotNone(echo.process.poll())

    def test_stale_measurement_is_hidden(self):
        probe = object.__new__(ConnectionProbe)
        probe.values = {"arm": {"duration_ms": 1.2, "sample_monotonic_ns": time.monotonic_ns() - 4_000_000_000}}
        self.assertEqual(probe.snapshot(), {})
        probe.store("arm", 2.3)
        self.assertEqual(probe.snapshot()["arm"]["duration_ms"], 2.3)
        probe.store("arm")
        self.assertIsNone(probe.snapshot()["arm"]["duration_ms"])
