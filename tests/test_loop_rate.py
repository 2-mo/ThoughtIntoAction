import unittest
from robot_core.loop_rate import LoopRate


class LoopRateTests(unittest.TestCase):
    def test_measures_execution_instead_of_config_and_expires(self):
        rate = LoopRate(50)
        self.assertIsNone(rate.snapshot(0)['observed_hz'])
        for stamp in range(0, 1_000_000_001, 40_000_000):
            rate.tick(stamp)
        self.assertAlmostEqual(rate.snapshot(1_000_000_000)['observed_hz'], 25)
        self.assertEqual(rate.snapshot(1_000_000_000)['configured_hz'], 50)
        self.assertIsNone(rate.snapshot(1_600_000_000)['observed_hz'])
        for stamp in range(2_000_000_000, 3_000_000_001, 20_000_000):
            rate.tick(stamp)
        self.assertAlmostEqual(rate.snapshot(3_000_000_000)['observed_hz'], 50)


if __name__ == '__main__':
    unittest.main()
