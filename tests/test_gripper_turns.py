"""Exercise the compiled SDK compensation without constructing hardware."""
import math
import unittest
import arx5_interface as sdk


class GripperTurnsTests(unittest.TestCase):
    def test_recorded_startup_frame_and_round_trip(self):
        self.assertEqual(sdk.quest_gripper_turns_version, 1)
        turns = sdk._GripperTurns()
        value = turns.feedback(6.186579690241857, -3.4, .082)
        self.assertAlmostEqual(value, .0023299001732040676)
        self.assertAlmostEqual(turns.offset(), 2 * math.pi)
        for width in (0., .00233, .04, .077, .082):
            command = turns.command(width, -3.4, .082)
            self.assertAlmostEqual(command, width / .082 * -3.4 + 2 * math.pi)
            self.assertAlmostEqual(turns.feedback(command, -3.4, .082), width)

    def test_previous_negative_74_mm_error(self):
        turns = sdk._GripperTurns()
        self.assertAlmostEqual(turns.feedback(-.074 / .082 * -3.4, -3.4, .082),
                               .07753564564374298)

    def test_valid_original_feedback_stays_unchanged(self):
        for sign in (-1, 1):
            turns = sdk._GripperTurns()
            self.assertAlmostEqual(turns.feedback(sign * 1.7, sign * 3.4, .082), .041)
            self.assertEqual(turns.offset(), 0.)

    def test_running_fault_is_not_reinterpreted_as_another_turn(self):
        turns = sdk._GripperTurns()
        turns.feedback(6.186579690241857, -3.4, .082)
        # A later jump to zero remains outside the SDK's 5 mm tolerance.
        self.assertGreater(turns.feedback(0., -3.4, .082), .087)
        self.assertAlmostEqual(turns.offset(), 2 * math.pi)

    def test_invalid_or_ambiguous_initial_readout_is_rejected(self):
        for raw, readout, width in ((1., -3.4, .082), (float('nan'), -3.4, .082),
                                     (0., 0., .082), (0., -7., .082), (0., -3.4, 0.)):
            with self.subTest(raw=raw, readout=readout, width=width):
                with self.assertRaises(RuntimeError):
                    sdk._GripperTurns().feedback(raw, readout, width)
        with self.assertRaises(RuntimeError):
            sdk._GripperTurns().command(0., -3.4, .082)


if __name__ == '__main__':
    unittest.main()
