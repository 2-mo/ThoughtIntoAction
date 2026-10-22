"""Clock alignment, delayed tracking, absent samples and candidate pause metrics."""
import copy
import math
import unittest

from embodied.applications.operations.benchmark import analyze_motion, compare_reports


def trace(lag=.04, paused=False):
    rows = []
    base = 1_000_000_000
    for n in range(200):
        t = n * .01
        # Feedback at t; this tick's command is sent later, at t + 1 ms.
        source_t = max(0., t - lag)
        rows.append({'runtime_id': 'test', 'sequence': n,
            'timing': {'tick_ns': base + n * 10_000_000,
                       'feedback_received_ns': base + n * 10_000_000,
                       'command_submitted_ns': base + n * 10_000_000 + 1_000_000},
            'feedback': {'q': [math.sin(source_t)] * 7, 'dq': [0. if paused else math.cos(source_t)] * 7},
            'command': {'q': [math.sin(t + .001)] * 7, 'dq': [math.cos(t + .001)] * 7, 'submitted': True}})
    return rows


class BenchmarkTests(unittest.TestCase):
    def test_comparison_rejects_different_actions_and_preserves_unknown_metrics(self):
        report = {'schema': 'embodied.motion_benchmark/1', 'joint': 0, 'delta_rad': .1, 'repeats': 1,
                  'start_q': [0.] * 7,
                  'requested_speed_rad_s': 0, 'status': 'completed', 'runtime': {'backend': 'sim'},
                  'description': {'model_id': 'model'}, 'descriptor': {'parameters': {}},
                  'operations': [{'execute_to_settled_s': 2., 'planning_ms': 10., 'measurement': {}}]}
        current = copy.deepcopy(report)
        current['descriptor']['parameters']['created_utc'] = 'different run time'
        current['operations'][0]['execute_to_settled_s'] = 1.
        result = compare_reports(current, report)
        self.assertEqual(result['metrics']['execute_to_settled_s']['change_percent'], -50.)
        self.assertIsNone(result['metrics']['peak_feedback_rad_s']['change_percent'])
        self.assertFalse(result['configuration_changed'])
        current['start_q'][0] = .1
        with self.assertRaisesRegex(ValueError, 'starting joint'):
            compare_reports(current, report)
        current['start_q'][0] = .0005
        compare_reports(current, report)
        current['delta_rad'] = .2
        with self.assertRaisesRegex(ValueError, 'same model'):
            compare_reports(current, report)

    def test_measured_seven_axis_lag_and_cycles_use_independent_timestamps(self):
        report = analyze_motion(trace())
        self.assertAlmostEqual(report['estimated_position_lag_ms'], 40., delta=2.)
        self.assertEqual(report['cycle_ms']['p99'], 10.)
        self.assertEqual(len(report['reference_error_rms_rad']), 7)
        self.assertEqual(report['candidate_pause_count'], 0)
        self.assertGreater(report['reference_error_max_rad'][0], .02)

    def test_pause_is_counted_once_and_sample_gaps_do_not_become_cycles(self):
        rows = trace(paused=True)[:100]
        self.assertEqual(analyze_motion(rows)['candidate_pause_count'], 1)
        rows = rows[:20] + rows[30:]
        report = analyze_motion(rows)
        self.assertEqual(report['sequence_gaps'], 10)
        self.assertEqual(report['cycle_ms']['max'], 10.)
        self.assertIsNone(report['estimated_position_lag_ms'])

    def test_missing_sources_stay_unknown_and_invalid_identity_is_rejected(self):
        self.assertIsNone(analyze_motion([])['feedback_peak_velocity_rad_s'])
        rows = trace()[:2]
        rows[1]['runtime_id'] = 'other'
        with self.assertRaisesRegex(ValueError, 'identities'):
            analyze_motion(rows)
        rows = trace()[:2]
        rows[1]['feedback']['q'] = [float('nan')] * 7
        with self.assertRaisesRegex(ValueError, 'finite'):
            analyze_motion(rows)


if __name__ == '__main__':
    unittest.main()
