"""Gaps between kept camera anchors are recorded, not hidden (todo D-01)."""
import unittest
import importlib.util
from pathlib import Path

from embodied.data.export.exporters import anchor_gaps, annotate, runtime_timing, Schema2


def rows(stamps_ms):
    return [(None, None, None, {'host_received_monotonic_ns': int(ms * 1e6)}, None) for ms in stamps_ms]


class AnchorGapTests(unittest.TestCase):
    def test_regular_30fps_anchors_have_no_gap(self):
        self.assertEqual(anchor_gaps(rows([i * 1000 / 30 for i in range(30)])), [])

    def test_removed_segment_is_one_gap_with_its_length(self):
        stamps = [i * 1000 / 30 for i in range(10)] + [i * 1000 / 30 for i in range(40, 50)]
        self.assertEqual(anchor_gaps(rows(stamps)), [{'after_sample': 9, 'duration_ms': 1033.3, 'missing_frames': 30}])

    def test_jitter_below_one_and_a_half_periods_is_not_a_gap(self):
        self.assertEqual(anchor_gaps(rows([0, 33.3, 82, 115])), [])
        self.assertEqual(len(anchor_gaps(rows([0, 33.3, 84, 117]))), 1)

    def test_annotation_marks_the_episode_as_anomaly(self):
        report = annotate({}, rows([0, 33.3, 200]), control_stack='stack', backend='sim')
        self.assertEqual((report['control_stack'], report['backend'], report['anomaly']), ('stack', 'sim', True))
        self.assertEqual(report['gaps'][0]['missing_frames'], 4)


class RuntimeTimingTests(unittest.TestCase):
    def test_configured_rate_and_measured_stream_rates_are_distinct(self):
        metadata = {'runtime': {'parameters': {'effective_robot_config': {'control_hz': 500}}}}
        records = lambda stamps: [{'host_monotonic_ns': value} for value in stamps]
        timing = runtime_timing(metadata, {'state': records([1, 4_000_001, 8_000_001]),
                                           'command': records([2, 8_000_002])})
        self.assertEqual(timing['configured_control_hz'], 500)
        self.assertEqual(timing['feedback_mean_hz'], 250.)
        self.assertEqual(timing['command_mean_hz'], 125.)
        self.assertEqual(Schema2.layout, 'aligned30_runtime_v2')
        self.assertIsNone(Schema2.raw_hz)

    def test_unknown_configuration_and_gaps_do_not_fabricate_250_hz(self):
        timing = runtime_timing({'runtime': {}}, {
            'state': [{'host_monotonic_ns': value} for value in (1, 4_000_001, 20_000_001)],
            'command': [{'host_monotonic_ns': 1}]})
        self.assertIsNone(timing['configured_control_hz'])
        self.assertEqual(timing['feedback_mean_hz'], 100.)
        self.assertIsNone(timing['command_mean_hz'])

    def test_new_and_historical_layouts_select_the_same_action_adapter(self):
        spec = importlib.util.spec_from_file_location('openpi_training_entry',
            Path(__file__).resolve().parents[1] / 'scripts/training/openpi/run.py')
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        info = {'fps': 30, 'features': {'action': {'shape': [7]}, 'observation.state': {'shape': [7]},
                                      'observation.images.wrist': {'shape': [48, 64, 3]}}}
        action = {'action_definition': 'sent_joint_position + sent_gripper_width'}
        for layout in ('aligned30_raw250_v2', 'aligned30_runtime_v2'):
            self.assertEqual(entry.dataset_config({'layout': layout, **action}, info),
                             ('local/x5_kernel', 'pi05_x5_kernel'))
        info['features']['action']['shape'] = [8]
        with self.assertRaisesRegex(SystemExit, 'select an adapter'):
            entry.dataset_config({'layout': 'aligned30_runtime_v2', **action}, info)


if __name__ == '__main__':
    unittest.main()
