"""Gaps between kept camera anchors are recorded, not hidden (todo D-01)."""
import unittest

from dataset_tools.exporters import anchor_gaps, annotate


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


if __name__ == '__main__':
    unittest.main()
