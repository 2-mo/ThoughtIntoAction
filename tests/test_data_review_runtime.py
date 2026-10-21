"""`data review` on kernel (schema 2) episodes: validate, label, inspect."""
import contextlib
import io
import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from embodied.data import datasets
from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
from test_runtime_collection import tick


def episode(root):
    descriptor = {'runtime_id': 'test', 'backend': 'sim', 'parameters': {'model': {'manifest': 'm'}}}
    camera = {'stream_id': 'rgb', 'width': 32, 'height': 24, 'fps': 30}
    writer = RuntimeEpisodeWriter(root, descriptor, 'review', camera)
    origin = time.monotonic_ns()
    for i in range(60):
        writer.add_tick(tick(i, origin + i * 4_000_000))
        if i % 8 == 0:
            writer.add_frame({'image': np.zeros((24, 32, 3), np.uint8), 'stream_id': 'rgb',
                              'host_received_monotonic_ns': origin + i * 4_000_000 + 3_000_000, 'frame_number': i // 8})
    writer.close('duration')
    return writer.path


class RuntimeReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = episode(self.temp.name)

    def test_validate_reports_the_training_view(self):
        report = datasets.validate_runtime(self.path)
        self.assertTrue(report['valid'], report)
        self.assertEqual(report['cameras'], ['wrist_rgb'])
        self.assertGreater(report['training']['samples'], 1)
        self.assertEqual(report['training']['control_stack'], 'arx-televr kernel: sent_q at 250 Hz')

    def test_label_keeps_history_and_cli_dispatches_by_schema(self):
        datasets.label_episode(self.path, 'success', 'looks good')
        datasets.label_episode(self.path, 'discarded')
        meta = json.loads((self.path / 'episode.json').read_text())
        self.assertEqual(meta['outcome'], 'discarded')
        self.assertEqual([h['outcome'] for h in meta['outcome_history']], ['success', 'discarded'])
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(datasets.main(['inspect', str(self.path)]), 0)
        self.assertEqual(json.loads(out.getvalue())['stored']['frame'], 8)
        with self.assertRaisesRegex(ValueError, 'schema 1'):
            datasets.main(['replay', str(self.path)])

    def test_damaged_storage_is_invalid(self):
        (self.path / 'episode.mcap').write_bytes(b'broken')
        report = datasets.validate_runtime(self.path)
        self.assertFalse(report['valid'])


if __name__ == '__main__':
    unittest.main()
