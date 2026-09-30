"""Raw stream preservation, causal alignment, and incomplete episode handling."""
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from collection.capture import capture
from dataset_tools.runtime_v2 import load_episode
from episode_io.runtime_episode import RuntimeEpisodeWriter, split_tick
from observations.runtime_alignment import align


def tick(sequence, stamp, *, terminal=False, mode=3):
    return {'type': 'tick', 'version': 1, 'runtime_id': 'test', 'sequence': sequence,
        'producer_dropped': 0, 'observer_gaps': 0, 'phase': 3 if terminal else 1,
        'source': 'quest', 'source_sequence': None, 'error': None, 'terminal': terminal,
        'timing': {'completed_ns': stamp + 3_000_000, 'feedback_received_ns': stamp,
                   'command_started_ns': stamp + 1_000_000, 'command_submitted_ns': stamp + 2_000_000},
        'feedback': None if terminal else {'q': [sequence * .001] * 6, 'gripper_width': .04,
                                           'mode': mode, 'gripper_mode': mode},
        'command': None if terminal else {'q': [sequence * .002] * 6, 'gripper_width': .05,
                                         'mode': mode, 'gripper_mode': mode, 'submitted': True}}


class RuntimeCollectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.stamp = time.monotonic_ns()
        self.camera = {'stream_id': 'camera-one', 'width': 4, 'height': 4, 'fps': 30}

    def writer(self, camera=None):
        return RuntimeEpisodeWriter(self.temp.name, {'runtime_id': 'test'}, 'test capture', camera)

    def frame(self, stamp, number=1, **changes):
        return {'image': np.zeros((4, 4, 3), np.uint8), 'host_received_monotonic_ns': stamp,
                'frame_number': number, 'stream_id': 'camera-one', **changes}

    def test_roundtrip_preserves_independent_timestamps_and_raw_ticks(self):
        writer = self.writer(self.camera)
        rows = [tick(0, self.stamp), tick(1, self.stamp + 4_000_000), tick(2, self.stamp + 8_000_000, terminal=True)]
        for row in rows:
            writer.add_tick(row)
        # Image is after feedback from tick 1 but before its command submission.
        writer.add_frame(self.frame(self.stamp + 5_000_000))
        self.assertEqual(writer.close('runtime-terminal')['lifecycle'], 'complete')
        meta, stored = load_episode(writer.path)
        self.assertEqual(stored['tick'], rows)
        anchors, excluded = align(stored['state'], stored['command'], stored['frame'])
        self.assertEqual(excluded, {})
        self.assertEqual(anchors[0]['state_sequence'], 1)
        self.assertEqual(anchors[0]['action_sequence'], 0)
        self.assertEqual(anchors[0]['action_monotonic_ns'], self.stamp + 2_000_000)
        self.assertTrue(meta['terminal_received'])

    def test_latest_nonfollowing_command_does_not_reuse_old_action(self):
        following = split_tick(tick(0, self.stamp))
        home = split_tick(tick(1, self.stamp + 4_000_000, mode=0))
        anchors, excluded = align([following['state'], home['state']],
            [following['command'], home['command']], [self.frame(self.stamp + 7_000_000)])
        self.assertEqual(anchors, [])
        self.assertEqual(excluded, {'not-position-following': 1})

    def test_camera_restart_and_missing_frames_fail_recording(self):
        for changes in [{'number': 3}, {'stream_id': 'camera-two'}]:
            writer = self.writer(self.camera)
            writer.add_tick(tick(0, self.stamp))
            writer.add_frame(self.frame(self.stamp))
            with self.assertRaises(ValueError):
                writer.add_frame(self.frame(self.stamp + 33_000_000, **changes))
            writer.close('stream-error', 'camera incomplete')
            with self.assertRaisesRegex(ValueError, 'incomplete'):
                load_episode(writer.path)

    def test_manifest_cannot_hide_tick_gap_or_uncommitted_recording(self):
        writer = self.writer()
        writer.add_tick(tick(1, self.stamp))
        with self.assertRaisesRegex(ValueError, 'sequence gap'):
            writer.add_tick(tick(3, self.stamp + 4_000_000))
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            load_episode(writer.path)
        writer.close('stream-error', 'gap')
        path = writer.path / 'episode.json'
        manifest = json.loads(path.read_text())
        manifest.update(lifecycle='complete', error=None, last_sequence=3)
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'boundaries'):
            load_episode(writer.path)

    def test_premature_eof_fails_collector_but_explicit_terminal_completes(self):
        class Reader:
            mode, descriptor, terminal = 'full', {'runtime_id': 'test'}, False
            def __init__(self, rows):
                self.rows = iter(rows)
            def read(self):
                try:
                    row = next(self.rows)
                except StopIteration:
                    raise EOFError('observer disappeared')
                self.terminal = row['terminal']
                return copy.deepcopy(row)
        for terminal in (False, True):
            rows = [tick(0, self.stamp)]
            if terminal:
                rows.append(tick(1, self.stamp + 4_000_000, terminal=True))
            path, meta = capture(Reader(rows), self.temp.name, 'capture', 0)
            self.assertEqual(meta['lifecycle'], 'complete' if terminal else 'failed')
            if terminal:
                self.assertEqual(load_episode(path)[0]['kind'], 'quest_runtime_trace')


if __name__ == '__main__':
    unittest.main()
