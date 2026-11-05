"""Raw stream preservation, causal alignment, and incomplete episode handling."""
import copy
import json
import shutil
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from embodied.data.capture.capture import capture
from embodied.data.export.runtime_v2 import load_episode
from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter, split_tick
from embodied.data.alignment.runtime_alignment import align


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

    @unittest.skipUnless(shutil.which('ffmpeg'), 'MP4 recording requires ffmpeg')
    def test_three_cameras_as_mp4_align_for_export(self):
        self.check_three_cameras_for_export(video=True)

    def test_three_cameras_as_png_align_for_export(self):
        self.check_three_cameras_for_export(video=False)

    def check_three_cameras_for_export(self, *, video):
        """Left/right wrist and side images retain separate files, clocks and roles."""
        from embodied.data.export.exporters import Schema2
        cameras = {'wrist_rgb': {'stream_id': 'wrist', 'width': 64, 'height': 48, 'fps': 30},
                   'exterior_rgb': {'stream_id': 'exterior', 'width': 64, 'height': 48, 'fps': 30},
                   'right_wrist_rgb': {'stream_id': 'right', 'width': 64, 'height': 48, 'fps': 30}}
        writer = RuntimeEpisodeWriter(self.temp.name, {'runtime_id': 'test'}, 'three cameras', cameras=cameras, video=video,
            camera_roles={'left_wrist_0_rgb': 'wrist_rgb', 'right_wrist_0_rgb': 'right_wrist_rgb', 'base_0_rgb': 'exterior_rgb'})
        writer.manifest['kind'] = 'quest_runtime_episode'
        ticks = [tick(i, self.stamp + i * 4_000_000) for i in range(40)]
        for row in ticks:
            writer.add_tick(row)
        for n in range(1, 5):
            stamp = self.stamp + n * 33_000_000
            image = np.full((48, 64, 3), 40 * n, np.uint8)
            writer.add_frame({'image': image, 'host_received_monotonic_ns': stamp, 'frame_number': n, 'stream_id': 'wrist'})
            writer.add_frame({'image': image, 'host_received_monotonic_ns': stamp - 5_000_000, 'frame_number': n,
                              'stream_id': 'exterior'}, 'exterior_rgb')
            writer.add_frame({'image': 255 - image, 'host_received_monotonic_ns': stamp - 3_000_000,
                              'frame_number': n, 'stream_id': 'right'}, 'right_wrist_rgb')
        writer.add_tick(tick(40, self.stamp + 160_000_000, terminal=True))
        manifest = writer.close('runtime-terminal')
        self.assertEqual(manifest['lifecycle'], 'complete', manifest['error'])
        self.assertEqual(manifest['counts']['frame'], 4)
        self.assertEqual(manifest['counts']['frame:exterior_rgb'], 4)
        self.assertEqual(manifest['counts']['frame:right_wrist_rgb'], 4)
        self.assertEqual(manifest['camera_roles']['right_wrist_0_rgb'], 'right_wrist_rgb')
        for camera_id in cameras:
            artifact = f'{camera_id}.mp4' if video else f'{camera_id}/000002.png'
            self.assertTrue((writer.path / artifact).is_file())
        meta, stored = load_episode(writer.path)
        for camera_id in ('exterior_rgb', 'right_wrist_rgb'):
            frame = stored['frame:' + camera_id][2]
            self.assertEqual(frame['video_frame_index'] if video else frame['image'],
                             2 if video else f'{camera_id}/000002.png')
        meta['outcome'] = 'success'
        (writer.path / 'episode.json').write_text(json.dumps(meta))
        meta['runtime']['parameters'] = {'model': {'manifest': 'm'}}
        meta['runtime']['backend'] = 'sim'
        (writer.path / 'episode.json').write_text(json.dumps(meta))
        _, rows, report, _ = Schema2().prepare(writer.path)
        self.assertGreaterEqual(len(rows), 2)
        for row in rows:
            frames = row[3]['camera_frames']
            self.assertEqual(frames['wrist_rgb']['host_received_monotonic_ns'] - frames['exterior_rgb']['host_received_monotonic_ns'],
                             5_000_000)
            self.assertEqual(frames['wrist_rgb']['host_received_monotonic_ns'] - frames['right_wrist_rgb']['host_received_monotonic_ns'], 3_000_000)
        self.assertEqual(set(report['cameras']), {'wrist_rgb', 'right_wrist_rgb', 'exterior_rgb'})
        self.assertEqual(report['cameras']['exterior_rgb']['repeated_images'], 0)

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
