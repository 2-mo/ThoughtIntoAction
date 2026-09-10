"""Offline raw H264, exact clocks and 50Hz state / 30FPS observation alignment."""
import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from quest_teleop.export_lerobot import prepare_episode
from quest_teleop.recording import EpisodeWriter
from quest_teleop.video import read_frame


class TrainingExportTests(unittest.TestCase):
    def test_raw_video_keeps_all_frame_timestamps_and_task_name(self):
        with tempfile.TemporaryDirectory() as folder:
            camera = {'width': 64, 'height': 48, 'fps': 30}
            writer = EpisodeWriter(folder, {'camera': camera}, '拿/杯子', rgb_video=True)
            timestamps = []
            for i in range(6):
                stamp = time.monotonic_ns()
                timestamps.append(stamp)
                writer.add('frame', {'image': np.full((48, 64, 3), i*30, dtype=np.uint8),
                    'host_received_monotonic_ns': stamp, 'device_timestamp_ms': 123.125+i*1000/30,
                    'device_timestamp_domain': 'hardware_clock', 'frame_number': i})
                writer.add('state', {'host_monotonic_ns': stamp, 'requested_monotonic_ns': stamp-123})
            result = writer.stop(timeout=35)
            self.assertEqual(result['lifecycle'], 'complete')
            self.assertTrue(writer.id.startswith('拿_杯子_'))
            frames = [json.loads(s) for s in (writer.path/'frames.jsonl').read_text().splitlines()]
            self.assertEqual([f['host_received_monotonic_ns'] for f in frames], timestamps)
            self.assertEqual([f['video_frame_index'] for f in frames], list(range(6)))
            self.assertEqual(frames[0]['device_timestamp_ms'], 123.125)
            image = read_frame(writer.path, frames[-1])
            self.assertEqual(image.shape, (48,64,3))
            self.assertLess(abs(float(image.mean())-150), 5)
            self.assertEqual(len(list(writer.path.glob('wrist_rgb/*.png'))), 0)

    def test_50hz_grid_keeps_commands_and_reuses_only_preceding_30fps_images(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            states, frames = [], []
            origin = 1234567890000000
            for i in range(11):
                states.append({'host_monotonic_ns': origin+i*20_000_000,
                    'requested_monotonic_ns': origin+i*20_000_000-1,
                    'actual_pos': [i/100]*6, 'gripper_actual_pos': .04,
                    'requested_pos': [i/100+.1]*6, 'requested_gripper_pos': .02})
            for i in range(7):
                image = f'{i}.png'; (root/image).touch()
                frames.append({'host_received_monotonic_ns': origin+round(i*1e9/30), 'image': image})
            meta = {'kind':'quest_teleop_episode', 'schema_version':1, 'lifecycle':'complete',
                    'outcome':'unknown', 'counts':{'state':11,'frame':7}, 'id':'sample','task':'pick',
                    'camera':{'width':64,'height':48,'fps':30}}
            (root/'episode.json').write_text(json.dumps(meta))
            for name, rows in [('states',states),('frames',frames)]:
                (root/(name+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
            _, rows, report = prepare_episode(root)
            self.assertEqual(len(rows), 11)
            self.assertGreater(report['repeated_images'], 0)
            self.assertLessEqual(report['image_max_age_ms'], 34)
            np.testing.assert_allclose(rows[5][0][:6], .05)
            np.testing.assert_allclose(rows[5][1][:6], .15)
            _, every_row, _ = prepare_episode(root, fps=240, preserve_samples=True)
            self.assertEqual(len(every_row), len(states))
            self.assertEqual([row[4]['host_monotonic_ns'] for row in every_row], [row['host_monotonic_ns'] for row in states])
            self.assertEqual(rows[1][3]['image'], '0.png')  # 20ms must not use the 33ms image.
            states[5]['requested_pos'] = None
            (root/'states.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in states))
            with self.assertRaises((TypeError, ValueError)):
                prepare_episode(root)

class X5PolicyTests(unittest.TestCase):
    def test_wrist_only_mask_and_seven_dimensional_outputs(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'integrations'/'openpi'))
        from x5_policy import X5Inputs, X5Outputs
        data = X5Inputs()({'wrist_image': np.ones((3,48,64), dtype=np.float32),
                           'state': np.zeros(7), 'actions': np.ones((25,7)), 'prompt': 'pick cup'})
        self.assertEqual(data['image']['left_wrist_0_rgb'].shape, (48,64,3))
        self.assertEqual(data['image']['left_wrist_0_rgb'].dtype, np.uint8)
        self.assertTrue(data['image_mask']['left_wrist_0_rgb'])
        self.assertFalse(data['image_mask']['base_0_rgb'])
        self.assertFalse(data['image_mask']['right_wrist_0_rgb'])
        self.assertEqual(X5Outputs()({'actions': np.zeros((25,32))})['actions'].shape, (25,7))
