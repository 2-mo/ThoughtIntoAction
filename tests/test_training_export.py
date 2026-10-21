"""Offline raw H264, exact clocks and 50Hz state / 30FPS observation alignment."""
from embodied.data.episode.mcap_io import read_records
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from embodied.data.episode.recording import EpisodeWriter
from embodied.data.episode.video import read_frame


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
            _, frames = read_records(writer.path/'episode.mcap')
            self.assertEqual([f['host_received_monotonic_ns'] for f in frames], timestamps)
            self.assertEqual([f['video_frame_index'] for f in frames], list(range(6)))
            self.assertEqual(frames[0]['device_timestamp_ms'], 123.125)
            image = read_frame(writer.path, frames[-1])
            self.assertEqual(image.shape, (48,64,3))
            self.assertLess(abs(float(image.mean())-150), 5)
            self.assertEqual(len(list(writer.path.glob('wrist_rgb/*.png'))), 0)


class X5PolicyTests(unittest.TestCase):
    def test_exterior_image_is_used_in_base_camera_slot(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'integrations'/'openpi'))
        from x5_policy import X5Inputs
        data = X5Inputs()({'wrist_image':np.zeros((48,64,3),np.uint8), 'state':np.zeros(7),
                          'exterior_image':np.ones((3,48,64),np.float32), 'exterior_valid':np.array([True])})
        self.assertTrue(data['image_mask']['base_0_rgb'])
        self.assertEqual(int(data['image']['base_0_rgb'].min()),255)
        self.assertFalse(data['image_mask']['right_wrist_0_rgb'])

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
