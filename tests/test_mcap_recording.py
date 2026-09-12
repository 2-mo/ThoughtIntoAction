import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from mcap.reader import make_reader
from quest_teleop.recording import EpisodeWriter
from quest_teleop.datasets import load_episode
from quest_teleop.replay import RecordedEpisode, WebReplay
from quest_teleop.export_lerobot import load_episode as load_training
from robot_core.model import RobotModel
from unittest.mock import patch

class McapRecordingTests(unittest.TestCase):
    def test_roundtrip_video_replay_training_and_clock(self):
        with tempfile.TemporaryDirectory() as folder:
            writer = EpisodeWriter(folder, {'robot': RobotModel(eef='eef_link').metadata(),
                'camera': {'width':64, 'height':48, 'fps':30}}, 'MCAP 测试', rgb_video=True, record_format='mcap')
            start = writer.manifest['started_monotonic_ns']
            for i in range(3):
                stamp = start + i * 33_333_333
                row = {'host_monotonic_ns': stamp, 'requested_monotonic_ns': stamp, 'actual_pos': [i*.01]*6, 'actual_vel': [.1]*6,
                       'actual_torque': [.2]*6, 'requested_pos': [.1]*6, 'requested_gripper_pos': .03,
                       'gripper_actual_pos': .02, 'gripper_actual_vel': .001, 'source': 'quest'}
                writer.add('state', row)
                writer.add('frame', {'host_received_monotonic_ns': stamp, 'image': np.full((48,64,3), i*80, np.uint8)})
            result = writer.stop()
            self.assertEqual(result['lifecycle'], 'complete', result)
            self.assertFalse((writer.path/'states.jsonl').exists())
            root, meta, states, frames = load_episode(writer.path)
            self.assertEqual(len(states), 3)
            self.assertEqual(len(frames), 3)
            self.assertEqual(states[-1], row)
            self.assertEqual(frames[-1]['video_frame_index'], 2)
            with (root/'episode.mcap').open('rb') as stream:
                messages = list(make_reader(stream, validate_crcs=True).iter_messages())
            self.assertEqual(len(messages), 6)
            self.assertEqual(messages[0][2].log_time, meta['started_unix_ns'])
            _, training_states, training_frames, _ = load_training(root)
            self.assertEqual(training_states, states)
            self.assertEqual(training_frames, frames)
            episode = RecordedEpisode(root)
            review = episode.review(.04)
            self.assertEqual(review['sample_index'], 1)
            self.assertEqual(review['actual_vel'], [.1]*6)
            self.assertTrue(review['image_base64'])
            self.assertIsNone(review['target_q'])
            with patch('quest_teleop.replay.time.monotonic', return_value=10) as clock:
                replay = WebReplay(episode)
                replay.control('speed', speed=.25)
                clock.return_value = 10.1
                self.assertAlmostEqual(replay.seconds(), .025)
                replay.control('pause')
                clock.return_value = 20
                self.assertAlmostEqual(replay.seconds(), .025)
                replay.control('seek', .04)
                self.assertEqual(replay.frame()['torque_history'], review['torque_history'])
            content = (root/'episode.mcap').read_bytes()
            (root/'episode.mcap').write_bytes(content[:60])
            with self.assertRaises(ValueError): load_episode(root)

if __name__ == '__main__': unittest.main()
