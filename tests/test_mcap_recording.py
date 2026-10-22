import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from mcap.reader import make_reader
from embodied.data.episode.recording import EpisodeWriter
from embodied.data.datasets import load_episode
from embodied.data.replay import RecordedEpisode, WebReplay
from embodied.data.export.lerobot import load_episode as load_training
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from unittest.mock import patch, MagicMock
from types import SimpleNamespace

class McapRecordingTests(unittest.TestCase):
    def test_roundtrip_video_replay_training_and_clock(self):
        from legacy.robot_control.hardware import OfficialX5HardwareAdapter
        backend = OfficialX5HardwareAdapter.__new__(OfficialX5HardwareAdapter)
        backend.arm = MagicMock()
        self.addCleanup(setattr, backend, 'arm', None)
        backend.robot_config = SimpleNamespace(joint_dof=6, joint_pos_min=[-3]*6, joint_pos_max=[3]*6)
        backend.actuate = False
        backend._motor_feedback = None
        backend._gripper_mode = 'position'
        backend._requested_joints = backend._requested_monotonic_ns = None
        backend._requested_gripper_pos = backend._last_submission = None
        feedback = SimpleNamespace(pos=lambda: np.zeros(6), vel=lambda: np.zeros(6),
            torque=lambda: np.zeros(6), gripper_pos=.02, gripper_vel=.001, gripper_torque=.1, timestamp=1.)
        output = SimpleNamespace(pos=lambda: np.ones(6), vel=lambda: np.full(6, .2),
            torque=lambda: np.full(6, .3), gripper_pos=.03, gripper_vel=.002, gripper_torque=.4, timestamp=1.01)
        gains = SimpleNamespace(kp=lambda: np.full(6, 10.), kd=lambda: np.full(6, .5), gripper_kp=4., gripper_kd=.2)
        backend.arm.get_joint_state.return_value = feedback
        backend.arm.get_joint_cmd.return_value = output
        backend.arm.get_gain.return_value = gains
        backend.arm.get_timestamp.return_value = 1.02
        captured = backend.get_recording_snapshot()
        gripper_keys = ('gripper_sdk_cmd_pos', 'gripper_sdk_cmd_vel',
                        'gripper_sdk_cmd_torque', 'gripper_kp', 'gripper_kd')
        with tempfile.TemporaryDirectory() as folder:
            writer = EpisodeWriter(folder, {'robot': X5PinocchioModel(eef='eef_link').metadata(),
                'camera': {'width':64, 'height':48, 'fps':30}}, 'MCAP 测试', rgb_video=True)
            start = writer.manifest['started_monotonic_ns']
            for i in range(3):
                stamp = start + i * 33_333_333
                row = {**{key: captured[key] for key in gripper_keys}, 'host_monotonic_ns': stamp, 'requested_monotonic_ns': stamp, 'actual_pos': [i*.01]*6, 'actual_vel': [.1]*6,
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
            self.assertEqual([row[key] for key in gripper_keys], [.03, .002, .4, 4., .2])
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
            self.assertEqual(review['command_details']['gripper_sdk_cmd_pos'], .03)
            self.assertTrue(review['image_base64'])
            self.assertIsNone(review['target_q'])
            with patch('embodied.data.replay.time.monotonic', return_value=10) as clock:
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
