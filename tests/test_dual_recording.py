"""Dual capture, persisted quality, causal review/export; fake devices only."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from embodied.data.episode.mcap_io import read_camera_records
from embodied.data.episode.recording import EpisodeWriter
from embodied.data.datasets import validate_episode
from embodied.data.export.lerobot import prepare_camera_episode
from embodied.data.replay import RecordedEpisode
from legacy.quest_teleop.runtime.session import TeleopSession
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.core.project import load_config
from test_recording import FakeCamera, snapshot


def make_dual(root, *, broken=None, count=12):
    cameras = {key: {'width': 64, 'height': 48, 'fps': 30, 'stream_id': key}
               for key in ('wrist_rgb', 'exterior_rgb')}
    limits = dict(min_duration_s=.1, max_state_gap_s=.1, max_frame_gap_s=.15, max_sync_skew_s=.06)
    writer = EpisodeWriter(root, {'robot': X5PinocchioModel().metadata(), 'camera': cameras['wrist_rgb'],
        'cameras': cameras, 'hand_eye_calibrated': False, 'task_run_id': 'dual-fixture',
        'quality_limits': limits, 'control': {'actuate': True, 'gripper_width_m': .082,
        'control_rate_hz': 50, 'recording_rate_hz': 240}}, 'dual capture', rgb_video=True, queue_size=1024)
    origin = writer.manifest['started_monotonic_ns']
    for index in range(count * 8 + 4):
        stamp = origin + round(index * 1e9 / 240)
        row = snapshot(index)
        row.update(host_monotonic_ns=stamp, requested_monotonic_ns=stamp,
                   feedback_timestamp=stamp/1e9, command_timestamp=stamp/1e9, controller_timestamp=stamp/1e9)
        row['submission']['host_monotonic_ns'] = stamp
        writer.add('state', row)
    for key in cameras:
        for index in range(count):
            if key == 'exterior_rgb' and broken == 'missing' and index == 5:
                continue
            stamp = origin + round(index * 1e9 / 30) + (10_000_000 if key == 'wrist_rgb' else 5_000_000)
            writer.add('frame', {'camera_id': key, 'frame_number': index,
                'host_received_monotonic_ns': stamp, 'device_timestamp_ms': 1000 + index * 1000/30,
                'device_timestamp_domain': 'hardware_clock',
                'stream_id': 'restarted' if key == 'exterior_rgb' and broken == 'restart' and index > 4 else key,
                'image': np.full((48, 64, 3), (10, 40, 180) if key == 'wrist_rgb' else (180, 80, 20), np.uint8)})
    with patch('embodied.data.episode.recording.time.monotonic_ns', return_value=origin + round(count * 1e9 / 30) + 20_000_000):
        result = writer.stop()
    assert result['lifecycle'] == 'complete', result
    return writer.path, limits


class DualRecordingTests(unittest.TestCase):
    def test_roundtrip_quality_review_and_causal_training_rows(self):
        with tempfile.TemporaryDirectory() as folder:
            path, settings = make_dual(folder)
            meta = json.loads((path / 'episode.json').read_text())
            states, frames = read_camera_records(path / 'episode.mcap')
            self.assertEqual(meta['camera_counts'], {'wrist_rgb': 12, 'exterior_rgb': 12})
            self.assertEqual(meta['counts']['frame'], 12)
            self.assertTrue((path / 'exterior_rgb.mp4').is_file())
            for key, rows in frames.items():
                self.assertEqual([f['video_frame_index'] for f in rows], list(range(12)))
                self.assertTrue(all(f['video'] == key + '.mp4' for f in rows))
            report = validate_episode(path, settings=settings)
            self.assertTrue(report['valid'], report)
            self.assertFalse(report['training_eligible'])  # Unknown is not task success.
            self.assertEqual(set(report['cameras']), set(frames))
            self.assertLess(report['camera_pair_alignment']['exterior_rgb']['max_s'], .006)
            review = RecordedEpisode(path)
            self.assertIsNone(review.review(0)['image_base64'])  # Never show a future image.
            images = review.review(.02)['cameras']
            self.assertTrue(all(image['image_base64'] for image in images.values()))
            self.assertNotEqual(images['wrist_rgb']['image_base64'], images['exterior_rgb']['image_base64'])
            self.assertEqual(images['exterior_rgb']['host_received_monotonic_ns'], frames['exterior_rgb'][0]['host_received_monotonic_ns'])
            self.assertIsNone(review.review(1)['cameras']['exterior_rgb']['image_base64'])
            _, rows, exported, raw = prepare_camera_episode(path)
            self.assertEqual(len(raw), len(states))
            self.assertEqual(exported['cameras']['exterior_rgb']['repeated_images'], 0)
            # Export metadata names the control stack and records anchor gaps (todo D-01, D-02).
            self.assertEqual(exported['backend'], 'real')
            self.assertIn(exported['control_stack'].split(':')[0], ('legacy.robot_control.service', 'unknown'))
            self.assertEqual(exported['gaps'], [])
            self.assertFalse(exported['anomaly'])
            for row in rows:
                cameras = row[3]['camera_frames']
                self.assertLessEqual(cameras['exterior_rgb']['host_received_monotonic_ns'], row[3]['host_received_monotonic_ns'])
                self.assertEqual(row[3]['host_received_monotonic_ns'] - cameras['exterior_rgb']['host_received_monotonic_ns'], 5_000_000)

    def test_external_missing_frames_or_restart_fail_quality_and_export(self):
        with tempfile.TemporaryDirectory() as folder:
            for failure in ('missing', 'restart'):
                path, settings = make_dual(folder, broken=failure)
                report = validate_episode(path, settings=settings)
                self.assertFalse(report['valid'])
                self.assertTrue(report['cameras']['wrist_rgb']['valid'])
                self.assertFalse(report['cameras']['exterior_rgb']['valid'])
                with self.assertRaisesRegex(ValueError, 'exterior_rgb'):
                    prepare_camera_episode(path)

    def test_session_quality_is_background_and_external_disconnect_stops_recording(self):
        with tempfile.TemporaryDirectory() as folder:
            config = copy.deepcopy(load_config())
            root = Path(folder)
            config['runtime']['socket'] = str(root / 'session.sock')
            config['paths'].update(dataset_dir=str(root / 'data'), log_dir=str(root / 'logs'))
            config['recording'].update(camera_ids=['wrist_rgb', 'exterior_rgb'], min_duration_s=.1,
                                       min_free_bytes=0, package_on_task_end=False)
            cameras = {key: FakeCamera(key, 64, 48, 30) for key in config['recording']['camera_ids']}
            for key in cameras:
                config['cameras'][key] = dict(serial=key, width=64, height=48, fps=30, enabled=True, hand_eye_calibrated=False)
            session = TeleopSession(config, X5PinocchioModel().metadata(),
                {'actuate': True, 'gripper_width_m': .082, 'control_rate_hz': 50},
                camera_factory=lambda serial, *args, **kwargs: cameras[serial]).start()
            stop = threading.Event()
            def publish():
                sequence = 0
                while not stop.wait(.02):
                    sequence += 1
                    session.publish(snapshot(sequence))
            publisher = threading.Thread(target=publish)
            publisher.start()
            release = threading.Event()
            check = session._check_quality
            def delayed(episode):
                release.wait(5)
                return check(episode)
            try:
                deadline = time.monotonic() + 3
                while not all(s['frame'] for s in session.camera_states.values()):
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.02)
                with patch.object(session, '_check_quality', side_effect=delayed):
                    first = session.dispatch({'command': 'record.start', 'task': 'dual'})
                    time.sleep(.35)
                    session.dispatch({'command': 'record.stop'})
                    self.assertEqual(session.status()['last_episode']['quality']['status'], 'checking')
                    session.dispatch({'command': 'record.start', 'task': 'next'})
                    self.assertIsNotNone(session.recording)
                    cameras['exterior_rgb'].fail = True
                    deadline = time.monotonic() + 3
                    while session.recording is not None:
                        self.assertLess(time.monotonic(), deadline)
                        time.sleep(.02)
                    self.assertEqual(session.last_episode['lifecycle'], 'interrupted')
                    self.assertTrue(session.camera_states['wrist_rgb']['frame'])
                    release.set()
                    for job in session.quality_jobs:
                        job.result(timeout=10)
                    saved = json.loads((Path(first['path']) / 'quality.json').read_text())
                    self.assertEqual(saved['status'], 'passed', saved)
                    self.assertEqual(session.status()['last_episode']['quality']['status'], 'failed')
                    self.assertNotEqual(first['id'], session.status()['last_episode']['quality']['episode'])
            finally:
                release.set()
                stop.set()
                publisher.join()
                session.close()


if __name__ == '__main__':
    unittest.main()
