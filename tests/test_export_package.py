"""Run the portable archive against the real, pinned CPU LeRobot environment."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import numpy as np
from mcap_fixture import write_records

from embodied.data.task_package import finalize_task
from embodied.data.export.decode import BatchFrameReader
from embodied.data.episode.video import RGBVideoWriter, SequentialFrameReader, read_frame


def make_episode(root, name, count=120, png=False):
    import cv2
    path = root / name
    path.mkdir(parents=True)
    origin = 4552089789885210
    # The first camera frame precedes the first robot state and must be excluded.
    states = [dict(host_monotonic_ns=origin+1+round(i*1e9/240),
        requested_monotonic_ns=origin+round(i*1e9/240)-10,
        actual_pos=[i*.0001]*6, gripper_actual_pos=.04,
        requested_pos=[i*.0002]*6, requested_gripper_pos=.03) for i in range(count*8)]
    frames = []
    writer = None if png else RGBVideoWriter(path/'wrist_rgb.mp4', 64, 48, 30)
    try:
        for i in range(count):
            image = np.full((48, 64, 3), (i % 200, 40, 180), dtype=np.uint8)
            frame = dict(host_received_monotonic_ns=origin+round(i*1e9/30))
            if png:
                frame['image'] = f'{i}.png'
                assert cv2.imwrite(str(path/frame['image']), image)
            else:
                writer.write(image)
                frame.update(video='wrist_rgb.mp4', video_frame_index=i)
            frames.append(frame)
    finally:
        if writer:
            writer.close()
    meta = dict(kind='quest_teleop_episode', schema_version=1, lifecycle='complete',
        outcome='success', task_run_id='test-run', counts=dict(state=len(states), frame=count),
        id=name, task='pick cup', robot={'model': 'x5'}, camera=dict(width=64, height=48, fps=30))
    (path/'episode.json').write_text(json.dumps(meta))
    write_records(path, states=states, frames=frames)
    return path, states, frames


class SequentialReaderTests(unittest.TestCase):
    def test_sequential_decode_matches_seek_and_opens_video_once(self):
        import cv2
        with tempfile.TemporaryDirectory() as directory:
            path, _, frames = make_episode(Path(directory), 'video', count=8)
            expected = [read_frame(path, frames[i]) for i in (0, 2, 2, 7)]
            with patch('cv2.VideoCapture', wraps=cv2.VideoCapture) as opened:
                with SequentialFrameReader() as reader:
                    for i, pixels in zip((0, 2, 2, 7), expected):
                        np.testing.assert_array_equal(reader.read(path, frames[i]), pixels)
                    with self.assertRaisesRegex(ValueError, 'ordered'):
                        reader.read(path, frames[1])
                self.assertEqual(opened.call_count, 1)
                self.assertIsNone(reader.capture)


@unittest.skipUnless(importlib.util.find_spec('lerobot'), 'requires scripts/datasets/setup_export.sh environment')
class PortableExportTests(unittest.TestCase):
    def test_dual_camera_portable_export_keeps_pixels_and_independent_clocks(self):
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path, states, wrist = make_episode(root/'raw', 'dual', count=8)
            exterior = []
            writer = RGBVideoWriter(path/'exterior_rgb.mp4', 64, 48, 30)
            try:
                for index, frame in enumerate(wrist):
                    frame.update(camera_id='wrist_rgb', frame_number=index, device_timestamp_ms=index*1000/30,
                                 device_timestamp_domain='hardware_clock', stream_id='wrist')
                    writer.write(np.full((48,64,3), (180,80,20), np.uint8))
                    exterior.append({**frame, 'camera_id':'exterior_rgb', 'video':'exterior_rgb.mp4',
                        'stream_id':'exterior', 'host_received_monotonic_ns':frame['host_received_monotonic_ns']-5_000_000})
            finally:
                writer.close()
            meta = json.loads((path/'episode.json').read_text())
            meta['camera']['stream_id'] = 'wrist'
            meta.update(cameras={'wrist_rgb':meta['camera'], 'exterior_rgb':{**meta['camera'], 'stream_id':'exterior'}},
                        camera_counts={'wrist_rgb':8, 'exterior_rgb':8})
            (path/'episode.json').write_text(json.dumps(meta))
            (path/'quality.json').write_text(json.dumps({'status':'passed', 'episode':'dual'}))
            write_records(path, states=states, frames=[*wrist, *exterior])
            archive = finalize_task(root/'raw', ['dual'], root/'dual task', 240)
            with zipfile.ZipFile(archive) as package:
                self.assertIn('dual task/episodes/dual/quality.json', package.namelist())
                self.assertEqual(package.read('dual task/episodes/dual/exterior_rgb.mp4'), (path/'exterior_rgb.mp4').read_bytes())
                package.extractall(root/'unpacked')
            bundle = root/'unpacked/dual task'
            done = subprocess.run(['bash', str(bundle/'转换为LeRobot.sh'), '--decoder','opencv','--decode-workers','0'],
                cwd=root, env={**os.environ, 'LEROBOT_EXPORT_PYTHON':sys.executable,'PYTHONPATH':''},
                capture_output=True,text=True,timeout=90)
            self.assertEqual(done.returncode,0,done.stdout+done.stderr)
            output = bundle/'lerobot_dataset'
            loaded = LeRobotDataset('local/quest_x5',root=output,video_backend='pyav')
            self.assertEqual(len(loaded),7)
            sample = loaded[0]
            self.assertEqual(sample['source.rgb_monotonic_ns'].item()-sample['source.exterior_rgb_monotonic_ns'].item(),5_000_000)
            self.assertTrue(sample['observation.exterior_valid'].item())
            self.assertEqual(tuple(sample['observation.images.exterior'].shape),(3,48,64))
            self.assertGreater(sample['observation.images.wrist'][0].mean().item(),.6)
            self.assertLess(sample['observation.images.exterior'][0].mean().item(),.15)
            manifest = json.loads((output/'meta/quest_export.json').read_text())
            self.assertEqual(set(manifest['cameras']),{'wrist_rgb','exterior_rgb'})
            self.assertEqual(manifest['episodes'][0]['cameras']['exterior_rgb']['max_age_ms'],5.)
            with np.load(output/'robot/episode_000000.npz') as raw:
                self.assertEqual(len(raw['state']),len(states))

    def test_runtime_schema2_portable_export_preserves_independent_clocks(self):
        from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
        from test_runtime_collection import tick
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        from embodied.data.export.lerobot import export
        import time
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            descriptor = {'runtime_id':'test', 'backend':'sim', 'parameters':{'model':{'manifest':{'model':'fixture'}}}}
            camera = {'stream_id':'rgb', 'fps':30, 'height':48, 'width':64}
            writer = RuntimeEpisodeWriter(root/'raw',descriptor,'independent clocks',camera)
            origin = time.monotonic_ns()
            for i in range(80):
                row = tick(i,origin+i*4_000_000)
                row.update(version=2, source='phone', owner_epoch=1, source_sequence=i, operation_id='')
                writer.add_tick(row)
                if i % 8 == 0:
                    writer.add_frame({'image':np.zeros((48,64,3),np.uint8), 'stream_id':'rgb',
                        'host_received_monotonic_ns':origin+i*4_000_000+3_000_000, 'frame_number':i//8})
            writer.close('duration')
            archive = finalize_task(root/'raw',[writer.path.name],root/'runtime task',250)
            with zipfile.ZipFile(archive) as package:
                package.extractall(root/'unpacked')
            bundle = root/'unpacked/runtime task'
            env = {**os.environ,'LEROBOT_EXPORT_PYTHON':sys.executable,'PYTHONPATH':''}
            done = subprocess.run(['bash',str(bundle/'转换为LeRobot.sh'),'--decoder','opencv','--decode-workers','0'],
                cwd=root, env=env, capture_output=True, text=True, timeout=90)
            self.assertEqual(done.returncode,0,done.stdout+done.stderr)
            output = bundle/'lerobot_dataset'
            manifest = json.loads((output/'meta/quest_export.json').read_text())
            self.assertEqual(manifest['layout'],'aligned30_raw250_v2')
            self.assertEqual(manifest['control_stacks'], ['arx-televr kernel: sent_q at 250 Hz'])
            self.assertEqual(manifest['anomalous_episodes'], [])
            self.assertEqual(manifest['episodes'][0]['gaps'], [])
            loaded = LeRobotDataset('local/quest_x5',root=output,video_backend='pyav')
            self.assertEqual(len(loaded),10)
            self.assertGreater(loaded[0]['source.action_monotonic_ns'].item(), loaded[0]['source.state_monotonic_ns'].item())
            with np.load(output/'robot/episode_000000.npz') as raw:
                np.testing.assert_array_equal(raw['command_timestamp_ns']-raw['feedback_timestamp_ns'],2_000_000)
                self.assertEqual(len(raw['raw_ticks_json']),80)
            make_episode(root/'raw','legacy',count=4,png=True)
            with self.assertRaisesRegex(ValueError,'Mixed episode schemas'):
                export(root/'raw',root/'mixed','local/test',progress=False,decoder='opencv',decode_workers=0)

    def test_batched_decoders_preserve_exact_pixels_and_order_across_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _, frames = make_episode(Path(directory), 'video', count=41)
            selected = frames[1::2]
            with BatchFrameReader('opencv', 0, 7) as reader:
                expected = np.stack(list(reader.frames(path, selected)))
            for backend, workers in [('torchcodec', 0), ('torchcodec', 2), ('opencv', 2)]:
                with self.subTest(backend=backend, workers=workers):
                    with BatchFrameReader(backend, workers, 7) as reader:
                        actual = np.stack(list(reader.frames(path, selected)))
                    np.testing.assert_array_equal(actual, expected)

    def test_extracted_archive_runs_outside_checkout_and_loads_both_media_formats(self):
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        with tempfile.TemporaryDirectory(prefix='portable export 中文 ') as directory:
            root = Path(directory)
            raw = root/'raw'
            episodes = [make_episode(raw, 'a_video'), make_episode(raw, 'b_png', count=7, png=True)]
            rejected = raw/'c_discarded'
            rejected.mkdir()
            (rejected/'episode.json').write_text(json.dumps(dict(lifecycle='complete', outcome='discarded',
                task='pick cup', task_run_id='test-run', kind='quest_teleop_episode', schema_version=1)))
            archive = finalize_task(raw, ['a_video', 'b_png', 'c_discarded'], root/'task archive', 240)
            with zipfile.ZipFile(archive) as package:
                self.assertEqual((package.getinfo('task archive/转换为LeRobot.sh').external_attr >> 16) & 0o777, 0o755)
                package.extractall(root/'extracted')
            bundle = root/'extracted'/'task archive'
            env = {**os.environ, 'LEROBOT_EXPORT_PYTHON': sys.executable, 'PYTHONPATH': ''}
            command = ['bash', str(bundle/'转换为LeRobot.sh')]
            failed = subprocess.run(command, cwd=root, env=env, capture_output=True, text=True, timeout=90)
            self.assertNotEqual(failed.returncode, 0)
            self.assertFalse((bundle/'lerobot_dataset').exists())
            result = subprocess.run([*command, '--skip-invalid'], cwd=root, env=env,
                capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            for stage in ('检查数据', '转换第', '写入表格与统计', '验证数据集'):
                self.assertIn(stage, result.stderr)
            output = bundle/'lerobot_dataset'
            manifest = json.loads((output/'meta/quest_export.json').read_text())
            self.assertEqual(len(manifest['rejected']), 1)
            self.assertGreater(manifest['conversion']['frames_per_second'], 0)
            self.assertFalse((output/'images').exists())
            self.assertEqual(len(list(output.rglob('*.mp4'))), 2)
            loaded = LeRobotDataset('local/quest_x5', root=output, video_backend='pyav',
                delta_timestamps={'action': [i/30 for i in range(15)]})
            self.assertEqual(len(loaded), 119+6)
            offset = 0
            for episode_index, (path, states, frames) in enumerate(episodes):
                report = manifest['episodes'][episode_index]
                self.assertEqual(report['excluded_camera_anchors'], 1)
                with np.load(output/'robot'/f'episode_{episode_index:06d}.npz') as sidecar:
                    np.testing.assert_array_equal(sidecar['timestamp_ns'], [s['host_monotonic_ns'] for s in states])
                    np.testing.assert_array_equal(sidecar['action_timestamp_ns'], [s['requested_monotonic_ns'] for s in states])
                    np.testing.assert_array_equal(sidecar['action'], [[*s['requested_pos'], s['requested_gripper_pos']] for s in states])
                for local_index in (0, (len(frames)-2)//2, len(frames)-2):
                    sample = loaded[offset+local_index]
                    frame = frames[local_index+1]
                    index = np.searchsorted([s['host_monotonic_ns'] for s in states], frame['host_received_monotonic_ns'], side='right')-1
                    self.assertEqual(sample['source.rgb_monotonic_ns'].item(), frame['host_received_monotonic_ns'])
                    self.assertEqual(sample['source.state_monotonic_ns'].item(), states[index]['host_monotonic_ns'])
                    np.testing.assert_allclose(sample['observation.state'][:6], states[index]['actual_pos'])
                    np.testing.assert_allclose(sample['action'][0, :6], states[index]['requested_pos'])
                    expected_rgb = read_frame(path, frame)[..., ::-1].copy().transpose(2, 0, 1)/255
                    np.testing.assert_allclose(sample['observation.images.wrist'], expected_rgb, atol=5/255)
                    self.assertEqual(tuple(sample['action'].shape), (15, 7))
                offset += len(frames)-1
            rerun = subprocess.run([*command, '--skip-invalid'], cwd=root, env=env,
                capture_output=True, text=True, timeout=90)
            self.assertNotEqual(rerun.returncode, 0)
            self.assertIn('Output already exists', rerun.stderr)

    def test_decoder_failure_removes_partial_output(self):
        from embodied.data.export.lerobot import export
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path, _, _ = make_episode(root, 'video', count=4)
            with patch.object(SequentialFrameReader, 'read', side_effect=ValueError('broken RGB')):
                with self.assertRaisesRegex(ValueError, 'broken RGB'):
                    export(path, root/'dataset', 'local/test', progress=False, decoder='opencv', decode_workers=0)
            self.assertFalse((root/'dataset').exists())
            self.assertEqual(list(root.glob('dataset.partial-*')), [])


if __name__ == '__main__':
    unittest.main()
