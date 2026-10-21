"""Offline conversion of raw Quest episodes to OpenPI's pinned LeRobot v2.1."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import sys
import time
import uuid
from contextlib import ExitStack

import numpy as np
from embodied.data.episode.cameras import camera_metadata

LEROBOT_REVISION = '0cf864870cf29f4738d3ade893e6fd13fbd7cdb5'
FEATURE_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_m']


def load_episode(path, max_gap_s=.1):
    """Read and validate one episode; do not retain all episodes in memory."""
    from embodied.data.episode.schema import TELEOP, read
    schema, meta = read(path)
    if schema != TELEOP:
        raise ValueError(f'Expected a {TELEOP} episode, found {schema}')
    if meta.get('lifecycle') != 'complete' or meta.get('outcome') in ('discarded', 'failure'):
        raise ValueError('Episode is not complete or is discarded/failed')
    if meta.get('error') or any(meta.get('dropped', {}).values()):
        raise ValueError('Writer error or dropped samples')
    if meta.get('record_storage', {}).get('format') != 'mcap':
        raise ValueError('Only MCAP episodes are supported')
    from embodied.data.episode.mcap_io import read_camera_records
    states, camera_frames = read_camera_records(path / 'episode.mcap')
    images = camera_frames['wrist_rgb']
    if any(meta.get('counts', {}).get(k) != n for k, n in [('state', len(states)), ('frame', len(images))]):
        raise ValueError('Manifest counts disagree with stored records')
    clocks = [np.array([r[k] for r in rows], dtype=np.int64) for rows, k in
              [(states, 'host_monotonic_ns'), (images, 'host_received_monotonic_ns')]]
    for clock in clocks:
        if len(clock) < 2 or np.any(np.diff(clock) <= 0) or np.max(np.diff(clock)) > max_gap_s * 1e9:
            raise ValueError('Missing, unordered or gapped timestamps')
    if 'cameras' in meta:
        from embodied.data.episode.quality import inspect_camera
        cameras = camera_metadata(meta)
        if set(camera_frames) != set(cameras):
            raise ValueError('Missing or undeclared recording camera channel')
        for camera_id, camera in cameras.items():
            frames = camera_frames[camera_id]
            if meta.get('camera_counts', {}).get(camera_id) != len(frames):
                raise ValueError(camera_id + ': camera count mismatch')
            report = inspect_camera(path, camera, frames, clocks[0], max_gap_s=max_gap_s,
                max_skew_s=meta.get('quality_limits', {}).get('max_sync_skew_s', .06), check_images=False)
            if report['errors']:
                raise ValueError(camera_id + ': ' + '; '.join(report['errors']))
    for state in states:
        if "feedback_valid" in state and (len(state["feedback_valid"]) != 7 or any(v is not True for v in state["feedback_valid"])):
            raise ValueError("Invalid or faulted motor feedback in recording")
        if state.get('preset_name') or state.get('source') in ('preset', 'replay'):
            raise ValueError('Episode includes positioning/replay; collect a clean demonstration')
        values = np.asarray([[*state['actual_pos'], state['gripper_actual_pos']],
                             [*state['requested_pos'], state['requested_gripper_pos']]], dtype=np.float64)
        if values.shape != (2, 7) or not np.isfinite(values).all():
            raise ValueError('Missing/nonfinite 7D state or command')
        if np.any(values[:, -1] < 0) or np.any(values[:, -1] > .082):
            raise ValueError('Gripper outside calibrated width range')
        requested = state.get('requested_monotonic_ns')
        if type(requested) is not int or requested > state['host_monotonic_ns']:
            raise ValueError('Missing/noncausal integer action timestamp')
    from embodied.data.episode.video import media_path
    checked, video_indices = set(), {}
    for frame in images:
        reference = frame.get('video', frame.get('image', ''))
        if not reference:
            raise ValueError('Missing image path')
        if reference not in checked:
            if not media_path(path, frame).is_file():
                raise ValueError('Missing source RGB')
            checked.add(reference)
        if 'video' in frame:
            index = frame.get('video_frame_index')
            if type(index) is not int or index < 0 or index <= video_indices.get(reference, -1):
                raise ValueError('Invalid or unordered video frame index')
            video_indices[reference] = index
    return meta, states, images, clocks


def prepare_schema1_camera_episode(path):
    from embodied.data.export.multirate import camera_rows
    meta, states, frames, _ = load_episode(path)
    if meta['camera']['fps'] != 30:
        raise ValueError('Expected original camera FPS 30')
    rows = camera_rows(states, frames, path)
    exclusion_reasons = {'stale-robot-data': len(frames) - len(rows)}
    cameras = camera_metadata(meta)
    other_frames, clocks = {}, {}
    if len(cameras) > 1:
        from embodied.data.episode.mcap_io import read_camera_records
        _, all_frames = read_camera_records(path / 'episode.mcap')
        other_frames = {key: all_frames[key] for key in cameras if key != 'wrist_rgb'}
        clocks = {key: np.asarray([f['host_received_monotonic_ns'] for f in stream], np.int64)
                  for key, stream in other_frames.items()}
        selected = []
        max_age_ns = meta.get('quality_limits', {}).get('max_sync_skew_s', .06) * 1e9
        for state, action, root, wrist, source in rows:
            stamp = wrist['host_received_monotonic_ns']
            matched = {'wrist_rgb': wrist}
            for key, clock in clocks.items():
                index = int(np.searchsorted(clock, stamp, side='right') - 1)
                if index < 0 or stamp - clock[index] > max_age_ns:
                    break
                matched[key] = other_frames[key][index]
            if len(matched) == len(cameras):
                selected.append((state, action, root, {**wrist, 'camera_frames': matched}, source))
        exclusion_reasons['camera-unmatched'] = len(rows) - len(selected)
        rows = selected
    if len(rows) < 2:
        raise ValueError('Insufficient causal camera anchors')
    report = {'id': meta['id'], 'task': meta['task'], 'source': str(path),
              'outcome': meta['outcome'], 'samples': len(rows),
              'robot_samples': len(states), 'observation_samples': len(rows),
              'excluded_camera_anchors': len(frames)-len(rows),
              'exclusion_reasons': {key: value for key, value in exclusion_reasons.items() if value},
              'start_monotonic_ns': frames[0]['host_received_monotonic_ns'],
              'end_monotonic_ns': frames[-1]['host_received_monotonic_ns'],
              'state_max_age_ms': max(r[3]['host_received_monotonic_ns']-r[4]['host_monotonic_ns'] for r in rows)/1e6,
              'image_max_age_ms': 0, 'repeated_images': 0}
    report['cameras'] = {key: {
        'source_frames': len(other_frames[key]) if key in other_frames else len(frames),
        'repeated_images': len(rows) - len({r[3].get('camera_frames', {'wrist_rgb': r[3]})[key]['host_received_monotonic_ns'] for r in rows}),
        'max_age_ms': max(r[3]['host_received_monotonic_ns'] - r[3].get('camera_frames', {'wrist_rgb': r[3]})[key]['host_received_monotonic_ns'] for r in rows) / 1e6
    } for key in cameras}
    return meta, rows, report, states


def prepare_camera_episode(path):
    from embodied.data.export.exporters import for_episode
    return for_episode(path).prepare(path)


def export(source, output, repo_id, task=None, fps=30, skip_invalid=False,
           video_threads=None, video_preset='veryfast', progress=True,
           decoder='torchcodec', decode_workers=0, decode_batch_size=32):
    if fps != 30:
        raise ValueError("Video observation FPS must be 30; robot actions remain in the full-rate sidecar")
    video_threads = video_threads if video_threads is not None else min(4, os.cpu_count() or 1)
    if video_threads < 1:
        raise ValueError('video_threads must be positive')
    if decode_workers < 0 or decode_batch_size < 1:
        raise ValueError('decode_workers must be nonnegative and decode_batch_size positive')
    started = time.perf_counter()
    from tqdm import tqdm
    from embodied.data.export.exporters import for_episode
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION
    from embodied.data.export.video import StreamingVideoWriter, MultiVideoWriter
    from embodied.data.export.decode import BatchFrameReader
    if CODEBASE_VERSION != 'v2.1':
        raise RuntimeError('Use the pinned export environment; this OpenPI adapter requires LeRobot v2.1')
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('Output already exists; choose a new directory')
    paths = [source] if (source/'episode.json').is_file() else sorted(p.parent for p in source.glob('*/episode.json'))
    plans, rejected, robot, shapes = [], [], None, None
    adapter = None
    for path in tqdm(paths, desc='检查数据', unit='条', disable=not progress, dynamic_ncols=True):
        try:
            if task is not None and json.loads((path/'episode.json').read_text()).get('task') != task:
                continue
            meta, rows, report, raw_states = prepare_camera_episode(path)
            current_adapter = for_episode(path)
            if adapter is not None and current_adapter.identity != adapter.identity:
                raise ValueError('Mixed episode schemas/action definitions; export separately')
            current_shapes = {key: (cam['height'], cam['width'], 3) for key, cam in camera_metadata(meta).items()}
            if any(cam['fps'] != 30 for cam in camera_metadata(meta).values()):
                raise ValueError('All recording cameras must use 30 FPS')
            if robot is not None and meta['robot'] != robot:
                raise ValueError('Mixed robot models/control frames; export separately')
            if shapes is not None and current_shapes != shapes:
                raise ValueError('Mixed camera sets or image resolutions; export separately')
            robot, shapes = meta['robot'], current_shapes
            adapter = current_adapter
            plans.append((path, report))
            del rows, raw_states
        except (ValueError, KeyError, TypeError, OSError) as exc:
            rejected.append({'path': str(path), 'error': str(exc)})
    if not plans or (rejected and not skip_invalid):
        raise ValueError('No valid episodes or invalid episodes found (use --skip-invalid explicitly): ' + json.dumps(rejected, ensure_ascii=False))
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(output.name + '.partial-' + uuid.uuid4().hex[:8])
    features = {key: {'dtype': 'float32', 'shape': (7,), 'names': FEATURE_NAMES}
                for key in ('observation.state', 'action')}
    for key, shape in shapes.items():
        features['observation.images.' + key[:-4]] = {'dtype': 'video', 'shape': shape, 'names': ['height', 'width', 'channels']}
        if key != 'wrist_rgb':
            features['observation.' + key[:-4] + '_valid'] = {'dtype': 'bool', 'shape': (1,), 'names': None}
            features['source.' + key + '_monotonic_ns'] = {'dtype': 'int64', 'shape': (1,), 'names': None}
    features['observation.wrist_valid'] = {'dtype': 'bool', 'shape': (1,), 'names': None}
    for key in ('source.state_monotonic_ns', 'source.action_monotonic_ns', 'source.rgb_monotonic_ns'):
        features[key] = {'dtype': 'int64', 'shape': (1,), 'names': None}
    dataset, writer = None, None
    total_frames = sum(report['observation_samples'] for _, report in plans)
    try:
        dataset = LeRobotDataset.create(repo_id=repo_id, root=staging, fps=fps,
            robot_type='stanford_x5', features=features, use_videos=True, video_backend='pyav')
        with ExitStack() as stack, \
             tqdm(total=total_frames, desc='转换视频', unit='帧', disable=not progress,
                  dynamic_ncols=True) as bar:
            readers = {key: stack.enter_context(BatchFrameReader(decoder, decode_workers, decode_batch_size)) for key in shapes}
            for episode_index, (path, expected_report) in enumerate(plans):
                bar.set_description(f'读取第 {episode_index+1}/{len(plans)} 条')
                meta, rows, report, raw_states = prepare_camera_episode(path)
                if report != expected_report or meta['robot'] != robot:
                    raise ValueError('Source changed during export')
                bar.set_description(f'保存原始状态 {episode_index+1}/{len(plans)}')
                adapter.save_sidecar(staging / 'robot' / f'episode_{episode_index:06d}.npz', raw_states)
                writer = MultiVideoWriter({})
                for key, shape in shapes.items():
                    feature = 'observation.images.' + key[:-4]
                    video_path = staging / dataset.meta.get_video_file_path(episode_index, feature)
                    writer.writers[feature] = StreamingVideoWriter(video_path, shape, len(rows), fps, video_preset, video_threads)
                dataset.image_writer = writer
                bar.set_description(f'转换第 {episode_index+1}/{len(plans)} 条')
                streams = {key: [row[3].get('camera_frames', {'wrist_rgb': row[3]})[key] for row in rows] for key in shapes}
                pixel_batches = {key: readers[key].frames(path, stream) for key, stream in streams.items()}
                try:
                    for index, (state, action, raw_root, frame, source_state) in enumerate(rows):
                        sample = {'observation.state': state, 'action': action, 'task': meta['task'],
                            'observation.wrist_valid': np.array([frame['wrist_valid']], dtype=bool),
                            'source.state_monotonic_ns': np.array([source_state['host_monotonic_ns']], dtype=np.int64),
                            'source.action_monotonic_ns': np.array([source_state['requested_monotonic_ns']], dtype=np.int64),
                            'source.rgb_monotonic_ns': np.array([frame['host_received_monotonic_ns']], dtype=np.int64)}
                        for key in shapes:
                            sample['observation.images.' + key[:-4]] = next(pixel_batches[key])
                            if key != 'wrist_rgb':
                                sample['observation.' + key[:-4] + '_valid'] = np.array([True], dtype=bool)
                                sample['source.' + key + '_monotonic_ns'] = np.array([streams[key][index]['host_received_monotonic_ns']], np.int64)
                        dataset.add_frame(sample)
                        bar.update()
                finally:
                    for batch in pixel_batches.values():
                        batch.close()
                bar.set_description(f'写入表格与统计 {episode_index+1}/{len(plans)}')
                dataset.save_episode()
                writer.stop()
                writer, dataset.image_writer = None, None
                del rows, raw_states
        manifest = {'lerobot_revision': LEROBOT_REVISION, 'fps': fps, 'robot': robot,
                    'layout': adapter.layout, 'action_fps': 30, 'raw_robot_fps': adapter.raw_hz,
                    'episode_schema': adapter.identity[0], 'action_definition': adapter.identity[1],
                    'sidecar': 'robot/episode_{episode_index:06d}.npz',
                    'state': 'actual joints (rad) + actual gripper width (m)',
                    'action': adapter.identity[1] + ' (rad, m)',
                    'alignment': 'wrist RGB anchors; latest preceding robot state and other cameras within max skew; original clocks retained',
                    'camera': '30 FPS wrist anchors; other cameras may repeat on alignment; see per-camera report; H264 CRF20 GOP2',
                    'cameras': list(shapes),
                    'control_stacks': sorted({report['control_stack'] for _, report in plans}),
                    'backends': sorted({report['backend'] for _, report in plans}),
                    'gap_policy': 'gaps stay spliced at 1/fps (LeRobot timestamps); each is listed per episode and the episode is marked anomaly',
                    'anomalous_episodes': [report['id'] for _, report in plans if report['anomaly']],
                    'conversion': {'video_preset': video_preset, 'video_threads': video_threads,
                                   'decoder': decoder, 'decode_workers': decode_workers,
                                   'decode_batch_size': decode_batch_size, 'encoder': 'streaming'},
                    'episodes': [report for _, report in plans], 'rejected': rejected}
        # Verify the real pinned loader and action chunk indexing before publishing.
        with tqdm(total=3, desc='验证数据集', unit='步', disable=not progress, dynamic_ncols=True) as bar:
            loaded = LeRobotDataset(repo_id, root=staging, video_backend='pyav', delta_timestamps={'action': [i/fps for i in range(15)]})
            bar.update()
            first = loaded[0]
            bar.update()
            last = loaded[len(loaded)-1]
            if len(loaded) != total_frames or first['action'].shape != (15, 7) or last['observation.state'].shape != (7,):
                raise ValueError('Exported dataset failed loader validation')
            for key, shape in shapes.items():
                feature = 'observation.images.' + key[:-4]
                if tuple(first[feature].shape) != (3, *shape[:2]) or tuple(last[feature].shape) != (3, *shape[:2]):
                    raise ValueError('Exported camera failed loader validation: ' + key)
            bar.update()
        elapsed = time.perf_counter()-started
        manifest['conversion'].update(elapsed_s=round(elapsed, 3), frames_per_second=round(total_frames/elapsed, 2))
        (staging/'meta'/'quest_export.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        staging.rename(output)
    except BaseException:
        try:
            if writer is not None:
                writer.stop()
        finally:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    print(json.dumps({'path': str(output), 'episodes': len(plans), 'frames': len(loaded),
        'elapsed_s': round(elapsed, 2), 'frames_per_second': round(total_frames/elapsed, 2),
        'rejected': rejected}, ensure_ascii=False, indent=2))
    return output


def main():
    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'datasets')
    parser.add_argument('--output', type=Path, default=root/'datasets'/'lerobot'/datetime.now().strftime('x5_%Y%m%d-%H%M%S'))
    parser.add_argument('--repo-id', default='local/quest_x5')
    parser.add_argument('--task', help='Exact task text; omit to export all tasks')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--skip-invalid', action='store_true')
    parser.add_argument('--video-threads', type=int, default=min(4, os.cpu_count() or 1), help='H264 encoder threads (default: up to 4)')
    parser.add_argument('--video-preset', choices=('ultrafast', 'superfast', 'veryfast', 'faster', 'fast', 'medium', 'slow'), default='veryfast', help='Faster presets trade compression efficiency for speed; CRF stays 20')
    parser.add_argument('--no-progress', action='store_true', help='Disable conversion progress bars')
    parser.add_argument('--decoder', choices=('torchcodec', 'opencv'), default='torchcodec', help='CPU RGB decoder (default: torchcodec)')
    parser.add_argument('--decode-workers', type=int, default=0, help='Decoder subprocesses; 0 runs in-process (default: 0; avoids startup/IPC overhead on small tasks)')
    parser.add_argument('--decode-batch-size', type=int, default=32, help='Frames per bounded decode batch (default: 32)')
    args = parser.parse_args()
    if args.fps <= 0: parser.error('fps must be positive')
    if args.video_threads < 1: parser.error('video-threads must be positive')
    if args.decode_workers < 0: parser.error('decode-workers must be nonnegative')
    if args.decode_batch_size < 1: parser.error('decode-batch-size must be positive')
    print('正在加载 LeRobot 转换依赖…', file=sys.stderr, flush=True)
    export(args.source, args.output, args.repo_id, args.task, args.fps, args.skip_invalid,
           args.video_threads, args.video_preset, not args.no_progress,
           args.decoder, args.decode_workers, args.decode_batch_size)


if __name__ == '__main__':
    main()
