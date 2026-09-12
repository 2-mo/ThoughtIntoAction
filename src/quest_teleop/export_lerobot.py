"""Offline conversion of raw Quest episodes to OpenPI's pinned LeRobot v2.1."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
import uuid

import numpy as np

LEROBOT_REVISION = '0cf864870cf29f4738d3ade893e6fd13fbd7cdb5'
FEATURE_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_m']


def load_episode(path, max_gap_s=.1):
    """Read and validate one episode; do not retain all episodes in memory."""
    meta = json.loads((path / 'episode.json').read_text())
    if meta.get('kind') != 'quest_teleop_episode' or meta.get('schema_version') != 1:
        raise ValueError('Unknown episode format')
    if meta.get('lifecycle') != 'complete' or meta.get('outcome') in ('discarded', 'failure'):
        raise ValueError('Episode is not complete or is discarded/failed')
    if meta.get('error') or any(meta.get('dropped', {}).values()):
        raise ValueError('Writer error or dropped samples')
    if meta.get('record_storage', {}).get('format') != 'mcap':
        raise ValueError('Only MCAP episodes are supported')
    from .mcap_io import read_records
    states, images = read_records(path / 'episode.mcap')
    if any(meta.get('counts', {}).get(k) != n for k, n in [('state', len(states)), ('frame', len(images))]):
        raise ValueError('Manifest counts disagree with stored records')
    clocks = [np.array([r[k] for r in rows], dtype=np.int64) for rows, k in
              [(states, 'host_monotonic_ns'), (images, 'host_received_monotonic_ns')]]
    for clock in clocks:
        if len(clock) < 2 or np.any(np.diff(clock) <= 0) or np.max(np.diff(clock)) > max_gap_s * 1e9:
            raise ValueError('Missing, unordered or gapped timestamps')
    for state in states:
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
    from .video import media_path
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


def prepare_episode(path, fps=50, max_gap_s=.1, preserve_samples=False):
    """Uniform host-clock grid; causal image/state/command selection, no extrapolation."""
    meta, states, images, clocks = load_episode(path, max_gap_s)
    start, end = max(c[0] for c in clocks), min(c[-1] for c in clocks)
    grid = start + np.rint(np.arange(max(0, math.floor((end-start)*fps/1e9)+1)) * 1e9 / fps).astype(np.int64)
    if preserve_samples:
        grid = clocks[0].copy()
    if len(grid) < 2:
        raise ValueError('No usable overlapping image/state interval')
    indices = [np.maximum(0, np.searchsorted(c, grid, side='right') - 1) for c in clocks]
    if any(np.max(grid - c[ix]) > max_gap_s * 1e9 for c, ix in zip(clocks, indices)):
        raise ValueError('Stale observation on synchronized grid')
    selected = []
    for t, si, fi in zip(grid, *indices):
        state, frame = states[int(si)], images[int(fi)]
        if state.get('preset_name') or state.get('source') in ('preset', 'replay'):
            raise ValueError('Episode includes positioning/replay; collect a clean demonstration')
        observation = np.asarray([*state['actual_pos'], state['gripper_actual_pos']], dtype=np.float32)
        # Use commanded targets, not the later measured response. Force hold remains
        # a deployment-controller behavior, as it is during collection.
        command = np.asarray([*state['requested_pos'], state['requested_gripper_pos']], dtype=np.float32)
        if observation.shape != (7,) or command.shape != (7,) or not np.isfinite([observation, command]).all():
            raise ValueError('Missing/nonfinite 7D state or command')
        if not (0 <= observation[-1] <= .082 and 0 <= command[-1] <= .082):
            raise ValueError('Gripper outside calibrated width range')
        image = (path / frame.get('video', frame.get('image', ''))).resolve()
        if not image.is_relative_to(path.resolve()) or not image.is_file():
            raise ValueError('Missing image or image path escapes episode')
        frame = {**frame, "wrist_valid": bool(t >= clocks[1][0])}
        selected.append((observation, command, path, frame, state))
    report = {'id': meta['id'], 'task': meta['task'], 'source': str(path.resolve()),
              'outcome': meta['outcome'], 'samples': len(grid),
              'start_monotonic_ns': int(grid[0]), 'end_monotonic_ns': int(grid[-1]),
              'state_max_age_ms': float(np.max(grid-clocks[0][indices[0]]) / 1e6),
              'image_max_age_ms': float(np.max(grid-clocks[1][indices[1]]) / 1e6),
              'repeated_images': len(grid) - len(set(indices[1].tolist()))}
    return meta, selected, report


def prepare_camera_episode(path):
    from .multirate import camera_rows
    meta, states, frames, _ = load_episode(path)
    if meta['camera']['fps'] != 30:
        raise ValueError('Expected original camera FPS 30')
    rows = camera_rows(states, frames, path)
    if len(rows) < 2:
        raise ValueError('Insufficient causal camera anchors')
    report = {'id': meta['id'], 'task': meta['task'], 'source': str(path),
              'outcome': meta['outcome'], 'samples': len(rows),
              'robot_samples': len(states), 'observation_samples': len(rows),
              'excluded_camera_anchors': len(frames)-len(rows),
              'start_monotonic_ns': frames[0]['host_received_monotonic_ns'],
              'end_monotonic_ns': frames[-1]['host_received_monotonic_ns'],
              'state_max_age_ms': max(r[3]['host_received_monotonic_ns']-r[4]['host_monotonic_ns'] for r in rows)/1e6,
              'image_max_age_ms': 0, 'repeated_images': 0}
    return meta, rows, report, states


def export(source, output, repo_id, task=None, fps=30, skip_invalid=False, preserve_samples=False,
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
    from .multirate import save_robot_sidecar
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION
    from .export_video import StreamingVideoWriter
    from .export_decode import BatchFrameReader
    if CODEBASE_VERSION != 'v2.1':
        raise RuntimeError('Use the pinned export environment; this OpenPI adapter requires LeRobot v2.1')
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('Output already exists; choose a new directory')
    paths = [source] if (source/'episode.json').is_file() else sorted(p.parent for p in source.glob('*/episode.json'))
    plans, rejected, robot, shape = [], [], None, None
    for path in tqdm(paths, desc='检查数据', unit='条', disable=not progress, dynamic_ncols=True):
        try:
            if task is not None and json.loads((path/'episode.json').read_text()).get('task') != task:
                continue
            meta, rows, report, raw_states = prepare_camera_episode(path)
            current_shape = (meta['camera']['height'], meta['camera']['width'], 3)
            if robot is not None and meta['robot'] != robot:
                raise ValueError('Mixed robot models/control frames; export separately')
            if shape is not None and current_shape != shape:
                raise ValueError('Mixed image resolutions; export separately')
            robot, shape = meta['robot'], current_shape
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
    features['observation.images.wrist'] = {'dtype': 'video', 'shape': shape, 'names': ['height', 'width', 'channels']}
    features['observation.wrist_valid'] = {'dtype': 'bool', 'shape': (1,), 'names': None}
    for key in ('source.state_monotonic_ns', 'source.action_monotonic_ns', 'source.rgb_monotonic_ns'):
        features[key] = {'dtype': 'int64', 'shape': (1,), 'names': None}
    dataset, writer = None, None
    total_frames = sum(report['observation_samples'] for _, report in plans)
    try:
        dataset = LeRobotDataset.create(repo_id=repo_id, root=staging, fps=fps,
            robot_type='stanford_x5', features=features, use_videos=True, video_backend='pyav')
        with BatchFrameReader(decoder, decode_workers, decode_batch_size) as reader, \
             tqdm(total=total_frames, desc='转换视频', unit='帧', disable=not progress,
                  dynamic_ncols=True) as bar:
            for episode_index, (path, expected_report) in enumerate(plans):
                bar.set_description(f'读取第 {episode_index+1}/{len(plans)} 条')
                meta, rows, report, raw_states = prepare_camera_episode(path)
                if report != expected_report or meta['robot'] != robot:
                    raise ValueError('Source changed during export')
                bar.set_description(f'保存原始状态 {episode_index+1}/{len(plans)}')
                save_robot_sidecar(staging / 'robot' / f'episode_{episode_index:06d}.npz', raw_states)
                video_path = staging / dataset.meta.get_video_file_path(episode_index, 'observation.images.wrist')
                writer = StreamingVideoWriter(video_path, shape, len(rows), fps, video_preset, video_threads)
                dataset.image_writer = writer
                bar.set_description(f'转换第 {episode_index+1}/{len(plans)} 条')
                pixel_batches = reader.frames(path, [row[3] for row in rows])
                try:
                    for (state, action, raw_root, frame, source_state), pixels in zip(rows, pixel_batches, strict=True):
                        dataset.add_frame({'observation.state': state, 'action': action,
                            'observation.images.wrist': pixels, 'task': meta['task'],
                            'observation.wrist_valid': np.array([frame['wrist_valid']], dtype=bool),
                            'source.state_monotonic_ns': np.array([source_state['host_monotonic_ns']], dtype=np.int64),
                            'source.action_monotonic_ns': np.array([source_state['requested_monotonic_ns']], dtype=np.int64),
                            'source.rgb_monotonic_ns': np.array([frame['host_received_monotonic_ns']], dtype=np.int64)})
                        bar.update()
                finally:
                    pixel_batches.close()
                bar.set_description(f'写入表格与统计 {episode_index+1}/{len(plans)}')
                dataset.save_episode()
                writer.stop()
                writer, dataset.image_writer = None, None
                del rows, raw_states
        manifest = {'lerobot_revision': LEROBOT_REVISION, 'fps': fps, 'robot': robot,
                    'layout': 'aligned30_raw240_v1', 'action_fps': 30, 'raw_robot_fps': 240,
                    'sidecar': 'robot/episode_{episode_index:06d}.npz',
                    'state': 'actual joints (rad) + actual gripper width (m)',
                    'action': 'requested absolute joints (rad) + requested gripper width (m)',
                    'alignment': 'one observation per real RGB frame; causal robot state; exact clocks and all robot samples in sidecar',
                    'camera': '30 FPS, no repeated images; original resolution; H264 CRF20 GOP2',
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
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'datasets')
    parser.add_argument('--output', type=Path, default=root/'datasets'/'lerobot'/datetime.now().strftime('x5_%Y%m%d-%H%M%S'))
    parser.add_argument('--repo-id', default='local/quest_x5')
    parser.add_argument('--task', help='Exact task text; omit to export all tasks')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--skip-invalid', action='store_true')
    parser.add_argument('--preserve-samples', action='store_true', help='Compatibility flag; full robot samples are always retained in robot/*.npz')
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
    export(args.source, args.output, args.repo_id, args.task, args.fps, args.skip_invalid, args.preserve_samples,
           args.video_threads, args.video_preset, not args.no_progress,
           args.decoder, args.decode_workers, args.decode_batch_size)


if __name__ == '__main__':
    main()
