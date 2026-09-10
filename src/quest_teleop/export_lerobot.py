"""Offline conversion of raw Quest episodes to OpenPI's pinned LeRobot v2.1."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import uuid

import numpy as np

LEROBOT_REVISION = '0cf864870cf29f4738d3ade893e6fd13fbd7cdb5'
FEATURE_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_m']


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def prepare_episode(path, fps=50, max_gap_s=.1, preserve_samples=False):
    """Uniform host-clock grid; causal image/state/command selection, no extrapolation."""
    meta = json.loads((path / 'episode.json').read_text())
    if meta.get('kind') != 'quest_teleop_episode' or meta.get('schema_version') != 1:
        raise ValueError('Unknown episode format')
    if meta.get('lifecycle') != 'complete' or meta.get('outcome') in ('discarded', 'failure'):
        raise ValueError('Episode is not complete or is discarded/failed')
    if meta.get('error') or any(meta.get('dropped', {}).values()):
        raise ValueError('Writer error or dropped samples')
    states, images = read_rows(path / 'states.jsonl'), read_rows(path / 'frames.jsonl')
    if any(meta.get('counts', {}).get(k) != n for k, n in [('state', len(states)), ('frame', len(images))]):
        raise ValueError('Manifest counts disagree with stored records')
    clocks = [np.array([r[k] for r in rows], dtype=np.int64) for rows, k in
              [(states, 'host_monotonic_ns'), (images, 'host_received_monotonic_ns')]]
    for clock in clocks:
        if len(clock) < 2 or np.any(np.diff(clock) <= 0) or np.max(np.diff(clock)) > max_gap_s * 1e9:
            raise ValueError('Missing, unordered or gapped timestamps')
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
    camera = meta['camera']
    report = {'id': meta['id'], 'task': meta['task'], 'source': str(path.resolve()),
              'outcome': meta['outcome'], 'samples': len(grid),
              'start_monotonic_ns': int(grid[0]), 'end_monotonic_ns': int(grid[-1]),
              'state_max_age_ms': float(np.max(grid-clocks[0][indices[0]]) / 1e6),
              'image_max_age_ms': float(np.max(grid-clocks[1][indices[1]]) / 1e6),
              'repeated_images': len(grid) - len(set(indices[1].tolist()))}
    return meta, selected, report


def export(source, output, repo_id, task=None, fps=50, skip_invalid=False, preserve_samples=False):
    from PIL import Image
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION
    import lerobot.common.datasets.lerobot_dataset as dataset_module
    from lerobot.common.datasets.video_utils import encode_video_frames
    from functools import partial
    dataset_module.encode_video_frames = partial(encode_video_frames, vcodec='h264', crf=20)
    from .video import read_frame
    if CODEBASE_VERSION != 'v2.1':
        raise RuntimeError('Use the pinned export environment; this OpenPI adapter requires LeRobot v2.1')
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('Output already exists; choose a new directory')
    paths = [source] if (source/'episode.json').is_file() else sorted(p.parent for p in source.glob('*/episode.json'))
    plans, rejected, robot, shape = [], [], None, None
    for path in paths:
        meta = json.loads((path/'episode.json').read_text())
        if task is not None and meta.get('task') != task:
            continue
        try:
            meta, rows, report = prepare_episode(path, fps, preserve_samples=preserve_samples)
            current_shape = (meta['camera']['height'], meta['camera']['width'], 3)
            if robot is not None and meta['robot'] != robot:
                raise ValueError('Mixed robot models/control frames; export separately')
            if shape is not None and current_shape != shape:
                raise ValueError('Mixed image resolutions; export separately')
            robot, shape = meta['robot'], current_shape
            plans.append((meta, rows, report))
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
    dataset = LeRobotDataset.create(repo_id=repo_id, root=staging, fps=fps,
                                   robot_type='stanford_x5', features=features, use_videos=True, video_backend="pyav")
    try:
        for meta, rows, report in plans:
            cached_key, cached_pixels = None, None
            for state, action, raw_root, frame, source_state in rows:
                key = (frame.get('image'), frame.get('video'), frame.get('video_frame_index'))
                if key != cached_key:
                    pixels_bgr = read_frame(raw_root, frame)
                    if pixels_bgr is None: raise ValueError('Cannot decode source RGB')
                    cached_pixels, cached_key = pixels_bgr[..., ::-1].copy(), key
                pixels = cached_pixels if frame["wrist_valid"] else np.zeros_like(cached_pixels)
                if pixels.shape != shape:
                    raise ValueError('Actual image resolution differs from metadata')
                dataset.add_frame({'observation.state': state, 'action': action,
                                   'observation.images.wrist': pixels, 'task': meta['task'],
                    'observation.wrist_valid': np.array([frame['wrist_valid']], dtype=bool),
                    'source.state_monotonic_ns': np.array([source_state['host_monotonic_ns']], dtype=np.int64),
                    'source.action_monotonic_ns': np.array([source_state['requested_monotonic_ns']], dtype=np.int64),
                    'source.rgb_monotonic_ns': np.array([frame['host_received_monotonic_ns'] if frame['wrist_valid'] else 0], dtype=np.int64)})
            dataset.save_episode()
        manifest = {'lerobot_revision': LEROBOT_REVISION, 'fps': fps, 'robot': robot,
                    'state': 'actual joints (rad) + actual gripper width (m)',
                    'action': 'requested absolute joints (rad) + requested gripper width (m)',
                    'alignment': ('all recorded state rows, nominal index/fps training clock; exact source clocks retained' if preserve_samples else 'uniform host monotonic grid, latest preceding image and state; no future observations'),
                    'camera': 'original RGB resolution, H264 CRF20; training video repeats causal 30fps source images on the action grid; raw video stays 30fps',
                    'episodes': [p[2] for p in plans], 'rejected': rejected}
        (staging/'meta'/'quest_export.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        # Verify the real pinned loader and action chunk indexing before publishing.
        loaded = LeRobotDataset(repo_id, root=staging, video_backend='pyav', delta_timestamps={'action': [i/fps for i in range(10)]})
        first, last = loaded[0], loaded[len(loaded)-1]
        assert first['action'].shape == (10, 7) and last['observation.state'].shape == (7,)
        staging.rename(output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    print(json.dumps({'path': str(output), 'episodes': len(plans), 'frames': len(loaded), 'rejected': rejected}, ensure_ascii=False, indent=2))
    return output


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'datasets')
    parser.add_argument('--output', type=Path, default=root/'datasets'/'lerobot'/datetime.now().strftime('x5_%Y%m%d-%H%M%S'))
    parser.add_argument('--repo-id', default='local/quest_x5')
    parser.add_argument('--task', help='Exact task text; omit to export all tasks')
    parser.add_argument('--fps', type=int, default=50)
    parser.add_argument('--skip-invalid', action='store_true')
    parser.add_argument('--preserve-samples', action='store_true')
    args = parser.parse_args()
    if args.fps <= 0: parser.error('fps must be positive')
    export(args.source, args.output, args.repo_id, args.task, args.fps, args.skip_invalid, args.preserve_samples)


if __name__ == '__main__':
    main()
