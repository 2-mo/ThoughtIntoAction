"""Exercise real dual-video GR00T export using a synthetic, measured-state episode."""
from pathlib import Path
import argparse
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import numpy as np
import pyarrow.parquet as pq
from embodied.data.groot.config import load_collection_config
from embodied.data.groot.exporter import export_dataset
from embodied.data.groot.validator import validate_raw_episode


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--groot-stats', action='store_true', default=True,
                        help='Compatibility flag: full acceptance always generates GR00T statistics')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cfg = load_collection_config()
    raw = output / 'raw'
    episode = raw / 'success' / 'synthetic-episode'
    write(episode / 'episode.json', {
        'session_id': 'synthetic-episode', 'dataset_id': cfg.dataset_id,
        'batch_id': cfg.batch_id, 'task_instruction': cfg.task_instruction,
        'stage': 'accepted', 'synthetic': True,
        'limits': {'joint_position_min_rad': [-2.] * 6,
                   'joint_position_max_rad': [2.] * 6, 'gripper_width_m': .088},
    })
    states = []
    for i in range(401):
        q = .001 * min(320, max(0, i - 30))
        states.append({'host_monotonic_ns': 1_000_000_000 + i * 10_000_000,
                       'joint_position': [q, 0., 0., 0., 0., 0.], 'joint_velocity': [0.] * 6,
                       'joint_torque': [0.] * 6, 'gripper_position': .003,
                       'eef_pose_6d': [.3 + q / 10, 0., .2, 0., 0., 0.]})
    write(episode / 'state/summary.json', {'sample_count': len(states), 'error': None})
    (episode / 'state/states.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in states))
    for camera, color in zip(cfg.cameras, ('red', 'blue')):
        path = episode / 'cameras' / camera.video_key
        write(path / 'camera.json', {
            'stage': 'complete', 'video_key': camera.video_key, 'width': camera.width,
            'height': camera.height, 'fps': camera.fps, 'frame_count': 121,
            'dropped_queue_frames': 0, 'error': None,
            'storage': {'container': 'matroska', 'codec': 'h264',
                        'video_file': 'video.mkv', 'frame_metadata_file': 'frames.jsonl'},
        })
        frames = [{'host_monotonic_ns': 1_000_000_000 + round(i * 1e9 / camera.fps),
                   'frame_index': i} for i in range(121)]
        (path / 'frames.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in frames))
        subprocess.run(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
                        '-f', 'lavfi', '-i', f'color=c={color}:s={camera.width}x{camera.height}:r={camera.fps}',
                        '-frames:v', '121', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                        str(path / 'video.mkv')], check=True)
    report = validate_raw_episode(episode, config=cfg)
    write(output / 'raw-validation.json', report)
    assert report['ok'], report
    result = export_dataset(raw, output / 'lerobot', config=cfg,
                            generate_stats=args.groot_stats, allow_incomplete=True)
    table = pq.read_table(next((output / 'lerobot/data').rglob('*.parquet')))
    state = np.asarray(table['observation.state'].to_pylist())
    action = np.asarray(table['action'].to_pylist())
    assert state.shape == action.shape and state.shape[1] == 16
    assert np.allclose(action[:-1, 10:], state[1:, 10:])
    assert np.allclose(action[-1, 10:], state[-1, 10:])
    videos = list((output / 'lerobot/videos').rglob('*.mp4'))
    assert len(videos) == 2
    write(output / 'result.json', {'frames': len(state), 'videos': len(videos),
          'action': 'measured-next-state joint positions', 'groot_stats': args.groot_stats,
          'export': result})
    print(json.dumps({'frames': len(state), 'videos': len(videos), 'result': str(output / 'result.json')}))


if __name__ == '__main__':
    main()
