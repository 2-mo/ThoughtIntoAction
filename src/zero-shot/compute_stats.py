"""Compute experimental X5 quantiles from validated real raw recordings, without video decoding."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
import numpy as np
from robot_core.model import RobotModel
from quest_teleop.export_lerobot import load_episode
from quest_teleop.multirate import camera_rows
from x5_mapping import SCHEMA, action_chunks


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, default=ROOT / 'datasets/.raw')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--horizon', type=int, default=50, help='Must match base checkpoint chunk_size')
    args = p.parse_args()
    if args.horizon < 1 or args.output.exists():
        p.error('Use a positive horizon and a new output file')
    robot = RobotModel()
    included, rejected, state_arrays, delta_arrays = [], [], [], []
    for path in sorted(args.source.glob('*/episode.json')):
        meta = json.loads(path.read_text())
        if meta.get('outcome') not in ('success', 'unknown'):
            rejected.append({'episode': path.parent.name, 'reason': 'outcome: ' + str(meta.get('outcome'))})
            continue
        try:
            meta, states, frames, _ = load_episode(path.parent)
            if meta['robot'] != robot.metadata():
                raise ValueError('Robot metadata differs')
            for row in states:
                robot.joints(row['actual_pos'])
                robot.joints(row['requested_pos'])
        except (ValueError, KeyError, TypeError, OSError) as exc:
            rejected.append({'episode': path.parent.name, 'reason': str(exc)})
            continue
        root = path.parent
        if meta['camera']['fps'] != 30 or not all(s['actuate'] for s in states):
            rejected.append({'episode': path.parent.name, 'reason': 'Need 30 FPS actuated recordings'})
            continue
        aligned = camera_rows(states, frames, root)
        if len(aligned) < args.horizon:
            rejected.append({'episode': path.parent.name, 'reason': 'Fewer frames than action horizon'})
            continue
        state = np.stack([x[0] for x in aligned]).astype(float)
        action = np.stack([x[1] for x in aligned]).astype(float)
        state_arrays.append(state)
        delta_arrays.append(action_chunks(state, action, args.horizon).reshape(-1, 7))
        included.append({'episode': str(root.resolve()), 'outcome': meta['outcome'], 'anchors': len(state),
                         'files_sha256': {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                          for f in root.iterdir() if f.name in ('episode.mcap', 'episode.json')}})
        print(f'统计 {len(included)} 条 / {sum(len(x) for x in state_arrays)} 个图像锚点', flush=True)
    if not included:
        raise ValueError('No valid episodes: ' + str(rejected))
    def stats(values):
        return {'q01': np.quantile(values, .01, axis=0).tolist(),
                'q99': np.quantile(values, .99, axis=0).tolist(), 'count': len(values)}
    result = {'schema': SCHEMA, 'experimental': True, 'robot': robot.metadata(), 'fps': 30,
              'horizon': args.horizon, 'tail': 'repeat final action within each episode',
              'interpretation': 'Six delta joints relative to observation anchor; absolute gripper metres. '
                                'Computed from X5 recordings, not base pretraining statistics; unknown outcomes included.',
              'state': stats(np.concatenate(state_arrays)), 'action': stats(np.concatenate(delta_arrays)),
              'episodes': included, 'rejected': rejected}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f'统计完成：{len(included)} 条纳入，{len(rejected)} 条排除；{args.output}', flush=True)


if __name__ == '__main__':
    main()
