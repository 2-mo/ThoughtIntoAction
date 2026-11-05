#!/usr/bin/env python3
"""Recover the right-arm gripper command for episodes recorded before D-07.

Those episodes store one `gripper_width` (the left arm's) per tick, but the
kernel wrote every tick's per-arm telemetry to `tuning.jsonl` during the run.
This tool aligns each episode's tick timestamps against that log and writes a
sidecar with the right gripper's commanded width (`arms[1].gripper_target`).

Read-only on the episodes; the sidecar is a new file next to them. The right
gripper's *measured* width is not recoverable: `sdk_raw_feedback.gripper` was
null in this SDK build and the tick stream only carried the left arm.

Usage:
    python scripts/datasets/recover_right_gripper.py \
        --run outputs/runtime/control/<RUN> --episodes outputs/datasets/runtime [--write]
"""
import argparse
import bisect
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))


def tuning_path(run: Path) -> Path:
    candidates = [run / 'tuning.jsonl']
    candidates += sorted(run.glob('**/tuning.jsonl'))
    for path in candidates:
        if path.is_file():
            return path
    raise SystemExit(f'no tuning.jsonl under {run}')


def read_timeline(path: Path):
    """(monotonic_ns, left target, right target) for every telemetry record."""
    times, left, right = [], [], []
    with path.open() as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            stamp = record.get('monotonic_s')
            arms = record.get('arms') or []
            if stamp is None or len(arms) < 2:
                continue
            times.append(int(float(stamp) * 1e9))
            left.append(arms[0].get('gripper_target'))
            right.append(arms[1].get('gripper_target'))
    order = np.argsort(np.asarray(times, dtype=np.int64))
    return (np.asarray(times, dtype=np.int64)[order],
            np.asarray([left[i] if isinstance(left[i], (int, float)) else np.nan for i in order], dtype=np.float64),
            np.asarray([right[i] if isinstance(right[i], (int, float)) else np.nan for i in order], dtype=np.float64))


def episode_ticks(episode: Path):
    """Tick completion timestamps and the recorded (left) gripper command."""
    from mcap.reader import make_reader

    stamps, commands = [], []
    with (episode / 'episode.mcap').open('rb') as handle:
        for _schema, channel, message in make_reader(handle).iter_messages(log_time_order=True):
            if channel.topic != '/robot/command':
                continue
            row = json.loads(message.data)
            stamps.append(int(row['host_monotonic_ns']))
            commands.append(row.get('gripper_width'))
    return np.asarray(stamps, dtype=np.int64), np.asarray(
        [c if isinstance(c, (int, float)) else np.nan for c in commands], dtype=np.float64)


def align(times, stamps):
    """Nearest-record index and |dt| in nanoseconds for every tick."""
    indices = np.searchsorted(times, stamps)
    indices = np.clip(indices, 1, len(times) - 1)
    left_idx = indices - 1
    nearest = np.where(np.abs(times[indices] - stamps) < np.abs(times[left_idx] - stamps), indices, left_idx)
    return nearest, np.abs(times[nearest] - stamps)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='kernel run directory holding tuning.jsonl')
    parser.add_argument('--episodes', type=Path, default=ROOT / 'outputs/datasets/runtime')
    parser.add_argument('--write', action='store_true', help='write the sidecar files (default: report only)')
    args = parser.parse_args(argv)

    log = tuning_path(args.run)
    print(f'reading {log} ...', flush=True)
    times, left_targets, right_targets = read_timeline(log)
    if len(times) < 2:
        raise SystemExit('tuning log has no per-arm telemetry')
    print(f'telemetry records {len(times)}  {times[0]/1e9:.1f} .. {times[-1]/1e9:.1f} (monotonic s)', flush=True)

    episodes = sorted(p.parent for p in args.episodes.glob('*/episode.json'))
    total = recovered = 0
    worst = 0.0
    for episode in episodes:
        stamps, recorded_left = episode_ticks(episode)
        if not len(stamps):
            print(f'{episode.name}: no command rows, skipped')
            continue
        inside = (stamps >= times[0]) & (stamps <= times[-1])
        if not inside.all():
            print(f'{episode.name}: {int((~inside).sum())}/{len(stamps)} ticks outside the log window')
        nearest, delta = align(times, stamps[inside])
        right = right_targets[nearest]
        left = left_targets[nearest]
        check = recorded_left[inside]
        mask = np.isfinite(check) & np.isfinite(left)
        agreement = (np.abs(check[mask] - left[mask]) < 1e-9).mean() if mask.any() else float('nan')
        max_ms = float(delta.max()) / 1e6 if len(delta) else float('nan')
        worst = max(worst, max_ms)
        total += 1
        recovered += 1
        print(f'{episode.name}: ticks {len(stamps)}  matched {int(inside.sum())}  '
              f'|dt| max {max_ms:.2f} ms  left-agreement {100*agreement:.1f}%  '
              f'right target {np.nanmin(right):.4f}..{np.nanmax(right):.4f} m', flush=True)
        if args.write:
            target = episode / 'recovered'
            target.mkdir(exist_ok=True)
            np.savez_compressed(
                target / 'gripper_right.npz',
                schema_version=np.asarray(1),
                source=np.asarray(str(log)),
                run_id=np.asarray(args.run.name),
                tick_host_monotonic_ns=stamps[inside],
                gripper_right_target_m=right.astype(np.float64),
                gripper_left_target_m=left.astype(np.float64),
                gripper_left_recorded_m=check.astype(np.float64),
                alignment_error_ns=delta.astype(np.int64),
                out_of_window_ticks=np.asarray(int((~inside).sum())),
                note=np.asarray('right gripper command recovered from kernel tuning.jsonl; '
                                'measured width unavailable in this SDK build'),
            )
    print(f'\nepisodes {total}, recovered {recovered}, worst |dt| {worst:.2f} ms')
    if not args.write:
        print('report only; re-run with --write to store sidecars')


if __name__ == '__main__':
    main()
