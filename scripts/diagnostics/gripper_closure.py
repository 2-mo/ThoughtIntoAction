"""Compare EG2 closure attempts in existing control logs."""
import argparse
from collections import deque
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def attempts(path, since):
    """Group consecutive ticks that request the closed endpoint."""
    attempt = None
    with path.open() as stream:
        for line in stream:
            if not line.endswith('\n'):
                break  # A running kernel may still be writing this row.
            row = json.loads(line)
            stamp = row['monotonic_s']
            if stamp < since:
                continue
            if row.get('command_gripper_target') != 0:
                if attempt:
                    yield attempt
                    attempt = None
                continue
            if attempt is None:
                attempt = dict(start=stamp, minimum=None, modes=set(), pending=set(),
                               blocked=False, errors=set(), first_closed=None, position=None, force=None)
            attempt['duration'] = stamp - attempt['start']
            pending = row.get('gripper_pending')
            if pending:
                attempt['pending'].add(pending[0])
            attempt['blocked'] |= bool(row.get('gripper_needs_release'))
            feedback = row.get('gripper_raw') or {}
            if feedback.get('error'):
                attempt['errors'].add(feedback['error'])
            if feedback.get('enable') != 1 or feedback.get('status') != 1 or feedback.get('error') != 0:
                continue
            position = feedback['actpos']
            attempt['position'] = position
            attempt['force'] = feedback.get('current_force')
            minimum = attempt['minimum']
            attempt['minimum'] = position if minimum is None else min(position, minimum)
            attempt['modes'].add(feedback['mode'])
            if position <= 1 and attempt['first_closed'] is None:
                attempt['first_closed'] = attempt['duration']
    if attempt:
        yield attempt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pair', type=Path, default=ROOT / 'outputs/runtime/realman/pair.json')
    parser.add_argument('--since', type=float, default=0, help='只分析此主机单调时间之后的记录')
    args = parser.parse_args()
    try:
        runs = json.loads(args.pair.read_text())['runs']
        for side in ('left', 'right'):
            path = Path(runs[side]) / 'tuning.jsonl'
            print(f'\n{side}: {path}')
            recent = deque(attempts(path, args.since), maxlen=5)
            if not recent:
                print('没有全闭合目标记录')
            for item in recent:
                delay = item['first_closed']
                closed = '未到达' if delay is None else f'{delay:.2f}s'
                print(f"t={item['start']:.2f}s 持续={item['duration']:.2f}s "
                      f"actpos最低/末值={item['minimum']}/{item['position']} 到闭合端(≤1)={closed} "
                      f"末次压力={item['force']} 模式={sorted(item['modes'])} 等待目标={sorted(item['pending'])} "
                      f"曾等待松键={item['blocked']} 故障码={sorted(item['errors'])}")
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'读取夹爪日志失败：{exc}\n')


if __name__ == '__main__':
    main()
