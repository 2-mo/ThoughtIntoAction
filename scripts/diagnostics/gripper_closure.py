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
                               blocked=False, errors=set(), decisions={}, close_ack=None, first_closed=None, position=None, force=None)
                attempt.update(sent_at=None, samples={}, last_feedback=None, max_query_ms=0.)
            attempt['duration'] = stamp - attempt['start']
            request = row.get('gripper_position_request')
            if request and request['position'] == 1 and attempt['sent_at'] is None:
                attempt['sent_at'] = request['sent_monotonic_s']
                attempt['origin'] = request['origin_actpos']
            pending = row.get('gripper_pending')
            if pending:
                attempt['pending'].add(pending[0])
            decision = row.get('gripper_decision')
            if decision:
                attempt['decisions'].setdefault(decision, attempt['duration'])
            if (decision == 'accepted' and pending and pending[0] == 1
                    and attempt['close_ack'] is None):
                attempt['close_ack'] = (attempt['duration'], row['gripper_ack_ms'])
            attempt['blocked'] |= bool(row.get('gripper_needs_release'))
            feedback = row.get('gripper_raw') or {}
            if feedback.get('error'):
                attempt['errors'].add(feedback['error'])
            if feedback.get('enable') != 1 or feedback.get('status') != 1 or feedback.get('error') != 0:
                continue
            position = feedback['actpos']
            received = row.get('gripper_feedback_received_monotonic_s')
            sent_at = attempt['sent_at']
            if (sent_at is not None and received is not None and received >= sent_at
                    and received != attempt['last_feedback']):
                attempt['last_feedback'] = received
                elapsed = received - sent_at
                attempt['samples'].setdefault(int(elapsed),
                    (elapsed, position, feedback['mode'], feedback.get('current_force')))
                attempt['max_query_ms'] = max(attempt['max_query_ms'], row['gripper_query_ms'])
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
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument('--pair', type=Path, default=ROOT / 'outputs/runtime/realman/pair.json')
    inputs.add_argument('--run', type=Path, help='单臂运行目录，包含 tuning.jsonl')
    parser.add_argument('--since', type=float, default=0, help='只分析此主机单调时间之后的记录')
    args = parser.parse_args()
    try:
        runs = ({args.run.name: args.run} if args.run else json.loads(args.pair.read_text())['runs'])
        for side, directory in sorted(runs.items()):
            path = Path(directory) / 'tuning.jsonl'
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
                if item['decisions']:
                    decisions = ', '.join(f'{key}@{delay:.2f}s' for key, delay in item['decisions'].items())
                    print(f'  驱动决策首次出现: {decisions}')
                if item['close_ack'] is not None:
                    elapsed, latency = item['close_ack']
                    print(f'  位置 1 获确认: 全闭目标后 {elapsed:.2f}s，TCP 应答 {latency:.1f}ms')
                if item['samples']:
                    print(f"  下发时 actpos={item['origin']}，反馈查询最长 {item['max_query_ms']:.1f}ms")
                    for elapsed, position, mode, force in item['samples'].values():
                        print(f'  +{elapsed:.2f}s actpos={position} mode={mode} current_force={force}')
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'读取夹爪日志失败：{exc}\n')


if __name__ == '__main__':
    main()
