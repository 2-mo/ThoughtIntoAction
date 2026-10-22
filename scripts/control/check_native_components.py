#!/usr/bin/env python3
"""Fault acceptance: actual native processes + MuJoCo, synthetic XR only."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]
from embodied.contracts.rpc import request


def case(output, component, fault_signal, active=True):
    output.mkdir()
    env = {**os.environ, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'PYTHONDONTWRITEBYTECODE': '1'}
    with (output/'supervisor.log').open('w') as log:
        runtime = subprocess.Popen([sys.executable, '-B', '-m', 'embodied.control', 'run', '--backend', 'sim',
            '--managed', '--input', 'idle', '--input-plugin', 'arx.demo', '--headless',
            '--controller-plugin', 'arx.joint_target', '--record-tuning'], cwd=ROOT, env=env,
            stdout=log, stderr=subprocess.STDOUT)
        directory = None
        lease = None
        sequence = 0
        suspended = []
        def call(action, **fields):
            return request(directory/'commands.sock', {'version':1, 'runtime_id':identity['runtime_id'],
                'action':action, 'sent_monotonic_ns':time.monotonic_ns(), **fields})
        def owned(action, **fields):
            nonlocal sequence
            sequence += 1
            return call(action, token=lease['token'], epoch=lease['epoch'], sequence=sequence, **fields)
        def until(predicate, timeout=12):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                if runtime.poll() is not None:
                    raise RuntimeError((output/'supervisor.log').read_text())
                value = predicate()
                if value:
                    return value
                time.sleep(.02)
            raise TimeoutError(f'Condition timed out: {output}')
        try:
            def ready():
                nonlocal directory, identity
                for path in (ROOT/'outputs/runtime/control').glob('*/native-processes.json'):
                    ident = json.loads((path.parent/'runtime.json').read_text())
                    if ident['supervisor_pid'] == runtime.pid:
                        directory, identity = path.parent, ident
                        return True
            identity = None
            until(ready)
            until(lambda: call('status')['ingresses']['native']['ready'])
            processes = json.loads((directory/'native-processes.json').read_text())
            lease = call('acquire', source='operator_a' if active else 'phone',
                         ingress='native' if active else 'rpc', purpose='teleop')
            q_start = lease['q']
            def moving():
                owned('heartbeat')
                state = call('status')
                assert state['owner'], state
                return state if state['phase'] == 'teleop' and max(abs(a-b) for a,b in zip(state['q'],q_start)) > .002 else None
            if active:
                before = until(moving)
            else:
                owned('joint', q=q_start)
                before = call('status')
            targets = list(processes) if component == 'all' else [component]
            started = time.monotonic()
            for name in targets:
                os.kill(runtime.pid if name == 'supervisor' else processes[name], fault_signal)
                if fault_signal == signal.SIGSTOP:
                    suspended.append(runtime.pid if name == 'supervisor' else processes[name])
            if active:
                def revoked():
                    state = call('status')
                    if state['owner']:
                        try:
                            owned('heartbeat')
                        except ValueError:
                            pass  # A health withdrawal may win the next tick.
                    return state if state['owner'] is None else None
                after = until(revoked, 1.2)
                delay = time.monotonic()-started
                assert after['phase'] == 'hold' and not after['stopped'], after
                assert after['last_revocation']['reason'].startswith('native'), after
                until(lambda: max(map(abs,call('status')['dq'])) < .05)
                # A different ingress still works after this dependency failed.
                lease = call('acquire',source='phone', purpose='teleop')
                sequence = 0
                owned('joint',q=call('status')['q'])
                time.sleep(.02)
                assert call('status')['owner']['source'] == 'phone'
            else:
                for _ in range(35):
                    owned('joint', q=q_start)
                    assert call('status')['owner']['source'] == 'phone'
                    time.sleep(.02)
                after, delay = call('status'), time.monotonic()-started
                assert after['last_revocation'] is None, after
                owned('release')
                lease = call('acquire',source='new_label',ingress='native')
                until(lambda: call('status')['owner'] is None, 1)
            call('stop')
            for pid in suspended:
                os.kill(pid,signal.SIGCONT)
            suspended.clear()
            assert runtime.wait(timeout=15) == 0, (output/'supervisor.log').read_text()
            report = json.loads((directory/'report.json').read_text())
            assert not report['error'] and 245 <= report['achieved_hz'] <= 255, report
            rows = [json.loads(line) for line in (directory/'tuning.jsonl').read_text().splitlines()]
            # Native faults must never trigger the coordinator's Disabled/Home path.
            control_rows = [row for row in rows if row['phase'] == 'control']
            assert not any(row['command_mode'] == 'disabled' for row in control_rows), 'Unexpected Home on fault'
            result = {'runtime':str(directory), 'component':component, 'signal':fault_signal.name,
                      'active':active, 'withdrawal_observed_s':delay if active else None,
                      'before':before, 'after':after, 'report':report}
            (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(f'PASS {output.name}: {report["achieved_hz"]:.2f} Hz; observed {delay:.3f}s',flush=True)
            return result
        finally:
            for pid in suspended:
                try: os.kill(pid,signal.SIGCONT)
                except ProcessLookupError: pass
            if runtime.poll() is None:
                runtime.send_signal(signal.SIGINT)
                try: runtime.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    runtime.kill(); runtime.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--quick',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    results=[]
    for name in ('input','coordinator','arm','gripper'):
        results.append(case(args.output/f'exit-{name}',name,signal.SIGTERM))
    if not args.quick:
        for name in ('input','coordinator','arm','gripper','supervisor'):
            results.append(case(args.output/f'stall-{name}',name,signal.SIGSTOP))
    results.append(case(args.output/'inactive-all','all',signal.SIGTERM,active=False))
    (args.output/'result.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
