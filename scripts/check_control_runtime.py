#!/usr/bin/env python3
"""Process-level simulation acceptance; never constructs a real robot backend."""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def until(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if result := predicate():
            return result
        time.sleep(.05)
    raise TimeoutError('Simulation acceptance condition was not reached')


def child(supervisor, marker):
    path = Path(f'/proc/{supervisor}/task/{supervisor}/children')
    for pid in path.read_text().split():
        try:
            args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
            if marker.encode() in args:
                return int(pid)
        except FileNotFoundError:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', action='store_true', help='Use Xvfb to exercise actual viewer exit')
    parser.add_argument('--keep-observer', action='store_true', help='Verify clean terminal delivery instead of observer failure')
    parser.add_argument('--output', type=Path, required=True, help='New output directory')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    display = runtime = interrupted = None
    with (output / 'runtime.log').open('w') as log:
        try:
            if args.viewer:
                display = subprocess.Popen(['Xvfb', '-displayfd', '1', '-screen', '0', '800x600x24', '-nolisten', 'tcp'],
                    stdout=subprocess.PIPE, stderr=log, text=True)
                if not select.select([display.stdout], [], [], 5)[0]:
                    raise RuntimeError('Xvfb did not select a display')
                env['DISPLAY'] = ':' + display.stdout.readline().strip()
                env.pop('WAYLAND_DISPLAY', None)
                env.pop('SSH_CONNECTION', None)
            command = [str(ROOT / 'run.sh'), 'control', 'start', '--backend', 'sim',
                       '--input', 'demo', '--duration', '12', '--record-tuning']
            if not args.viewer:
                command.append('--headless')
            runtime = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)

            def find_run():
                for path in (ROOT / 'outputs/runtime/control').glob('*/runtime.json'):
                    try:
                        if json.loads(path.read_text())['supervisor_pid'] == runtime.pid and (path.parent / 'backend.ready').exists():
                            return path.parent
                    except (ValueError, FileNotFoundError):
                        pass
            directory = until(find_run)
            if args.viewer:
                viewer = until(lambda: child(runtime.pid, '_viewer'))
                time.sleep(.7)
                os.kill(viewer, signal.SIGTERM)
                until(lambda: '桌面视图已退出' in (output / 'runtime.log').read_text())
                assert runtime.poll() is None, 'Viewer exit stopped control'
            base = [str(ROOT / 'run.sh'), 'collect', '--run', str(directory), '--task', 'simulation acceptance']
            successful = subprocess.run(base + ['--duration', '1', '--output', str(output / 'complete')],
                cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
            (output / 'collector.log').write_text(successful.stdout)
            assert successful.returncode == 0, successful.stdout
            episode = next((output / 'complete').glob('*/episode.json')).parent
            verified = subprocess.run([str(ROOT / 'run.sh'), 'runtime-data', 'validate', str(episode)],
                cwd=ROOT, capture_output=True, text=True, timeout=10)
            assert verified.returncode == 0, verified.stderr
            with (output / 'interrupted.log').open('w') as interrupted_log:
                interrupted = subprocess.Popen(base + ['--output', str(output / 'interrupted')],
                    cwd=ROOT, stdout=interrupted_log, stderr=subprocess.STDOUT)
                until(lambda: list((output / 'interrupted').glob('*/episode.json')))
                time.sleep(.3)
                if not args.keep_observer:
                    os.kill(until(lambda: child(runtime.pid, 'embodied.control.observer')), signal.SIGTERM)
                assert interrupted.wait(timeout=25) == (0 if args.keep_observer else 1), 'Wrong collector exit status'
            if not args.keep_observer:
                until(lambda: '监看反馈服务已退出' in (output / 'runtime.log').read_text())
                assert runtime.poll() is None, 'Observer exit stopped control'
            assert runtime.wait(timeout=25) == 0, (output / 'runtime.log').read_text()
            failed = json.loads(next((output / 'interrupted').glob('*/episode.json')).read_text())
            if args.keep_observer:
                assert failed['lifecycle'] == 'complete' and failed['terminal_received'] and not failed['error']
            else:
                assert failed['lifecycle'] == 'failed' and failed['error']
            report = json.loads((directory / 'report.json').read_text())
            assert report['park_completed'] and report['error'] is None
            assert report['ticks'] > 2500 and report['tick_telemetry']['dropped'] == 0
            result = {'runtime': str(directory), 'viewer_exit_isolated': args.viewer,
                      'observer_exit_isolated': not args.keep_observer, 'complete_capture_validated': str(episode),
                      'broken_stream_marked_failed': not args.keep_observer,
                      'terminal_capture_complete': args.keep_observer, 'report': report}
            (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            print(json.dumps({key: value for key, value in result.items() if key != 'report'}, indent=2))
        finally:
            for process in (interrupted, runtime, display):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == '__main__':
    main()
