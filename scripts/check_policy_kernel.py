#!/usr/bin/env python3
"""Policy executor on the MuJoCo kernel with a stand-in policy; no model, camera or hardware."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from embodied.contracts.runtime.control import ControlSession
from embodied.policy.kernel_runner import run
from embodied.sensors.synthetic import SyntheticCamera

ROOT = Path(__file__).resolve().parents[1]


class StandInPolicy:
    """Answers each observation with a chunk that walks joint 1 by +0.004 rad per step."""

    def __init__(self):
        self.calls = 0

    def predict(self, cycle, prompt):
        self.calls += 1
        observation = json.loads((cycle / 'observation.json').read_text())
        state = np.asarray(observation['state_physical'])
        actions = np.repeat(state[None], 15, axis=0)
        actions[:, 0] += .004 * np.arange(1, 16)
        # Alternate the fine-tuned and zero-shot output keys.
        np.savez(cycle / 'prediction.npz', **{'actions_x5' if self.calls % 2 else 'actions_x5_experimental': actions})
        return {'arm_service_id': observation['arm_service_id'], 'rgb_monotonic_ns': observation['rgb_monotonic_ns']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    with (output / 'runtime.log').open('w') as log:
        runtime = subprocess.Popen([str(ROOT / 'run.sh'), 'control', 'start', '--backend', 'sim', '--input', 'idle',
                                    '--managed', '--headless'], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline, directory = time.monotonic() + 20, None
            while directory is None and time.monotonic() < deadline:
                if runtime.poll() is not None:
                    raise RuntimeError((output / 'runtime.log').read_text())
                for path in (ROOT / 'control/run').glob('*/runtime.json'):
                    if json.loads(path.read_text())['supervisor_pid'] == runtime.pid and (path.parent / 'backend.ready').exists():
                        directory = path.parent
                time.sleep(.05)
            assert directory, 'kernel did not start'
            with ControlSession(directory, source='policy', purpose='operation') as session:
                start = np.asarray(session.status()['q'])
                (output / 'policy').mkdir()
                with SyntheticCamera(width=64, height=48) as camera:
                    summary = run(session, StandInPolicy(), camera, output / 'policy', 'stand-in', seconds=3, chunk_steps=10)
                time.sleep(.3)
                end = np.asarray(session.status()['q'])
            moved = float(end[0] - start[0])
            assert summary['status'] == 'time_limit', summary
            assert summary['cycles'] >= 2 and summary['commands'] >= 20, summary
            assert moved > .02, f'joint 1 moved only {moved:.4f} rad'
            assert np.max(np.abs(end[1:] - start[1:])) < .02, 'other joints drifted'
            print(f'PASS: {summary["cycles"]} chunks, {summary["commands"]} actions through the gateway, '
                  f'joint 1 moved {moved:.3f} rad, kernel holding afterwards. {directory}')
        finally:
            runtime.send_signal(2)
            runtime.wait(timeout=30)


if __name__ == '__main__':
    main()
