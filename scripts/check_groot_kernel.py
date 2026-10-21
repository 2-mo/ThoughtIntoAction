#!/usr/bin/env python3
"""GR00T kernel runtime on the MuJoCo kernel with a stand-in policy; no model, camera or hardware."""
import argparse
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from embodied.contracts.runtime.control import ControlSession
from embodied.policy.groot.cameras import RgbFrame, SynchronizedFrames
from embodied.policy.groot.checkpoint import ACTION_HORIZON
from embodied.policy.groot.kernel_runtime import kernel_snapshot, run
from embodied.workstation.control import sim_kernel


class StandInCameras:
    def __init__(self):
        self.number = 0

    def latest(self):
        self.number += 1
        frame = lambda: RgbFrame(np.zeros((48, 64, 3), np.uint8), time.monotonic_ns(), 0.0, self.number)
        return SynchronizedFrames(exterior=frame(), wrist=frame())


class StandInPolicy:
    """Each chunk moves the TCP 2 mm per step along base +Y, keeping orientation, gripper open."""

    def __init__(self):
        self.pending, self.submitted = None, 0

    def submit(self, observation, observation_ns, options):
        assert set(observation['video']) == {'exterior_rgb', 'wrist_rgb'}
        self.pending, self.submitted = observation, self.submitted + 1

    def poll(self):
        if self.pending is None:
            return None
        eef = self.pending['state']['eef_9d'][0, 0].astype(np.float64)
        steps = np.arange(1, ACTION_HORIZON + 1)[:, None]
        chunk = np.repeat(eef[None], ACTION_HORIZON, axis=0)
        chunk[:, 1] += .002 * steps[:, 0]
        self.pending = None
        return SimpleNamespace(error=None, actions={'eef_9d': chunk[None].astype(np.float32),
                                                    'gripper_position': np.full((1, ACTION_HORIZON, 1), .08, np.float32)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    with sim_kernel(output / 'runtime.log') as directory:
        with ControlSession(directory, source='policy', purpose='operation') as session:
            # Move away from the singular zero pose first, as the GR00T home/training start would.
            plan = session.preview('movej', q=[0., .9, .9, -.4, 0., 0.], speed=.4)
            session.wait(session.execute(plan)['operation_id'], timeout=30)
            before = kernel_snapshot(session)
            policy = StandInPolicy()
            summary = run(session, policy, StandInCameras(), 'stand-in', seconds=3)
            time.sleep(.3)
            after = kernel_snapshot(session)
        moved = after.eef_pose_6d[:3] - before.eef_pose_6d[:3]
        (output / 'summary.json').write_text(json.dumps({**summary, 'moved_m': moved.tolist()}, indent=2))
        assert summary['status'] == 'time_limit' and summary['chunks'] >= 2 and summary['rejected_chunks'] == 0, summary
        assert moved[1] > .02 and abs(moved[0]) < .01 and abs(moved[2]) < .01, moved
        print(f'PASS: {summary["chunks"]} chunks, {summary["commands"]} Cartesian targets, '
              f'TCP moved {moved[1] * 1000:.0f} mm along +Y, holding afterwards. {directory}')


if __name__ == '__main__':
    main()
