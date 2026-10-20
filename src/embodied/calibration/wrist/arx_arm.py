"""Existing hand-eye workflow on the arx command/feedback plugin boundary."""
import hashlib
from pathlib import Path
import time
import numpy as np

from embodied.contracts.runtime.control import ControlSession


class ArxCalibrationArm:
    def __init__(self, robot, directory, manual=True):
        self.robot = robot
        self.client = ControlSession(directory, source='handeye')
        parameters = self.client.parameters
        manifest = parameters['model']['manifest']
        expected = manifest['sha256'][Path(parameters['model']['xml_path']).with_suffix('.urdf').name]
        if hashlib.sha256(robot.urdf.read_bytes()).hexdigest() != expected:
            raise ValueError('Calibration model differs from the active arx model')
        self.manual = manual

    def __enter__(self):
        self.client.__enter__()
        try:
            self.gripper = self.client.status()['gripper_width']
            self.client.command('free-drive' if self.manual else 'hold', ttl_ms=250)
            return self
        except BaseException:
            self.client.close()
            raise

    def snapshot(self):
        if self.manual:
            self.client.command('free-drive', ttl_ms=250)
        state = self.client.status()
        return {'q': self.robot.joints(state['q']).tolist(), 'dq': state['dq'],
                'gripper_m': state['gripper_width'], 'sdk_timestamp': None,
                'feedback_clock': 'host monotonic_ns (arx feedback age)',
                'host_monotonic_ns': state['feedback_monotonic_ns']}

    def enable(self):
        state = self.snapshot()
        if np.max(np.abs(state['dq'])) > .02:
            raise ValueError('Reference pose is moving')
        self.manual = False
        self.gripper = state['gripper_m']
        self.client.command('hold')

    def command(self, q):
        if self.manual:
            raise RuntimeError('Enable calibration hold before moving')
        self.client.command('joint', q=self.robot.joints(q, margin=.05).tolist(),
                            gripper_width=self.gripper, ttl_ms=100)

    def damp(self):
        # This compatibility method terminates the calibration operation. The
        # shared runtime's handover policy is measured hold, with no Home move.
        self.manual = False
        self.client.command('hold')

    def __exit__(self, *exc):
        self.client.close()
