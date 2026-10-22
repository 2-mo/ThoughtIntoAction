"""The arm side of hand-eye calibration: one kernel lease for teaching, holding and previewed moves.

Limits, the table plane and motion bounds are the kernel's; this only sequences them.
"""
import threading
import time

import numpy as np

from embodied.contracts.runtime.control import ControlSession

KEEPALIVE_S = .08


class Refused(ValueError):
    """The kernel declined to plan a move (limits, table plane, bounds); nothing moved."""


class CalibrationArm:
    def __init__(self, directory):
        self.session = ControlSession(directory, source='handeye', purpose='operation')
        self.teaching = threading.Event()
        self.stop = threading.Event()
        self.error = None
        self.thread = None

    def __enter__(self):
        self.session.__enter__()
        self.thread = threading.Thread(target=self._keepalive, name='handeye-teach', daemon=True)
        self.thread.start()
        return self

    @property
    def robot(self):
        """What the samples are expressed in, for the session record."""
        identity = self.session.identity
        return {'runtime_id': identity['runtime_id'], 'backend': identity.get('backend'),
                'model_id': self.session.status()['model_id'], 'base_frame': 'base_link',
                'eef_frame': 'tcp_link (link6 + 0.145 m along +x)', 'joint_unit': 'rad', 'length_unit': 'm'}

    def _keepalive(self):
        # Free-drive is a short-TTL target: renew it while teaching, keeping the gripper where it is.
        while not self.stop.wait(KEEPALIVE_S):
            if self.teaching.is_set():
                try:
                    self.session.command('free-drive', gripper_width=self.width, ttl_ms=200,
                                         sent_monotonic_ns=time.monotonic_ns())
                except Exception as exc:
                    self.error = str(exc)
                    self.teaching.clear()

    def _check(self):
        if self.error:
            raise RuntimeError(f'Kernel control lost: {self.error}')

    def snapshot(self):
        self._check()
        status = self.session.status()
        return {'q': status['q'], 'dq': status['dq'], 'pose': status['pose'], 'gripper_width': status['gripper_width'],
                'host_monotonic_ns': status['feedback_monotonic_ns']}

    def teach(self):
        """Arm free to move by hand, gripper held at its current width."""
        self._check()
        self.width = float(self.session.status()['gripper_width'])
        self.teaching.set()

    def hold(self, settle_s=.3):
        """Hold the measured pose and wait until the arm is still."""
        self.teaching.clear()
        self.session.command('hold')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if np.max(np.abs(self.snapshot()['dq'])) < .01:
                break
            time.sleep(.02)
        time.sleep(settle_s)

    def move(self, q, speed):
        """Kernel MoveJ: preview (limits, table plane, derivative bounds), execute, wait until settled."""
        self.teaching.clear()
        try:
            plan = self.session.preview('movej', q=[float(v) for v in q], speed=float(speed))
        except RuntimeError as exc:
            raise Refused(str(exc)) from exc
        self.session.wait(self.session.execute(plan)['operation_id'], timeout=float(plan['times'][-1]) + 15)
        return plan

    def __exit__(self, *exc):
        self.teaching.clear()
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1)
        try:
            self.session.command('hold')
        except Exception:
            pass  # Releasing the lease also makes the kernel hold.
        self.session.close()
