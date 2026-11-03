"""Latest-only, read-only feedback for independent displays; never opens devices."""
import json
import os
from pathlib import Path
import threading
import time
import uuid

from embodied.core.project import PROJECT_ROOT

STATE_PATH = PROJECT_ROOT / 'outputs/runtime/realman/state.json'


class StateMirror:
    def __init__(self, path=STATE_PATH):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream_id = uuid.uuid4().hex
        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.error = None
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._write, name='realman-display', daemon=True)
        self.thread.start()

    def update(self, controls, *, fresh, timestamp, paused, reason):
        arms = []
        for c in controls:
            b = c.backend
            arms.append({'hand': c.hand, 'host': c.config['host'], 'q': b.state.q.tolist(),
                'target_q_deg': b.last_joint_target.tolist() if b.last_joint_target is not None else None,
                'feedback_at': b.feedback_at,
                'fault': b.state.fault, 'following': c.gate.following,
                'arm_blocked_reason': getattr(c, 'blocked_reason', None),
                'grip': c.grip, 'open': getattr(c, 'open_down', False),
                'trigger': getattr(c, 'trigger', 0.),
                'gripper': getattr(c, 'latest_gripper', None),
                'linear_speed_m_s': c.config['linear_speed_m_s'],
                'angular_speed_rad_s': c.config['angular_speed_rad_s'],
                'joint_speed_deg_s': c.config.get('joint_speed_deg_s'),
                'command_diagnostics': b.command_diagnostics})
        with self.lock:
            self.sequence += 1
            self.latest = {'schema': 1, 'stream_id': self.stream_id, 'pid': os.getpid(),
                'boot_id': self.boot_id, 'sequence': self.sequence,
                'sample_monotonic_ns': time.monotonic_ns(), 'xr_fresh': fresh,
                'xr_timestamp_ns': timestamp, 'paused': paused, 'reason': reason, 'arms': arms}

    def _write(self):
        temporary = self.path.with_name(f'.state-{self.stream_id}.tmp')
        try:
            while not self.closed.is_set():
                with self.lock:
                    state = self.latest
                if state is not None:
                    temporary.write_text(json.dumps(state, allow_nan=False))
                    temporary.replace(self.path)
                self.closed.wait(.1)
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)
            print(f'RealMan display publication failed: {exc}', flush=True)
        finally:
            temporary.unlink(missing_ok=True)

    def close(self):
        self.closed.set()
        self.thread.join(timeout=2)


def latest_state(path=STATE_PATH):
    try:
        state = json.loads(Path(path).read_text())
        if (state['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip() or
                not 0 <= time.monotonic_ns() - state['sample_monotonic_ns'] < 2_000_000_000):
            return None
        os.kill(state['pid'], 0)
        return state
    except (OSError, ValueError, KeyError, TypeError):
        return None
