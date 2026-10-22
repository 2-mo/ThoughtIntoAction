"""Command client with explicit leases. Does not import a robot or VR SDK."""
import json
import os
from pathlib import Path
import threading
import time

from embodied.contracts.rpc import request


def live_runs(root):
    """Kernel run directories under `root` (outputs/runtime/control) of this boot whose supervisor is alive, oldest first."""
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    runs = []
    for path in sorted(Path(root).glob('*/commands.sock')):
        try:
            identity = json.loads((path.parent / 'runtime.json').read_text())
            os.kill(identity['supervisor_pid'], 0)
        except (OSError, ValueError, KeyError):
            continue
        if identity.get('boot_id') == boot and (path.parent / 'runtime.ready').exists():
            runs.append(path.parent)
    return runs


class ControlClient:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.identity = json.loads((self.directory / 'runtime.json').read_text())
        if self.identity['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            raise ValueError('Runtime belongs to an earlier host boot')
        self.parameters = json.loads((self.directory / 'parameters.json').read_text())

    def request(self, action, **fields):
        return request(self.directory / 'commands.sock', {'version': 1,
            'runtime_id': self.identity['runtime_id'], 'action': action,
            'sent_monotonic_ns': time.monotonic_ns(), **fields})

    def status(self):
        status = self.request('status')
        age = time.monotonic_ns() - status['feedback_monotonic_ns']
        if not 0 <= age < 150_000_000:
            raise RuntimeError('Runtime feedback is stale')
        return status

    def describe(self):
        """Robot limits, model identity and workspace from the running kernel; no lease."""
        return self.request('describe')

    def preview(self, kind='movej', **fields):
        return self.request('preview', kind=kind, **fields)


class ControlSession(ControlClient):
    def __init__(self, directory, source='cli', metadata=None, *, ingress='rpc', purpose='operation'):
        super().__init__(directory)
        self.source = source
        self.ingress, self.purpose = ingress, purpose
        self.metadata = metadata or {}
        self.lease = None
        self.sequence = 0
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.error = None
        self.thread = None

    def __enter__(self):
        self.lease = super().request('acquire', source=self.source, metadata=self.metadata,
                                             ingress=self.ingress, purpose=self.purpose)
        self.thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.thread.start()
        return self

    def command(self, action, **fields):
        with self.lock:
            if self.error:
                raise RuntimeError(f'Control lease lost: {self.error}')
            if self.lease is None:
                raise RuntimeError('Acquire control first')
            self.sequence += 1
            result = super().request(action, token=self.lease['token'], epoch=self.lease['epoch'],
                                     sequence=self.sequence, **fields)
            if action in ('simulation-restore', 'simulation-reset'):
                # Keep heartbeats on the epoch returned by this discontinuity.
                self.lease['epoch'] = result['epoch']
            return result

    def _heartbeat(self):
        while not self.stop_event.wait(.1):
            try:
                self.command('heartbeat')
            except Exception as exc:
                self.error = str(exc)
                return

    def execute(self, plan):
        return self.command('execute', plan_id=plan['plan_id'])

    def wait(self, operation_id, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for op in self.status()['operations']:
                if op['operation_id'] == operation_id and op['status'] != 'running':
                    if op['status'] != 'completed':
                        raise RuntimeError(f'Operation {op["status"]}: {op.get("reason")}')
                    return op
            time.sleep(.02)
        self.command('cancel')
        raise TimeoutError('Operation timed out')

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)
        if self.lease:
            try:
                self.command('release')
            except (OSError, ValueError, RuntimeError, EOFError):
                pass  # Worker lease expiry independently enters hold.
            self.lease = None

    def __exit__(self, *exc):
        self.close()
