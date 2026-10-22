"""Collector-owned heartbeat, separate from immutable recording completion metadata."""
import os
from pathlib import Path
import threading
import time

from embodied.core.storage import save_json

INTERVAL_S = .25
STALE_AFTER_S = 2.


class CaptureStatus:
    def __init__(self, writer, frames, dropped):
        self.writer, self.frames, self.dropped = writer, frames, dropped
        self.state, self.error, self.reason = 'recording', None, None
        self.progress_ns = time.monotonic_ns()
        self.stopped = threading.Event()
        self.identity = {'pid': os.getpid(), 'process_start_ticks':
                         Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19]}
        self.thread = threading.Thread(target=self._run, name='capture-heartbeat', daemon=True)

    def payload(self):
        from embodied.data.episode.runtime_episode import frame_kind
        manifest = self.writer.manifest
        return {'schema': 'embodied.capture_status/1', 'schema_version': 1, **self.identity,
            'runtime_id': manifest['runtime']['runtime_id'], 'boot_id': manifest['runtime'].get('boot_id'),
            'host_monotonic_ns': time.monotonic_ns(), 'state': self.state,
            'started_monotonic_ns': manifest['started_monotonic_ns'],
            'episode': {'id': manifest['id'], 'path': str(self.writer.path.resolve()), 'task': manifest['task']},
            'counts': dict(self.writer.counts),
            'camera_counts': {key: self.writer.counts[frame_kind(key)] for key in self.writer.cameras},
            'queue_size': self.frames.qsize(), 'queue_capacity': self.frames.maxsize,
            'dropped': dict(self.dropped), 'error': self.error or self.writer.error,
            'stop_reason': self.reason, 'last_progress_monotonic_ns': self.progress_ns}

    def publish(self):
        save_json(self.writer.path / 'capture-status.json', self.payload())

    def start(self):
        self.publish()
        self.thread.start()

    def _run(self):
        while not self.stopped.wait(INTERVAL_S):
            try:
                self.publish()
            except Exception as exc:
                self.error = f'Capture heartbeat write failed: {exc}'
                return

    def close(self, manifest):
        self.stopped.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('Capture heartbeat writer did not stop')
        self.state, self.reason = manifest['lifecycle'], manifest['stop_reason']
        self.error = manifest.get('error') or self.error
        self.publish()

    def fail(self, error):
        """Stop heartbeat even when final manifest publication itself failed."""
        self.stopped.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2)
        self.state, self.error = 'failed', str(error)
        try:
            self.publish()
        except OSError:
            pass  # Original storage failure is raised by the collector.
