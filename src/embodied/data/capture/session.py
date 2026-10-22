"""A bounded collector context; robot control remains in the calling task thread."""
import threading
import math

from embodied.contracts.runtime import RuntimeReader
from embodied.data.capture.capture import capture


class CaptureSession:
    def __init__(self, run, *, output, task, cameras, demonstration=None, camera_roles=None,
                 anchor=None, video=True, ready_timeout=10., reader=None):
        if not math.isfinite(ready_timeout) or ready_timeout <= 0:
            raise ValueError('ready_timeout must be finite and positive')
        self.run, self.reader = run, reader
        self.options = dict(root=output, task=task, duration=0, cameras=cameras, video=video,
                            demonstration=demonstration, camera_roles=camera_roles, anchor=anchor)
        self.ready_timeout = ready_timeout
        self.stop_event, self.ready, self.finished = threading.Event(), threading.Event(), threading.Event()
        self.path = self.manifest = self.error = self.writer = None
        self.thread = None

    def _created(self, writer):
        self.writer, self.path = writer, writer.path

    def _run(self):
        try:
            self.path, self.manifest = capture(self.reader, **self.options, stop_event=self.stop_event,
                                               ready=self.ready, on_created=self._created)
        except BaseException as exc:
            self.error = exc
        finally:
            self.finished.set()
            self.ready.set()

    def __enter__(self):
        if self.thread is not None:
            raise RuntimeError('Capture contexts cannot be reused')
        self.reader = self.reader or RuntimeReader(self.run, mode='full')
        self.thread = threading.Thread(target=self._run, name='episode-collector', daemon=True)
        self.thread.start()
        try:
            if not self.ready.wait(self.ready_timeout):
                raise TimeoutError('Recording did not receive robot feedback and two frames from every camera')
            self.check()
            if self.finished.is_set():
                raise RuntimeError('Recording ended before the task could start')
            return self
        except BaseException:
            self.stop()
            raise

    def check(self):
        if self.error is not None:
            raise RuntimeError(f'Capture failed: {self.error}') from self.error
        if self.manifest is not None and self.manifest['lifecycle'] != 'complete':
            raise RuntimeError(f"Capture failed: {self.manifest.get('error')}; retained at {self.path}")
        if self.manifest is not None and self.manifest.get('stop_reason') != 'context-exit':
            raise RuntimeError('Recording ended independently of the task context: ' + str(self.manifest.get('stop_reason')))

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=10)
            if self.thread.is_alive():
                self.reader.close()
                self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError('Collector is still finalizing; episode is not committed')
        if self.reader is not None:
            self.reader.close()

    def __exit__(self, exc_type, exc, traceback):
        self.stop()
        if exc is None:
            self.check()
        return False
