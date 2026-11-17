"""Workbench source lifecycles. CLI leaves authority available to motion tools."""
import time
from embodied.contracts.runtime.control import ControlSession


class IdleSource:
    def __init__(self, directory):
        self.directory = directory

    def run(self, duration=0):
        deadline = time.monotonic() + duration if duration else float('inf')
        while time.monotonic() < deadline:
            time.sleep(.1)


class NativeSession(IdleSource):
    """Lease keeper for an explicitly connected ingress; it produces no XR data."""
    def __init__(self, directory, ingress='native', source='quest'):
        super().__init__(directory)
        self.ingress, self.source = ingress, source

    def run(self, duration=0):
        # A lease needs a settled robot. Right after a kernel start the settle
        # window has not filled yet (and a slower vendor reports later), so
        # keep asking instead of giving up on the first refusal.
        deadline = time.monotonic() + duration if duration else float('inf')
        retry_until = time.monotonic() + 30
        while True:
            try:
                with ControlSession(self.directory, source=self.source, ingress=self.ingress,
                                    purpose='teleop') as session:
                    while time.monotonic() < deadline:
                        if session.error:
                            raise RuntimeError(session.error)
                        time.sleep(.05)
                return
            except RuntimeError as exc:
                if 'has not settled' not in str(exc) or time.monotonic() >= retry_until:
                    raise
                time.sleep(.2)


QuestSource = NativeSession  # Compatibility import; registry uses the explicit role.
