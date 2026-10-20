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
        with ControlSession(self.directory, source=self.source, ingress=self.ingress, purpose='teleop') as session:
            deadline = time.monotonic() + duration if duration else float('inf')
            while time.monotonic() < deadline:
                if session.error:
                    raise RuntimeError(session.error)
                time.sleep(.05)


QuestSource = NativeSession  # Compatibility import; registry uses the explicit role.
