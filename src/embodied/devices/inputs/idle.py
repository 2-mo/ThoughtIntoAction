"""Workbench source lifecycles. CLI leaves authority available to motion tools."""
import time
import json
from pathlib import Path
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
        deadline = time.monotonic() + duration if duration else float('inf')
        session = None
        request = Path(self.directory) / 'native-source.json'
        try:
            while time.monotonic() < deadline:
                enabled = json.loads(request.read_text())['enabled'] if request.exists() else True
                if enabled and session is None:
                    session = ControlSession(self.directory, source=self.source, ingress=self.ingress, purpose='teleop')
                    session.__enter__()
                elif not enabled and session is not None:
                    session.close()
                    session = None
                if session is not None and session.error:
                    raise RuntimeError(session.error)
                time.sleep(.05)
        finally:
            if session is not None:
                session.close()


QuestSource = NativeSession  # Compatibility import; registry uses the explicit role.
