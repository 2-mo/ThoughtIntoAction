"""Cooperative CAN ownership for teleop and calibration using the shared adapter."""

import fcntl
import hashlib
import os
from pathlib import Path
import tempfile


class DeviceLease:
    def __init__(self, device):
        key = hashlib.sha256(device.encode()).hexdigest()[:16]
        self.path = Path(tempfile.gettempdir()) / f"quest-x5-{os.getuid()}-{key}.lock"
        self.stream = self.path.open("a+")
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.close()
            raise RuntimeError(f"{device} is owned by another teleop/calibration process") from None

    def close(self):
        if not self.stream.closed:
            fcntl.flock(self.stream, fcntl.LOCK_UN)
            self.stream.close()

    def __del__(self):
        if hasattr(self, "stream"):
            self.close()
