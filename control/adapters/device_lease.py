"""Same cooperative lock key as QuestArmTeleop's legacy DeviceLease.

Kept dependency-free so the control distribution also works on its own.
"""
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile


class DeviceLease:
    def __init__(self, interface):
        key = hashlib.sha256(interface.encode()).hexdigest()[:16]
        self.path = Path(tempfile.gettempdir()) / f"quest-x5-{os.getuid()}-{key}.lock"
        self.stream = self.path.open('a+')
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.close()
            raise RuntimeError(f'{interface} is owned by another teleop/calibration process') from None

    def close(self):
        self.stream.close()
