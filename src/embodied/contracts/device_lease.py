"""Cooperative CAN ownership for teleop and calibration using the shared adapter."""

from functools import wraps
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


def leased_operation(device):
    """Exclusive ownership for a finite, direct motor maintenance operation."""
    def decorate(method):
        @wraps(method)
        def run(*args, **kwargs):
            lease = DeviceLease(device())
            try:
                return method(*args, **kwargs)
            finally:
                lease.close()
        return run
    return decorate


def leased_connect(device):
    def decorate(method):
        @wraps(method)
        def connect(self, *args, **kwargs):
            if getattr(self, '_component_device_lease', None) is not None:
                return method(self, *args, **kwargs)
            name = device(self)
            if name is None:
                return method(self, *args, **kwargs)
            lease = DeviceLease(name)
            try:
                result = method(self, *args, **kwargs)
            except BaseException:
                lease.close()
                raise
            self._component_device_lease = lease
            return result
        return connect
    return decorate


def leased_disconnect(method):
    @wraps(method)
    def disconnect(self, *args, **kwargs):
        result = method(self, *args, **kwargs)
        lease = getattr(self, '_component_device_lease', None)
        if lease is not None:
            lease.close()
            self._component_device_lease = None
        return result
    return disconnect
