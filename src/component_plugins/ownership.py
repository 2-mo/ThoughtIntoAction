"""Share the existing arx worker's device lease with migrated device adapters."""
from functools import wraps
from robot_core.device_lease import DeviceLease


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
