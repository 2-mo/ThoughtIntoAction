"""Original native health ABI plus a separate supervisor dependency heartbeat."""
import ctypes as C
import time
from arx5_vr.ipc.services import create_node, open_service, make_publisher, send


class TeleopGate(C.Structure):
    _fields_ = [('timestamp_us', C.c_uint64), ('sequence', C.c_uint64),
                ('engaged', C.c_uint8), ('tracking_valid', C.c_uint8),
                ('filter_valid', C.c_uint8), ('action', C.c_int32), ('reserved', C.c_uint8 * 4)]

    @classmethod
    def type_name(cls):
        return 'TeleopGate'


class TeleopRouteHealth(C.Structure):
    _fields_ = [('timestamp_us', C.c_uint64), ('sequence', C.c_uint64),
                ('state', C.c_uint8), ('reserved', C.c_uint8 * 7)]

    @classmethod
    def type_name(cls):
        return 'TeleopRouteHealth'


class DependencyHealth(C.Structure):
    _fields_ = [('monotonic_ns', C.c_uint64), ('sequence', C.c_uint64),
                ('failed_mask', C.c_uint64)]

    @classmethod
    def type_name(cls):
        return 'QuestArmDependencyHealthV1'


class DependencyPublisher:
    def __init__(self, prefix, dependencies):
        self.dependencies = dependencies
        self.node = create_node()
        self.publisher = make_publisher(open_service(self.node, f'{prefix}/managed/dependencies', DependencyHealth))
        self.sequence = 0

    def publish(self, processes):
        self.sequence += 1
        mask = sum(1 << i for i, name in enumerate(self.dependencies)
                   if name not in processes or processes[name].poll() is not None)
        send(self.publisher, DependencyHealth(time.monotonic_ns(), self.sequence, mask))

    def close(self):
        self.publisher = self.node = None

