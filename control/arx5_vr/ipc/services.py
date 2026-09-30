"""Small iceoryx2 helpers with the native bus capacity contract."""
from __future__ import annotations

import ctypes
import re
import time

STATE_BUFFER = 1024
STATE_MAX_PUBLISHERS = STATE_MAX_SUBSCRIBERS = STATE_MAX_NODES = 16


def validate_prefix(prefix: str) -> str:
    """Restrict each run to a single explicit, nonglobal namespace."""
    if not re.fullmatch(r"arx5-[A-Za-z0-9_.-]{1,96}", prefix):
        raise ValueError("IPC prefix must start with arx5- and contain only letters, digits, _, . or -")
    return prefix


def create_node(name: str | None = None):
    import iceoryx2 as iox2
    builder = iox2.NodeBuilder.new()
    if name:
        builder = builder.name(iox2.NodeName.new(name))
    return builder.create(iox2.ServiceType.Ipc)


def open_service(node, name: str, payload_type: type, *, mode: bool = False):
    import iceoryx2 as iox2
    builder = (node.service_builder(iox2.ServiceName.new(name))
               .publish_subscribe(payload_type)
               .max_publishers(STATE_MAX_PUBLISHERS)
               .max_subscribers(STATE_MAX_SUBSCRIBERS)
               .max_nodes(STATE_MAX_NODES))
    if not mode:
        builder = builder.subscriber_max_buffer_size(STATE_BUFFER).history_size(STATE_BUFFER)
    for attempt in range(201):
        try:
            return builder.open_or_create()
        except iox2.PublishSubscribeOpenOrCreateError as exc:
            if str(exc) != "SystemInFlux" or attempt == 200:
                raise
            time.sleep(.01)


def make_publisher(service):
    return service.publisher_builder().create()


def make_subscriber(service):
    return service.subscriber_builder().create()


def send(publisher, payload: ctypes.Structure) -> None:
    sample = publisher.loan_uninit()
    sample = sample.write_payload(payload)
    sample.send()


def drain_latest(subscriber):
    """Copy the newest sample so its lifetime exceeds the loan."""
    latest = None
    while True:
        sample = subscriber.receive()
        if sample is None:
            return latest
        value = sample.payload().contents
        latest = type(value)()
        ctypes.memmove(ctypes.byref(latest), ctypes.byref(value), ctypes.sizeof(value))
