import socket
import threading
import time

import pytest

from embodied.devices.phone.transports.udp import (
    UdpTransport,
    UdpTransportConfig,
    UdpTransportStateError,
)


def local_transport(
    **changes: object,
) -> UdpTransport:
    values: dict[str, object] = {
        "bind_host": "127.0.0.1",
        "bind_port": 0,
        "receive_timeout_s": 0.05,
        "max_drain_datagrams": 64,
    }
    values.update(changes)
    config = UdpTransportConfig(**values)
    return UdpTransport(config)


def test_udp_receives_and_replies_to_last_peer() -> None:
    transport = local_transport()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(0.5)
    transport.start()
    try:
        client.sendto(b"pose", transport.local_address)

        assert transport.receive() == b"pose"
        assert transport.last_peer is not None
        transport.send(b"pong")
        assert client.recvfrom(64)[0] == b"pong"
    finally:
        transport.stop()
        client.close()


def test_udp_receive_drains_burst_and_returns_newest() -> None:
    events: dict[str, int] = {}

    def record(event: str, count: int) -> None:
        events[event] = events.get(event, 0) + count

    transport = UdpTransport(
        UdpTransportConfig(
            bind_host="127.0.0.1",
            bind_port=0,
            receive_timeout_s=0.1,
            max_drain_datagrams=128,
            receive_buffer_bytes=1 << 20,
        ),
        event_callback=record,
    )
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    transport.start()
    try:
        for sequence in range(50):
            client.sendto(sequence.to_bytes(4, "little"), transport.local_address)

        assert transport.receive() == (49).to_bytes(4, "little")
        assert events["udp.datagram"] == 50
        assert events["udp.superseded"] == 49
    finally:
        transport.stop()
        client.close()


def test_udp_receive_batch_preserves_bounded_datagrams() -> None:
    events: dict[str, int] = {}

    def record(event: str, count: int) -> None:
        events[event] = events.get(event, 0) + count

    transport = UdpTransport(
        UdpTransportConfig(
            bind_host="127.0.0.1",
            bind_port=0,
            receive_timeout_s=0.1,
            max_drain_datagrams=8,
        ),
        event_callback=record,
    )
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    transport.start()
    try:
        for payload in (b"pose-0", b"settings", b"pose-1"):
            client.sendto(payload, transport.local_address)

        assert transport.receive_batch() == (b"pose-0", b"settings", b"pose-1")
        assert events == {"udp.datagram": 3}
    finally:
        transport.stop()
        client.close()


def test_single_datagram_drain_does_not_wait_for_receive_timeout() -> None:
    transport = local_transport(receive_timeout_s=1.0)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    transport.start()
    try:
        client.sendto(b"ready", transport.local_address)
        started_at = time.monotonic()

        assert transport.receive() == b"ready"
        assert time.monotonic() - started_at < 0.25
    finally:
        transport.stop()
        client.close()


def test_udp_timeout_and_idempotent_restart() -> None:
    transport = local_transport()

    transport.start()
    first_address = transport.local_address
    transport.start()
    assert transport.receive() is None
    transport.stop()
    transport.stop()
    transport.start()
    try:
        assert transport.local_address[1] > 0
        assert first_address[1] > 0
    finally:
        transport.stop()


def test_udp_stop_unblocks_a_blocking_receive() -> None:
    transport = local_transport(receive_timeout_s=1.0)
    result: list[bytes | None] = []
    entered = threading.Event()

    def receive() -> None:
        entered.set()
        result.append(transport.receive())

    transport.start()
    thread = threading.Thread(target=receive)
    thread.start()
    assert entered.wait(0.5)
    time.sleep(0.01)

    transport.stop()
    thread.join(0.5)

    assert not thread.is_alive()
    assert result == [None]


def test_udp_send_requires_a_running_transport_and_destination() -> None:
    transport = local_transport()
    with pytest.raises(UdpTransportStateError, match="running"):
        transport.send(b"early")

    transport.start()
    try:
        with pytest.raises(UdpTransportStateError, match="peer"):
            transport.send(b"no destination")
    finally:
        transport.stop()


@pytest.mark.parametrize(
    "config",
    [
        UdpTransportConfig(bind_host="127.0.0.1", bind_port=0),
        UdpTransportConfig(
            bind_host="127.0.0.1",
            bind_port=0,
            remote_host="127.0.0.1",
            remote_port=5005,
        ),
    ],
)
def test_udp_config_accepts_server_and_fixed_peer_modes(
    config: UdpTransportConfig,
) -> None:
    assert config.bind_port == 0


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"bind_host": ""}, "bind_host"),
        ({"bind_port": -1}, "bind_port"),
        ({"bind_port": 65_536}, "bind_port"),
        ({"remote_host": "127.0.0.1"}, "configured together"),
        ({"remote_port": 5005}, "configured together"),
        ({"receive_timeout_s": 0.0}, "receive_timeout_s"),
        ({"max_datagram_size": 0}, "max_datagram_size"),
        ({"max_drain_datagrams": -1}, "max_drain_datagrams"),
        ({"receive_buffer_bytes": 0}, "receive_buffer_bytes"),
    ],
)
def test_udp_config_rejects_invalid_values(
    changes: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        UdpTransportConfig(**changes)
