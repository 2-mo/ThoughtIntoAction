import pytest

from embodied.devices.phone.transports.mock import MockTransport, TransportStateError


def test_mock_transport_is_latest_only() -> None:
    transport = MockTransport()
    transport.start()
    transport.inject_received(b"old")
    transport.inject_received(b"new")

    assert transport.receive() == b"new"
    assert transport.receive() is None


def test_mock_transport_lifecycle_and_send() -> None:
    transport = MockTransport()
    with pytest.raises(TransportStateError):
        transport.send(b"early")

    transport.start()
    transport.start()
    transport.send(b"reply")
    assert transport.latest_sent == b"reply"
    transport.stop()
    transport.stop()
    assert transport.receive() is None
    with pytest.raises(TransportStateError):
        transport.inject_received(b"late")
