"""Bounded in-memory transport for protocol-independent tests."""

import threading


class TransportStateError(RuntimeError):
    pass


class MockTransport:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._running = False
        self._received: bytes | None = None
        self._sent: bytes | None = None

    @property
    def latest_sent(self) -> bytes | None:
        with self._lock:
            return self._sent

    def start(self) -> None:
        with self._lock:
            self._running = True

    def inject_received(self, payload: bytes) -> None:
        with self._lock:
            if not self._running:
                raise TransportStateError("transport is not running")
            self._received = bytes(payload)

    def receive(self) -> bytes | None:
        with self._lock:
            if not self._running:
                return None
            payload = self._received
            self._received = None
            return payload

    def send(self, payload: bytes) -> None:
        with self._lock:
            if not self._running:
                raise TransportStateError("transport is not running")
            self._sent = bytes(payload)

    def stop(self) -> None:
        with self._lock:
            self._running = False
            self._received = None
