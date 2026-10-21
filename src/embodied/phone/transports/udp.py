"""Bounded, latest-only IPv4 UDP byte transport."""

from __future__ import annotations

import math
import socket
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import cast

type EventCallback = Callable[[str, int], None]
type SocketAddress = tuple[str, int]


class UdpTransportStateError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class UdpTransportConfig:
    bind_host: str = "0.0.0.0"
    bind_port: int = 5005
    remote_host: str | None = None
    remote_port: int | None = None
    receive_timeout_s: float = 0.1
    max_datagram_size: int = 2_048
    max_drain_datagrams: int = 64
    receive_buffer_bytes: int = 262_144

    def __post_init__(self) -> None:
        if not self.bind_host.strip():
            raise ValueError("bind_host must not be blank")
        if (
            isinstance(self.bind_port, bool)
            or not isinstance(self.bind_port, int)
            or not 0 <= self.bind_port <= 65_535
        ):
            raise ValueError("bind_port must be in [0, 65535]")
        if (self.remote_host is None) != (self.remote_port is None):
            raise ValueError("remote_host and remote_port must be configured together")
        if self.remote_host is not None and not self.remote_host.strip():
            raise ValueError("remote_host must not be blank")
        if self.remote_port is not None and (
            isinstance(self.remote_port, bool)
            or not isinstance(self.remote_port, int)
            or not 1 <= self.remote_port <= 65_535
        ):
            raise ValueError("remote_port must be in [1, 65535]")
        if not math.isfinite(self.receive_timeout_s) or self.receive_timeout_s <= 0.0:
            raise ValueError("receive_timeout_s must be positive and finite")
        for name, value, allow_zero in (
            ("max_datagram_size", self.max_datagram_size, False),
            ("max_drain_datagrams", self.max_drain_datagrams, True),
            ("receive_buffer_bytes", self.receive_buffer_bytes, False),
        ):
            minimum = 0 if allow_zero else 1
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise ValueError(f"{name} must be at least {minimum}")


class UdpTransport:
    def __init__(
        self,
        config: UdpTransportConfig | None = None,
        *,
        event_callback: EventCallback | None = None,
    ) -> None:
        self._config = config or UdpTransportConfig()
        self._event_callback = event_callback
        self._lock = threading.Lock()
        self._receive_lock = threading.Lock()
        self._socket: socket.socket | None = None
        self._local_address: SocketAddress | None = None
        self._configured_peer: SocketAddress | None = None
        self._last_peer: SocketAddress | None = None

    @property
    def local_address(self) -> SocketAddress:
        with self._lock:
            if self._local_address is None:
                raise UdpTransportStateError("UDP transport is not running")
            return self._local_address

    @property
    def last_peer(self) -> SocketAddress | None:
        with self._lock:
            return self._last_peer

    def _emit(self, event: str, count: int = 1) -> None:
        callback = self._event_callback
        if callback is not None and count > 0:
            callback(event, count)

    def start(self) -> None:
        with self._lock:
            if self._socket is not None:
                return
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                udp_socket.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_RCVBUF,
                    self._config.receive_buffer_bytes,
                )
                udp_socket.settimeout(self._config.receive_timeout_s)
                udp_socket.bind((self._config.bind_host, self._config.bind_port))
                local_address = cast(SocketAddress, udp_socket.getsockname())
                configured_peer: SocketAddress | None = None
                if self._config.remote_host is not None:
                    assert self._config.remote_port is not None
                    resolved = socket.getaddrinfo(
                        self._config.remote_host,
                        self._config.remote_port,
                        socket.AF_INET,
                        socket.SOCK_DGRAM,
                    )
                    configured_peer = cast(SocketAddress, resolved[0][4])
            except Exception:
                udp_socket.close()
                raise
            self._socket = udp_socket
            self._local_address = local_address
            self._configured_peer = configured_peer
            self._last_peer = None

    def _active(self, udp_socket: socket.socket) -> bool:
        with self._lock:
            return self._socket is udp_socket

    def receive(self) -> bytes | None:
        datagrams = self.receive_batch()
        if not datagrams:
            return None
        self._emit("udp.superseded", len(datagrams) - 1)
        return datagrams[-1]

    def receive_batch(self) -> tuple[bytes, ...]:
        with self._receive_lock:
            with self._lock:
                udp_socket = self._socket
            if udp_socket is None:
                return ()
            try:
                first, first_peer = udp_socket.recvfrom(self._config.max_datagram_size)
            except TimeoutError:
                return ()
            except OSError:
                if not self._active(udp_socket):
                    return ()
                raise
            if not self._active(udp_socket):
                return ()

            datagrams = [(bytes(first), cast(SocketAddress, first_peer))]
            udp_socket.setblocking(False)
            try:
                for _ in range(self._config.max_drain_datagrams):
                    try:
                        candidate, candidate_peer = udp_socket.recvfrom(
                            self._config.max_datagram_size
                        )
                    except BlockingIOError:
                        break
                    except OSError:
                        if not self._active(udp_socket):
                            return ()
                        raise
                    datagrams.append(
                        (bytes(candidate), cast(SocketAddress, candidate_peer))
                    )
            finally:
                udp_socket.settimeout(self._config.receive_timeout_s)

            peer = datagrams[-1][1]
            with self._lock:
                if self._socket is not udp_socket:
                    return ()
                self._last_peer = peer
            self._emit("udp.datagram", len(datagrams))
            return tuple(payload for payload, _ in datagrams)

    def send(self, payload: bytes) -> None:
        if not isinstance(payload, bytes):
            raise TypeError("UDP payload must be bytes")
        with self._lock:
            udp_socket = self._socket
            peer = self._configured_peer or self._last_peer
        if udp_socket is None:
            raise UdpTransportStateError("UDP transport is not running")
        if peer is None:
            raise UdpTransportStateError("UDP peer is not known")
        try:
            udp_socket.sendto(payload, peer)
        except OSError:
            if not self._active(udp_socket):
                raise UdpTransportStateError(
                    "UDP transport stopped during send"
                ) from None
            raise
        self._emit("udp.sent")

    def stop(self) -> None:
        with self._lock:
            udp_socket = self._socket
            local_address = self._local_address
            self._socket = None
            self._local_address = None
            self._configured_peer = None
            self._last_peer = None
        if udp_socket is None:
            return

        if local_address is not None:
            wake_host = local_address[0]
            if wake_host == "0.0.0.0":
                wake_host = "127.0.0.1"
            wake_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                wake_socket.sendto(b"", (wake_host, local_address[1]))
            except OSError:
                pass
            finally:
                wake_socket.close()
        with self._receive_lock:
            udp_socket.close()
