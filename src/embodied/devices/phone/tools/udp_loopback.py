#!/usr/bin/env python3
"""Run a finite local protocol Ping/Pong RTT and clock-offset test."""

from __future__ import annotations

import argparse
import socket
import threading
import time
from collections.abc import Sequence

from embodied.devices.phone.inputs.arkit import ArkitInputSource
from embodied.devices.phone.protocol.clock_sync import ClockSyncEstimator, clock_sample
from embodied.devices.phone.protocol.packet import (
    PingPacket,
    PongPacket,
    SourceType,
    decode_packet,
    encode_packet,
)
from embodied.devices.phone.transports.udp import UdpTransport, UdpTransportConfig

from embodied.devices.phone.tools._common import (
    EventCounts,
    distribution_with_count,
    print_report,
    validate_nonnegative,
    validate_port,
    validate_positive,
)


def run_loopback(
    *,
    bind_host: str,
    port: int,
    count: int,
    interval_ms: float,
    timeout_ms: float,
) -> dict[str, object]:
    if not bind_host.strip():
        raise ValueError("bind_host must not be blank")
    validate_port(port, allow_zero=True)
    if isinstance(count, bool) or count <= 0:
        raise ValueError("count must be positive")
    validate_nonnegative("interval_ms", interval_ms)
    validate_positive("timeout_ms", timeout_ms)

    events = EventCounts()
    transport = UdpTransport(
        UdpTransportConfig(
            bind_host=bind_host,
            bind_port=port,
            receive_timeout_s=min(timeout_ms / 1_000.0, 0.1),
        ),
        event_callback=events,
    )
    source = ArkitInputSource(transport, event_callback=events)
    stop_event = threading.Event()
    source.start()

    def serve() -> None:
        while not stop_event.is_set():
            source.latest()

    thread = threading.Thread(target=serve, name="udp-loopback-server")
    thread.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(timeout_ms / 1_000.0)
    estimator = ClockSyncEstimator(
        max_rtt_ns=round(timeout_ms * 1_000_000.0),
        sample_capacity=max(count, 8),
        low_rtt_sample_count=min(count, 8),
    )
    rtt_ms: list[float] = []
    received = 0
    address = transport.local_address
    try:
        for sequence in range(count):
            t0 = time.monotonic_ns()
            client.sendto(
                encode_packet(
                    PingPacket(
                        source_type=SourceType.ARKIT,
                        sequence=sequence,
                        session_id=1,
                        client_time_ns=t0,
                    )
                ),
                address,
            )
            try:
                payload = client.recvfrom(512)[0]
            except TimeoutError:
                continue
            t3 = time.monotonic_ns()
            decoded = decode_packet(payload, expected_source=SourceType.ARKIT)
            if not isinstance(decoded, PongPacket):
                continue
            sample = clock_sample(
                decoded.client_time_ns,
                decoded.server_receive_time_ns,
                decoded.server_send_time_ns,
                t3,
            )
            if estimator.add_exchange(
                sample.client_send_time_ns,
                sample.server_receive_time_ns,
                sample.server_send_time_ns,
                sample.client_receive_time_ns,
            ):
                rtt_ms.append(sample.rtt_ns / 1_000_000.0)
                received += 1
            if interval_ms > 0.0:
                time.sleep(interval_ms / 1_000.0)
    finally:
        client.close()
        stop_event.set()
        source.stop()
        thread.join(1.0)
    if thread.is_alive():
        raise RuntimeError("loopback server did not stop")
    return {
        "sent": count,
        "received": received,
        "lost": count - received,
        "rtt_ms": distribution_with_count(rtt_ms),
        "clock_offset_ns": estimator.offset_ns,
        "minimum_rtt_ns": estimator.minimum_rtt_ns,
        "events": events.snapshot(),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--interval-ms", type=float, default=10.0)
    parser.add_argument("--timeout-ms", type=float, default=100.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run_loopback(
        bind_host=args.bind_host,
        port=args.port,
        count=args.count,
        interval_ms=args.interval_ms,
        timeout_ms=args.timeout_ms,
    )
    print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
