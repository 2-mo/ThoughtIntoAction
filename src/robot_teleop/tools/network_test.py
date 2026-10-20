#!/usr/bin/env python3
"""Exercise local UDP loss, reorder, jitter, and latest-only behavior."""

from __future__ import annotations

import argparse
import random
import socket
import threading
import time
from collections.abc import Sequence

from robot_teleop.core.models import Pose, Quaternion, TrackingQuality, Vector3
from robot_teleop.inputs.arkit import ArkitInputSource
from robot_teleop.protocol.packet import PosePacket, SourceType, encode_packet
from robot_teleop.transports.udp import UdpTransport, UdpTransportConfig

from robot_teleop.tools._common import (
    EventCounts,
    print_report,
    validate_nonnegative,
    validate_port,
    validate_positive,
    validate_probability,
)


def _packet(sequence: int) -> bytes:
    return encode_packet(
        PosePacket(
            source_type=SourceType.ARKIT,
            sequence=sequence,
            session_id=1,
            client_time_ns=(sequence + 1) * 1_000,
            pose=Pose(
                Vector3(sequence * 0.00001, 0.0, 0.0),
                Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            linear_velocity=None,
            angular_velocity=None,
            connected=True,
            clutch_active=False,
            recenter_requested=False,
            estop_requested=False,
            gripper=None,
            tracking_quality=TrackingQuality.NORMAL,
        )
    )


def run_network_test(
    *,
    bind_host: str,
    port: int,
    rate_hz: float,
    duration_s: float,
    loss_rate: float,
    reorder_rate: float,
    jitter_ms: float,
    seed: int,
) -> dict[str, object]:
    if not bind_host.strip():
        raise ValueError("bind_host must not be blank")
    validate_port(port, allow_zero=True)
    validate_positive("rate_hz", rate_hz)
    validate_positive("duration_s", duration_s)
    validate_probability("loss_rate", loss_rate)
    validate_probability("reorder_rate", reorder_rate)
    validate_nonnegative("jitter_ms", jitter_ms)

    events = EventCounts()
    transport = UdpTransport(
        UdpTransportConfig(
            bind_host=bind_host,
            bind_port=port,
            receive_timeout_s=0.02,
            max_drain_datagrams=64,
            receive_buffer_bytes=1 << 20,
        ),
        event_callback=events,
    )
    source = ArkitInputSource(transport, event_callback=events)
    source.start()
    received_sequences: list[int] = []
    stop_event = threading.Event()

    def receive() -> None:
        while not stop_event.is_set():
            input_state = source.latest()
            if input_state is not None:
                received_sequences.append(input_state.sequence)

    thread = threading.Thread(target=receive, name="network-test-receiver")
    thread.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = transport.local_address
    random_source = random.Random(seed)
    attempted = max(1, round(duration_s * rate_hz))
    sent_sequences: list[int] = []
    simulated_lost = 0
    simulated_reordered = 0
    pending: tuple[int, bytes] | None = None
    started_at = time.monotonic()
    try:
        for sequence in range(attempted):
            deadline = started_at + sequence / rate_hz
            deadline += random_source.uniform(0.0, jitter_ms / 1_000.0)
            delay = deadline - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            payload = _packet(sequence)
            if random_source.random() < loss_rate:
                simulated_lost += 1
                continue
            if pending is None and random_source.random() < reorder_rate:
                pending = (sequence, payload)
                simulated_reordered += 1
                continue
            sender.sendto(payload, destination)
            sent_sequences.append(sequence)
            if pending is not None:
                sender.sendto(pending[1], destination)
                sent_sequences.append(pending[0])
                pending = None
        if pending is not None:
            sender.sendto(pending[1], destination)
            sent_sequences.append(pending[0])
        send_finished_at = time.monotonic()
        time.sleep(0.05)
    finally:
        sender.close()
        stop_event.set()
        source.stop()
        thread.join(1.0)
    if thread.is_alive():
        raise RuntimeError("network test receiver did not stop")
    send_duration_s = max(send_finished_at - started_at, 1e-9)
    total_duration_s = max(time.monotonic() - started_at, 1e-9)
    expected_latest = max(sent_sequences, default=None)
    last_sequence = received_sequences[-1] if received_sequences else None
    return {
        "attempted": attempted,
        "sent": len(sent_sequences),
        "received": len(received_sequences),
        "simulated_lost": simulated_lost,
        "simulated_reordered": simulated_reordered,
        "last_sequence": last_sequence,
        "expected_latest_sequence": expected_latest,
        "latest_reached": last_sequence == expected_latest,
        "send_fps": len(sent_sequences) / send_duration_s,
        "receive_fps": len(received_sequences) / total_duration_s,
        "send_duration_s": send_duration_s,
        "total_duration_s": total_duration_s,
        "events": events.snapshot(),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=120.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--loss-rate", type=float, default=0.0)
    parser.add_argument("--reorder-rate", type=float, default=0.0)
    parser.add_argument("--jitter-ms", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=1)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run_network_test(
        bind_host=args.bind_host,
        port=args.port,
        rate_hz=args.rate_hz,
        duration_s=args.duration,
        loss_rate=args.loss_rate,
        reorder_rate=args.reorder_rate,
        jitter_ms=args.jitter_ms,
        seed=args.seed,
    )
    print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
