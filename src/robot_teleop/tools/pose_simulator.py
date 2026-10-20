#!/usr/bin/env python3
"""Send finite ARKit-format Pose traffic without enabling any robot hardware."""

from __future__ import annotations

import argparse
import math
import random
import secrets
import socket
import time
from collections.abc import Sequence
from dataclasses import dataclass, replace

from robot_teleop.core.models import Pose, Quaternion, TrackingQuality, Vector3
from robot_teleop.protocol.clock_sync import ClockSyncEstimator, clock_sample
from robot_teleop.protocol.packet import (
    PingPacket,
    PongPacket,
    PosePacket,
    SourceType,
    decode_packet,
    encode_packet,
)

from robot_teleop.tools._common import (
    distribution_with_count,
    print_report,
    validate_nonnegative,
    validate_port,
    validate_positive,
    validate_probability,
)


@dataclass(frozen=True, slots=True)
class PoseSimulatorConfig:
    host: str = "127.0.0.1"
    port: int = 5005
    rate_hz: float = 60.0
    duration_s: float = 5.0
    loss_rate: float = 0.0
    reorder_rate: float = 0.0
    jitter_ms: float = 0.0
    seed: int = 1
    session_id: int = 1
    clutch_active: bool = False
    position_amplitude_m: float = 0.01
    ping_interval_s: float = 1.0

    def __post_init__(self) -> None:
        if not self.host.strip():
            raise ValueError("host must not be blank")
        validate_port(self.port)
        validate_positive("rate_hz", self.rate_hz)
        validate_positive("duration_s", self.duration_s)
        validate_probability("loss_rate", self.loss_rate)
        validate_probability("reorder_rate", self.reorder_rate)
        validate_nonnegative("jitter_ms", self.jitter_ms)
        validate_nonnegative("position_amplitude_m", self.position_amplitude_m)
        validate_positive("ping_interval_s", self.ping_interval_s)
        if isinstance(self.session_id, bool) or not 1 <= self.session_id <= 0xFFFFFFFF:
            raise ValueError("session_id must be a nonzero UInt32")


def _pose(config: PoseSimulatorConfig, sequence: int, phase: float) -> PosePacket:
    return PosePacket(
        source_type=SourceType.ARKIT,
        sequence=sequence,
        session_id=config.session_id,
        client_time_ns=time.monotonic_ns(),
        pose=Pose(
            Vector3(config.position_amplitude_m * math.sin(phase), 0.0, 0.0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        linear_velocity=None,
        angular_velocity=None,
        connected=True,
        clutch_active=config.clutch_active,
        recenter_requested=False,
        estop_requested=False,
        gripper=0.5,
        tracking_quality=TrackingQuality.NORMAL,
    )


def run_simulator(config: PoseSimulatorConfig) -> dict[str, object]:
    random_source = random.Random(config.seed)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("0.0.0.0", 0))
    udp_socket.setblocking(False)
    destination = (config.host, config.port)
    sample_count = max(1, round(config.duration_s * config.rate_hz))
    estimator = ClockSyncEstimator(max_rtt_ns=500_000_000)
    rtt_ms: list[float] = []
    pending: bytes | None = None
    pose_sent = 0
    simulated_lost = 0
    simulated_reordered = 0
    ping_sequence = 0
    pings_sent = 0
    pongs_received = 0
    next_ping_s = 0.0
    started_at = time.monotonic()

    def drain_pongs() -> None:
        nonlocal pongs_received
        while True:
            try:
                payload = udp_socket.recvfrom(512)[0]
            except BlockingIOError:
                return
            received_at_ns = time.monotonic_ns()
            try:
                decoded = decode_packet(payload, expected_source=SourceType.ARKIT)
            except ValueError:
                continue
            if not isinstance(decoded, PongPacket):
                continue
            try:
                sample = clock_sample(
                    decoded.client_time_ns,
                    decoded.server_receive_time_ns,
                    decoded.server_send_time_ns,
                    received_at_ns,
                )
            except ValueError:
                continue
            if estimator.add_exchange(
                sample.client_send_time_ns,
                sample.server_receive_time_ns,
                sample.server_send_time_ns,
                sample.client_receive_time_ns,
            ):
                rtt_ms.append(sample.rtt_ns / 1_000_000.0)
                pongs_received += 1

    try:
        for sequence in range(sample_count):
            deadline = started_at + sequence / config.rate_hz
            deadline += random_source.uniform(0.0, config.jitter_ms / 1_000.0)
            delay = deadline - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elapsed_s = time.monotonic() - started_at
            if elapsed_s >= next_ping_s:
                ping = PingPacket(
                    source_type=SourceType.ARKIT,
                    sequence=ping_sequence,
                    session_id=config.session_id,
                    client_time_ns=time.monotonic_ns(),
                )
                udp_socket.sendto(encode_packet(ping), destination)
                ping_sequence = (ping_sequence + 1) & 0xFFFFFFFF
                pings_sent += 1
                next_ping_s += config.ping_interval_s

            payload = encode_packet(_pose(config, sequence, 2.0 * math.pi * elapsed_s))
            if random_source.random() < config.loss_rate:
                simulated_lost += 1
            elif pending is None and random_source.random() < config.reorder_rate:
                pending = payload
                simulated_reordered += 1
            else:
                udp_socket.sendto(payload, destination)
                pose_sent += 1
                if pending is not None:
                    udp_socket.sendto(pending, destination)
                    pose_sent += 1
                    pending = None
            drain_pongs()
        if pending is not None:
            udp_socket.sendto(pending, destination)
            pose_sent += 1

        release = replace(_pose(config, sample_count, 0.0), clutch_active=False)
        udp_socket.sendto(encode_packet(release), destination)
        pose_sent += 1
        send_finished_at = time.monotonic()
        end_wait = time.monotonic() + 0.05
        while time.monotonic() < end_wait:
            drain_pongs()
            time.sleep(0.001)
    finally:
        udp_socket.close()

    elapsed_s = max(send_finished_at - started_at, 1e-9)
    return {
        "hardware_enabled": False,
        "clutch_active": config.clutch_active,
        "session_id": config.session_id,
        "pose_attempted": sample_count + 1,
        "pose_sent": pose_sent,
        "simulated_lost": simulated_lost,
        "simulated_reordered": simulated_reordered,
        "send_fps": pose_sent / elapsed_s,
        "send_duration_s": elapsed_s,
        "pings_sent": pings_sent,
        "pongs_received": pongs_received,
        "rtt_ms": distribution_with_count(rtt_ms),
        "clock_offset_ns": estimator.offset_ns,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--rate-hz", type=float, default=60.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--loss-rate", type=float, default=0.0)
    parser.add_argument("--reorder-rate", type=float, default=0.0)
    parser.add_argument("--jitter-ms", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--session-id", type=int)
    parser.add_argument("--clutch", action="store_true")
    parser.add_argument("--position-amplitude-m", type=float, default=0.01)
    parser.add_argument("--ping-interval", type=float, default=1.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    session_id = args.session_id or secrets.randbelow(0xFFFFFFFF) + 1
    report = run_simulator(
        PoseSimulatorConfig(
            host=args.host,
            port=args.port,
            rate_hz=args.rate_hz,
            duration_s=args.duration,
            loss_rate=args.loss_rate,
            reorder_rate=args.reorder_rate,
            jitter_ms=args.jitter_ms,
            seed=args.seed,
            session_id=session_id,
            clutch_active=args.clutch,
            position_amplitude_m=args.position_amplitude_m,
            ping_interval_s=args.ping_interval,
        )
    )
    print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
