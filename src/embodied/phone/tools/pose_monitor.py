#!/usr/bin/env python3
"""Monitor finite ARKit-format UDP traffic without a robot backend."""

from __future__ import annotations

import argparse
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass

from embodied.phone.core.mapping import MappingScales
from embodied.phone.inputs.arkit import ArkitInputSource
from embodied.phone.transports.udp import UdpTransport, UdpTransportConfig

from embodied.phone.tools._common import (
    EventCounts,
    distribution_with_count,
    print_report,
    validate_port,
    validate_positive,
)
from embodied.phone.tools.control_observations import ControlObservations


@dataclass(frozen=True, slots=True)
class PoseMonitorConfig:
    bind_host: str = "0.0.0.0"
    port: int = 5005
    duration_s: float = 5.0
    receive_timeout_s: float = 0.1
    exit_after_round_trip: bool = False

    def __post_init__(self) -> None:
        if not self.bind_host.strip():
            raise ValueError("bind_host must not be blank")
        validate_port(self.port, allow_zero=True)
        validate_positive("duration_s", self.duration_s)
        validate_positive("receive_timeout_s", self.receive_timeout_s)


def run_monitor(
    config: PoseMonitorConfig,
    *,
    ready_event: threading.Event | None = None,
) -> dict[str, object]:
    events = EventCounts()
    transport = UdpTransport(
        UdpTransportConfig(
            bind_host=config.bind_host,
            bind_port=config.port,
            receive_timeout_s=config.receive_timeout_s,
            max_drain_datagrams=64,
            receive_buffer_bytes=1 << 20,
        ),
        event_callback=events,
    )
    received = 0
    last_input = None
    previous_receive_ns: int | None = None
    interarrival_ms: list[float] = []
    last_settings: MappingScales | None = None
    control_observations = ControlObservations()
    started_at = time.monotonic()

    def apply_settings(settings: MappingScales) -> MappingScales:
        nonlocal last_settings
        last_settings = settings
        return settings

    source = ArkitInputSource(
        transport,
        event_callback=events,
        settings_callback=apply_settings,
    )
    source.start()
    if ready_event is not None:
        ready_event.set()
    try:
        deadline = started_at + config.duration_s
        while time.monotonic() < deadline:
            input_state = source.latest()
            if input_state is None:
                continue
            now_ns = time.monotonic_ns()
            if previous_receive_ns is not None:
                interarrival_ms.append((now_ns - previous_receive_ns) / 1_000_000.0)
            previous_receive_ns = now_ns
            received += 1
            last_input = input_state
            control_observations.observe(input_state)
            if config.exit_after_round_trip and last_settings is not None:
                current_events = events.snapshot()
                if current_events.get("packet.ping", 0) > 0:
                    break
    finally:
        source.stop()
    elapsed_s = max(time.monotonic() - started_at, 1e-9)
    event_snapshot = events.snapshot()
    round_trip_complete = (
        received > 0
        and last_settings is not None
        and event_snapshot.get("packet.pong_sent", 0) > 0
        and event_snapshot.get("packet.settings_ack_sent", 0) > 0
    )
    return {
        "hardware_enabled": False,
        "round_trip_complete": round_trip_complete,
        "pose_received": received,
        "receive_fps": received / elapsed_s,
        "interarrival_ms": distribution_with_count(interarrival_ms),
        "last_sequence": None if last_input is None else last_input.sequence,
        "last_session_id": None if last_input is None else last_input.session_id,
        "last_tracking": (
            None if last_input is None else last_input.tracking_quality.value
        ),
        "last_position": (
            None if last_input is None else list(last_input.pose.position.as_tuple())
        ),
        "last_mapping_scales": (
            None
            if last_settings is None
            else {
                "position": last_settings.position_scale,
                "rotation": last_settings.rotation_scale,
            }
        ),
        "control_observations": control_observations.report(),
        "events": event_snapshot,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--receive-timeout", type=float, default=0.1)
    parser.add_argument("--exit-after-round-trip", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run_monitor(
        PoseMonitorConfig(
            bind_host=args.bind_host,
            port=args.port,
            duration_s=args.duration,
            receive_timeout_s=args.receive_timeout,
            exit_after_round_trip=args.exit_after_round_trip,
        )
    )
    print_report(report)
    return int(args.exit_after_round_trip and not report["round_trip_complete"])


if __name__ == "__main__":
    raise SystemExit(main())
