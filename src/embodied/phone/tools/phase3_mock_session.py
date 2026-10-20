"""Run a local ARKit-UDP-to-MockRobot session with explicit reset authority."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import cast

from embodied.phone.config import load_profile
from embodied.phone.registry import ApplicationBundle, create_application

from embodied.phone.tools._common import print_report, validate_port

ROOT = Path(__file__).resolve().parents[4]
PHASE3_METRICS_SAMPLE_CAPACITY = 131_072
type LineReader = Callable[[str], str]
type Reporter = Callable[[dict[str, object]], None]


@dataclass(frozen=True, slots=True)
class Phase3MockSessionConfig:
    bind_host: str = "0.0.0.0"
    port: int = 5005

    def __post_init__(self) -> None:
        if not self.bind_host.strip():
            raise ValueError("bind_host must not be blank")
        validate_port(self.port)


def create_phase3_mock_session(
    config: Phase3MockSessionConfig,
) -> ApplicationBundle:
    """Build the fixed Mock-only composition root used by the local console."""
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")
    if profile.udp is None:
        raise RuntimeError("ARKit Mock profile must define UDP settings")
    bundle = create_application(
        replace(
            profile,
            udp=replace(
                profile.udp,
                bind_host=config.bind_host,
                bind_port=config.port,
            ),
        ),
        enable_hardware=False,
        metrics_sample_capacity=PHASE3_METRICS_SAMPLE_CAPACITY,
    )
    if bundle.hardware_enabled:
        raise RuntimeError("Phase 3 Mock session must never enable hardware")
    return bundle


def _status(bundle: ApplicationBundle, *, event: str) -> dict[str, object]:
    return {
        "event": event,
        "hardware_enabled": False,
        "state": bundle.application.state.value,
        "estop_reset_ready": bundle.application.estop_reset_ready,
        "receiver_error": bundle.application.receiver_error,
        "servo_error": bundle.application.servo_error,
        "workers": bundle.application.worker_count,
        "mock_safe_stopped": bundle.robot.safe_stopped,
        "mock_safe_stop_count": bundle.robot.safe_stop_count,
    }


def _runtime_report(bundle: ApplicationBundle) -> dict[str, object]:
    """Return a non-real-time snapshot for the local operator console."""
    report = _status(bundle, event="report")
    metrics = bundle.metrics.report()
    protocol_events = cast(dict[str, int], metrics["protocol_events"])
    report["estimated_sequence_loss"] = protocol_events.get("sequence.loss", 0)
    report["metrics"] = metrics
    return report


def run_phase3_mock_session(
    config: Phase3MockSessionConfig,
    *,
    read_line: LineReader = input,
    report: Reporter = print_report,
) -> None:
    """Keep a Mock-only session alive until a local operator ends it."""
    bundle = create_phase3_mock_session(config)
    bundle.application.start()
    report(_status(bundle, event="started"))
    try:
        while True:
            try:
                command = (
                    read_line("phase3-mock [status|report|reset|quit] > ")
                    .strip()
                    .lower()
                )
            except EOFError:
                command = "quit"
            if command in {"quit", "exit"}:
                report(_status(bundle, event="stopping"))
                return
            if command == "status":
                report(_status(bundle, event="status"))
                continue
            if command == "report":
                report(_runtime_report(bundle))
                continue
            if command == "reset":
                completed = bundle.application.reset_estop()
                result = _status(
                    bundle, event="reset_completed" if completed else "reset_rejected"
                )
                result["local_reset_command"] = True
                report(result)
                continue
            report(
                {
                    "event": "unknown_command",
                    "hardware_enabled": False,
                    "command": command,
                }
            )
    finally:
        bundle.application.stop()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        run_phase3_mock_session(
            Phase3MockSessionConfig(bind_host=args.bind_host, port=args.port)
        )
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
