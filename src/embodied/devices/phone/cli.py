"""Command-line composition entry point; dry-run unless explicitly gated."""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path

from embodied.devices.phone.config import ConfigurationError, TeleopProfile, load_profile
from embodied.devices.phone.core.models import (
    Pose,
    Quaternion,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.devices.phone.inputs.mock import MockInputSource
from embodied.devices.phone.registry import (
    ComponentUnavailableError,
    HardwareEnableRequiredError,
    create_application,
)
from embodied.devices.phone.robots.arx5 import Arx5BackendError


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robot-teleop")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run = subparsers.add_parser("run")
    run.add_argument(
        "--config",
        type=Path,
        default=Path("configs/phone/profiles/mock_mock.yaml"),
    )
    run.add_argument("--input")
    run.add_argument("--transport")
    run.add_argument("--policy")
    run.add_argument("--robot")
    run.add_argument("--controller")
    run.add_argument("--robot-interface")
    run.add_argument("--duration", type=float, default=1.0)
    run.add_argument("--enable-hardware", action="store_true")
    run.add_argument("--allow-active-command", action="store_true")
    run.add_argument("--allow-gripper-command", action="store_true")
    run.add_argument("--gripper-only", action="store_true")
    run.add_argument("--gripper-open-readout", type=float)
    run.add_argument("--allow-unverified-safety-envelope", action="store_true")
    run.add_argument("--debug-web", action="store_true")
    run.add_argument("--debug-web-host", default="0.0.0.0")
    run.add_argument("--debug-web-port", type=int, default=8080)
    return parser


def _overrides(profile: TeleopProfile, args: argparse.Namespace) -> TeleopProfile:
    return replace(
        profile,
        input_name=args.input or profile.input_name,
        transport_name=args.transport or profile.transport_name,
        policy_name=args.policy or profile.policy_name,
        robot_name=args.robot or profile.robot_name,
        controller_name=args.controller or profile.controller_name,
    )


def _mock_input(sequence: int, elapsed_s: float, *, clutch: bool) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=1,
        timestamp_ns=sequence * 16_666_667,
        pose=Pose(
            Vector3(0.005 * math.sin(elapsed_s), 0.0, 0.0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=clutch,
        recenter_requested=False,
        estop_requested=False,
        gripper=0.5,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="mock-cli",
    )


def _run(args: argparse.Namespace) -> int:
    if not math.isfinite(args.duration) or args.duration < 0.0:
        raise ConfigurationError("duration must be non-negative and finite")
    profile = _overrides(load_profile(args.config), args)
    bundle = create_application(
        profile,
        enable_hardware=args.enable_hardware,
        robot_interface=args.robot_interface,
        allow_active_command=args.allow_active_command,
        allow_gripper_command=args.allow_gripper_command,
        gripper_only=args.gripper_only,
        gripper_open_readout=args.gripper_open_readout,
        allow_unverified_safety_envelope=args.allow_unverified_safety_envelope,
        debug_web_host=args.debug_web_host if args.debug_web else None,
        debug_web_port=args.debug_web_port,
    )
    started_at = time.monotonic()
    sequence = 0
    bundle.application.start()
    try:
        deadline = started_at + args.duration
        next_mobile_recover_attempt_at = started_at
        if isinstance(bundle.input_source, MockInputSource):
            while (
                time.monotonic() < deadline
                and bundle.application.servo_error is None
                and bundle.application.receiver_error is None
            ):
                now = time.monotonic()
                bundle.input_source.push(
                    _mock_input(sequence, now - started_at, clutch=True)
                )
                sequence += 1
                threading.Event().wait(min(1.0 / 60.0, max(0.0, deadline - now)))
            bundle.input_source.push(
                _mock_input(
                    sequence,
                    time.monotonic() - started_at,
                    clutch=False,
                )
            )
            threading.Event().wait(0.02)
        else:
            while time.monotonic() < deadline:
                now = time.monotonic()
                # A mobile Recover request is already correlated with a newer,
                # normal, released input by EStopResetGate. Complete it here so
                # the operator never needs an Ubuntu console or process restart.
                if (
                    bundle.application.estop_reset_ready
                    and now >= next_mobile_recover_attempt_at
                ):
                    try:
                        bundle.application.reset_estop()
                    except Exception as exc:
                        bundle.metrics.record_fault(
                            f"automatic mobile recover failed: {exc}"
                        )
                    next_mobile_recover_attempt_at = now + 1.0
                elif not bundle.application.estop_reset_ready:
                    next_mobile_recover_attempt_at = now
                # Keep the read-only dashboard alive after a latched hardware
                # stop so the operator can see the concrete failure instead of
                # getting a silent connection drop.
                if (
                    bundle.application.servo_error is not None
                    or bundle.application.receiver_error is not None
                ):
                    threading.Event().wait(
                        min(0.25, max(0.0, deadline - time.monotonic()))
                    )
                    continue
                threading.Event().wait(min(0.01, max(0.0, deadline - time.monotonic())))
        safe_stop_count = bundle.robot.safe_stop_count
    finally:
        bundle.application.stop()
    report = {
        "hardware_enabled": bundle.hardware_enabled,
        "components": {
            "input": profile.input_name,
            "transport": profile.transport_name,
            "policy": profile.policy_name,
            "robot": profile.robot_name,
            "controller": profile.controller_name,
        },
        "metrics": bundle.metrics.report(),
        "receiver_error": bundle.application.receiver_error,
        "servo_error": bundle.application.servo_error,
        "safe_stop_count": safe_stop_count,
    }
    print(json.dumps(report, sort_keys=True))
    return int(
        report["receiver_error"] is not None
        or report["servo_error"] is not None
        or safe_stop_count > 0
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "run":
            return _run(args)
    except (
        ComponentUnavailableError,
        ConfigurationError,
        HardwareEnableRequiredError,
        Arx5BackendError,
    ) as exc:
        print(f"robot-teleop: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
