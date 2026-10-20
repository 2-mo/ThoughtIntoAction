import socket
from typing import cast

from embodied.phone.tools.phase3_mock_session import (
    Phase3MockSessionConfig,
    run_phase3_mock_session,
)
from embodied.phone.tools.phase3_mock_validation import (
    Phase3MockValidationConfig,
    run_phase3_mock_validation,
)


def test_phase3_mock_validation_exercises_the_full_udp_session() -> None:
    report = run_phase3_mock_validation(
        Phase3MockValidationConfig(duration_s=0.10, stream_rate_hz=90.0)
    )

    assert report["hardware_enabled"] is False
    assert report["workers_stopped"] is True
    checks = cast(dict[str, bool], report["checks"])
    assert all(checks.values())
    assert checks["mapping_axes_scales_orientation"]
    assert checks["clutch_recenter_continuity"]
    assert checks["tracking_and_session_recovery"]
    assert checks["invalid_input_rejection"]
    assert checks["latest_only_burst"]
    assert checks["explicit_estop_reset"]
    assert checks["watchdog_safe_stop"]

    metrics = cast(dict[str, object], report["metrics"])
    protocol_events = cast(dict[str, int], metrics["protocol_events"])
    watchdog_stages = cast(dict[str, int], metrics["watchdog_stages"])
    input_rejections = cast(dict[str, int], metrics["input_rejections"])
    assert protocol_events["udp.superseded"] > 0
    assert watchdog_stages["hold"] > 0
    assert watchdog_stages["brake"] > 0
    assert watchdog_stages["safe_stop"] >= 1
    assert input_rejections["input position jump exceeds the limit"] == 1


def test_phase3_mock_console_never_enables_hardware_or_auto_resets() -> None:
    reserve_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve_socket.bind(("127.0.0.1", 0))
    port = reserve_socket.getsockname()[1]
    reserve_socket.close()
    commands = iter(("status", "report", "reset", "quit"))
    reports: list[dict[str, object]] = []

    run_phase3_mock_session(
        Phase3MockSessionConfig(bind_host="127.0.0.1", port=port),
        read_line=lambda _: next(commands),
        report=reports.append,
    )

    assert [report["event"] for report in reports] == [
        "started",
        "status",
        "report",
        "reset_rejected",
        "stopping",
    ]
    assert all(report["hardware_enabled"] is False for report in reports)
    assert reports[2]["estimated_sequence_loss"] == 0
    metrics = cast(dict[str, object], reports[2]["metrics"])
    assert isinstance(metrics["cycle_count"], int)
    sample_window = cast(dict[str, object], metrics["sample_window"])
    assert sample_window["capacity"] == 131_072
    assert reports[3]["local_reset_command"] is True
