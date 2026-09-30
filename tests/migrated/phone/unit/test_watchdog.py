import pytest

from robot_teleop.runtime.watchdog import Watchdog, WatchdogConfig, WatchdogStage


def test_watchdog_progresses_through_ordered_stages() -> None:
    watchdog = Watchdog(
        WatchdogConfig(
            hold_packet_age_ms=50.0,
            brake_packet_age_ms=100.0,
            stop_packet_age_ms=300.0,
        )
    )
    watchdog.reset(1_000_000_000)

    assert watchdog.evaluate(None, 1_050_000_000).stage is WatchdogStage.FRESH
    assert watchdog.evaluate(None, 1_050_000_001).stage is WatchdogStage.HOLD
    assert watchdog.evaluate(None, 1_100_000_001).stage is WatchdogStage.BRAKE
    assert watchdog.evaluate(None, 1_300_000_001).stage is WatchdogStage.SAFE_STOP


def test_latest_receipt_time_restarts_watchdog_age() -> None:
    watchdog = Watchdog(WatchdogConfig())
    watchdog.reset(0)

    decision = watchdog.evaluate(900_000_000, 910_000_000)

    assert decision.stage is WatchdogStage.FRESH
    assert decision.age_ms == pytest.approx(10.0)


def test_watchdog_thresholds_must_be_strictly_ordered() -> None:
    with pytest.raises(ValueError, match="hold < brake < stop"):
        WatchdogConfig(
            hold_packet_age_ms=100.0,
            brake_packet_age_ms=50.0,
            stop_packet_age_ms=300.0,
        )
