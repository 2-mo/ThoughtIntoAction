import json
from pathlib import Path

from robot_teleop.cli import main

ROOT = Path(__file__).parents[4]


def test_cli_runs_finite_mock_session_and_prints_one_report(capsys: object) -> None:
    exit_code = main(
        [
            "run",
            "--config",
            str(ROOT / "configs/phone/profiles/mock_mock.yaml"),
            "--duration",
            "0.05",
        ]
    )
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)

    assert exit_code == 0
    assert report["hardware_enabled"] is False
    assert report["components"]["robot"] == "mock"
    assert report["metrics"]["cycle_count"] > 0
    assert captured.err == ""


def test_cli_rejects_arx5_before_any_default_hardware_initialization(
    capsys: object,
) -> None:
    exit_code = main(
        [
            "run",
            "--config",
            str(ROOT / "configs/phone/profiles/arkit_arx5.yaml"),
            "--duration",
            "0.01",
        ]
    )
    captured = capsys.readouterr()  # type: ignore[attr-defined]

    assert exit_code == 2
    assert captured.out == ""
    assert "explicit --enable-hardware" in captured.err
