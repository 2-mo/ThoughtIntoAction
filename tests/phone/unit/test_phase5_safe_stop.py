from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest

from embodied.phone.tools.phase5_safe_stop import run_safe_stop_validation

ROOT = Path(__file__).resolve().parents[3]
MODEL_CONFIG = ROOT / "configs/phone/robots/arx5.yaml"


@dataclass
class FakeJointState:
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    torque_values: tuple[float, ...]
    timestamp: float

    def pos(self) -> tuple[float, ...]:
        return self.position

    def vel(self) -> tuple[float, ...]:
        return self.velocity

    def torque(self) -> tuple[float, ...]:
        return self.torque_values


class FakeController:
    def __init__(
        self,
        *,
        post_stop_velocity: float = 0.0,
        fail_read: bool = False,
        fail_damping: bool = False,
        freeze_timestamp: bool = False,
    ) -> None:
        self.position = (0.0, 1.0, 1.0, 0.0, 0.0, 0.0)
        self.reads = 0
        self.damping_calls = 0
        self.post_stop_velocity = post_stop_velocity
        self.fail_read = fail_read
        self.fail_damping = fail_damping
        self.freeze_timestamp = freeze_timestamp

    def get_joint_state(self) -> FakeJointState:
        self.reads += 1
        if self.fail_read:
            raise RuntimeError("injected read failure")
        timestamp = 10.0 if self.freeze_timestamp else 10.0 + self.reads * 0.01
        speed = self.post_stop_velocity if self.damping_calls else 0.2
        return FakeJointState(
            position=self.position,
            velocity=(speed, 0.0, 0.0, 0.0, 0.0, 0.0),
            torque_values=(0.0,) * 6,
            timestamp=timestamp,
        )

    def set_to_damping(self) -> None:
        self.damping_calls += 1
        if self.fail_damping:
            raise RuntimeError("injected damping failure")


def fake_factory(controller: FakeController):
    def factory(
        _sdk: object,
        model: str,
        interface: str,
        joint_count: int,
    ) -> FakeController:
        assert model == "X5"
        assert interface == "fake0"
        assert joint_count == 6
        return controller

    return factory


def test_safe_stop_dry_run_never_loads_sdk() -> None:
    loader_calls = 0

    def loader() -> object:
        nonlocal loader_calls
        loader_calls += 1
        return object()

    report = run_safe_stop_validation(
        model_config_path=MODEL_CONFIG,
        sdk_loader=loader,
    )

    assert report["dry_run"] is True
    assert report["safe_stop_passed"] is False
    assert report["command_writes_enabled"] is False
    assert loader_calls == 0


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"confirm_operator_present": False, "allow_stop_command": True}, "operator"),
        (
            {"confirm_operator_present": True, "allow_stop_command": False},
            "allow-stop-command",
        ),
        (
            {
                "confirm_operator_present": True,
                "allow_stop_command": True,
                "robot_interface": "",
            },
            "robot-interface",
        ),
    ],
)
def test_safe_stop_requires_explicit_hardware_gates(
    kwargs: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        run_safe_stop_validation(
            model_config_path=MODEL_CONFIG,
            enable_hardware=True,
            **kwargs,
        )


def test_safe_stop_fake_hardware_sends_one_damping_command() -> None:
    controller = FakeController(post_stop_velocity=0.01)
    sleeps: list[float] = []
    clock_values = iter(range(100, 10_000, 100))

    report = run_safe_stop_validation(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_stop_command=True,
        post_stop_samples=2,
        sample_interval_ms=50.0,
        max_post_stop_velocity_rad_s=0.05,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        sleeper=sleeps.append,
        clock=lambda: next(clock_values),
    )

    assert report["safe_stop_passed"] is True
    assert report["command_writes_enabled"] is True
    assert report["servo_command_writes_enabled"] is False
    assert report["stop_command_sent"] is True
    assert report["sample_count"] == 3
    assert report["timestamp_progress"] is True
    assert report["host_sample_progress"] is True
    assert report["maximum_post_stop_joint_velocity_rad_s"] == pytest.approx(0.01)
    assert controller.damping_calls == 1
    assert sleeps == pytest.approx([0.05, 0.05])
    samples = report["samples"]
    assert isinstance(samples, list)
    assert samples[0]["phase"] == "before_stop"
    assert [sample["phase"] for sample in samples[1:]] == [
        "after_stop",
        "after_stop",
    ]


def test_safe_stop_reports_damping_failure() -> None:
    controller = FakeController(fail_damping=True)

    report = run_safe_stop_validation(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_stop_command=True,
        post_stop_samples=1,
        sample_interval_ms=0.0,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        sleeper=lambda _seconds: None,
    )

    assert report["safe_stop_passed"] is False
    assert report["stop_command_sent"] is False
    assert report["stop_command_error"] == "injected damping failure"
    assert (
        "safe-stop command failed: injected damping failure"
        in report["blocking_reasons"]
    )
    assert controller.damping_calls == 1


def test_safe_stop_reports_post_stop_velocity_failure() -> None:
    controller = FakeController(post_stop_velocity=0.2)

    report = run_safe_stop_validation(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_stop_command=True,
        post_stop_samples=1,
        sample_interval_ms=0.0,
        max_post_stop_velocity_rad_s=0.05,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        sleeper=lambda _seconds: None,
    )

    assert report["safe_stop_passed"] is False
    assert "post-stop joint velocity exceeded threshold" in report["blocking_reasons"]
    assert controller.damping_calls == 1


def test_safe_stop_attempts_damping_even_when_feedback_read_fails() -> None:
    controller = FakeController(fail_read=True)

    report = run_safe_stop_validation(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_stop_command=True,
        post_stop_samples=1,
        sample_interval_ms=0.0,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        sleeper=lambda _seconds: None,
    )

    assert report["safe_stop_passed"] is False
    assert report["stop_command_sent"] is True
    assert controller.damping_calls == 1
    assert any(
        "injected read failure" in reason for reason in report["blocking_reasons"]
    )


def test_safe_stop_reports_timestamp_stall() -> None:
    controller = FakeController(freeze_timestamp=True)

    report = run_safe_stop_validation(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_stop_command=True,
        post_stop_samples=2,
        sample_interval_ms=0.0,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        sleeper=lambda _seconds: None,
    )

    assert report["safe_stop_passed"] is False
    assert "SDK feedback timestamp did not advance" in report["blocking_reasons"]
    assert controller.damping_calls == 1
