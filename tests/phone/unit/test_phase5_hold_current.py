from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

from embodied.phone.tools.phase5_hold_current import run_hold_current

ROOT = Path(__file__).resolve().parents[3]
MODEL_CONFIG = ROOT / "configs/phone/robots/arx5.yaml"


@dataclass
class FakeJointState:
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    timestamp: float
    gripper_pos: float = 0.01
    gripper_vel: float = 0.0
    gripper_torque: float = 0.0

    def pos(self) -> tuple[float, ...]:
        return self.position

    def vel(self) -> tuple[float, ...]:
        return self.velocity

    def torque(self) -> tuple[float, ...]:
        return (0.0,) * len(self.position)


class FakeCommand:
    def __init__(
        self,
        position: Any,
        velocity: Any,
        acceleration: Any,
        gripper_position: float,
        gripper_velocity: float,
        gripper_torque: float,
    ) -> None:
        self.position = tuple(float(value) for value in position)
        self.velocity = tuple(float(value) for value in velocity)
        self.acceleration = tuple(float(value) for value in acceleration)
        self.gripper_position = gripper_position
        self.gripper_velocity = gripper_velocity
        self.gripper_torque = gripper_torque
        self.timestamp = 0.0


class FakeSdk:
    JointState = FakeCommand


class FakeController:
    def __init__(
        self,
        *,
        drift_per_read: float = 0.0,
        fail_command: bool = False,
    ) -> None:
        self.base = (0.0, 1.0, 1.0, 0.0, 0.0, 0.0)
        self.reads = 0
        self.command_writes = 0
        self.damping_calls = 0
        self.commands: list[FakeCommand] = []
        self.drift_per_read = drift_per_read
        self.fail_command = fail_command

    def get_joint_state(self) -> FakeJointState:
        self.reads += 1
        drift = self.drift_per_read * max(0, self.reads - 2)
        return FakeJointState(
            position=(self.base[0] + drift, *self.base[1:]),
            velocity=(0.0,) * 6,
            timestamp=10.0 + self.reads * 0.01,
        )

    def get_timestamp(self) -> float:
        return 20.0 + self.reads * 0.01

    def set_joint_cmd(self, command: object) -> None:
        if self.fail_command:
            raise RuntimeError("injected command failure")
        self.command_writes += 1
        self.commands.append(command)  # type: ignore[arg-type]

    def set_to_damping(self) -> None:
        self.damping_calls += 1


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


def test_hold_current_dry_run_never_loads_sdk() -> None:
    loader_calls = 0

    def loader() -> object:
        nonlocal loader_calls
        loader_calls += 1
        return FakeSdk()

    report = run_hold_current(model_config_path=MODEL_CONFIG, sdk_loader=loader)

    assert report["dry_run"] is True
    assert report["hold_passed"] is False
    assert report["command_writes_enabled"] is False
    assert loader_calls == 0


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"confirm_operator_present": False, "allow_command_writes": True}, "operator"),
        (
            {"confirm_operator_present": True, "allow_command_writes": False},
            "allow-command-writes",
        ),
        (
            {
                "confirm_operator_present": True,
                "allow_command_writes": True,
                "robot_interface": "",
            },
            "robot-interface",
        ),
    ],
)
def test_hold_current_requires_explicit_hardware_gates(
    kwargs: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        run_hold_current(
            model_config_path=MODEL_CONFIG,
            enable_hardware=True,
            **kwargs,
        )


def test_hold_current_fake_hardware_writes_current_position_and_damps() -> None:
    controller = FakeController()
    sleeps: list[float] = []
    clock_values = iter(range(100, 10_000, 100))

    report = run_hold_current(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_command_writes=True,
        duration_s=0.06,
        command_rate_hz=50.0,
        sdk_loader=lambda: FakeSdk(),
        controller_factory=fake_factory(controller),
        sleeper=sleeps.append,
        clock=lambda: next(clock_values),
    )

    assert report["hold_passed"] is True
    assert report["command_writes_enabled"] is True
    assert report["command_writes"] == 3
    assert report["sample_count"] == 4
    assert report["timestamp_progress"] is True
    assert report["maximum_abs_joint_drift_rad"] == pytest.approx(0.0)
    assert controller.command_writes == 3
    assert controller.damping_calls == 1
    assert sleeps == pytest.approx([0.02, 0.02])
    assert all(command.position == controller.base for command in controller.commands)
    assert all(command.velocity == (0.0,) * 6 for command in controller.commands)
    assert all(command.acceleration == (0.0,) * 6 for command in controller.commands)


def test_hold_current_reports_joint_drift_failure_but_still_damps() -> None:
    controller = FakeController(drift_per_read=0.04)

    report = run_hold_current(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_command_writes=True,
        duration_s=0.04,
        command_rate_hz=50.0,
        max_joint_drift_rad=0.01,
        sdk_loader=lambda: FakeSdk(),
        controller_factory=fake_factory(controller),
        sleeper=lambda _seconds: None,
    )

    assert report["hold_passed"] is False
    assert report["maximum_abs_joint_drift_rad"] > 0.01
    assert "joint drift exceeded the configured threshold" in report["blocking_reasons"]
    assert controller.damping_calls == 1


def test_hold_current_reports_command_error_and_damps() -> None:
    controller = FakeController(fail_command=True)

    report = run_hold_current(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        confirm_operator_present=True,
        allow_command_writes=True,
        duration_s=0.04,
        command_rate_hz=50.0,
        sdk_loader=lambda: FakeSdk(),
        controller_factory=fake_factory(controller),
        sleeper=lambda _seconds: None,
    )

    assert report["hold_passed"] is False
    assert report["command_writes"] == 0
    assert report["command_error"] == "injected command failure"
    assert "injected command failure" in report["blocking_reasons"]
    assert controller.damping_calls == 1
