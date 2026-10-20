from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import pytest

from embodied.phone.core.models import Pose, Quaternion, Vector3
from embodied.phone.robots.arx5 import Arx5BackendConfig
from embodied.phone.tools.arx5_state_readiness import run_state_readiness

ROOT = Path(__file__).resolve().parents[3]
MODEL_CONFIG = ROOT / "configs/phone/robots/arx5.yaml"


@dataclass
class FakeJointState:
    timestamp: float = 10.0
    gripper_pos: float = -3.24846
    gripper_vel: float = 0.0
    gripper_torque: float = 0.0

    def pos(self) -> tuple[float, ...]:
        return (0.0, 1.0, 1.0, 0.0, 0.0, 0.0)

    def vel(self) -> tuple[float, ...]:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def torque(self) -> tuple[float, ...]:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class FakeController:
    def __init__(self) -> None:
        self.joint = FakeJointState()
        self.state_reads = 0
        self.damping_calls = 0

    def get_joint_state(self) -> FakeJointState:
        self.state_reads += 1
        self.joint.timestamp += 0.01
        return self.joint

    def set_to_damping(self) -> None:
        self.damping_calls += 1


def fake_factory(
    controller: FakeController,
) -> Callable[[object, Arx5BackendConfig], FakeController]:
    def factory(_sdk: object, config: Arx5BackendConfig) -> FakeController:
        assert config.interface_name == "fake0"
        assert config.disconnect_damping_enabled is False
        assert config.damping_on_connect_error is False
        assert config.feedback_fault_damping_enabled is False
        assert config.background_send_recv is True
        assert config.interpolation_method == "cubic"
        return controller

    return factory


def fake_tcp_pose(_joint_position: tuple[float, ...]) -> Pose:
    return Pose(
        position=Vector3(0.35, 0.0, 0.30),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def test_state_readiness_dry_run_never_loads_sdk() -> None:
    loader_calls = 0

    def loader() -> object:
        nonlocal loader_calls
        loader_calls += 1
        return object()

    report = run_state_readiness(
        model_config_path=MODEL_CONFIG,
        sdk_loader=loader,
    )

    assert report["dry_run"] is True
    assert report["hardware_enabled"] is False
    assert report["controller_constructed"] is False
    assert report["command_writes_enabled"] is False
    assert report["readiness_passed"] is False
    assert loader_calls == 0


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"state_only": False, "confirm_operator_present": True}, "--state-only"),
        (
            {
                "state_only": True,
                "confirm_operator_present": False,
                "robot_interface": "fake0",
            },
            "operator",
        ),
        (
            {
                "state_only": True,
                "confirm_operator_present": True,
                "robot_interface": "",
            },
            "--robot-interface",
        ),
    ],
)
def test_state_readiness_hardware_requires_all_explicit_gates(
    kwargs: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        run_state_readiness(
            model_config_path=MODEL_CONFIG,
            enable_hardware=True,
            **kwargs,
        )


def test_state_readiness_fake_hardware_reads_state_without_damping() -> None:
    controller = FakeController()

    report = run_state_readiness(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        state_only=True,
        confirm_operator_present=True,
        samples=2,
        sample_interval_ms=0.0,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=fake_tcp_pose,
    )

    assert report["dry_run"] is False
    assert report["readiness_passed"] is True
    assert report["controller_constructed"] is True
    assert report["read_error"] is None
    assert report["sdk_state_reads"] == 2
    assert report["timestamp_progress"] is True
    assert report["host_sample_progress"] is True
    assert report["sdk_background_send_recv"] is True
    assert report["sdk_interpolation_method"] == "cubic"
    assert report["command_writes_enabled"] is False
    assert report["disconnect_damping_enabled"] is False
    assert report["feedback_fault_damping_enabled"] is False
    assert controller.state_reads == 3
    assert controller.damping_calls == 0
    samples = report["samples"]
    assert isinstance(samples, list)
    assert samples[0]["gripper_position_raw"] == pytest.approx(-3.24846)
    assert samples[1]["sdk_timestamp_s"] > samples[0]["sdk_timestamp_s"]


def test_state_readiness_rejects_nonadvancing_fake_timestamp() -> None:
    controller = FakeController()

    def frozen_state() -> FakeJointState:
        controller.state_reads += 1
        return controller.joint

    controller.get_joint_state = frozen_state  # type: ignore[method-assign]

    report = run_state_readiness(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        state_only=True,
        confirm_operator_present=True,
        samples=2,
        sample_interval_ms=0.0,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=fake_tcp_pose,
    )

    assert controller.damping_calls == 0
    assert report["timestamp_progress"] is False
    assert report["readiness_passed"] is False
    assert report["blocking_reasons"] == ("SDK feedback timestamp did not advance",)
    assert math.isfinite(controller.joint.timestamp)


def test_state_readiness_reports_nonadvancing_loop_samples_without_damping() -> None:
    controller = FakeController()
    clock_values = iter((0, 10_000_000, 220_000_000))

    def frozen_after_connect() -> FakeJointState:
        controller.state_reads += 1
        if controller.state_reads == 1:
            controller.joint.timestamp += 0.01
        return controller.joint

    controller.get_joint_state = frozen_after_connect  # type: ignore[method-assign]

    report = run_state_readiness(
        model_config_path=MODEL_CONFIG,
        robot_interface="fake0",
        enable_hardware=True,
        state_only=True,
        confirm_operator_present=True,
        samples=2,
        sample_interval_ms=0.0,
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=fake_tcp_pose,
        clock=lambda: next(clock_values),
    )

    assert report["readiness_passed"] is False
    assert report["read_error"] is None
    assert report["sdk_state_reads"] == 2
    assert report["timestamp_progress"] is False
    assert report["host_sample_progress"] is True
    assert report["blocking_reasons"] == ("SDK feedback timestamp did not advance",)
    assert controller.damping_calls == 0
