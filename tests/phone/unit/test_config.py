from dataclasses import replace
from pathlib import Path

import pytest

from embodied.devices.phone.config import ConfigurationError, load_profile
from embodied.devices.phone.registry import (
    ComponentUnavailableError,
    HardwareEnableRequiredError,
    create_application,
)

ROOT = Path(__file__).parents[3]


def test_mock_profile_loads_mapping_and_safety_defaults() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/mock_mock.yaml")

    assert profile.input_name == "mock"
    assert profile.robot_name == "mock"
    assert profile.rate_hz == 200.0
    assert profile.mapping.axes.robot_from_input == (0, 1, 2)
    assert profile.watchdog.hold_packet_age_ms == 50.0
    assert not profile.hardware_requested
    assert profile.udp is None


def test_arkit_mock_profile_loads_strict_udp_configuration() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")

    assert profile.input_name == "arkit"
    assert profile.transport_name == "udp"
    assert profile.robot_name == "mock"
    assert profile.receiver_poll_interval_s == 0.0
    assert profile.udp is not None
    assert profile.udp.bind_host == "0.0.0.0"
    assert profile.udp.bind_port == 5005
    assert profile.udp.max_drain_datagrams == 64
    assert not profile.hardware_requested


def test_udp_profile_rejects_invalid_network_values(tmp_path: Path) -> None:
    source = (ROOT / "configs/phone/profiles/arkit_mock.yaml").read_text(encoding="utf-8")
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(source.replace("bind_port: 5005", "bind_port: 70000"))

    with pytest.raises(ConfigurationError, match="bind_port"):
        load_profile(invalid)


def test_config_file_cannot_authorize_unregistered_hardware() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/mock_mock.yaml")
    arx_profile = replace(
        profile,
        robot_name="arx5",
        hardware_requested=True,
    )

    with pytest.raises(HardwareEnableRequiredError):
        create_application(arx_profile, enable_hardware=False)
    with pytest.raises(ComponentUnavailableError, match="arx5"):
        create_application(arx_profile, enable_hardware=True)


def test_phase_zero_factory_builds_only_dry_run_bundle() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/mock_mock.yaml")

    bundle = create_application(profile, enable_hardware=False)

    assert not bundle.hardware_enabled
    assert bundle.profile.robot_name == "mock"
