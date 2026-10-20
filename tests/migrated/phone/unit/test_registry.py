from dataclasses import replace
from pathlib import Path

import pytest

from embodied.phone.config import load_profile
from embodied.phone.inputs.arkit import ArkitInputSource
from embodied.phone.registry import (
    CONTROLLERS,
    INPUTS,
    POLICIES,
    ROBOTS,
    TRANSPORTS,
    ComponentUnavailableError,
    HardwareEnableRequiredError,
    create_application,
)
from embodied.phone.transports.udp import UdpTransport

ROOT = Path(__file__).parents[4]


def test_phase_zero_registry_is_explicit_and_hardware_free() -> None:
    assert set(INPUTS) == {"arkit", "mock", "replay"}
    assert set(TRANSPORTS) == {"mock", "udp"}
    assert set(POLICIES) == {"relative-pose"}
    assert set(ROBOTS) == {"arx5", "mock"}
    assert set(CONTROLLERS) == {"cartesian-position"}


def test_arx5_requires_both_hardware_gate_and_interface_name() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_arx5.yaml")

    with pytest.raises(HardwareEnableRequiredError, match="explicit"):
        create_application(profile, enable_hardware=False)

    with pytest.raises(ComponentUnavailableError, match="robot-interface"):
        create_application(profile, enable_hardware=True)


def test_registry_builds_arkit_udp_to_mock_without_hardware() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")
    assert profile.udp is not None
    profile = replace(profile, udp=replace(profile.udp, bind_port=0))

    bundle = create_application(profile, enable_hardware=False)

    assert isinstance(bundle.input_source, ArkitInputSource)
    assert isinstance(bundle.transport, UdpTransport)
    assert not bundle.hardware_enabled


def test_registry_rejects_incompatible_input_transport_pair() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")

    with pytest.raises(ComponentUnavailableError, match="requires udp"):
        create_application(
            replace(profile, transport_name="mock"),
            enable_hardware=False,
        )
