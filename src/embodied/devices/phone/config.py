"""Strict, explicit YAML profile loading outside the canonical core."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import yaml

from embodied.devices.phone.core.mapping import AxisMapping, MappingOutputFrame, PoseMapping
from embodied.devices.phone.core.models import Vector3
from embodied.devices.phone.core.safety import TargetSafetyLimits
from embodied.devices.phone.runtime.watchdog import WatchdogConfig
from embodied.devices.phone.transports.udp import UdpTransportConfig


class ConfigurationError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class TeleopProfile:
    input_name: str
    transport_name: str
    policy_name: str
    robot_name: str
    controller_name: str
    rate_hz: float
    receiver_poll_interval_s: float
    state_stale_timeout_ms: float
    mapping: PoseMapping
    watchdog: WatchdogConfig
    target_safety: TargetSafetyLimits
    udp: UdpTransportConfig | None
    hardware_requested: bool


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ConfigurationError(f"{name} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _section(parent: dict[str, object], key: str) -> dict[str, object]:
    if key not in parent:
        raise ConfigurationError(f"missing configuration section: {key}")
    return _mapping(parent[key], key)


def _text(parent: dict[str, object], key: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{key} must be a non-empty string")
    return value.strip()


def _number(parent: dict[str, object], key: str) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{key} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigurationError(f"{key} must be finite")
    return result


def _integer(parent: dict[str, object], key: str) -> int:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{key} must be an integer")
    return value


def _optional_text(parent: dict[str, object], key: str) -> str | None:
    value = parent.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{key} must be a non-empty string")
    return value.strip()


def _optional_integer(parent: dict[str, object], key: str) -> int | None:
    value = parent.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{key} must be an integer")
    return value


def _boolean(parent: dict[str, object], key: str) -> bool:
    value = parent.get(key)
    if not isinstance(value, bool):
        raise ConfigurationError(f"{key} must be a boolean")
    return value


def _vector3(value: object, name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ConfigurationError(f"{name} must contain exactly three numbers")
    numbers: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ConfigurationError(f"{name} must contain only numbers")
        numbers.append(float(item))
    return Vector3(*numbers)


def _pose_mapping(raw: dict[str, object]) -> PoseMapping:
    axes = _section(raw, "axes")
    source_indices = {"input_x": 0, "input_y": 1, "input_z": 2}
    indices: list[int] = []
    signs: list[int] = []
    for output_name in ("robot_x", "robot_y", "robot_z"):
        axis = _mapping(axes.get(output_name), f"axes.{output_name}")
        source = _text(axis, "source")
        if source not in source_indices:
            raise ConfigurationError(f"unsupported axis source: {source}")
        sign_value = axis.get("sign")
        if isinstance(sign_value, bool) or sign_value not in (-1, 1):
            raise ConfigurationError("axis sign must be -1 or 1")
        indices.append(source_indices[source])
        signs.append(sign_value)
    return PoseMapping(
        axes=AxisMapping(
            (indices[0], indices[1], indices[2]),
            (signs[0], signs[1], signs[2]),
        ),
        position_scale=_number(raw, "position_scale"),
        rotation_scale=_number(raw, "rotation_scale"),
        output_frame=_mapping_output_frame(raw),
    )


def _mapping_output_frame(raw: dict[str, object]) -> MappingOutputFrame:
    value = raw.get("output_frame", MappingOutputFrame.TOOL.value)
    if not isinstance(value, str):
        raise ConfigurationError("mapping.output_frame must be 'tool' or 'base'")
    try:
        return MappingOutputFrame(value)
    except ValueError as exc:
        raise ConfigurationError(
            "mapping.output_frame must be 'tool' or 'base'"
        ) from exc


def load_profile(path: Path) -> TeleopProfile:
    try:
        loaded: object = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigurationError(f"could not load profile {path}: {exc}") from exc
    root = _mapping(loaded, "profile")
    runtime = _section(root, "runtime")
    control = _section(root, "control")
    receiver = _section(root, "receiver")
    mapping = _section(root, "mapping")
    safety = _section(root, "safety")
    hardware = _section(root, "hardware")
    input_name = _text(runtime, "input")
    transport_name = _text(runtime, "transport")
    udp: UdpTransportConfig | None = None
    if transport_name == "udp":
        network = _section(root, "network")
        try:
            udp = UdpTransportConfig(
                bind_host=_text(network, "bind_host"),
                bind_port=_integer(network, "bind_port"),
                remote_host=_optional_text(network, "remote_host"),
                remote_port=_optional_integer(network, "remote_port"),
                receive_timeout_s=_number(network, "receive_timeout_ms") / 1_000.0,
                max_datagram_size=_integer(network, "max_datagram_size"),
                max_drain_datagrams=_integer(network, "max_drain_datagrams"),
                receive_buffer_bytes=_integer(network, "receive_buffer_bytes"),
            )
        except ValueError as exc:
            raise ConfigurationError(f"invalid UDP configuration: {exc}") from exc
    workspace_raw = safety.get("workspace")
    workspace_minimum = None
    workspace_maximum = None
    if workspace_raw is not None:
        workspace = _mapping(workspace_raw, "safety.workspace")
        workspace_minimum = _vector3(workspace.get("minimum_m"), "minimum_m")
        workspace_maximum = _vector3(workspace.get("maximum_m"), "maximum_m")
    return TeleopProfile(
        input_name=input_name,
        transport_name=transport_name,
        policy_name=_text(runtime, "policy"),
        robot_name=_text(runtime, "robot"),
        controller_name=_text(runtime, "controller"),
        rate_hz=_number(control, "rate_hz"),
        receiver_poll_interval_s=_number(receiver, "poll_interval_ms") / 1_000.0,
        state_stale_timeout_ms=_number(safety, "state_stale_timeout_ms"),
        mapping=_pose_mapping(mapping),
        watchdog=WatchdogConfig(
            hold_packet_age_ms=_number(safety, "hold_packet_age_ms"),
            brake_packet_age_ms=_number(safety, "brake_packet_age_ms"),
            stop_packet_age_ms=_number(safety, "stop_packet_age_ms"),
        ),
        target_safety=TargetSafetyLimits(
            maximum_target_step_m=_number(safety, "maximum_target_step_m"),
            maximum_rotation_step_rad=_number(safety, "maximum_rotation_step_rad"),
            maximum_linear_velocity_m_s=_number(safety, "maximum_linear_velocity_m_s"),
            maximum_angular_velocity_rad_s=_number(
                safety, "maximum_angular_velocity_rad_s"
            ),
            workspace_minimum_m=workspace_minimum,
            workspace_maximum_m=workspace_maximum,
        ),
        udp=udp,
        hardware_requested=_boolean(hardware, "enabled"),
    )
