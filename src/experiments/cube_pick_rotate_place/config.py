"""Configuration and validation for the cube manipulation experiment."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from arx5_paths import PROJECT_ROOT


DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.json")


@dataclass(frozen=True)
class CameraConfig:
    serial: str | None
    width: int
    height: int
    fps: int
    warmup_frames: int


@dataclass(frozen=True)
class TableConfig:
    center_x_m: float
    center_y_m: float
    top_z_m: float
    width_m: float
    depth_m: float
    yaw_rad: float


@dataclass(frozen=True)
class PerceptionConfig:
    required_samples: int
    maximum_frames: int
    minimum_confidence: float
    minimum_size_m: float
    maximum_size_m: float
    maximum_xy_rms_m: float
    maximum_yaw_rms_rad: float
    maximum_depth_residual_m: float
    depth_patch_radius_px: int


@dataclass(frozen=True)
class MotionConfig:
    rotation_rad: float
    place_offset_x_m: float
    place_offset_y_m: float
    approach_height_m: float
    lift_height_m: float
    tcp_z_offset_m: float
    minimum_initial_clearance_m: float
    pregrasp_width_m: float
    precontact_width_m: float
    release_width_m: float
    gripper_duration_s: float
    closing_torque_nm: float
    minimum_gripper_width_m: float
    gripper_timeout_s: float
    contact_hold_s: float
    settle_s: float
    velocity_scale: float


@dataclass(frozen=True)
class ExperimentConfig:
    interface: str
    calibration_path: Path
    cube_edge_m: float
    camera: CameraConfig
    table: TableConfig
    perception: PerceptionConfig
    motion: MotionConfig


def _finite(name: str, value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _millimetres(name: str, value: Any) -> float:
    return _finite(name, value) / 1000.0


def _resolve_project_path(value: Any) -> Path:
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def load_config(path: str | Path = DEFAULT_CONFIG_PATH) -> ExperimentConfig:
    """Load the checked JSON configuration used by both preview and execution."""
    source = Path(path).expanduser().resolve()
    data = json.loads(source.read_text(encoding="utf-8"))
    camera_data = data["camera"]
    table_data = data["table"]
    perception_data = data["perception"]
    motion_data = data["motion"]
    camera = CameraConfig(
        serial=None if camera_data.get("serial") in (None, "") else str(camera_data["serial"]),
        width=int(camera_data["width"]),
        height=int(camera_data["height"]),
        fps=int(camera_data["fps"]),
        warmup_frames=int(camera_data["warmup_frames"]),
    )
    table = TableConfig(
        center_x_m=_millimetres("table.center_x_mm", table_data["center_x_mm"]),
        center_y_m=_millimetres("table.center_y_mm", table_data["center_y_mm"]),
        top_z_m=_millimetres("table.top_z_mm", table_data["top_z_mm"]),
        width_m=_millimetres("table.width_mm", table_data["width_mm"]),
        depth_m=_millimetres("table.depth_mm", table_data["depth_mm"]),
        yaw_rad=math.radians(_finite("table.yaw_deg", table_data["yaw_deg"])),
    )
    perception = PerceptionConfig(
        required_samples=int(perception_data["required_samples"]),
        maximum_frames=int(perception_data["maximum_frames"]),
        minimum_confidence=_finite(
            "perception.minimum_confidence",
            perception_data["minimum_confidence"],
        ),
        minimum_size_m=_millimetres(
            "perception.minimum_size_mm",
            perception_data["minimum_size_mm"],
        ),
        maximum_size_m=_millimetres(
            "perception.maximum_size_mm",
            perception_data["maximum_size_mm"],
        ),
        maximum_xy_rms_m=_millimetres(
            "perception.maximum_xy_rms_mm",
            perception_data["maximum_xy_rms_mm"],
        ),
        maximum_yaw_rms_rad=math.radians(
            _finite(
                "perception.maximum_yaw_rms_deg",
                perception_data["maximum_yaw_rms_deg"],
            )
        ),
        maximum_depth_residual_m=_millimetres(
            "perception.maximum_depth_residual_mm",
            perception_data["maximum_depth_residual_mm"],
        ),
        depth_patch_radius_px=int(perception_data["depth_patch_radius_px"]),
    )
    motion = MotionConfig(
        rotation_rad=math.radians(_finite("motion.rotation_deg", motion_data["rotation_deg"])),
        place_offset_x_m=_millimetres(
            "motion.place_offset_x_mm", motion_data["place_offset_x_mm"]
        ),
        place_offset_y_m=_millimetres(
            "motion.place_offset_y_mm", motion_data["place_offset_y_mm"]
        ),
        approach_height_m=_millimetres(
            "motion.approach_height_mm", motion_data["approach_height_mm"]
        ),
        lift_height_m=_millimetres(
            "motion.lift_height_mm", motion_data["lift_height_mm"]
        ),
        tcp_z_offset_m=_millimetres(
            "motion.tcp_z_offset_mm", motion_data["tcp_z_offset_mm"]
        ),
        minimum_initial_clearance_m=_millimetres(
            "motion.minimum_initial_clearance_mm",
            motion_data["minimum_initial_clearance_mm"],
        ),
        pregrasp_width_m=_millimetres(
            "motion.pregrasp_width_mm", motion_data["pregrasp_width_mm"]
        ),
        precontact_width_m=_millimetres(
            "motion.precontact_width_mm", motion_data["precontact_width_mm"]
        ),
        release_width_m=_millimetres(
            "motion.release_width_mm", motion_data["release_width_mm"]
        ),
        gripper_duration_s=_finite(
            "motion.gripper_duration_s", motion_data["gripper_duration_s"]
        ),
        closing_torque_nm=_finite(
            "motion.closing_torque_nm", motion_data["closing_torque_nm"]
        ),
        minimum_gripper_width_m=_millimetres(
            "motion.minimum_gripper_width_mm",
            motion_data["minimum_gripper_width_mm"],
        ),
        gripper_timeout_s=_finite(
            "motion.gripper_timeout_s", motion_data["gripper_timeout_s"]
        ),
        contact_hold_s=_finite("motion.contact_hold_s", motion_data["contact_hold_s"]),
        settle_s=_finite("motion.settle_s", motion_data["settle_s"]),
        velocity_scale=_finite("motion.velocity_scale", motion_data["velocity_scale"]),
    )
    config = ExperimentConfig(
        interface=str(data.get("interface", "can0")),
        calibration_path=_resolve_project_path(data["calibration_path"]),
        cube_edge_m=_millimetres("cube_edge_mm", data["cube_edge_mm"]),
        camera=camera,
        table=table,
        perception=perception,
        motion=motion,
    )
    validate_config(config)
    return config


def validate_config(config: ExperimentConfig) -> None:
    if not config.interface.strip():
        raise ValueError("interface must not be empty")
    if not 0.01 <= config.cube_edge_m <= 0.08:
        raise ValueError("cube_edge_mm must be in [10, 80]")
    if config.camera.width <= 0 or config.camera.height <= 0 or config.camera.fps <= 0:
        raise ValueError("camera dimensions and fps must be positive")
    if config.camera.warmup_frames < 0:
        raise ValueError("camera.warmup_frames must not be negative")
    if config.table.width_m <= 0.0 or config.table.depth_m <= 0.0:
        raise ValueError("table dimensions must be positive")
    perception = config.perception
    if perception.required_samples < 3:
        raise ValueError("perception.required_samples must be at least 3")
    if perception.maximum_frames < perception.required_samples:
        raise ValueError("perception.maximum_frames must cover required_samples")
    if not 0.0 <= perception.minimum_confidence <= 1.0:
        raise ValueError("perception.minimum_confidence must be in [0, 1]")
    if not 0.0 < perception.minimum_size_m < perception.maximum_size_m:
        raise ValueError("perception size bounds are invalid")
    if perception.maximum_xy_rms_m <= 0.0 or perception.maximum_yaw_rms_rad <= 0.0:
        raise ValueError("perception stability limits must be positive")
    if perception.maximum_depth_residual_m <= 0.0:
        raise ValueError("maximum depth residual must be positive")
    if not 0 <= perception.depth_patch_radius_px <= 8:
        raise ValueError("depth_patch_radius_px must be in [0, 8]")
    motion = config.motion
    if motion.approach_height_m <= 0.0 or motion.lift_height_m <= 0.0:
        raise ValueError("approach and lift heights must be positive")
    if motion.minimum_initial_clearance_m <= 0.0:
        raise ValueError("minimum initial clearance must be positive")
    if not 0.0 < motion.pregrasp_width_m <= 0.088:
        raise ValueError("pregrasp width must be in (0, 88] mm")
    if not (
        config.cube_edge_m + 0.006
        <= motion.precontact_width_m
        < motion.pregrasp_width_m
    ):
        raise ValueError(
            "precontact width must leave at least 3 mm clearance per finger "
            "and remain below pregrasp width"
        )
    if not 0.0 < motion.release_width_m <= 0.088:
        raise ValueError("release width must be in (0, 88] mm")
    if not 0.02 <= motion.closing_torque_nm <= 0.30:
        raise ValueError("closing torque must be in [0.02, 0.30] N.m")
    if not 0.0 <= motion.minimum_gripper_width_m < config.cube_edge_m:
        raise ValueError("minimum gripper width must be below the cube edge")
    if not 0.01 <= motion.velocity_scale <= 1.0:
        raise ValueError("velocity scale must be in [0.01, 1.0]")
    if motion.gripper_duration_s < 0.5 or motion.gripper_timeout_s < 0.5:
        raise ValueError("gripper durations must be at least 0.5 seconds")
    if not 0.0 <= motion.contact_hold_s <= 5.0 or not 0.0 <= motion.settle_s <= 5.0:
        raise ValueError("gripper hold and settle times must be in [0, 5] seconds")
