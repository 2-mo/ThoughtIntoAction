"""D455 RGB/RGB-D cube observation in the ARX X5 base frame."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
import numpy as np

from legacy.hand_eye.core.geometry import camera_to_base
from legacy.hand_eye.core.projection import (
    normalize_image_points,
    normalized_to_image_points,
)
from legacy.hand_eye.core.schema import CameraIntrinsics
from legacy.hand_eye.core.transforms import invert_transform, validate_transform
from legacy.arx_tools.rgb_cube_detector import RgbCubeDetector

from legacy.cube.config import ExperimentConfig, PerceptionConfig, TableConfig


@dataclass(frozen=True)
class CalibrationData:
    T_base_camera: np.ndarray
    camera: CameraIntrinsics
    source_path: Path


@dataclass(frozen=True)
class FramePose:
    base_xyz_m: tuple[float, float, float]
    yaw_rad: float
    size_m: float
    confidence: float
    source: str
    depth_residual_m: float | None
    timestamp: float
    depth_xy_delta_m: float | None = None
    depth_consistent: bool = False
    candidate_id: str | None = None
    color: str | None = None


@dataclass(frozen=True)
class CubeObservation:
    base_xyz_m: tuple[float, float, float]
    yaw_rad: float
    size_m: float
    confidence: float
    position_source: str
    accepted_samples: int
    depth_samples: int
    xy_rms_m: float
    yaw_rms_rad: float
    timestamp: float
    candidate_id: str | None = None
    color: str | None = None

    def as_dict(self) -> dict[str, Any]:
        result = {
            "base_xyz_m": list(self.base_xyz_m),
            "yaw_rad": self.yaw_rad,
            "yaw_deg": math.degrees(self.yaw_rad),
            "size_m": self.size_m,
            "confidence": self.confidence,
            "position_source": self.position_source,
            "accepted_samples": self.accepted_samples,
            "depth_samples": self.depth_samples,
            "xy_rms_m": self.xy_rms_m,
            "yaw_rms_rad": self.yaw_rms_rad,
            "timestamp": self.timestamp,
        }
        if self.candidate_id is not None:
            result["candidate_id"] = self.candidate_id
        if self.color is not None:
            result["color"] = self.color
        return result


@dataclass(frozen=True)
class CaptureResult:
    observation: CubeObservation
    depth_available: bool
    debug_image_path: Path | None


@dataclass(frozen=True)
class SceneCaptureResult:
    """A static, calibrated snapshot of every stable tabletop cube."""

    observations: tuple[CubeObservation, ...]
    frames_captured: int
    depth_available: bool
    debug_image_path: Path | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "cube_count": len(self.observations),
            "frames_captured": self.frames_captured,
            "depth_available": self.depth_available,
            "debug_image_path": (
                None if self.debug_image_path is None else str(self.debug_image_path)
            ),
            "cubes": [observation.as_dict() for observation in self.observations],
        }


def load_enabled_calibration(path: str | Path) -> CalibrationData:
    source = Path(path).expanduser().resolve()
    value = json.loads(source.read_text(encoding="utf-8"))
    profile = value.get("profile")
    if profile not in {"eye-to-hand", "dual-camera-hand-eye"}:
        raise RuntimeError("calibration is not an eye-to-hand result")
    if value.get("enabled") is not True:
        raise RuntimeError("eye-to-hand calibration is not enabled")
    if value.get("stale") is True:
        raise RuntimeError("eye-to-hand calibration is stale")
    validation = value.get("validation")
    if not isinstance(validation, dict) or validation.get("passed") is not True:
        raise RuntimeError("eye-to-hand calibration has not passed validation")
    camera = value.get("camera")
    if profile == "dual-camera-hand-eye":
        cameras = value.get("cameras")
        if isinstance(cameras, dict):
            camera = cameras.get("eye_to_hand", camera)
    if not isinstance(camera, dict):
        raise RuntimeError("eye-to-hand camera intrinsics are unavailable")
    return CalibrationData(
        T_base_camera=validate_transform(
            np.asarray(value["T_base_camera"], dtype=np.float64),
            name="T_base_camera",
        ),
        camera=CameraIntrinsics.from_dict(camera),
        source_path=source,
    )


class RealSenseFrameSource:
    """Own one D455 pipeline and degrade cleanly to RGB-only capture."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        fps: int,
        serial: str | None,
        warmup_frames: int,
        rgb_only: bool = False,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.serial = serial
        self.warmup_frames = int(warmup_frames)
        self.rgb_only = bool(rgb_only)
        self.pipeline: Any | None = None
        self.align: Any | None = None
        self.depth_scale: float | None = None
        self.depth_available = False
        self.intrinsics: CameraIntrinsics | None = None
        self.active_serial: str | None = None

    def _start_pipeline(self, rs: Any, with_depth: bool) -> tuple[Any, Any]:
        pipeline = rs.pipeline()
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.rgb8,
            self.fps,
        )
        if with_depth:
            config.enable_stream(
                rs.stream.depth,
                self.width,
                self.height,
                rs.format.z16,
                self.fps,
            )
        try:
            return pipeline, pipeline.start(config)
        except BaseException:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            raise

    def start(self) -> CameraIntrinsics:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is required; use .venv-handeye/bin/python"
            ) from exc
        requested_depth = not self.rgb_only
        try:
            pipeline, profile = self._start_pipeline(rs, requested_depth)
            depth_available = requested_depth
        except BaseException as depth_error:
            if not requested_depth:
                raise RuntimeError(f"D455 RGB stream failed to start: {depth_error}") from depth_error
            print(f"D455 depth unavailable; continuing RGB-only: {depth_error}")
            try:
                pipeline, profile = self._start_pipeline(rs, False)
            except BaseException as rgb_error:
                raise RuntimeError(f"D455 RGB stream failed to start: {rgb_error}") from rgb_error
            depth_available = False

        while True:
            try:
                device = profile.get_device()
                active_serial = str(device.get_info(rs.camera_info.serial_number))
                if self.serial and active_serial != self.serial:
                    raise RuntimeError(
                        f"opened D455 serial {active_serial}, expected {self.serial}"
                    )
                color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
                values = color_profile.get_intrinsics()
                intrinsics = CameraIntrinsics(
                    width=int(values.width),
                    height=int(values.height),
                    fps=self.fps,
                    camera_matrix=np.asarray(
                        [
                            [values.fx, 0.0, values.ppx],
                            [0.0, values.fy, values.ppy],
                            [0.0, 0.0, 1.0],
                        ],
                        dtype=np.float64,
                    ),
                    distortion_coefficients=np.asarray(values.coeffs[:5], dtype=np.float64),
                    distortion_model=str(values.model),
                    serial=active_serial,
                )
                depth_scale = None
                align = None
                if depth_available:
                    depth_scale = float(device.first_depth_sensor().get_depth_scale())
                    align = rs.align(rs.stream.color)
                break
            except BaseException as depth_setup_error:
                if not depth_available:
                    try:
                        pipeline.stop()
                    except RuntimeError:
                        pass
                    raise
                print(f"D455 depth setup unavailable; continuing RGB-only: {depth_setup_error}")
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
                try:
                    pipeline, profile = self._start_pipeline(rs, False)
                except BaseException as rgb_error:
                    raise RuntimeError(
                        f"D455 RGB stream failed after depth setup error: {rgb_error}"
                    ) from rgb_error
                depth_available = False

        self.pipeline = pipeline
        self.align = align
        self.depth_scale = depth_scale
        self.depth_available = depth_available
        self.intrinsics = intrinsics
        self.active_serial = active_serial
        try:
            for _ in range(max(0, self.warmup_frames)):
                self.read()
        except BaseException:
            self.stop()
            raise
        return intrinsics

    def read(self) -> tuple[np.ndarray, np.ndarray | None, float]:
        if self.pipeline is None:
            raise RuntimeError("D455 is not started")
        frames = self.pipeline.wait_for_frames(5000)
        color_frames = frames
        depth_m: np.ndarray | None = None
        if self.align is not None:
            try:
                color_frames = self.align.process(frames)
                depth_frame = color_frames.get_depth_frame()
                if depth_frame and self.depth_scale is not None:
                    depth_m = (
                        np.asanyarray(depth_frame.get_data()).astype(np.float64)
                        * self.depth_scale
                    )
            except RuntimeError:
                # A missing depth sample must not take down RGB perception.
                color_frames = frames
                depth_m = None
        color = color_frames.get_color_frame()
        if not color:
            color = frames.get_color_frame()
        if not color:
            raise RuntimeError("D455 did not return an RGB frame")
        return np.asanyarray(color.get_data()).copy(), depth_m, time.time()

    def stop(self) -> None:
        pipeline = self.pipeline
        self.pipeline = None
        self.align = None
        self.depth_scale = None
        self.depth_available = False
        if pipeline is not None:
            try:
                pipeline.stop()
            except RuntimeError:
                pass

    def __enter__(self) -> "RealSenseFrameSource":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()


def project_table_polygon(
    table: TableConfig,
    calibration: CalibrationData,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Project the configured tabletop into RGB pixels for detector masking."""
    cosine = math.cos(table.yaw_rad)
    sine = math.sin(table.yaw_rad)
    local = np.asarray(
        [
            [-table.width_m / 2.0, -table.depth_m / 2.0],
            [table.width_m / 2.0, -table.depth_m / 2.0],
            [table.width_m / 2.0, table.depth_m / 2.0],
            [-table.width_m / 2.0, table.depth_m / 2.0],
        ],
        dtype=np.float64,
    )
    points_base = np.column_stack(
        (
            table.center_x_m + cosine * local[:, 0] - sine * local[:, 1],
            table.center_y_m + sine * local[:, 0] + cosine * local[:, 1],
            np.full(4, table.top_z_m, dtype=np.float64),
        )
    )
    T_camera_base = invert_transform(calibration.T_base_camera)
    points_camera = (
        T_camera_base[:3, :3] @ points_base.T
        + T_camera_base[:3, 3, None]
    ).T
    # The D455 is mounted at the table edge, so a valid large tabletop can
    # straddle the camera plane. Clip it against the RGB viewing frustum in
    # camera space before projection; projecting a near-plane intersection
    # outside the sensor would make the distortion inverse numerically diverge.
    image_corners = np.asarray(
        [
            [0.0, 0.0],
            [intrinsics.width - 1.0, 0.0],
            [intrinsics.width - 1.0, intrinsics.height - 1.0],
            [0.0, intrinsics.height - 1.0],
        ],
        dtype=np.float64,
    )
    corner_rays = normalize_image_points(image_corners, intrinsics)
    x_min, y_min = np.min(corner_rays, axis=0)
    x_max, y_max = np.max(corner_rays, axis=0)

    def clip_half_space(
        polygon: list[np.ndarray],
        signed_distance,
    ) -> list[np.ndarray]:
        result: list[np.ndarray] = []
        for index, current in enumerate(polygon):
            previous = polygon[index - 1]
            current_distance = float(signed_distance(current))
            previous_distance = float(signed_distance(previous))
            current_inside = current_distance >= 0.0
            previous_inside = previous_distance >= 0.0
            if current_inside != previous_inside:
                fraction = previous_distance / (previous_distance - current_distance)
                result.append(previous + fraction * (current - previous))
            if current_inside:
                result.append(current)
        return result

    clipped = [point.copy() for point in points_camera]
    for distance in (
        lambda point: point[2] - 0.05,
        lambda point: point[0] - x_min * point[2],
        lambda point: x_max * point[2] - point[0],
        lambda point: point[1] - y_min * point[2],
        lambda point: y_max * point[2] - point[1],
    ):
        clipped = clip_half_space(clipped, distance)
        if len(clipped) < 3:
            break
    if len(clipped) < 3:
        raise RuntimeError("configured tabletop is outside the D455 field of view")
    visible = np.asarray(clipped, dtype=np.float64)
    normalized = visible[:, :2] / visible[:, 2, None]
    return normalized_to_image_points(normalized, intrinsics)


def intersect_pixel_with_base_plane(
    pixel_xy: tuple[float, float] | list[float] | np.ndarray,
    plane_z_m: float,
    intrinsics: CameraIntrinsics,
    T_base_camera: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    pixel = np.asarray(pixel_xy, dtype=np.float64).reshape(1, 2)
    normalized = normalize_image_points(pixel, intrinsics)[0]
    ray_camera = np.asarray([normalized[0], normalized[1], 1.0], dtype=np.float64)
    transform = validate_transform(T_base_camera, name="T_base_camera")
    ray_base = transform[:3, :3] @ ray_camera
    origin_base = transform[:3, 3]
    if abs(float(ray_base[2])) < 1e-7:
        raise RuntimeError("D455 ray is parallel to the cube top plane")
    scale = (float(plane_z_m) - float(origin_base[2])) / float(ray_base[2])
    if not math.isfinite(scale) or scale <= 0.0:
        raise RuntimeError("cube top plane is behind the D455")
    return origin_base + ray_base * scale, ray_camera * scale


def _inside_table(point_xy: np.ndarray, table: TableConfig, margin_m: float) -> bool:
    delta = np.asarray(point_xy, dtype=np.float64) - np.asarray(
        [table.center_x_m, table.center_y_m], dtype=np.float64
    )
    cosine = math.cos(table.yaw_rad)
    sine = math.sin(table.yaw_rad)
    local_x = cosine * delta[0] + sine * delta[1]
    local_y = -sine * delta[0] + cosine * delta[1]
    return (
        abs(float(local_x)) <= table.width_m / 2.0 - margin_m
        and abs(float(local_y)) <= table.depth_m / 2.0 - margin_m
    )


def _median_depth(
    depth_m: np.ndarray | None,
    center_px: tuple[float, float] | list[float],
    radius: int,
) -> float | None:
    if depth_m is None or depth_m.ndim != 2:
        return None
    x = int(round(float(center_px[0])))
    y = int(round(float(center_px[1])))
    height, width = depth_m.shape
    if not 0 <= x < width or not 0 <= y < height:
        return None
    patch = depth_m[
        max(0, y - radius) : min(height, y + radius + 1),
        max(0, x - radius) : min(width, x + radius + 1),
    ].reshape(-1)
    valid = patch[np.isfinite(patch) & (patch > 0.08) & (patch < 4.0)]
    return None if valid.size < 3 else float(np.median(valid))


def estimate_frame_pose(
    detection: dict[str, Any],
    depth_m: np.ndarray | None,
    calibration: CalibrationData,
    intrinsics: CameraIntrinsics,
    config: ExperimentConfig,
    timestamp: float,
) -> FramePose:
    if detection.get("status") != "tracking":
        raise ValueError("cube is not currently tracked")
    confidence = float(detection.get("confidence", 0.0))
    if confidence < config.perception.minimum_confidence:
        raise ValueError("cube confidence is below the configured threshold")
    center_px = detection.get("center_px")
    axis_px = detection.get("axis_px")
    corners_px = detection.get("corners_px")
    if not isinstance(center_px, list) or not isinstance(axis_px, list):
        raise ValueError("cube center or axis is missing")
    top_z = config.table.top_z_m + config.cube_edge_m
    center_base, _ = intersect_pixel_with_base_plane(
        center_px,
        top_z,
        intrinsics,
        calibration.T_base_camera,
    )

    axis = np.asarray(axis_px, dtype=np.float64)
    axis /= max(float(np.linalg.norm(axis)), 1e-9)
    axis_length = max(8.0, float(detection.get("size_px", 20.0)) * 0.4)
    before, _ = intersect_pixel_with_base_plane(
        np.asarray(center_px, dtype=np.float64) - axis * axis_length,
        top_z,
        intrinsics,
        calibration.T_base_camera,
    )
    after, _ = intersect_pixel_with_base_plane(
        np.asarray(center_px, dtype=np.float64) + axis * axis_length,
        top_z,
        intrinsics,
        calibration.T_base_camera,
    )
    yaw = math.atan2(float(after[1] - before[1]), float(after[0] - before[0]))

    if not isinstance(corners_px, list) or len(corners_px) != 4:
        raise ValueError("cube corners are missing")
    corners = [
        intersect_pixel_with_base_plane(
            corner,
            top_z,
            intrinsics,
            calibration.T_base_camera,
        )[0]
        for corner in corners_px
    ]
    sizes = [
        float(np.linalg.norm(corners[(index + 1) % 4][:2] - corners[index][:2]))
        for index in range(4)
    ]
    size_m = float(np.mean(sizes))
    if not config.perception.minimum_size_m <= size_m <= config.perception.maximum_size_m:
        raise ValueError(f"detected cube size {1000.0 * size_m:.1f} mm is implausible")

    # The table plane is the high-precision metric constraint for this setup.
    # D455 depth is sampled from the same pixel ray, so it cannot improve its
    # lateral location; use it to reject inconsistent captures instead of
    # letting one noisy depth sample move the grasp target.
    source = "rgb_plane"
    depth_residual: float | None = None
    depth_xy_delta: float | None = None
    depth_consistent = False
    depth_value = _median_depth(
        depth_m,
        center_px,
        config.perception.depth_patch_radius_px,
    )
    if depth_value is not None:
        normalized = normalize_image_points(
            np.asarray(center_px, dtype=np.float64).reshape(1, 2),
            intrinsics,
        )[0]
        point_camera = np.asarray(
            [normalized[0] * depth_value, normalized[1] * depth_value, depth_value],
            dtype=np.float64,
        )
        point_base = camera_to_base(point_camera, calibration.T_base_camera)
        depth_residual = float(point_base[2] - top_z)
        depth_xy_delta = float(np.linalg.norm(point_base[:2] - center_base[:2]))
        if abs(depth_residual) <= config.perception.maximum_depth_residual_m:
            depth_consistent = True
            source = "rgb_plane+depth_check"

    if not _inside_table(center_base[:2], config.table, config.cube_edge_m / 2.0):
        raise ValueError("detected cube center is outside the configured tabletop")
    return FramePose(
        base_xyz_m=(
            float(center_base[0]),
            float(center_base[1]),
            float(config.table.top_z_m + config.cube_edge_m / 2.0),
        ),
        yaw_rad=float(yaw),
        size_m=size_m,
        confidence=confidence,
        source=source,
        depth_residual_m=depth_residual,
        timestamp=float(timestamp),
        depth_xy_delta_m=depth_xy_delta,
        depth_consistent=depth_consistent,
        candidate_id=(
            str(detection["candidate_id"])
            if detection.get("candidate_id") not in (None, "")
            else None
        ),
        color=(
            str(detection["color"])
            if isinstance(detection.get("color"), str)
            else None
        ),
    )


def _wrap_square_angle(value: np.ndarray | float) -> np.ndarray | float:
    period = math.pi / 2.0
    return (np.asarray(value) + period / 2.0) % period - period / 2.0


def _square_mean(values: np.ndarray) -> float:
    vector = np.mean(np.exp(4.0j * values))
    if abs(vector) < 1e-9:
        raise RuntimeError("cube yaw samples are inconsistent")
    return float(_wrap_square_angle(np.angle(vector) / 4.0))


def stabilize_observations(
    samples: list[FramePose],
    perception: PerceptionConfig,
) -> CubeObservation:
    """Reject outliers and combine square-symmetric pose samples robustly."""
    if len(samples) < perception.required_samples:
        raise RuntimeError(
            f"only {len(samples)} valid cube samples; need {perception.required_samples}"
        )
    xy = np.asarray([sample.base_xyz_m[:2] for sample in samples], dtype=np.float64)
    yaw = np.asarray([sample.yaw_rad for sample in samples], dtype=np.float64)
    median_xy = np.median(xy, axis=0)
    center_yaw = _square_mean(yaw)
    xy_error = np.linalg.norm(xy - median_xy, axis=1)
    yaw_error = np.abs(np.asarray(_wrap_square_angle(yaw - center_yaw)))
    xy_limit = max(0.006, 3.0 * float(np.median(xy_error)))
    yaw_limit = max(math.radians(10.0), 3.0 * float(np.median(yaw_error)))
    accepted = (xy_error <= xy_limit) & (yaw_error <= yaw_limit)
    if int(np.count_nonzero(accepted)) < perception.required_samples:
        raise RuntimeError("cube observations contain too many position/yaw outliers")

    indices = np.flatnonzero(accepted)
    accepted_samples = [samples[int(index)] for index in indices]
    depth_samples = [sample for sample in accepted_samples if sample.depth_consistent]
    stable_xy = np.asarray([sample.base_xyz_m[:2] for sample in accepted_samples], dtype=np.float64)
    final_xy = np.median(stable_xy, axis=0)
    accepted_xy = xy[accepted]
    xy_rms = float(np.sqrt(np.mean(np.sum((accepted_xy - final_xy) ** 2, axis=1))))
    accepted_yaw = yaw[accepted]
    final_yaw = _square_mean(accepted_yaw)
    yaw_rms = float(
        np.sqrt(np.mean(np.asarray(_wrap_square_angle(accepted_yaw - final_yaw)) ** 2))
    )
    if xy_rms > perception.maximum_xy_rms_m:
        raise RuntimeError(
            f"cube position is not stable: RMS {1000.0 * xy_rms:.1f} mm"
        )
    if yaw_rms > perception.maximum_yaw_rms_rad:
        raise RuntimeError(
            f"cube yaw is not stable: RMS {math.degrees(yaw_rms):.1f} deg"
        )
    candidate_ids = {sample.candidate_id for sample in accepted_samples if sample.candidate_id}
    if len(candidate_ids) > 1:
        raise RuntimeError("cube observations contain multiple candidate IDs")
    colors = {sample.color for sample in accepted_samples if sample.color}
    color = next(iter(colors)) if len(colors) == 1 else None
    return CubeObservation(
        base_xyz_m=(
            float(final_xy[0]),
            float(final_xy[1]),
            float(np.median([sample.base_xyz_m[2] for sample in accepted_samples])),
        ),
        yaw_rad=final_yaw,
        size_m=float(np.median([sample.size_m for sample in accepted_samples])),
        confidence=float(np.median([sample.confidence for sample in accepted_samples])),
        position_source="rgb_plane+depth_check" if len(depth_samples) >= 3 else "rgb_plane",
        accepted_samples=len(accepted_samples),
        depth_samples=len(depth_samples),
        xy_rms_m=xy_rms,
        yaw_rms_rad=yaw_rms,
        timestamp=max(sample.timestamp for sample in accepted_samples),
        candidate_id=next(iter(candidate_ids), None),
        color=color,
    )


def _annotate_debug_image(
    rgb: np.ndarray,
    detection: dict[str, Any],
    pose: FramePose | None,
    roi_polygon: np.ndarray,
) -> np.ndarray:
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    polygon = np.rint(roi_polygon).astype(np.int32)
    cv2.polylines(image, [polygon], True, (80, 160, 80), 1, cv2.LINE_AA)
    corners = detection.get("corners_px")
    center = detection.get("center_px")
    axis = detection.get("axis_px")
    if isinstance(corners, list) and len(corners) == 4:
        cv2.polylines(
            image,
            [np.rint(np.asarray(corners)).astype(np.int32)],
            True,
            (20, 220, 255),
            2,
            cv2.LINE_AA,
        )
    if isinstance(center, list) and len(center) == 2:
        point = tuple(np.rint(center).astype(int))
        cv2.circle(image, point, 4, (255, 255, 255), -1, cv2.LINE_AA)
        if isinstance(axis, list) and len(axis) == 2:
            endpoint = tuple(
                np.rint(np.asarray(center) + np.asarray(axis) * 45.0).astype(int)
            )
            cv2.arrowedLine(image, point, endpoint, (20, 220, 255), 2, cv2.LINE_AA)
    if pose is not None:
        text = (
            f"{pose.source}  x={1000 * pose.base_xyz_m[0]:.1f} "
            f"y={1000 * pose.base_xyz_m[1]:.1f} yaw={math.degrees(pose.yaw_rad):.1f}"
        )
        cv2.putText(image, text, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
    return image


def _annotate_scene_debug_image(
    rgb: np.ndarray,
    detection: dict[str, Any],
    pose: FramePose | None,
    roi_polygon: np.ndarray,
    observations: tuple[CubeObservation, ...],
    calibration: CalibrationData,
    intrinsics: CameraIntrinsics,
    config: ExperimentConfig,
) -> np.ndarray:
    """Overlay every accepted static-scene pose on the final RGB frame."""
    image = _annotate_debug_image(rgb, detection, pose, roi_polygon)
    T_camera_base = invert_transform(calibration.T_base_camera)
    colors = {
        "black": (0, 220, 255),
        "blue": (255, 120, 20),
        "white": (235, 235, 235),
    }
    for index, observation in enumerate(observations, start=1):
        point_base = np.asarray(
            [
                observation.base_xyz_m[0],
                observation.base_xyz_m[1],
                config.table.top_z_m + config.cube_edge_m,
            ],
            dtype=np.float64,
        )
        point_camera = T_camera_base[:3, :3] @ point_base + T_camera_base[:3, 3]
        if point_camera[2] <= 1e-6:
            continue
        pixel = normalized_to_image_points(
            (point_camera[:2] / point_camera[2]).reshape(1, 2),
            intrinsics,
        )[0]
        x, y = np.rint(pixel).astype(int)
        color = colors.get(observation.color or "", (90, 255, 90))
        cv2.drawMarker(
            image,
            (x, y),
            color,
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=20,
            thickness=2,
            line_type=cv2.LINE_AA,
        )
        label = observation.candidate_id or f"cube-{index}"
        cv2.putText(
            image,
            label,
            (x + 9, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2,
            cv2.LINE_AA,
        )
    return image


def capture_cube_observation(
    config: ExperimentConfig,
    *,
    rgb_only: bool = False,
    debug_image_path: str | Path | None = None,
) -> CaptureResult:
    calibration = load_enabled_calibration(config.calibration_path)
    expected_serial = config.camera.serial or calibration.camera.serial
    source = RealSenseFrameSource(
        width=config.camera.width,
        height=config.camera.height,
        fps=config.camera.fps,
        serial=expected_serial,
        warmup_frames=config.camera.warmup_frames,
        rgb_only=rgb_only,
    )
    samples: list[FramePose] = []
    last_rgb: np.ndarray | None = None
    last_detection: dict[str, Any] = {"status": "lost"}
    last_pose: FramePose | None = None
    result: CubeObservation | None = None
    with source:
        intrinsics = source.intrinsics
        if intrinsics is None:
            raise RuntimeError("D455 active RGB intrinsics are unavailable")
        if calibration.camera.serial and source.active_serial != calibration.camera.serial:
            raise RuntimeError(
                "active D455 does not match the camera used for eye-to-hand calibration"
            )
        roi_polygon = project_table_polygon(config.table, calibration, intrinsics)
        detector = RgbCubeDetector(downsample=2)
        for _ in range(config.perception.maximum_frames):
            rgb, depth_m, timestamp = source.read()
            detection = detector.process(
                rgb,
                timestamp=timestamp,
                roi_polygon=roi_polygon,
            )
            pose: FramePose | None = None
            try:
                pose = estimate_frame_pose(
                    detection,
                    depth_m,
                    calibration,
                    intrinsics,
                    config,
                    timestamp,
                )
            except (RuntimeError, ValueError):
                pass
            if pose is not None:
                samples.append(pose)
            last_rgb = rgb
            last_detection = detection
            last_pose = pose
            if len(samples) >= config.perception.required_samples:
                try:
                    result = stabilize_observations(samples, config.perception)
                except RuntimeError:
                    continue
                break
        depth_available = source.depth_available

    written_path: Path | None = None
    if result is None and debug_image_path is not None and last_rgb is not None:
        written_path = Path(debug_image_path).expanduser().resolve()
        written_path.parent.mkdir(parents=True, exist_ok=True)
        debug = _annotate_debug_image(
            last_rgb,
            last_detection,
            last_pose,
            roi_polygon,
        )
        if not cv2.imwrite(str(written_path), debug):
            raise RuntimeError(f"failed to write debug image: {written_path}")
    if result is None:
        try:
            result = stabilize_observations(samples, config.perception)
        except RuntimeError as exc:
            raise RuntimeError(
                f"{exc}; last detector status={last_detection.get('status', 'unknown')}"
            ) from exc
    if written_path is None and debug_image_path is not None and last_rgb is not None:
        written_path = Path(debug_image_path).expanduser().resolve()
        written_path.parent.mkdir(parents=True, exist_ok=True)
        debug = _annotate_debug_image(
            last_rgb,
            last_detection,
            last_pose,
            roi_polygon,
        )
        if not cv2.imwrite(str(written_path), debug):
            raise RuntimeError(f"failed to write debug image: {written_path}")
    return CaptureResult(
        observation=result,
        depth_available=depth_available,
        debug_image_path=written_path,
    )


def capture_cube_scene_observation(
    config: ExperimentConfig,
    *,
    rgb_only: bool = False,
    debug_image_path: str | Path | None = None,
) -> SceneCaptureResult:
    """Capture one static scene and return all individually stable cube poses.

    This intentionally scans for the complete frame budget. A pick sequence may
    rely on this initial snapshot, so returning after the first stable object
    would silently omit a later-detected cube.
    """
    calibration = load_enabled_calibration(config.calibration_path)
    expected_serial = config.camera.serial or calibration.camera.serial
    source = RealSenseFrameSource(
        width=config.camera.width,
        height=config.camera.height,
        fps=config.camera.fps,
        serial=expected_serial,
        warmup_frames=config.camera.warmup_frames,
        rgb_only=rgb_only,
    )
    samples_by_candidate: dict[str, list[FramePose]] = {}
    last_rgb: np.ndarray | None = None
    last_detection: dict[str, Any] = {"status": "lost"}
    last_pose: FramePose | None = None
    roi_polygon: np.ndarray | None = None
    frames_captured = 0
    with source:
        intrinsics = source.intrinsics
        if intrinsics is None:
            raise RuntimeError("D455 active RGB intrinsics are unavailable")
        if calibration.camera.serial and source.active_serial != calibration.camera.serial:
            raise RuntimeError(
                "active D455 does not match the camera used for eye-to-hand calibration"
            )
        roi_polygon = project_table_polygon(config.table, calibration, intrinsics)
        detector = RgbCubeDetector(downsample=2, require_table_roi=True)
        for _ in range(config.perception.maximum_frames):
            rgb, depth_m, timestamp = source.read()
            detection = detector.process(
                rgb,
                timestamp=timestamp,
                roi_polygon=roi_polygon,
            )
            candidates = detection.get("candidates")
            if isinstance(candidates, list):
                for candidate in candidates:
                    if not isinstance(candidate, dict):
                        continue
                    candidate_id = candidate.get("candidate_id")
                    if (
                        candidate.get("status") != "tracking"
                        or not isinstance(candidate_id, str)
                        or not candidate_id
                        or int(candidate.get("stable_frames", 0)) < 3
                    ):
                        continue
                    try:
                        pose = estimate_frame_pose(
                            candidate,
                            depth_m,
                            calibration,
                            intrinsics,
                            config,
                            timestamp,
                        )
                    except (RuntimeError, ValueError):
                        continue
                    samples_by_candidate.setdefault(candidate_id, []).append(pose)
                    last_pose = pose
            last_rgb = rgb
            last_detection = detection
            frames_captured += 1
        depth_available = source.depth_available

    written_path: Path | None = None
    if debug_image_path is not None and last_rgb is not None and roi_polygon is not None:
        written_path = Path(debug_image_path).expanduser().resolve()
        written_path.parent.mkdir(parents=True, exist_ok=True)
        debug = _annotate_debug_image(
            last_rgb,
            last_detection,
            last_pose,
            roi_polygon,
        )
        if not cv2.imwrite(str(written_path), debug):
            raise RuntimeError(f"failed to write debug image: {written_path}")

    observations: list[CubeObservation] = []
    for candidate_id, samples in sorted(samples_by_candidate.items()):
        try:
            observation = stabilize_observations(samples, config.perception)
        except RuntimeError:
            continue
        observations.append(observation)
    if not observations:
        raise RuntimeError("no stable tabletop cubes found in scene scan")
    observations.sort(key=lambda observation: observation.candidate_id or "")
    if written_path is not None and last_rgb is not None and roi_polygon is not None:
        debug = _annotate_scene_debug_image(
            last_rgb,
            last_detection,
            last_pose,
            roi_polygon,
            tuple(observations),
            calibration,
            intrinsics,
            config,
        )
        if not cv2.imwrite(str(written_path), debug):
            raise RuntimeError(f"failed to write debug image: {written_path}")
    return SceneCaptureResult(
        observations=tuple(observations),
        frames_captured=frames_captured,
        depth_available=depth_available,
        debug_image_path=written_path,
    )
