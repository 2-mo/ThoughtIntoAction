"""RealSense pixel normalization for OpenCV PnP."""

from __future__ import annotations

import numpy as np

from legacy.hand_eye.core.schema import CameraIntrinsics


def normalize_image_points(
    pixels: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Map RGB pixels to undistorted normalized camera coordinates."""
    values = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    fx = intrinsics.camera_matrix[0, 0]
    fy = intrinsics.camera_matrix[1, 1]
    ppx = intrinsics.camera_matrix[0, 2]
    ppy = intrinsics.camera_matrix[1, 2]
    distorted = np.column_stack(
        ((values[:, 0] - ppx) / fx, (values[:, 1] - ppy) / fy)
    )
    model = _model_name(intrinsics.distortion_model)
    if model in {"none", "distortion.none"}:
        return distorted
    if model.endswith("inverse_brown_conrady"):
        return _inverse_brown_to_undistorted(
            distorted, intrinsics.distortion_coefficients
        )
    if model.endswith("brown_conrady") or model.endswith("modified_brown_conrady"):
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV is required to normalize Brown-Conrady pixels") from exc
        return cv2.undistortPoints(
            values.reshape(-1, 1, 2),
            intrinsics.camera_matrix,
            intrinsics.distortion_coefficients,
        ).reshape(-1, 2)
    raise ValueError(f"unsupported RealSense distortion model: {model}")


def normalized_to_image_points(
    normalized: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Project undistorted normalized coordinates back to RGB pixels."""
    values = np.asarray(normalized, dtype=np.float64).reshape(-1, 2)
    model = _model_name(intrinsics.distortion_model)
    if model in {"none", "distortion.none"}:
        distorted = values
    elif model.endswith("inverse_brown_conrady"):
        # The RealSense inverse model maps distorted pixels to undistorted
        # rays. Invert it numerically; D455 coefficients are mild and this
        # fixed-point correction converges rapidly over the valid image.
        distorted = values.copy()
        for _ in range(12):
            estimate = _inverse_brown_to_undistorted(
                distorted, intrinsics.distortion_coefficients
            )
            distorted -= estimate - values
    elif model.endswith("brown_conrady") or model.endswith("modified_brown_conrady"):
        distorted = _forward_brown(values, intrinsics.distortion_coefficients)
    else:
        raise ValueError(f"unsupported RealSense distortion model: {model}")
    fx = intrinsics.camera_matrix[0, 0]
    fy = intrinsics.camera_matrix[1, 1]
    ppx = intrinsics.camera_matrix[0, 2]
    ppy = intrinsics.camera_matrix[1, 2]
    return np.column_stack(
        (distorted[:, 0] * fx + ppx, distorted[:, 1] * fy + ppy)
    )


def normalized_error_to_pixels(
    observed: np.ndarray,
    projected: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> float:
    first = normalized_to_image_points(observed, intrinsics)
    second = normalized_to_image_points(projected, intrinsics)
    delta = first - second
    return float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))


def _inverse_brown_to_undistorted(
    values: np.ndarray,
    coefficients: np.ndarray,
) -> np.ndarray:
    original_x = values[:, 0]
    original_y = values[:, 1]
    x = original_x.copy()
    y = original_y.copy()
    k1, k2, p1, p2, k3 = _five_coefficients(coefficients)
    # This is the exact ten-iteration algorithm used by
    # librealsense rs2_deproject_pixel_to_point for
    # RS2_DISTORTION_INVERSE_BROWN_CONRADY.
    for _ in range(10):
        r2 = x * x + y * y
        inverse_radial = 1.0 / (
            1.0 + ((k3 * r2 + k2) * r2 + k1) * r2
        )
        xq = x / inverse_radial
        yq = y / inverse_radial
        delta_x = 2.0 * p1 * xq * yq + p2 * (r2 + 2.0 * xq * xq)
        delta_y = 2.0 * p2 * xq * yq + p1 * (r2 + 2.0 * yq * yq)
        x = (original_x - delta_x) * inverse_radial
        y = (original_y - delta_y) * inverse_radial
    return np.column_stack((x, y))


def _forward_brown(values: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    x = values[:, 0]
    y = values[:, 1]
    k1, k2, p1, p2, k3 = _five_coefficients(coefficients)
    r2 = x * x + y * y
    factor = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    return np.column_stack(
        (
            x * factor + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
            y * factor + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y),
        )
    )


def _five_coefficients(coefficients: np.ndarray) -> tuple[float, ...]:
    padded = np.zeros(5, dtype=np.float64)
    source = np.asarray(coefficients, dtype=np.float64).reshape(-1)
    padded[: min(5, source.size)] = source[:5]
    return tuple(float(value) for value in padded)


def _model_name(value: str) -> str:
    return str(value).strip().lower().replace(" ", "_")
