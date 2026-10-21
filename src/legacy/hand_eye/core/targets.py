"""High-precision ChArUco and chessboard target detection."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from legacy.hand_eye.core.projection import normalize_image_points, normalized_error_to_pixels
from legacy.hand_eye.core.schema import CameraIntrinsics
from legacy.hand_eye.core.transforms import make_transform


@dataclass(frozen=True)
class TargetDetection:
    T_camera_target: np.ndarray
    reprojection_error_px: float
    annotated: np.ndarray
    label: str
    point_count: int


class ArucoTarget:
    def __init__(
        self,
        *,
        marker_length_m: float,
        marker_id: int = 0,
        dictionary: str = "DICT_4X4_50",
    ) -> None:
        cv2 = _cv2()
        if marker_length_m <= 0.0:
            raise ValueError("ArUco marker length must be positive")
        if marker_id < 0:
            raise ValueError("ArUco marker ID must be non-negative")
        self.marker_length_m = float(marker_length_m)
        self.marker_id = int(marker_id)
        self.dictionary_name = dictionary
        dictionary_id = getattr(cv2.aruco, dictionary)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if self.marker_id >= self.dictionary.bytesList.shape[0]:
            raise ValueError(
                f"marker ID {self.marker_id} is outside {dictionary}"
            )
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5
        parameters.cornerRefinementMaxIterations = 50
        parameters.cornerRefinementMinAccuracy = 0.01
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, parameters)

    def detect(
        self,
        image: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> TargetDetection | None:
        cv2 = _cv2()
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None or not len(ids):
            return None
        matches = np.flatnonzero(ids.reshape(-1) == self.marker_id)
        if not len(matches):
            return None
        chosen = int(matches[0])
        pixels = corners[chosen].reshape(4, 2)
        half = self.marker_length_m / 2.0
        # OpenCV ArUco corner order is top-left, top-right, bottom-right,
        # bottom-left. This is also the exact order required by IPPE_SQUARE.
        object_points = np.asarray(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        detection = _solve_square_target(
            object_points,
            pixels,
            intrinsics,
            image,
            axis_length=self.marker_length_m * 0.55,
        )
        if detection is None:
            return None
        transform, error, annotated = detection
        cv2.aruco.drawDetectedMarkers(
            annotated,
            [corners[chosen]],
            ids[chosen : chosen + 1],
        )
        return TargetDetection(
            T_camera_target=transform,
            reprojection_error_px=error,
            annotated=annotated,
            label=f"aruco_{self.dictionary_name.lower()}_id{self.marker_id}",
            point_count=4,
        )

    def generate_image(
        self,
        path: Path,
        *,
        dpi: int = 300,
        margin_m: float = 0.010,
    ) -> dict[str, Any]:
        cv2 = _cv2()
        pixels_per_metre = float(dpi) / 0.0254
        marker_pixels = int(round(self.marker_length_m * pixels_per_metre))
        margin_pixels = int(round(margin_m * pixels_per_metre))
        marker = cv2.aruco.generateImageMarker(
            self.dictionary,
            self.marker_id,
            marker_pixels,
            borderBits=1,
        )
        image = cv2.copyMakeBorder(
            marker,
            margin_pixels,
            margin_pixels,
            margin_pixels,
            margin_pixels,
            cv2.BORDER_CONSTANT,
            value=255,
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"failed to write target image: {path}")
        total_length_m = self.marker_length_m + 2.0 * margin_m
        return {
            "path": str(path),
            "dpi": int(dpi),
            "pixel_size": [int(image.shape[1]), int(image.shape[0])],
            "printed_width_mm": total_length_m * 1000.0,
            "printed_height_mm": total_length_m * 1000.0,
            "marker_length_mm": self.marker_length_m * 1000.0,
            "marker_id": self.marker_id,
            "dictionary": self.dictionary_name,
            "white_margin_mm": margin_m * 1000.0,
            "print_scale_percent": 100,
        }


class CharucoTarget:
    def __init__(
        self,
        *,
        squares_x: int = 7,
        squares_y: int = 5,
        square_length_m: float = 0.025,
        marker_length_m: float = 0.018,
        dictionary: str = "DICT_4X4_50",
        min_corners: int = 10,
    ) -> None:
        cv2 = _cv2()
        if squares_x < 3 or squares_y < 3:
            raise ValueError("ChArUco board must have at least 3x3 squares")
        if not 0.0 < marker_length_m < square_length_m:
            raise ValueError("marker length must be smaller than square length")
        self.squares_x = int(squares_x)
        self.squares_y = int(squares_y)
        self.square_length_m = float(square_length_m)
        self.marker_length_m = float(marker_length_m)
        self.dictionary_name = dictionary
        self.min_corners = int(min_corners)
        dictionary_id = getattr(cv2.aruco, dictionary)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.board = cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y),
            self.square_length_m,
            self.marker_length_m,
            self.dictionary,
        )
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5
        parameters.cornerRefinementMaxIterations = 50
        parameters.cornerRefinementMinAccuracy = 0.01
        self.detector = cv2.aruco.CharucoDetector(
            self.board,
            detectorParams=parameters,
        )

    def detect(
        self,
        image: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> TargetDetection | None:
        cv2 = _cv2()
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.detector.detectBoard(image)
        )
        if (
            charuco_ids is None
            or charuco_corners is None
            or len(charuco_ids) < self.min_corners
        ):
            return None
        ids = charuco_ids.reshape(-1).astype(int)
        object_points = np.asarray(self.board.getChessboardCorners())[ids]
        pixels = charuco_corners.reshape(-1, 2)
        detection = _solve_planar_target(
            object_points,
            pixels,
            intrinsics,
            image,
            axis_length=self.square_length_m * 2.5,
        )
        if detection is None:
            return None
        transform, error, annotated = detection
        if marker_ids is not None and marker_corners is not None:
            cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)
        cv2.aruco.drawDetectedCornersCharuco(
            annotated, charuco_corners, charuco_ids, (92, 255, 132)
        )
        return TargetDetection(
            T_camera_target=transform,
            reprojection_error_px=error,
            annotated=annotated,
            label=(
                f"charuco_{self.squares_x}x{self.squares_y}_"
                f"{self.dictionary_name.lower()}"
            ),
            point_count=len(ids),
        )

    def generate_image(
        self,
        path: Path,
        *,
        dpi: int = 300,
        margin_m: float = 0.010,
    ) -> dict[str, Any]:
        cv2 = _cv2()
        board_width_m = self.squares_x * self.square_length_m
        board_height_m = self.squares_y * self.square_length_m
        width_m = board_width_m + 2.0 * margin_m
        height_m = board_height_m + 2.0 * margin_m
        pixels_per_metre = float(dpi) / 0.0254
        size = (
            int(round(width_m * pixels_per_metre)),
            int(round(height_m * pixels_per_metre)),
        )
        margin_pixels = int(round(margin_m * pixels_per_metre))
        image = self.board.generateImage(
            size,
            marginSize=margin_pixels,
            borderBits=1,
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"failed to write target image: {path}")
        return {
            "path": str(path),
            "dpi": int(dpi),
            "pixel_size": list(size),
            "printed_width_mm": width_m * 1000.0,
            "printed_height_mm": height_m * 1000.0,
            "board_width_mm": board_width_m * 1000.0,
            "board_height_mm": board_height_m * 1000.0,
            "square_length_mm": self.square_length_m * 1000.0,
            "white_margin_mm": margin_m * 1000.0,
            "print_scale_percent": 100,
        }


class ChessboardTarget:
    def __init__(
        self,
        *,
        columns: int = 8,
        rows: int = 6,
        square_length_m: float = 0.025,
    ) -> None:
        self.columns = int(columns)
        self.rows = int(rows)
        self.square_length_m = float(square_length_m)
        points = np.zeros((self.columns * self.rows, 3), dtype=np.float64)
        points[:, :2] = np.mgrid[0 : self.columns, 0 : self.rows].T.reshape(-1, 2)
        self.object_points = points * self.square_length_m

    def detect(
        self,
        image: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> TargetDetection | None:
        cv2 = _cv2()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern = (self.columns, self.rows)
        found, corners = cv2.findChessboardCornersSB(
            gray,
            pattern,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )
        if not found or corners is None:
            return None
        pixels = corners.reshape(-1, 2)
        detection = _solve_planar_target(
            self.object_points,
            pixels,
            intrinsics,
            image,
            axis_length=self.square_length_m * 2.5,
        )
        if detection is None:
            return None
        transform, error, annotated = detection
        cv2.drawChessboardCorners(annotated, pattern, corners, True)
        return TargetDetection(
            T_camera_target=transform,
            reprojection_error_px=error,
            annotated=annotated,
            label=f"chessboard_{self.columns}x{self.rows}",
            point_count=len(pixels),
        )


def build_target_detector(config: dict[str, Any]):
    target_type = str(config.get("type", "charuco")).lower()
    if target_type == "aruco":
        if "marker_length_m" not in config:
            raise ValueError("target.marker_length_m is required for ArUco")
        return ArucoTarget(
            marker_length_m=float(config["marker_length_m"]),
            marker_id=int(config.get("marker_id", 0)),
            dictionary=str(config.get("dictionary", "DICT_4X4_50")),
        )
    if target_type == "charuco":
        return CharucoTarget(
            squares_x=int(config.get("squares_x", 7)),
            squares_y=int(config.get("squares_y", 5)),
            square_length_m=float(config.get("square_length_m", 0.025)),
            marker_length_m=float(config.get("marker_length_m", 0.018)),
            dictionary=str(config.get("dictionary", "DICT_4X4_50")),
            min_corners=int(config.get("min_corners", 10)),
        )
    if target_type == "chessboard":
        return ChessboardTarget(
            columns=int(config.get("columns", 8)),
            rows=int(config.get("rows", 6)),
            square_length_m=float(config.get("square_length_m", 0.025)),
        )
    raise ValueError(f"unsupported target type: {target_type}")


def _solve_square_target(
    object_points: np.ndarray,
    pixels: np.ndarray,
    intrinsics: CameraIntrinsics,
    image: np.ndarray,
    *,
    axis_length: float,
) -> tuple[np.ndarray, float, np.ndarray] | None:
    cv2 = _cv2()
    normalized = normalize_image_points(pixels, intrinsics)
    unit_camera = np.eye(3, dtype=np.float64)
    no_distortion = np.zeros(5, dtype=np.float64)
    result = cv2.solvePnPGeneric(
        np.asarray(object_points, dtype=np.float64),
        normalized,
        unit_camera,
        no_distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not result[0]:
        return None
    candidates: list[tuple[float, np.ndarray, np.ndarray]] = []
    for rvec, tvec in zip(result[1], result[2]):
        if float(np.asarray(tvec).reshape(3)[2]) <= 0.0:
            continue
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            unit_camera,
            no_distortion,
        )
        error = normalized_error_to_pixels(
            normalized,
            projected.reshape(-1, 2),
            intrinsics,
        )
        candidates.append((error, rvec, tvec))
    if not candidates:
        return None
    _, rvec, tvec = min(candidates, key=lambda value: value[0])
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points,
            normalized,
            unit_camera,
            no_distortion,
            rvec,
            tvec,
        )
    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        unit_camera,
        no_distortion,
    )
    error = normalized_error_to_pixels(
        normalized,
        projected.reshape(-1, 2),
        intrinsics,
    )
    rotation, _ = cv2.Rodrigues(rvec)
    annotated = image.copy()
    _draw_normalized_axes(
        annotated,
        intrinsics,
        rvec,
        tvec,
        axis_length,
    )
    return make_transform(rotation, tvec.reshape(3)), error, annotated


def _solve_planar_target(
    object_points: np.ndarray,
    pixels: np.ndarray,
    intrinsics: CameraIntrinsics,
    image: np.ndarray,
    *,
    axis_length: float,
) -> tuple[np.ndarray, float, np.ndarray] | None:
    cv2 = _cv2()
    normalized = normalize_image_points(pixels, intrinsics)
    unit_camera = np.eye(3, dtype=np.float64)
    no_distortion = np.zeros(5, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(object_points, dtype=np.float64),
        normalized,
        unit_camera,
        no_distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(tvec[2]) <= 0.0:
        return None
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            np.asarray(object_points, dtype=np.float64),
            normalized,
            unit_camera,
            no_distortion,
            rvec,
            tvec,
        )
    projected, _ = cv2.projectPoints(
        np.asarray(object_points, dtype=np.float64),
        rvec,
        tvec,
        unit_camera,
        no_distortion,
    )
    error = normalized_error_to_pixels(
        normalized, projected.reshape(-1, 2), intrinsics
    )
    rotation, _ = cv2.Rodrigues(rvec)
    annotated = image.copy()
    _draw_normalized_axes(
        annotated,
        intrinsics,
        rvec,
        tvec,
        axis_length,
    )
    return make_transform(rotation, tvec.reshape(3)), error, annotated


def _draw_normalized_axes(
    image: np.ndarray,
    intrinsics: CameraIntrinsics,
    rvec: np.ndarray,
    tvec: np.ndarray,
    length: float,
) -> None:
    cv2 = _cv2()
    axes = np.asarray(
        [[0.0, 0.0, 0.0], [length, 0.0, 0.0], [0.0, length, 0.0], [0.0, 0.0, length]],
        dtype=np.float64,
    )
    projected, _ = cv2.projectPoints(
        axes,
        rvec,
        tvec,
        np.eye(3),
        np.zeros(5),
    )
    from legacy.hand_eye.core.projection import normalized_to_image_points

    pixels = normalized_to_image_points(projected.reshape(-1, 2), intrinsics)
    points = [tuple(int(round(component)) for component in point) for point in pixels]
    origin = points[0]
    cv2.line(image, origin, points[1], (40, 60, 255), 2, cv2.LINE_AA)
    cv2.line(image, origin, points[2], (70, 230, 80), 2, cv2.LINE_AA)
    cv2.line(image, origin, points[3], (255, 120, 40), 2, cv2.LINE_AA)


def _cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV contrib is required; run uv sync --only-group handeye"
        ) from exc
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("opencv-contrib-python is required for ChArUco")
    return cv2
