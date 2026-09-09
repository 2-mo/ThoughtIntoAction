"""ArUco square pose with planar ambiguity rejection and native RGB intrinsics."""

import cv2
import numpy as np

from robot_core.transforms import from_rt, pose_error


def target_spec(size_m, marker_id=0):
    if not np.isfinite(size_m) or not 0.005 <= size_m <= 1:
        raise ValueError("Measure the black marker square; size must be in metres")
    if not 0 <= marker_id < 50:
        raise ValueError("DICT_4X4_50 marker ID must be 0..49")
    return {"dictionary": "DICT_4X4_50", "id": marker_id,
            "black_square_size_m": float(size_m)}


def marker_svg(spec):
    """Physical SVG units describe the black square, excluding a white border."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    bits = cv2.aruco.generateImageMarker(dictionary, spec["id"], 6)
    cells = [f'<rect x="{x + 1}" y="{y + 1}" width="1" height="1"/>'
             for y in range(6) for x in range(6) if bits[y, x] == 0]
    side_mm = spec["black_square_size_m"] * 1000 * 8 / 6
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{side_mm:.6f}mm" '
            f'height="{side_mm:.6f}mm" viewBox="0 0 8 8">'
            '<rect width="8" height="8" fill="white"/>' + ''.join(cells) + '</svg>\n')


def normalize_pixels(pixels, intrinsics):
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
    k = np.asarray(intrinsics["matrix"], dtype=float)
    model = intrinsics["distortion_model"].split(".")[-1]
    if model == "none":
        return (pixels - k[:2, 2]) / np.diag(k)[:2]
    if model not in {"brown_conrady", "inverse_brown_conrady"}:
        # Modified Brown has a different tangential mapping. Never silently
        # treat it as standard OpenCV Brown as the desktop helper did.
        raise ValueError(f"Unsupported RGB distortion model: {model}")
    import pyrealsense2 as rs
    native = rs.intrinsics()
    native.width, native.height = intrinsics["width"], intrinsics["height"]
    native.fx, native.fy, native.ppx, native.ppy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    native.model = getattr(rs.distortion, model)
    native.coeffs = intrinsics["coefficients"]
    # SDK implementation avoids reimplementing inverse Brown conventions.
    return np.array([rs.rs2_deproject_pixel_to_point(native, p.tolist(), 1.)[:2] for p in pixels])


def square_pose(pixels, intrinsics, size_m):
    h = size_m / 2
    objects = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    observed = normalize_pixels(pixels, intrinsics)
    solutions = cv2.solvePnPGeneric(objects, observed, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    focal = np.diag(intrinsics["matrix"])[:2]
    for rotation, translation in zip(solutions[1], solutions[2]):
        t = from_rt(cv2.Rodrigues(rotation)[0], translation)
        if np.any((objects @ t[:3, :3].T + t[:3, 3])[:, 2] <= 0):
            continue
        projected = cv2.projectPoints(objects, rotation, translation, np.eye(3), None)[0].reshape(-1, 2)
        error = float(np.sqrt(np.mean(np.sum(((projected - observed) * focal)**2, axis=1))))
        if np.isfinite(error):
            candidates.append((error, t))
    candidates.sort(key=lambda item: item[0])
    if not candidates or candidates[0][0] > 1.0:
        raise ValueError("Marker PnP reprojection error exceeds 1 undistorted pixel")
    if len(candidates) > 1:
        gap = candidates[1][0] - candidates[0][0]
        _, angle = pose_error(candidates[0][1], candidates[1][1])
        if gap < 0.15 and angle > 5:
            raise ValueError("Ambiguous planar marker pose; tilt/view the board from another pose")
    return candidates[0][1], candidates[0][0]


class MarkerDetector:
    def __init__(self, spec, intrinsics):
        self.spec, self.intrinsics = spec, intrinsics
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

    def detect(self, image):
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None:
            raise ValueError("Marker not visible")
        matches = np.flatnonzero(ids.ravel() == self.spec["id"])
        if len(matches) != 1:
            raise ValueError("Expected exactly one configured marker")
        pixels = corners[matches[0]].reshape(4, 2)
        if np.any(pixels < 8) or np.any(pixels > np.array([image.shape[1], image.shape[0]]) - 8):
            raise ValueError("Marker too close to image boundary")
        t, error = square_pose(pixels, self.intrinsics, self.spec["black_square_size_m"])
        return {"T_camera_target": t, "reprojection_error_undistorted_px": error,
                "corners_px": pixels.tolist()}
