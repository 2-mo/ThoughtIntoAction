"""RGB-only detector for colored, square tabletop cubes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class BackgroundRegion:
    """One connected change region in the configured tabletop ROI."""

    bbox_px: tuple[int, int, int, int]
    center_px: tuple[float, float]
    corners_px: tuple[tuple[float, float], ...]
    area_px: float
    rectangularity: float
    aspect_ratio: float
    score: float
    accepted: bool
    rejection: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "bbox_px": list(self.bbox_px),
            "center_px": [round(float(value), 2) for value in self.center_px],
            "corners_px": [
                [round(float(value), 2) for value in corner]
                for corner in self.corners_px
            ],
            "area_px": round(float(self.area_px), 2),
            "rectangularity": round(float(self.rectangularity), 4),
            "aspect_ratio": round(float(self.aspect_ratio), 4),
            "score": round(float(self.score), 4),
            "accepted": bool(self.accepted),
            "rejection": self.rejection,
        }


class TableBackgroundModel:
    """Explicit, frozen empty-table background with candidate extraction.

    The model is intentionally never updated during normal tracking. A caller
    must request a new capture, which prevents a stationary cube from being
    absorbed into the background.
    """

    def __init__(self, *, downsample: int = 2, capture_frames: int = 24) -> None:
        self.downsample = max(1, int(downsample))
        self.default_capture_frames = max(8, int(capture_frames))
        self._model: np.ndarray | None = None
        self._samples: list[np.ndarray] = []
        self._capture_remaining = 0
        self._capture_target = 0
        self._shape: tuple[int, int] | None = None
        self._roi_signature: tuple[float, ...] | None = None
        self._last_regions: tuple[BackgroundRegion, ...] = ()
        self._last_changed_pixels = 0
        self._frames_since_capture = 0

    @property
    def ready(self) -> bool:
        return self._model is not None

    def reset(self) -> None:
        self._model = None
        self._samples.clear()
        self._capture_remaining = 0
        self._capture_target = 0
        self._shape = None
        self._roi_signature = None
        self._last_regions = ()
        self._last_changed_pixels = 0
        self._frames_since_capture = 0

    def request_capture(self, frame_count: int | None = None) -> dict[str, Any]:
        count = self.default_capture_frames if frame_count is None else int(frame_count)
        if not 8 <= count <= 90:
            raise ValueError("background capture frame count must be in [8, 90]")
        self._model = None
        self._samples.clear()
        self._capture_target = count
        self._capture_remaining = count
        self._last_regions = ()
        return self.status()

    def _set_roi_signature(
        self,
        image_shape: tuple[int, int],
        roi_polygon: np.ndarray | None,
    ) -> None:
        points = None if roi_polygon is None else np.asarray(roi_polygon, dtype=np.float64)
        signature = (
            float(image_shape[0]),
            float(image_shape[1]),
            *(() if points is None else tuple(np.round(points.reshape(-1), 2))),
        )
        if self._roi_signature is not None and signature != self._roi_signature:
            # A changed tabletop calibration invalidates the old background.
            self._model = None
            self._samples.clear()
            self._capture_remaining = 0
            self._capture_target = 0
        self._roi_signature = signature

    def status(self) -> dict[str, Any]:
        if self._capture_remaining > 0:
            state = "capturing"
        elif self._model is not None:
            state = "ready"
        else:
            state = "missing"
        return {
            "state": state,
            "ready": self._model is not None,
            "capture_remaining": self._capture_remaining,
            "capture_target": self._capture_target,
            "frames_since_capture": self._frames_since_capture,
            "changed_pixels": self._last_changed_pixels,
            "candidate_count": len(self._last_regions),
            "regions": [region.as_dict() for region in self._last_regions],
        }

    @staticmethod
    def _gray(image: np.ndarray, downsample: int) -> np.ndarray:
        rgb = np.asarray(image)
        reduced = cv2.resize(
            np.ascontiguousarray(rgb[..., :3], dtype=np.uint8),
            (max(1, rgb.shape[1] // downsample), max(1, rgb.shape[0] // downsample)),
            interpolation=cv2.INTER_AREA,
        )
        return cv2.GaussianBlur(cv2.cvtColor(reduced, cv2.COLOR_RGB2GRAY), (5, 5), 0)

    def _roi_mask(
        self,
        image_shape: tuple[int, int],
        roi_polygon: np.ndarray | None,
    ) -> np.ndarray:
        height, width = image_shape
        mask = np.zeros((height, width), dtype=np.uint8)
        if roi_polygon is None:
            mask[:, :] = 255
            return mask
        polygon = np.rint(np.asarray(roi_polygon, dtype=np.float64) / self.downsample)
        polygon = polygon.astype(np.int32).reshape(-1, 2)
        if len(polygon) >= 3:
            cv2.fillConvexPoly(mask, polygon, 255)
        return mask

    def _regions_from_mask(
        self,
        mask: np.ndarray,
        source_shape: tuple[int, int],
    ) -> tuple[BackgroundRegion, ...]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        reduced_area = float(mask.shape[0] * mask.shape[1])
        full_height, full_width = source_shape
        regions: list[BackgroundRegion] = []
        for contour in contours:
            reduced_contour_area = float(cv2.contourArea(contour))
            area = reduced_contour_area * self.downsample * self.downsample
            if area < 24.0:
                continue
            hull = cv2.convexHull(contour)
            rect = cv2.minAreaRect(hull)
            (cx, cy), (rect_width, rect_height), _ = rect
            short_side = min(float(rect_width), float(rect_height))
            long_side = max(float(rect_width), float(rect_height))
            if short_side < 2.0:
                continue
            aspect_ratio = long_side / max(short_side, 1e-6)
            rectangle_area = max(1.0, float(rect_width * rect_height))
            rectangularity = min(1.0, reduced_contour_area / rectangle_area)
            perimeter = cv2.arcLength(hull, True)
            polygon = cv2.approxPolyDP(hull, 0.04 * perimeter, True)
            raw_corners = (
                polygon.reshape(-1, 2).astype(np.float64)
                if len(polygon) == 4
                else cv2.boxPoints(rect).astype(np.float64)
            )
            center = np.asarray([cx, cy], dtype=np.float64)
            corners = RgbCubeDetector._ordered_corners(
                raw_corners * self.downsample,
                center * self.downsample,
            )
            x, y, width, height = cv2.boundingRect(hull)
            bbox = (
                int(x * self.downsample),
                int(y * self.downsample),
                int(width * self.downsample),
                int(height * self.downsample),
            )
            too_large = (
                area > reduced_area * 0.12
                or bbox[2] > full_width * 0.42
                or bbox[3] > full_height * 0.42
            )
            rejection = None
            if too_large:
                rejection = "large_change"
            elif aspect_ratio > 2.4:
                rejection = "aspect_ratio"
            elif rectangularity < 0.28:
                rejection = "rectangularity"
            accepted = rejection is None
            square_score = min(1.0, 1.5 / max(aspect_ratio, 1.0))
            score = min(
                1.0,
                0.42 * square_score
                + 0.34 * rectangularity
                + 0.24 * min(1.0, area / 900.0),
            )
            regions.append(
                BackgroundRegion(
                    bbox_px=bbox,
                    center_px=(float(center[0] * self.downsample), float(center[1] * self.downsample)),
                    corners_px=tuple((float(px), float(py)) for px, py in corners),
                    area_px=area,
                    rectangularity=rectangularity,
                    aspect_ratio=aspect_ratio,
                    score=score,
                    accepted=accepted,
                    rejection=rejection,
                )
            )
        regions.sort(key=lambda region: (region.accepted, region.score), reverse=True)
        return tuple(regions[:12])

    def update(
        self,
        image: np.ndarray,
        roi_polygon: np.ndarray | None,
    ) -> dict[str, Any]:
        gray = self._gray(image, self.downsample)
        self._set_roi_signature((int(image.shape[0]), int(image.shape[1])), roi_polygon)
        if self._capture_remaining > 0:
            self._samples.append(gray.copy())
            self._capture_remaining -= 1
            if self._capture_remaining == 0:
                self._model = np.median(np.stack(self._samples), axis=0).astype(np.float32)
                self._samples.clear()
                self._frames_since_capture = 0
            self._last_regions = ()
            self._last_changed_pixels = 0
            return self.status()
        if self._model is None:
            self._last_regions = ()
            self._last_changed_pixels = 0
            return self.status()

        roi_mask = self._roi_mask(gray.shape, roi_polygon)
        roi_pixels = roi_mask > 0
        if not np.any(roi_pixels):
            self._last_regions = ()
            self._last_changed_pixels = 0
            self._frames_since_capture += 1
            return self.status()
        difference = cv2.absdiff(gray, self._model.astype(np.uint8))
        # Keep the threshold conservative for wood grain, then use morphology
        # to join the two visible sides of a black cube.
        threshold = max(
            18.0,
            min(52.0, float(np.percentile(difference[roi_pixels], 92.0)) + 8.0),
        )
        changed = cv2.threshold(difference, threshold, 255, cv2.THRESH_BINARY)[1]
        changed = cv2.bitwise_and(changed, roi_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        changed = cv2.morphologyEx(changed, cv2.MORPH_OPEN, kernel)
        changed = cv2.morphologyEx(changed, cv2.MORPH_CLOSE, kernel, iterations=2)
        self._last_changed_pixels = int(np.count_nonzero(changed)) * self.downsample * self.downsample
        self._last_regions = self._regions_from_mask(
            changed,
            (int(image.shape[0]), int(image.shape[1])),
        )
        self._frames_since_capture += 1
        return self.status()


@dataclass(frozen=True)
class CubeDetection:
    status: str
    center_px: tuple[float, float] | None = None
    corners_px: tuple[tuple[float, float], ...] = ()
    axis_px: tuple[float, float] | None = None
    size_px: float | None = None
    confidence: float = 0.0
    timestamp: float = 0.0
    color: str | None = None

    def as_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "type": "cube_detection",
            "status": self.status,
            "confidence": round(float(self.confidence), 4),
            "timestamp": float(self.timestamp),
        }
        if self.center_px is not None:
            result["center_px"] = [round(float(value), 2) for value in self.center_px]
        if self.corners_px:
            result["corners_px"] = [
                [round(float(value), 2) for value in corner]
                for corner in self.corners_px
            ]
        if self.axis_px is not None:
            result["axis_px"] = [round(float(value), 5) for value in self.axis_px]
        if self.size_px is not None:
            result["size_px"] = round(float(self.size_px), 3)
        if self.color is not None:
            result["color"] = self.color
        return result


@dataclass
class _CandidateTrack:
    candidate_id: str
    detection: CubeDetection
    missed: int = 0
    observed_streak: int = 1
    observed_frames: int = 1


class CubeTracker:
    """Adaptive low-pass tracker with short occlusion retention."""

    def __init__(self, *, alpha: float = 0.35, max_missed: int = 12) -> None:
        self.alpha = float(np.clip(alpha, 0.05, 1.0))
        self.max_missed = max(1, int(max_missed))
        self._missed = 0
        self._last: CubeDetection | None = None

    @staticmethod
    def _blend(old: np.ndarray, new: np.ndarray, alpha: float) -> np.ndarray:
        return old + (new - old) * alpha

    def _motion_alpha(self, old: CubeDetection, new: CubeDetection) -> float:
        motion_px = float(
            np.linalg.norm(
                np.asarray(new.center_px, dtype=np.float64)
                - np.asarray(old.center_px, dtype=np.float64)
            )
        )
        size_px = max(12.0, float(old.size_px or new.size_px or 30.0))
        noise_px = float(np.clip(size_px * 0.035, 0.7, 1.6))
        if motion_px <= noise_px:
            return 0.04
        if motion_px <= noise_px * 3.0:
            fraction = (motion_px - noise_px) / (noise_px * 2.0)
            return 0.08 + fraction * (self.alpha - 0.08)
        if motion_px <= noise_px * 7.0:
            fraction = (motion_px - noise_px * 3.0) / (noise_px * 4.0)
            return self.alpha + fraction * (0.68 - self.alpha)
        return 0.72

    def update(self, detection: CubeDetection | None, timestamp: float) -> CubeDetection:
        if detection is not None:
            self._missed = 0
            if self._last is None:
                filtered = detection
            else:
                old = self._last
                motion_alpha = self._motion_alpha(old, detection)
                old_axis = np.asarray(old.axis_px, dtype=np.float64)
                raw_axis = np.asarray(detection.axis_px, dtype=np.float64)
                equivalent_axes = (
                    raw_axis,
                    -raw_axis,
                    np.asarray((-raw_axis[1], raw_axis[0])),
                    np.asarray((raw_axis[1], -raw_axis[0])),
                )
                aligned_axis = max(equivalent_axes, key=lambda value: float(value @ old_axis))
                center = self._blend(
                    np.asarray(old.center_px, dtype=np.float64),
                    np.asarray(detection.center_px, dtype=np.float64),
                    motion_alpha,
                )
                corners = self._blend(
                    np.asarray(old.corners_px, dtype=np.float64),
                    np.asarray(detection.corners_px, dtype=np.float64),
                    motion_alpha,
                )
                axis = self._blend(
                    old_axis,
                    aligned_axis,
                    min(motion_alpha, self.alpha),
                )
                axis /= max(float(np.linalg.norm(axis)), 1e-9)
                filtered = CubeDetection(
                    status="tracking",
                    center_px=(float(center[0]), float(center[1])),
                    corners_px=tuple((float(x), float(y)) for x, y in corners),
                    axis_px=(float(axis[0]), float(axis[1])),
                    size_px=float(old.size_px + (detection.size_px - old.size_px) * motion_alpha),
                    confidence=float(old.confidence + (detection.confidence - old.confidence) * self.alpha),
                    timestamp=timestamp,
                    color=detection.color or old.color,
                )
            self._last = filtered
            return filtered

        if self._last is not None:
            self._missed += 1
            if self._missed <= self.max_missed:
                return CubeDetection(
                    status="occluded",
                    center_px=self._last.center_px,
                    corners_px=self._last.corners_px,
                    axis_px=self._last.axis_px,
                    size_px=self._last.size_px,
                    confidence=max(0.0, self._last.confidence * 0.92),
                    timestamp=timestamp,
                    color=self._last.color,
                )
            self._last = None
        return CubeDetection(status="lost", timestamp=timestamp)

    @property
    def reference_center(self) -> np.ndarray | None:
        if self._last is None or self._last.center_px is None:
            return None
        return np.asarray(self._last.center_px, dtype=np.float64)

    def reset(self) -> None:
        self._missed = 0
        self._last = None


class RgbCubeDetector:
    """Track model detections, with the legacy geometry path kept for tests."""

    def __init__(
        self,
        *,
        downsample: int = 2,
        min_area_px: float = 36.0,
        max_area_fraction: float = 0.28,
        background_required: bool = False,
        background_capture_frames: int = 24,
        require_table_roi: bool = False,
    ) -> None:
        self.downsample = max(1, int(downsample))
        self.min_area_px = float(min_area_px)
        self.max_area_fraction = float(max_area_fraction)
        self.background_required = bool(background_required)
        self.require_table_roi = bool(require_table_roi)
        self.background = TableBackgroundModel(
            downsample=self.downsample,
            capture_frames=background_capture_frames,
        )
        self.tracker = CubeTracker()
        self._candidate_tracks: dict[str, _CandidateTrack] = {}
        self._next_candidate_id = 1
        self._selected_candidate_id: str | None = None
        self._candidate_center: np.ndarray | None = None
        self._candidate_streak = 0
        self._last_timestamp = time.monotonic()
        # Keep a candidate ID through a few missed camera frames.  The D455
        # stream can briefly lose a contour when exposure or depth alignment
        # changes; rebuilding the ID makes the simulator blink and resets its
        # pose filter.
        self._candidate_max_missed = 12

    def reset(self) -> None:
        self.tracker.reset()
        self._candidate_tracks.clear()
        self._next_candidate_id = 1
        self._selected_candidate_id = None
        self.background.reset()
        self._candidate_center = None
        self._candidate_streak = 0

    def request_background_capture(self, frame_count: int | None = None) -> dict[str, Any]:
        self.tracker.reset()
        self._candidate_tracks.clear()
        self._selected_candidate_id = None
        self._candidate_center = None
        self._candidate_streak = 0
        return self.background.request_capture(frame_count)

    def background_status(self) -> dict[str, Any]:
        return self.background.status()

    def _update_candidate_tracks(
        self,
        detections: list[CubeDetection],
        timestamp: float,
        selected_detection: CubeDetection | None,
    ) -> tuple[list[dict[str, Any]], str | None]:
        """Associate frame candidates with short-lived, stable browser IDs."""
        unmatched = set(self._candidate_tracks)
        observed_ids: list[str] = []
        for detection in detections:
            center = np.asarray(detection.center_px, dtype=np.float64)
            best_id: str | None = None
            best_distance = math.inf
            for candidate_id in unmatched:
                previous = self._candidate_tracks[candidate_id].detection
                if previous.center_px is None:
                    continue
                distance = float(
                    np.linalg.norm(center - np.asarray(previous.center_px, dtype=np.float64))
                )
                gate = max(
                    20.0,
                    1.65 * max(float(detection.size_px or 0.0), float(previous.size_px or 0.0)),
                )
                previous_color = previous.color
                if (
                    previous_color is not None
                    and detection.color is not None
                    and previous_color != detection.color
                ):
                    continue
                if distance <= gate and distance < best_distance:
                    best_id = candidate_id
                    best_distance = distance
            if best_id is None:
                best_id = f"cube-{self._next_candidate_id}"
                self._next_candidate_id += 1
                self._candidate_tracks[best_id] = _CandidateTrack(best_id, detection)
            else:
                unmatched.remove(best_id)
                track = self._candidate_tracks[best_id]
                track.detection = detection
                track.missed = 0
                track.observed_streak += 1
                track.observed_frames += 1
            observed_ids.append(best_id)

        for candidate_id in unmatched:
            track = self._candidate_tracks[candidate_id]
            track.missed += 1
            track.observed_streak = 0
            if track.missed > self._candidate_max_missed:
                del self._candidate_tracks[candidate_id]
                continue
            previous = track.detection
            track.detection = CubeDetection(
                status="occluded",
                center_px=previous.center_px,
                corners_px=previous.corners_px,
                axis_px=previous.axis_px,
                size_px=previous.size_px,
                confidence=max(0.0, previous.confidence * 0.88),
                timestamp=timestamp,
                color=previous.color,
            )

        selected_id: str | None = None
        if selected_detection is not None and selected_detection.center_px is not None:
            selected_center = np.asarray(selected_detection.center_px, dtype=np.float64)
            eligible = [
                candidate_id
                for candidate_id in observed_ids
                if candidate_id in self._candidate_tracks
            ]
            if eligible:
                nearest_id = min(
                    eligible,
                    key=lambda candidate_id: float(
                        np.linalg.norm(
                            np.asarray(
                                self._candidate_tracks[candidate_id].detection.center_px,
                                dtype=np.float64,
                            )
                            - selected_center
                        )
                    ),
                )
                selected_id = nearest_id
                # Geometry scores can trade places by a pixel or two when
                # adjacent cubes have similar contrast.  Keep the current
                # selection unless the new candidate is clearly closer.
                previous_id = self._selected_candidate_id
                if previous_id in eligible and previous_id != nearest_id:
                    previous_distance = float(
                        np.linalg.norm(
                            np.asarray(
                                self._candidate_tracks[previous_id].detection.center_px,
                                dtype=np.float64,
                            )
                            - selected_center
                        )
                    )
                    nearest_distance = float(
                        np.linalg.norm(
                            np.asarray(
                                self._candidate_tracks[nearest_id].detection.center_px,
                                dtype=np.float64,
                            )
                            - selected_center
                        )
                    )
                    if nearest_distance + 4.0 >= previous_distance * 0.72:
                        selected_id = previous_id
        if selected_id is None and self._selected_candidate_id in self._candidate_tracks:
            selected_id = self._selected_candidate_id
        if selected_id is None and observed_ids:
            selected_id = observed_ids[0]
        self._selected_candidate_id = selected_id

        def candidate_number(candidate_id: str) -> int:
            try:
                return int(candidate_id.rsplit("-", 1)[1])
            except (IndexError, ValueError):
                return 0

        values: list[dict[str, Any]] = []
        for candidate_id in sorted(self._candidate_tracks, key=candidate_number):
            value = self._candidate_tracks[candidate_id].detection.as_dict()
            value.pop("type", None)
            value["candidate_id"] = candidate_id
            value["selected"] = candidate_id == selected_id
            value["stable_frames"] = self._candidate_tracks[
                candidate_id
            ].observed_streak
            value["observed_frames"] = self._candidate_tracks[
                candidate_id
            ].observed_frames
            values.append(value)
        return values, selected_id

    @staticmethod
    def _rgb_image(image: np.ndarray) -> np.ndarray:
        rgb = np.asarray(image)
        if rgb.ndim != 3 or rgb.shape[2] < 3:
            raise ValueError("RGB image must have shape (height, width, 3)")
        return np.ascontiguousarray(rgb[..., :3], dtype=np.uint8)

    @staticmethod
    def _ordered_corners(points: np.ndarray, center: np.ndarray) -> np.ndarray:
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
        return points[np.argsort(angles)]

    @staticmethod
    def _split_elongated_contour(
        mask: np.ndarray,
        contour: np.ndarray,
    ) -> list[np.ndarray]:
        """Split a touching row of square blobs into square-sized contours.

        A small closing kernel can turn two touching cubes of the same color
        into one long rectangle. Its aspect ratio contains enough information
        to recover the individual centers even when the contact seam has no
        background pixels.
        """
        rect = cv2.minAreaRect(contour)
        center = np.asarray(rect[0], dtype=np.float64)
        rect_width, rect_height = (float(value) for value in rect[1])
        short_side = min(rect_width, rect_height)
        long_side = max(rect_width, rect_height)
        ratio = long_side / max(short_side, 1e-6)
        if short_side < 10.0 or ratio < 1.5 or ratio > 4.5:
            return [contour]
        split_count = int(round(ratio))
        if split_count < 2 or split_count > 4:
            return [contour]

        component = np.zeros_like(mask)
        cv2.drawContours(component, [contour], -1, 255, -1)
        ys, xs = np.nonzero(component)
        if len(xs) < 2:
            return [contour]
        box = cv2.boxPoints(rect).astype(np.float64)
        edges = np.roll(box, -1, axis=0) - box
        long_edge = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
        long_axis = long_edge / max(float(np.linalg.norm(long_edge)), 1e-9)
        points = np.column_stack((xs, ys)).astype(np.float64)
        projections = (points - center) @ long_axis
        lower = float(np.min(projections))
        upper = float(np.max(projections))
        if upper - lower < short_side * 1.35:
            return [contour]

        pieces: list[np.ndarray] = []
        for index in range(split_count):
            start = lower + (upper - lower) * index / split_count
            end = lower + (upper - lower) * (index + 1) / split_count
            if index == split_count - 1:
                selected = (projections >= start) & (projections <= end)
            else:
                selected = (projections >= start) & (projections < end)
            if np.count_nonzero(selected) < 20:
                return [contour]
            piece = np.zeros_like(mask)
            piece[ys[selected], xs[selected]] = 255
            found, _ = cv2.findContours(
                piece,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            if not found:
                return [contour]
            pieces.append(max(found, key=cv2.contourArea))
        return pieces if len(pieces) == split_count else [contour]

    def _detect_table_color(
        self,
        image: np.ndarray,
        timestamp: float,
        roi_polygon: np.ndarray | None,
    ) -> tuple[CubeDetection | None, list[CubeDetection], list[dict[str, Any]]]:
        """Segment black, white, and blue cubes inside the calibrated tabletop."""
        if roi_polygon is None:
            return None, [], []
        rgb = self._rgb_image(image)
        height, width = rgb.shape[:2]
        polygon = np.rint(np.asarray(roi_polygon, dtype=np.float64)).astype(np.int32)
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillConvexPoly(roi_mask, polygon, 255)
        # Ignore the projected table boundary, where dark edge pixels and
        # sub-pixel calibration error otherwise create long false contours.
        roi_mask = cv2.erode(
            roi_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
        )
        roi_pixels = roi_mask > 0
        if np.count_nonzero(roi_pixels) < 400:
            return None, [], []

        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        tabletop_level = float(np.median(gray[roi_pixels]))
        dark_threshold = float(np.clip(tabletop_level - 42.0, 42.0, 92.0))
        # Keep black detection conservative. Saturated blue pixels are excluded
        # here and handled by the dedicated blue mask below.
        black = (
            (gray < dark_threshold)
            & ((hsv[..., 1] < 115) | ((hsv[..., 2] < 68) & (hsv[..., 1] < 65)))
            & roi_pixels
        ).astype(np.uint8) * 255
        blue = (
            (hsv[..., 0] >= 90)
            & (hsv[..., 0] <= 135)
            & (hsv[..., 1] >= 65)
            & (hsv[..., 2] >= 42)
            & roi_pixels
        ).astype(np.uint8) * 255
        # A white cube is identified by high value and low saturation relative
        # to the tabletop. The adaptive margin avoids treating the whole ROI as
        # white when exposure raises the table brightness.
        white_threshold = max(145.0, min(245.0, tabletop_level + 22.0))
        white = (
            (hsv[..., 1] <= 58)
            & (hsv[..., 2] >= white_threshold)
            & roi_pixels
        ).astype(np.uint8) * 255

        masks = (("black", black), ("blue", blue), ("white", white))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        reference = self.tracker.reference_center
        frame_diagonal = max(1.0, math.hypot(width, height))
        candidates: list[tuple[float, CubeDetection, dict[str, Any]]] = []
        for color, source_mask in masks:
            mask = cv2.morphologyEx(source_mask, cv2.MORPH_OPEN, kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            split_contours = [
                piece
                for contour in contours
                for piece in self._split_elongated_contour(mask, contour)
            ]
            for contour in split_contours:
                area = float(cv2.contourArea(contour))
                if not 90.0 <= area <= 6500.0:
                    continue
                hull = cv2.convexHull(contour)
                rect = cv2.minAreaRect(hull)
                center = np.asarray(rect[0], dtype=np.float64)
                rect_width, rect_height = (float(value) for value in rect[1])
                short_side = min(rect_width, rect_height)
                long_side = max(rect_width, rect_height)
                if short_side < 9.0 or long_side > 105.0:
                    continue
                aspect_ratio = long_side / max(short_side, 1e-6)
                rectangularity = area / max(1.0, rect_width * rect_height)
                if aspect_ratio > 1.65 or rectangularity < 0.58:
                    continue
                corners = self._ordered_corners(cv2.boxPoints(rect), center)
                contour_mask = np.zeros_like(gray)
                cv2.drawContours(contour_mask, [contour], -1, 255, -1)
                object_pixels = contour_mask > 0
                mean_gray = float(np.mean(gray[object_pixels]))
                mean_hsv = np.mean(hsv[object_pixels], axis=0)
                if color == "black":
                    appearance = float(
                        np.clip(
                            (dark_threshold - mean_gray) / max(dark_threshold, 1.0),
                            0.0,
                            1.0,
                        )
                    )
                elif color == "blue":
                    appearance = float(
                        np.clip(
                            0.55 * (float(mean_hsv[1]) - 55.0) / 145.0
                            + 0.45 * (float(mean_hsv[2]) - 42.0) / 213.0,
                            0.0,
                            1.0,
                        )
                    )
                else:
                    appearance = float(
                        np.clip(
                            (float(mean_hsv[2]) - white_threshold) / 80.0,
                            0.0,
                            1.0,
                        )
                    )
                square = float(np.clip((1.65 - aspect_ratio) / 0.65, 0.0, 1.0))
                geometry_score = float(
                    np.clip(
                        0.42 * square + 0.33 * rectangularity + 0.25 * appearance,
                        0.0,
                        1.0,
                    )
                )
                confidence = 0.78 + 0.18 * geometry_score
                detection = CubeDetection(
                    status="tracking",
                    center_px=(float(center[0]), float(center[1])),
                    corners_px=tuple((float(x), float(y)) for x, y in corners),
                    axis_px=(1.0, 0.0),
                    size_px=math.sqrt(max(area, 1.0)),
                    confidence=confidence,
                    timestamp=timestamp,
                    color=color,
                )
                continuity_penalty = 0.0
                if reference is not None:
                    continuity_penalty = min(
                        0.30,
                        float(np.linalg.norm(center - reference)) / frame_diagonal,
                    )
                x, y, box_width, box_height = cv2.boundingRect(hull)
                metadata = {
                    "bbox_px": [int(x), int(y), int(box_width), int(box_height)],
                    "center_px": [round(float(center[0]), 2), round(float(center[1]), 2)],
                    "area_px": round(area, 1),
                    "aspect_ratio": round(aspect_ratio, 3),
                    "rectangularity": round(rectangularity, 3),
                    "mean_gray": round(mean_gray, 1),
                    "mean_hsv": [round(float(value), 1) for value in mean_hsv],
                    "color": color,
                    "score": round(geometry_score, 4),
                }
                candidates.append((geometry_score - continuity_penalty, detection, metadata))
        if not candidates:
            return None, [], []
        candidates.sort(key=lambda item: item[0], reverse=True)
        # Avoid reporting the same cube twice when a very dark blue surface
        # also satisfies the conservative black mask.
        retained: list[tuple[float, CubeDetection, dict[str, Any]]] = []
        for candidate in candidates:
            bbox = candidate[2]["bbox_px"]
            x, y, box_width, box_height = (float(value) for value in bbox)
            duplicate = False
            for existing in retained:
                other = existing[2]["bbox_px"]
                ox, oy, ow, oh = (float(value) for value in other)
                ix0, iy0 = max(x, ox), max(y, oy)
                ix1, iy1 = min(x + box_width, ox + ow), min(y + box_height, oy + oh)
                intersection = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
                union = box_width * box_height + ow * oh - intersection
                if union > 0.0 and intersection / union > 0.55:
                    duplicate = True
                    break
            if not duplicate:
                retained.append(candidate)
            if len(retained) >= 8:
                break
        retained[0][2]["selected"] = True
        return (
            retained[0][1],
            [item[1] for item in retained],
            [item[2] for item in retained],
        )

    def _detect_once(
        self,
        image: np.ndarray,
        timestamp: float,
        roi_polygon: np.ndarray | None = None,
    ) -> CubeDetection | None:
        rgb = self._rgb_image(image)
        step = self.downsample
        reduced = cv2.resize(
            rgb,
            (max(1, rgb.shape[1] // step), max(1, rgb.shape[0] // step)),
            interpolation=cv2.INTER_AREA,
        )
        gray = cv2.cvtColor(reduced, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        hsv = cv2.cvtColor(reduced, cv2.COLOR_RGB2HSV)
        # Black-object segmentation adapts to exposure while retaining a
        # conservative absolute ceiling for a genuinely dark cube.
        background = float(np.percentile(gray, 65.0))
        threshold = min(110.0, max(30.0, background - 35.0))
        dark_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)[1]
        dark_mask = cv2.bitwise_and(
            dark_mask,
            (
                (hsv[..., 1] < 115)
                | ((hsv[..., 2] < 68) & (hsv[..., 1] < 65))
            ).astype(np.uint8) * 255,
        )
        # A cube can be wood/brown rather than pure black. Detect local
        # luminance contrast as a second, color-agnostic path so the textured
        # tabletop does not have to share the cube's absolute brightness.
        local_background = cv2.GaussianBlur(gray, (0, 0), sigmaX=9.0, sigmaY=9.0)
        local_contrast = cv2.absdiff(gray, local_background)
        contrast_mask = cv2.threshold(local_contrast, 16, 255, cv2.THRESH_BINARY)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        def prepare_mask(source: np.ndarray) -> np.ndarray:
            prepared = cv2.morphologyEx(source, cv2.MORPH_OPEN, kernel)
            prepared = cv2.morphologyEx(prepared, cv2.MORPH_CLOSE, kernel, iterations=2)
            prepared[[0, -1], :] = 0
            prepared[:, [0, -1]] = 0
            if roi_polygon is None:
                # Without the calibrated tabletop polygon, do not search the
                # entire image. The lower part is the only plausible workspace;
                # the stricter size gate below prevents chairs and large furniture
                # in the background from winning the square-contour score.
                workspace_top = int(round(prepared.shape[0] * 0.28))
                prepared[:workspace_top, :] = 0
            else:
                polygon = np.asarray(roi_polygon, dtype=np.float64).reshape(-1, 2) / step
                polygon = np.rint(polygon).astype(np.int32)
                roi_mask = np.zeros_like(prepared)
                cv2.fillConvexPoly(roi_mask, polygon, 255)
                margin = max(2, int(round(12 / step)))
                roi_mask = cv2.dilate(
                    roi_mask,
                    cv2.getStructuringElement(
                        cv2.MORPH_ELLIPSE,
                        (margin * 2 + 1, margin * 2 + 1),
                    ),
                )
                prepared = cv2.bitwise_and(prepared, roi_mask)
            return prepared

        mask = prepare_mask(dark_mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_colors: list[str | None] = ["black"] * len(contours)
        if not contours:
            mask = prepare_mask(contrast_mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour_colors = [None] * len(contours)
        if not contours:
            reduced_hsv = hsv
            reduced_gray_level = float(np.median(gray))
            reduced_white_threshold = max(145.0, min(245.0, reduced_gray_level + 22.0))
            color_sources = (
                (
                    "blue",
                    (reduced_hsv[..., 0] >= 90)
                    & (reduced_hsv[..., 0] <= 135)
                    & (reduced_hsv[..., 1] >= 65)
                    & (reduced_hsv[..., 2] >= 42),
                ),
                (
                    "white",
                    (reduced_hsv[..., 1] <= 58)
                    & (reduced_hsv[..., 2] >= reduced_white_threshold),
                ),
            )
            contours = []
            contour_colors = []
            for color, source in color_sources:
                color_mask = prepare_mask(source.astype(np.uint8) * 255)
                found, _ = cv2.findContours(
                    color_mask,
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE,
                )
                contours.extend(found)
                contour_colors.extend([color] * len(found))
        frame_area = float(mask.size)
        candidates: list[tuple[float, np.ndarray, np.ndarray, float, np.ndarray, str | None]] = []
        for contour, contour_color in zip(contours, contour_colors):
            area = float(cv2.contourArea(contour))
            if area < max(1.0, self.min_area_px / (step * step)):
                continue
            hull = cv2.convexHull(contour)
            rect = cv2.minAreaRect(hull)
            (_, _), (rect_width, rect_height), _ = rect
            max_area_fraction = self.max_area_fraction
            if roi_polygon is None:
                max_area_fraction = min(max_area_fraction, 0.06)
                center_x, center_y = rect[0]
                if (
                    center_x < reduced.shape[1] * 0.10
                    or center_x > reduced.shape[1] * 0.90
                    or center_y < reduced.shape[0] * 0.40
                ):
                    continue
                if (
                    rect_width > reduced.shape[1] * 0.20
                    or rect_height > reduced.shape[0] * 0.20
                ):
                    continue
            if area > frame_area * max_area_fraction:
                continue
            short_side = min(float(rect_width), float(rect_height))
            long_side = max(float(rect_width), float(rect_height))
            aspect_limit = 2.2 if roi_polygon is None else 3.2
            if short_side < 2.0 or long_side / short_side > aspect_limit:
                continue
            rectangle_area = max(1.0, float(rect_width * rect_height))
            rectangularity = min(1.0, area / rectangle_area)
            if rectangularity < 0.55:
                continue
            perimeter = cv2.arcLength(hull, True)
            polygon = cv2.approxPolyDP(hull, 0.035 * perimeter, True)
            polygon_score = 1.0 if len(polygon) == 4 else 0.65
            center_px = np.asarray(rect[0], dtype=np.float64) * step
            raw_corners = (
                polygon.reshape(4, 2).astype(np.float64)
                if len(polygon) == 4
                else cv2.boxPoints(rect).astype(np.float64)
            )
            corners = self._ordered_corners(raw_corners * step, center_px)
            edges = np.roll(corners, -1, axis=0) - corners
            axis = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
            axis /= max(float(np.linalg.norm(axis)), 1e-9)
            if axis[0] < 0 or (abs(axis[0]) < 1e-6 and axis[1] < 0):
                axis = -axis
            size_px = math.sqrt(max(area, 1.0)) * step
            aspect_score = min(1.0, short_side / long_side * 1.5)
            area_score = min(1.0, area / 120.0)
            score = min(
                1.0,
                0.35 * rectangularity + 0.25 * polygon_score
                + 0.2 * aspect_score + 0.2 * area_score,
            )
            candidates.append((score, center_px, corners, size_px, axis, contour_color))
        if not candidates:
            return None
        reference = self.tracker.reference_center
        if reference is None:
            selected = max(candidates, key=lambda item: item[0])
        else:
            frame_diagonal = max(1.0, math.hypot(rgb.shape[1], rgb.shape[0]))
            selected = max(
                candidates,
                key=lambda item: item[0]
                - min(0.55, float(np.linalg.norm(item[1] - reference)) / frame_diagonal),
            )
        score, center, corners, size_px, axis, color = selected
        if color is None:
            center_x = int(round(float(center[0]) / step))
            center_y = int(round(float(center[1]) / step))
            radius = max(2, int(round(size_px / step * 0.22)))
            x0 = max(0, center_x - radius)
            x1 = min(hsv.shape[1], center_x + radius + 1)
            y0 = max(0, center_y - radius)
            y1 = min(hsv.shape[0], center_y + radius + 1)
            patch_hsv = hsv[y0:y1, x0:x1].reshape(-1, 3)
            if patch_hsv.size:
                mean_hue, mean_saturation, mean_value = np.mean(patch_hsv, axis=0)
                if 90.0 <= mean_hue <= 135.0 and mean_saturation >= 65.0:
                    color = "blue"
                elif (
                    mean_saturation <= 58.0
                    and mean_value >= max(145.0, background + 22.0)
                ):
                    color = "white"
        return CubeDetection(
            status="tracking",
            center_px=(float(center[0]), float(center[1])),
            corners_px=tuple((float(x), float(y)) for x, y in corners),
            axis_px=(float(axis[0]), float(axis[1])),
            size_px=size_px,
            confidence=score,
            timestamp=timestamp,
            color=color,
        )

    @staticmethod
    def _translate_detection(
        detection: CubeDetection,
        offset_x: int,
        offset_y: int,
    ) -> CubeDetection:
        center = None
        if detection.center_px is not None:
            center = (
                float(detection.center_px[0] + offset_x),
                float(detection.center_px[1] + offset_y),
            )
        corners = tuple(
            (float(x + offset_x), float(y + offset_y))
            for x, y in detection.corners_px
        )
        return CubeDetection(
            status=detection.status,
            center_px=center,
            corners_px=corners,
            axis_px=detection.axis_px,
            size_px=detection.size_px,
            confidence=detection.confidence,
            timestamp=detection.timestamp,
            color=detection.color,
        )

    def _fallback_region_detection(
        self,
        region: BackgroundRegion,
        timestamp: float,
    ) -> CubeDetection:
        corners = np.asarray(region.corners_px, dtype=np.float64)
        if len(corners) == 4:
            edges = np.roll(corners, -1, axis=0) - corners
            axis = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
        else:
            axis = np.asarray([1.0, 0.0], dtype=np.float64)
        axis /= max(float(np.linalg.norm(axis)), 1e-9)
        if axis[0] < 0.0 or (abs(axis[0]) < 1e-6 and axis[1] < 0.0):
            axis = -axis
        return CubeDetection(
            status="tracking",
            center_px=region.center_px,
            corners_px=region.corners_px,
            axis_px=(float(axis[0]), float(axis[1])),
            size_px=math.sqrt(max(region.area_px, 1.0)),
            confidence=max(0.35, min(0.9, region.score * 0.88)),
            timestamp=timestamp,
        )

    def _detect_from_background_regions(
        self,
        image: np.ndarray,
        regions: tuple[BackgroundRegion, ...],
        timestamp: float,
    ) -> tuple[CubeDetection | None, list[dict[str, Any]]]:
        candidates = [region for region in regions if region.accepted]
        if not candidates:
            return None, [region.as_dict() for region in regions]
        detections: list[tuple[CubeDetection, BackgroundRegion]] = []
        height, width = image.shape[:2]
        for region in candidates:
            x, y, region_width, region_height = region.bbox_px
            pad = max(8, int(round(max(region_width, region_height) * 0.45)))
            x0 = max(0, x - pad)
            y0 = max(0, y - pad)
            x1 = min(width, x + region_width + pad)
            y1 = min(height, y + region_height + pad)
            crop = image[y0:y1, x0:x1]
            assisted = self._detect_once(crop, timestamp)
            if assisted is not None and assisted.center_px is not None:
                assisted = self._translate_detection(assisted, x0, y0)
                # A crop-level model can lock onto a dark edge or another
                # object. Accept its result only while it remains within the
                # background-change region and roughly matches its geometry.
                center = np.asarray(assisted.center_px, dtype=np.float64)
                region_center = np.asarray(region.center_px, dtype=np.float64)
                region_size = max(
                    4.0,
                    min(float(region.bbox_px[2]), float(region.bbox_px[3])) * 0.75,
                )
                model_size = float(assisted.size_px or 0.0)
                size_ratio = model_size / max(region_size, 1e-6)
                model_valid = (
                    float(np.linalg.norm(center - region_center)) <= region_size * 1.2
                    and 0.35 <= size_ratio <= 2.8
                )
                if model_valid:
                    assisted = CubeDetection(
                        status="tracking",
                        center_px=assisted.center_px,
                        corners_px=assisted.corners_px,
                        axis_px=assisted.axis_px,
                        size_px=assisted.size_px,
                        confidence=min(1.0, 0.62 * assisted.confidence + 0.38 * region.score),
                        timestamp=timestamp,
                        color=assisted.color,
                    )
                else:
                    assisted = None
            if assisted is None:
                assisted = self._fallback_region_detection(region, timestamp)
            detections.append((assisted, region))
        reference = self.tracker.reference_center
        if reference is None:
            selected, selected_region = max(
                detections,
                key=lambda item: (item[1].score, item[0].confidence),
            )
        else:
            frame_diagonal = max(1.0, math.hypot(width, height))
            selected, selected_region = max(
                detections,
                key=lambda item: item[0].confidence
                - min(
                    0.55,
                    float(np.linalg.norm(np.asarray(item[0].center_px) - reference))
                    / frame_diagonal,
                ),
            )
        annotated_regions = []
        for region in regions:
            value = region.as_dict()
            value["selected"] = region is selected_region
            value["model_assisted"] = region is selected_region and selected.confidence > region.score * 0.9
            annotated_regions.append(value)
        return selected, annotated_regions

    def process(
        self,
        image: np.ndarray,
        timestamp: float | None = None,
        *,
        roi_polygon: np.ndarray | None = None,
    ) -> dict[str, Any]:
        stamp = float(time.time() if timestamp is None else timestamp)
        if self.require_table_roi:
            background = self.background.status()
            detection, candidate_detections, color_candidates = self._detect_table_color(
                image,
                stamp,
                roi_polygon,
            )
            if roi_polygon is None:
                self.tracker.reset()
                self._candidate_tracks.clear()
                self._selected_candidate_id = None
            candidates, selected_candidate_id = self._update_candidate_tracks(
                candidate_detections,
                stamp,
                detection,
            )
            result = self.tracker.update(detection, stamp).as_dict()
            result["background"] = background
            result["fusion"] = "table-color"
            if result.get("color") is not None:
                result["cube_color"] = result["color"]
            result["color"] = {"candidates": color_candidates}
            result["candidates"] = candidates
            result["selected_candidate_id"] = selected_candidate_id
            return result
        if not self.background_required:
            # Keep an explicitly requested background capture alive for UI
            # feedback and diagnostics, but never make it a prerequisite for
            # the direct RGB geometry detector used by cube picking.
            background = self.background.status()
            if background["state"] == "capturing" or background["ready"]:
                background = self.background.update(image, roi_polygon)
            detection = self._detect_once(image, stamp, roi_polygon)
            result = self.tracker.update(detection, stamp).as_dict()
            result["background"] = background
            result["fusion"] = "rgb-geometry"
            return result

        background = self.background.update(image, roi_polygon)
        if background["state"] == "capturing":
            result = {
                "type": "cube_detection",
                "status": "background_capturing",
                "confidence": 0.0,
                "timestamp": stamp,
            }
            result["background"] = background
            return result
        if not background["ready"]:
            self._candidate_center = None
            self._candidate_streak = 0
            result = {
                "type": "cube_detection",
                "status": "background_missing",
                "confidence": 0.0,
                "timestamp": stamp,
            }
            result["background"] = background
            return result

        detection, regions = self._detect_from_background_regions(
            image,
            tuple(
                BackgroundRegion(
                    bbox_px=tuple(region["bbox_px"]),
                    center_px=tuple(region["center_px"]),
                    corners_px=tuple(tuple(corner) for corner in region["corners_px"]),
                    area_px=float(region["area_px"]),
                    rectangularity=float(region["rectangularity"]),
                    aspect_ratio=float(region["aspect_ratio"]),
                    score=float(region["score"]),
                    accepted=bool(region["accepted"]),
                    rejection=region.get("rejection"),
                )
                for region in background["regions"]
            ),
            stamp,
        )
        confirmed = detection
        if detection is None or detection.center_px is None:
            self._candidate_center = None
            self._candidate_streak = 0
            confirmed = None
        else:
            center = np.asarray(detection.center_px, dtype=np.float64)
            if self._candidate_center is not None:
                allowed_jump = max(10.0, float(detection.size_px or 20.0) * 1.6)
                if float(np.linalg.norm(center - self._candidate_center)) <= allowed_jump:
                    self._candidate_streak += 1
                else:
                    self._candidate_streak = 1
            else:
                self._candidate_streak = 1
            self._candidate_center = center
            if self._candidate_streak < 2:
                confirmed = None
        filtered = self.tracker.update(confirmed, stamp)
        result = filtered.as_dict()
        if detection is not None and confirmed is None and filtered.status == "lost":
            result["status"] = "background_confirming"
            result["confidence"] = round(float(detection.confidence) * 0.5, 4)
        result["background"] = dict(background)
        result["background"]["regions"] = regions
        if detection is None:
            result["fusion"] = "background_miss"
        elif confirmed is None:
            result["fusion"] = "background_confirming"
        else:
            result["fusion"] = "background+model"
        return result
