"""Accuracy-first depth visualization helpers.

The Turbo polynomial is Copyright 2019 Google LLC, Apache-2.0:
https://gist.github.com/mikhailov-work/0d177465a8151eb6ede1768d51d476c7
"""

from __future__ import annotations

import numpy as np


def _turbo_lut() -> np.ndarray:
    x = np.linspace(0.0, 1.0, 256, dtype=np.float64)
    powers = np.stack((np.ones_like(x), x, x**2, x**3), axis=1)
    high_powers = np.stack((x**4, x**5), axis=1)
    primary = np.array(
        [
            [0.13572138, 4.61539260, -42.66032258, 132.13108234],
            [0.09140261, 2.19418839, 4.84296658, -14.18503333],
            [0.10667330, 12.64194608, -60.58204836, 110.36276771],
        ],
        dtype=np.float64,
    )
    high = np.array(
        [
            [-152.94239396, 59.28637943],
            [4.27729857, 2.82956604],
            [-89.90310912, 27.34824973],
        ],
        dtype=np.float64,
    )
    rgb = powers @ primary.T + high_powers @ high.T
    return np.rint(np.clip(rgb, 0.0, 1.0) * 255.0).astype(np.uint8)


TURBO_LUT = _turbo_lut()


class DepthVisualizer:
    """Create a stable, non-fabricated RGB preview from a Z16 depth frame."""

    def __init__(
        self,
        *,
        depth_scale: float,
        min_distance_m: float = 0.2,
        max_distance_m: float = 4.0,
        min_valid_frames: int = 3,
        min_consistent_neighbors: int = 4,
        neighbor_tolerance_m: float = 0.03,
        neighbor_tolerance_ratio: float = 0.02,
    ) -> None:
        if depth_scale <= 0:
            raise ValueError("depth_scale must be positive")
        if not 0 <= min_consistent_neighbors <= 8:
            raise ValueError("min_consistent_neighbors must be in [0, 8]")
        if min_valid_frames <= 0 or not min_distance_m < max_distance_m:
            raise ValueError("invalid depth visualization range")
        self.depth_scale = float(depth_scale)
        self.min_distance_m = float(min_distance_m)
        self.max_distance_m = float(max_distance_m)
        self.min_valid_frames = int(min_valid_frames)
        self.min_consistent_neighbors = int(min_consistent_neighbors)
        self.neighbor_tolerance_m = float(neighbor_tolerance_m)
        self.neighbor_tolerance_ratio = float(neighbor_tolerance_ratio)
        self._valid_streak: np.ndarray | None = None

    def reset(self) -> None:
        self._valid_streak = None

    def _validity_mask(self, depth_units: np.ndarray) -> np.ndarray:
        depth_m = depth_units.astype(np.float32) * self.depth_scale
        valid = (
            (depth_units != 0)
            & (depth_m >= self.min_distance_m)
            & (depth_m <= self.max_distance_m)
        )
        if self.min_consistent_neighbors == 0:
            return valid

        padded = np.pad(depth_m, 1)
        support = np.zeros(depth_units.shape, dtype=np.uint8)
        tolerance = np.maximum(
            self.neighbor_tolerance_m,
            depth_m * self.neighbor_tolerance_ratio,
        )
        height, width = depth_units.shape
        for offset_y in range(3):
            for offset_x in range(3):
                if offset_y == 1 and offset_x == 1:
                    continue
                neighbor = padded[
                    offset_y : offset_y + height,
                    offset_x : offset_x + width,
                ]
                support += (
                    (neighbor >= self.min_distance_m)
                    & (neighbor <= self.max_distance_m)
                    & (np.abs(neighbor - depth_m) <= tolerance)
                )
        return valid & (support >= self.min_consistent_neighbors)

    def colorize(self, depth_units: np.ndarray) -> np.ndarray:
        if depth_units.ndim != 2:
            raise ValueError("depth frame must be a 2D Z16 array")
        valid = self._validity_mask(depth_units)
        if self._valid_streak is None or self._valid_streak.shape != depth_units.shape:
            self._valid_streak = np.zeros(depth_units.shape, dtype=np.uint8)
        self._valid_streak = np.where(
            valid,
            np.minimum(self._valid_streak, 254) + 1,
            0,
        ).astype(np.uint8)
        stable = valid & (self._valid_streak >= self.min_valid_frames)

        depth_m = depth_units.astype(np.float32) * self.depth_scale
        normalized = (self.max_distance_m - depth_m) / (
            self.max_distance_m - self.min_distance_m
        )
        indices = np.rint(np.clip(normalized, 0.0, 1.0) * 255.0).astype(
            np.uint8
        )
        colored = TURBO_LUT[indices].copy()
        colored[~stable] = 0
        return colored
