#!/usr/bin/env python3
"""CSV telemetry shared by the X5 simulation and hardware controller."""

import csv
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any


VECTOR_FIELDS = {
    "xr": ("x", "y", "z", "qx", "qy", "qz", "qw"),
    "eef_actual": ("x", "y", "z", "qx", "qy", "qz", "qw"),
    "eef_ik": ("x", "y", "z", "qx", "qy", "qz", "qw"),
    "xr_delta_world": ("x", "y", "z", "rx", "ry", "rz"),
    "q_actual": tuple(f"j{i}" for i in range(1, 7)),
    "q_ik": tuple(f"j{i}" for i in range(1, 7)),
    "q_command": tuple(f"j{i}" for i in range(1, 7)),
    "q_command_velocity": tuple(f"j{i}" for i in range(1, 7)),
    "q_command_torque": tuple(f"j{i}" for i in range(1, 7)),
}

FIELDS = [
    "host_time_ns",
    "xr_timestamp_ns",
    "source",
    "active",
    "xr_fresh",
    "command_sent",
    "rejection_reason",
    "right_grip",
    "right_trigger",
    "gripper_actual_m",
    "gripper_target_m",
    "gripper_command_velocity",
    "gripper_command_torque",
    "max_joint_error_rad",
    "max_ik_step_rad",
]
for prefix, names in VECTOR_FIELDS.items():
    FIELDS.extend(f"{prefix}_{name}" for name in names)


class TelemetryCSVLogger:
    """Write fixed-schema controller telemetry and flush it periodically."""

    def __init__(self, log_dir: str, source: str) -> None:
        directory = Path(log_dir).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = directory / f"x5_{source}_{timestamp}.csv"
        self._file = self.path.open("x", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._lock = threading.Lock()
        self._last_flush = time.monotonic()

    def write(self, row: dict[str, Any]) -> None:
        with self._lock:
            self._writer.writerow({name: row.get(name, "") for name in FIELDS})
            now = time.monotonic()
            if now - self._last_flush >= 1.0:
                self._file.flush()
                self._last_flush = now

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.flush()
                self._file.close()


def add_vector(row: dict[str, Any], prefix: str, values: Any) -> None:
    """Expand a vector into the logger's stable named columns."""
    names = VECTOR_FIELDS[prefix]
    if values is None:
        values = [""] * len(names)
    for name, value in zip(names, values):
        row[f"{prefix}_{name}"] = value
