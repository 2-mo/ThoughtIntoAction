"""Thread-safe handoff for low-rate canonical mapping settings."""

from threading import Lock

from robot_teleop.core.mapping import MappingScales, PoseMapping


class MappingSettingsStore:
    def __init__(self, initial_mapping: PoseMapping) -> None:
        self._mapping = initial_mapping
        self._lock = Lock()

    def load(self) -> PoseMapping:
        with self._lock:
            return self._mapping

    def update(self, scales: MappingScales) -> MappingScales:
        with self._lock:
            self._mapping = self._mapping.with_scales(scales)
            return self._mapping.scales
