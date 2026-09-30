from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class BackendSettings:
    host: str = "127.0.0.1"
    port: int = 8766
    interface: str = "can0"
    cert: Path | None = None
    key: Path | None = None
    origins: tuple[str, ...] = ()
    token: str | None = None
    camera_url: str | None = None
    camera_ca: Path | None = None
    camera_origin: str | None = None
    frontend_directory: Path | None = None
    ca_cert: Path | None = None
    auto_connect_arm: bool = False

    @property
    def allowed_origins(self) -> tuple[str, ...]:
        return self.origins or (
            "http://localhost:5173",
            "http://127.0.0.1:5173",
            "http://localhost:8000",
            "http://127.0.0.1:8000",
        )
