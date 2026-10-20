from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from embodied.core.project import port


@dataclass(frozen=True, slots=True)
class BackendSettings:
    host: str = "127.0.0.1"
    port: int = field(default_factory=lambda: port('console_backend'))
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
        return self.origins or tuple(f"http://{host}:{port(name)}" for name in ("frontend_dev", "console")
                                     for host in ("localhost", "127.0.0.1"))
