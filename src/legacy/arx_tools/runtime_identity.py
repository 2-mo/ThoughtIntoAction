"""Runtime source identity shared by long-lived ARX5 services."""

from __future__ import annotations

from datetime import datetime, timezone
import os
import subprocess

from embodied.core.arx5_paths import PROJECT_ROOT


def _source_revision() -> str:
    override = os.environ.get("ARX5_SOURCE_REVISION")
    if override:
        return override
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    revision = result.stdout.strip()
    return revision or "unknown"


SOURCE_REVISION = _source_revision()
SERVICE_STARTED_AT = datetime.now(timezone.utc).isoformat()
