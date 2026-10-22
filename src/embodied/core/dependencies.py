"""Pinned external installations, distinct from read-only reference repositories.

Only setup commands fetch/build these dependencies. Runtime resolves already prepared
packages, interpreters and the native SDK resources that cannot be packaged upstream.
"""
from __future__ import annotations

from pathlib import Path
import tomllib

PROJECT_ROOT = Path(__file__).resolve().parents[3]


def reject_reference_path(path: Path | str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if "third_party" in resolved.parts:
        raise ValueError(f"Reference checkout cannot be an executable dependency: {resolved}")
    return resolved


def dependency(name: str, *, root: Path = PROJECT_ROOT) -> dict:
    with (root / "dependencies.lock.toml").open("rb") as stream:
        entries = tomllib.load(stream)["managed"]
    for entry in entries:
        if entry["name"] == name:
            if len(entry["commit"]) != 40 or any(c not in "0123456789abcdef" for c in entry["commit"]):
                raise ValueError(f"Dependency must pin a full Git revision: {name}")
            return entry
    raise ValueError(f"Unknown managed dependency: {name}")


def dependency_root(name: str, *, root: Path = PROJECT_ROOT) -> Path:
    return reject_reference_path(root / dependency(name, root=root)["path"])


def dependency_python(name: str, *, root: Path = PROJECT_ROOT) -> Path:
    return reject_reference_path(root / dependency(name, root=root)["python"])
