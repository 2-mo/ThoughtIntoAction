"""Named device setups: save one, then load it with a single click.

The value a saved entry holds is the same validated document the setup page
edits (`configs/devices.json`); only the storage differs. Applying an entry
copies it back into that document, so every existing consumer keeps reading the
one file it already knows.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

from embodied.core.device_setup import SetupError, validate_setup
from embodied.core.project import PROJECT_ROOT

LIBRARY_DIR = PROJECT_ROOT / "configs/setups"
ACTIVE_PATH = PROJECT_ROOT / "outputs/runtime/setup/active.json"
NAME_LIMIT = 40


def _read(path: Path) -> dict | None:
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def _write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def _slug(name: str) -> str:
    """A filename that survives non-ASCII names; the display name stays verbatim."""
    base = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")[:32] or "setup"
    return base


def _free_slug(name: str) -> str:
    base = _slug(name)
    candidate, index = base, 2
    while (LIBRARY_DIR / f"{candidate}.json").exists():
        candidate, index = f"{base}-{index}", index + 1
    return candidate


def _name(value) -> str:
    if not isinstance(value, str) or not 1 <= len(value.strip()) <= NAME_LIMIT:
        raise SetupError(f"配置名称必须是 1..{NAME_LIMIT} 字符")
    if any(ord(character) < 32 for character in value):
        raise SetupError("配置名称包含控制字符")
    return value.strip()


def entries() -> list[dict]:
    """Saved setups, newest first, without their camera bodies."""
    found = []
    for path in LIBRARY_DIR.glob("*.json") if LIBRARY_DIR.is_dir() else ():
        stored = _read(path)
        if stored is None or not isinstance(stored.get("setup"), dict):
            continue
        document = stored["setup"]
        found.append({
            "slug": path.stem,
            "name": stored.get("name") or path.stem,
            "robots": len(document.get("robots") or ()),
            "cameras": len(document.get("cameras") or ()),
            "saved_at": stored.get("saved_at"),
        })
    return sorted(found, key=lambda entry: entry.get("saved_at") or 0, reverse=True)


def load(slug: str) -> dict:
    stored = _read(LIBRARY_DIR / f"{slug}.json")
    if stored is None or not isinstance(stored.get("setup"), dict):
        raise SetupError(f"没有这个配置：{slug}", 404)
    return {"slug": slug, "name": stored.get("name") or slug,
            "setup": validate_setup(stored["setup"])}


def save(name: str, document: dict, slug: str | None = None) -> dict:
    """Create or replace an entry; the document is validated before it lands."""
    setup = validate_setup(document)
    title = _name(name)
    if slug is None:
        slug = _free_slug(title)
    elif not re.fullmatch(r"[a-z0-9][a-z0-9-]{0,63}", slug or ""):
        raise SetupError(f"不合法的配置 ID：{slug}")
    if not isinstance(title, str):
        raise SetupError("配置名称必须是字符串")
    _write(LIBRARY_DIR / f"{slug}.json",
           {"version": 1, "name": title, "saved_at": time.time(), "setup": setup})
    return {"slug": slug, "name": title, "setup": setup}


def delete(slug: str) -> None:
    path = LIBRARY_DIR / f"{slug}.json"
    try:
        path.unlink()
    except OSError as exc:
        raise SetupError(f"没有这个配置：{slug}", 404) from exc
    if active() == slug:
        try:
            ACTIVE_PATH.unlink()
        except OSError:
            pass


def active() -> str | None:
    stored = _read(ACTIVE_PATH)
    value = stored.get("slug") if stored else None
    return value if isinstance(value, str) else None


def set_active(slug: str) -> None:
    _write(ACTIVE_PATH, {"slug": slug, "applied_at": time.time()})
