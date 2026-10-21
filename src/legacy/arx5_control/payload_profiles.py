"""Persistent named payload profiles shared by calibration and the web UI."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import time
from typing import Any

from embodied.core.arx5_paths import CALIBRATIONS_ROOT


DEFAULT_PAYLOAD_ROOT = CALIBRATIONS_ROOT / "payload"
PROFILE_NAME_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,47}")


def validate_profile_name(name: str) -> str:
    value = str(name).strip()
    if not PROFILE_NAME_PATTERN.fullmatch(value):
        raise RuntimeError(
            "负载档案名必须以字母或数字开头，且只能包含字母、数字、点、横线和下划线"
        )
    return value


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{time.time_ns()}.tmp")
    try:
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"文件不存在：{path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"JSON 无效：{path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"JSON 顶层必须是对象：{path}")
    return payload


def profile_path(name: str, root: Path = DEFAULT_PAYLOAD_ROOT) -> Path:
    return root / "profiles" / f"{validate_profile_name(name)}.json"


def validate_payload_result(payload: dict[str, Any]) -> dict[str, Any]:
    if payload.get("kind") != "added_payload":
        raise RuntimeError("负载档案必须来自 fit-payload 结果")
    mass = payload.get("estimated_payload_mass_kg")
    if isinstance(mass, bool) or not isinstance(mass, (int, float)):
        raise RuntimeError("负载档案缺少有效质量")
    mass = float(mass)
    if not math.isfinite(mass) or mass < 0.0:
        raise RuntimeError("负载质量必须是非负有限值")
    center = payload.get("center_of_mass_tcp_m")
    if center is not None and (
        not isinstance(center, list)
        or len(center) != 3
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in center
        )
    ):
        raise RuntimeError("负载重心必须是三个有限值或 null")
    mount_mode = payload.get("mount_mode")
    if mount_mode is not None and mount_mode not in {"fixed", "grasped"}:
        raise RuntimeError("负载安装方式必须是 fixed、grasped 或 null")
    return payload


def save_payload_profile(
    name: str,
    payload: dict[str, Any],
    *,
    root: Path = DEFAULT_PAYLOAD_ROOT,
    activate: bool = True,
) -> Path:
    profile_name = validate_profile_name(name)
    stored = dict(validate_payload_result(payload))
    stored["profile"] = profile_name
    destination = profile_path(profile_name, root)
    write_json_atomic(destination, stored)
    if activate:
        select_payload_profile(profile_name, root=root)
    return destination


def select_payload_profile(
    name: str | None,
    *,
    root: Path = DEFAULT_PAYLOAD_ROOT,
) -> None:
    profile_name = None if name is None else validate_profile_name(name)
    if profile_name is not None:
        load_payload_profile(profile_name, root=root)
    write_json_atomic(
        root / "active-profile.json",
        {
            "version": 1,
            "active_profile": profile_name,
        },
    )


def load_payload_profile(
    name: str,
    *,
    root: Path = DEFAULT_PAYLOAD_ROOT,
) -> dict[str, Any]:
    profile_name = validate_profile_name(name)
    payload = validate_payload_result(read_json_object(profile_path(profile_name, root)))
    result = dict(payload)
    result["profile"] = profile_name
    return result


def active_profile_name(root: Path = DEFAULT_PAYLOAD_ROOT) -> str | None:
    pointer = root / "active-profile.json"
    if not pointer.exists():
        return None
    payload = read_json_object(pointer)
    if payload.get("version") != 1:
        raise RuntimeError("当前负载档案指针版本无效")
    name = payload.get("active_profile")
    if name is None:
        return None
    if not isinstance(name, str):
        raise RuntimeError("当前负载档案名无效")
    return validate_profile_name(name)


def load_active_payload_profile(
    root: Path = DEFAULT_PAYLOAD_ROOT,
) -> dict[str, Any] | None:
    name = active_profile_name(root)
    return None if name is None else load_payload_profile(name, root=root)


def list_payload_profiles(
    root: Path = DEFAULT_PAYLOAD_ROOT,
) -> list[dict[str, Any]]:
    active = active_profile_name(root)
    profiles: list[dict[str, Any]] = []
    for path in sorted((root / "profiles").glob("*.json")):
        try:
            payload = load_payload_profile(path.stem, root=root)
        except RuntimeError:
            continue
        payload["active"] = payload["profile"] == active
        profiles.append(payload)
    return profiles


def payload_web_state(payload: dict[str, Any] | None) -> dict[str, Any]:
    if payload is None:
        return {
            "payload_profile": None,
            "payload_mass_kg": 0.0,
            "payload_center_of_mass_tcp_m": None,
            "payload_fit_rmse_nm": None,
            "payload_mount_mode": None,
        }
    validated = validate_payload_result(payload)
    return {
        "payload_profile": validated.get("profile"),
        "payload_mass_kg": float(validated["estimated_payload_mass_kg"]),
        "payload_center_of_mass_tcp_m": validated.get("center_of_mass_tcp_m"),
        "payload_fit_rmse_nm": validated.get("fit_rmse_nm"),
        "payload_mount_mode": validated.get("mount_mode"),
    }


@dataclass
class PayloadProfileReader:
    root: Path = DEFAULT_PAYLOAD_ROOT
    refresh_interval_s: float = 0.5
    _next_refresh: float = 0.0
    _cached: dict[str, Any] | None = None

    def web_state(self) -> dict[str, Any]:
        now = time.monotonic()
        if now >= self._next_refresh:
            try:
                self._cached = load_active_payload_profile(self.root)
            except RuntimeError:
                self._cached = None
            self._next_refresh = now + self.refresh_interval_s
        return payload_web_state(self._cached)
