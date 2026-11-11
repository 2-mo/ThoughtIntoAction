"""Device setup document: robots, cameras and their pairing.

Standard-library only so `embodied.core.project` can merge the saved document
without importing the console application. Hardware probing, kernel profile
compilation and the API live in the console.
"""
from __future__ import annotations

import json
import os
import re
import time
from pathlib import Path

from embodied.core.project import PROJECT_ROOT

# QUEST_DEVICES_FILE keeps tests and alternate rigs off the shared file.
DEVICES_PATH = Path(os.environ.get("QUEST_DEVICES_FILE") or (PROJECT_ROOT / "configs/devices.json"))
GENERATED_DIR = PROJECT_ROOT / "outputs/runtime/setup"

VENDORS = {
    "arx": {
        "label": "ARX X5",
        "implemented": True,
        "models": {"x5": {"label": "X5", "arm_counts": (1, 2)}},
        "robot_profile": {1: "arx5", 2: "arx5-dual"},
        "workbench_profile": {1: "real-quest", 2: "real-dual-quest"},
    },
    "realman": {
        "label": "RealMan",
        "implemented": False,
        "models": {},
        "note": "驱动与模型待接入：提供控制代码后在这里注册 profile 与 factory",
    },
}

CAMERA_ID = re.compile(r"[a-z][a-z0-9_]*_rgb")
ROBOT_ID = re.compile(r"[a-z][a-z0-9_-]{0,31}")



class SetupError(ValueError):
    """Invalid setup document (HTTP 400) or an unavailable vendor (HTTP 409)."""

    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


# --------------------------------------------------------------------------- storage


def load_setup() -> dict | None:
    """The saved setup document, or None when the page was never used."""
    try:
        value = json.loads(DEVICES_PATH.read_text())
    except (OSError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def save_setup(document: dict) -> dict:
    normalized = validate_setup(document)
    DEVICES_PATH.parent.mkdir(parents=True, exist_ok=True)
    temporary = DEVICES_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(normalized, ensure_ascii=False, indent=2) + "\n")
    os.replace(temporary, DEVICES_PATH)
    return normalized


def camera_entries(document: dict) -> dict:
    """Cameras in the `[cameras.<id>]` shape every existing consumer expects."""
    entries = {}
    for camera in document.get("cameras") or ():
        entries[camera["id"]] = {
            "label": camera.get("label") or camera["id"],
            "enabled": bool(camera.get("enabled", True)),
            "driver": camera.get("driver") or "realsense",
            "model": camera.get("model"),
            "serial": camera.get("serial"),
            "width": camera.get("width"),
            "height": camera.get("height"),
            "fps": camera.get("fps"),
            "mount": camera.get("mount"),
            "arm": camera.get("arm"),
        }
    return entries


# --------------------------------------------------------------------------- validation


def _text(value, field, *, required=True, limit=120):
    if value is None and not required:
        return None
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise SetupError(f"{field} 必须是非空字符串且不超过 {limit} 字符")
    return value.strip()


def _positive(value, field, default):
    if value is None:
        return default
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0 or value > 10000:
        raise SetupError(f"{field} 必须是正整数")
    return value


def validate_setup(document) -> dict:
    if not isinstance(document, dict) or document.get("version") != 1:
        raise SetupError("设备配置缺少 version: 1")

    robots_in = document.get("robots")
    if not isinstance(robots_in, list) or not 1 <= len(robots_in) <= 2:
        raise SetupError("需要 1 台或 2 台机械臂")
    robots, ids, sides, interfaces = [], set(), set(), set()
    for index, raw in enumerate(robots_in):
        if not isinstance(raw, dict):
            raise SetupError(f"第 {index + 1} 台机械臂配置格式错误")
        identifier = _text(raw.get("id"), f"robots[{index}].id", limit=32)
        if not ROBOT_ID.fullmatch(identifier):
            raise SetupError(f"机械臂 ID 只能是小写字母、数字、_ 或 -：{identifier}")
        if identifier in ids:
            raise SetupError(f"机械臂 ID 重复：{identifier}")
        ids.add(identifier)
        vendor = _text(raw.get("vendor"), f"robots[{index}].vendor", limit=32)
        spec = VENDORS.get(vendor)
        if spec is None:
            raise SetupError(f"未知厂商：{vendor}")
        if not spec.get("implemented"):
            raise SetupError(f"{spec['label']} 尚未接入：{spec.get('note', '驱动与 profile 待提供')}", 409)
        model = _text(raw.get("model"), f"robots[{index}].model", limit=32)
        if model not in spec["models"]:
            raise SetupError(f"{spec['label']} 不支持型号 {model}")
        side = raw.get("side")
        if len(robots_in) == 1:
            side = "single"
        elif side not in ("left", "right"):
            raise SetupError("双臂配置需要 left / right 各一台")
        if side in sides:
            raise SetupError(f"重复的臂侧：{side}")
        sides.add(side)
        interface = _text(raw.get("can_interface", "can0"), f"robots[{index}].can_interface", limit=32)
        if not re.fullmatch(r"can\d+", interface):
            raise SetupError(f"CAN 接口名不合法：{interface}")
        if interface in interfaces:
            raise SetupError(f"CAN 接口重复：{interface}")
        interfaces.add(interface)
        robots.append({
            "id": identifier,
            "vendor": vendor,
            "model": model,
            "side": side,
            "can_interface": interface,
            "can_device": raw.get("can_device"),
        })
    if len({robot["vendor"] for robot in robots}) != 1:
        raise SetupError("同一套设备只能选择一个厂商")
    if len(robots) == 2 and sides != {"left", "right"}:
        raise SetupError("双臂配置需要 left / right 各一台")
    if len(robots) == 2 and not spec["robot_profile"].get(2):
        raise SetupError("该厂商不支持双臂")

    cameras_in = document.get("cameras")
    if not isinstance(cameras_in, list) or not cameras_in:
        raise SetupError("至少需要配置一台相机")
    cameras, camera_ids, serials, wrist_arms = [], set(), set(), set()
    for index, raw in enumerate(cameras_in):
        if not isinstance(raw, dict):
            raise SetupError(f"第 {index + 1} 台相机配置格式错误")
        identifier = _text(raw.get("id"), f"cameras[{index}].id", limit=40)
        if not CAMERA_ID.fullmatch(identifier):
            raise SetupError(f"相机 ID 必须形如 wrist_rgb / exterior_rgb：{identifier}")
        if identifier in camera_ids:
            raise SetupError(f"相机 ID 重复：{identifier}")
        camera_ids.add(identifier)
        serial = raw.get("serial")
        serial = serial.strip() if isinstance(serial, str) else ""
        if len(serial) > 64:
            raise SetupError(f"cameras[{index}].serial 过长")
        if serial:
            if serial in serials:
                raise SetupError(f"相机序列号重复：{serial}")
            serials.add(serial)
        mount = raw.get("mount", "fixed")
        if mount not in ("wrist", "fixed"):
            raise SetupError("相机安装位置只能是 wrist 或 fixed")
        arm = raw.get("arm")
        if mount == "wrist":
            if len(robots) == 1:
                arm = robots[0]["id"]
            if arm not in ids:
                raise SetupError(f"{identifier} 的腕部相机必须绑定到已配置的机械臂")
            if arm in wrist_arms:
                raise SetupError(f"机械臂 {arm} 只能有一台腕部相机")
            wrist_arms.add(arm)
        else:
            arm = None
        cameras.append({
            "id": identifier,
            "label": _text(raw.get("label") or identifier, f"cameras[{index}].label", limit=60),
            "enabled": bool(raw.get("enabled", True)),
            "driver": _text(raw.get("driver") or "realsense", f"cameras[{index}].driver", limit=32),
            "model": _text(raw.get("model"), f"cameras[{index}].model", required=False, limit=40),
            "serial": serial,
            "width": _positive(raw.get("width"), "width", 640),
            "height": _positive(raw.get("height"), "height", 480),
            "fps": _positive(raw.get("fps"), "fps", 30),
            "mount": mount,
            "arm": arm,
        })
    return {"version": 1, "robots": robots, "cameras": cameras}


