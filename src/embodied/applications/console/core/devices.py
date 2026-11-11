"""Console side of the device setup: discovery, kernel profile compilation.

The validated document itself lives in `embodied.core.device_setup` so config
loading stays application-free.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

from embodied.core.device_setup import (  # noqa: F401  (re-exported for the console)
    DEVICES_PATH, GENERATED_DIR, ROBOT_ID, CAMERA_ID, VENDORS, SetupError, camera_entries, load_setup,
    save_setup, validate_setup,
)
from embodied.core.project import PROJECT_ROOT

_DISCOVERY = {"at": 0.0, "value": None}
DISCOVERY_TTL_S = 5.0


# --------------------------------------------------------------------------- compile


def compile_setup(document: dict) -> dict:
    """Write the kernel-facing robot profile for the selected vendor and arms.

    The ARX profile loader is unchanged; the generated JSON simply carries the
    CAN interfaces chosen on the page. Vendors register their own generator in
    `VENDORS[...]` when their control code lands.
    """
    vendor = document["robots"][0]["vendor"]
    spec = VENDORS[vendor]
    count = len(document["robots"])
    profile = spec["robot_profile"].get(count)
    if profile is None:
        raise SetupError(f"{spec['label']} 不支持 {count} 台机械臂", 409)
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    arms = sorted(document["robots"], key=lambda arm: arm["side"] != "left")
    if count == 1:
        robot = {"profile": profile, "can_interface": arms[0]["can_interface"]}
    else:
        robot = {"profile": profile, "version": 1,
                 "arms": [{"id": arm["side"], "can_interface": arm["can_interface"]} for arm in arms]}
    robot_path = GENERATED_DIR / f"{profile}.json"
    robot_path.write_text(json.dumps(robot, ensure_ascii=False, indent=2) + "\n")
    base_name = spec["workbench_profile"].get(count)
    base = json.loads((PROJECT_ROOT / "configs/workbench" / f"{base_name}.json").read_text())
    base["config"] = _portable(robot_path)
    workbench_path = GENERATED_DIR / f"{base_name}.json"
    workbench_path.write_text(json.dumps(base, ensure_ascii=False, indent=2) + "\n")
    return {"robot_profile": _portable(robot_path),
            "workbench_profile": _portable(workbench_path),
            "workbench_name": str(workbench_path)}


def _portable(path: Path) -> str:
    """Repo-relative when possible, absolute for generated files outside it."""
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def pending_restarts(document: dict, *, kernel_running: bool, camera_running: bool) -> list[dict]:
    """What the operator must restart for the saved setup to take effect."""
    items = []
    if camera_running:
        items.append({"kind": "sensor", "label": "相机服务", "reason": "相机清单或序列号已变化"})
    if kernel_running:
        items.append({"kind": "kernel", "label": "内核", "reason": "机械臂或 CAN 配置已变化"})
    return items


# --------------------------------------------------------------------------- discovery


def _realsense_devices() -> list[dict]:
    try:
        import pyrealsense2 as rs
    except Exception as exc:  # SDK missing on this host
        return [{"error": f"RealSense SDK 不可用：{exc}"}]
    found = []
    try:
        context = rs.context()
        for device in context.query_devices():
            name = device.get_info(rs.camera_info.name)
            found.append({
                "kind": "realsense",
                "name": name,
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": _firmware(device, rs),
                "model": "D405" if "D405" in name else "D435i" if "D435" in name else name,
            })
    except Exception as exc:
        return [{"error": f"枚举 RealSense 失败：{exc}"}]
    return found


def _can_adapters() -> tuple[list[dict], list[dict]]:
    adapters = []
    links = sorted(Path("/dev/serial/by-id").glob("usb-ARX_USB2CAN_*-if00")) if Path("/dev/serial/by-id").is_dir() else []
    for index, link in enumerate(links):
        try:
            device = os.path.realpath(link)
        except OSError:
            continue
        serial = link.name.split("usb-ARX_USB2CAN_")[-1].split("-if00")[0]
        adapters.append({"kind": "usb2can", "serial": serial, "device": device,
                         "interface": f"can{index}", "symlink": str(link)})
    interfaces = []
    for path in sorted(Path("/sys/class/net").glob("can*")):
        try:
            state = (path / "operstate").read_text().strip()
        except OSError:
            continue
        interfaces.append({"name": path.name, "state": state})
    return adapters, interfaces


def _firmware(device, rs):
    try:
        return device.get_info(rs.camera_info.firmware_version)
    except Exception:
        return None


def discover(force: bool = False) -> dict:
    now = time.monotonic()
    cached = _DISCOVERY["value"]
    if cached is not None and not force and now - _DISCOVERY["at"] < DISCOVERY_TTL_S:
        return cached
    adapters, interfaces = _can_adapters()
    value = {
        "cameras": _realsense_devices(),
        "can_adapters": adapters,
        "can_interfaces": interfaces,
        "vendors": {name: {"label": spec["label"], "implemented": spec["implemented"],
                           "models": {model: {"label": model_spec["label"], "arm_counts": list(model_spec["arm_counts"])}
                                      for model, model_spec in spec["models"].items()}}
                    for name, spec in VENDORS.items()},
        "generated_at": now,
    }
    _DISCOVERY["value"], _DISCOVERY["at"] = value, now
    return value
