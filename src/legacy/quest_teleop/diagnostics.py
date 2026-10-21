"""Read-only workstation checks. Does not start CAN, camera streams or XR services."""

import argparse
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys


def run_command(argv):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=4)
        return {"ok": result.returncode == 0, "detail": (result.stdout or result.stderr).strip()[:3000]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "detail": str(exc)}


def diagnose(config):
    from embodied.core.project import resolve_path
    from embodied.core.model import RobotModel
    from embodied.core.session_protocol import request
    checks = {}
    for package in ("arx5-interface", "numpy", "pin", "placo", "meshcat", "pyrealsense2", "opencv-python"):
        try:
            checks[package] = {"ok": True, "detail": importlib.metadata.version(package)}
        except importlib.metadata.PackageNotFoundError:
            checks[package] = {"ok": False, "detail": "not installed"}
    try:
        model = RobotModel(resolve_path(config["robot"]["urdf"]))
        checks["robot_model"] = {"ok": True, "detail": model.metadata()}
    except (ValueError, OSError) as exc:
        checks["robot_model"] = {"ok": False, "detail": str(exc)}
    checks["can_interface"] = run_command(["ip", "-json", "-details", "-statistics", "link", "show", config["robot"]["can_port"]])
    if checks["can_interface"]["ok"]:
        try:
            link = json.loads(checks["can_interface"]["detail"])[0]
            can_state = link.get("linkinfo", {}).get("info_data", {}).get("state")
            checks["can_interface"] = {"ok": "UP" in link.get("flags", []) and can_state not in {"BUS-OFF", "STOPPED"},
                                       "detail": link}
        except (ValueError, IndexError):
            checks["can_interface"]["ok"] = False
    checks["usb_arm_device"] = {"ok": Path(config["robot"]["can_device"]).exists(), "detail": config["robot"]["can_device"]}
    checks["quest_adb"] = run_command(["adb", "devices", "-l"])
    if checks["quest_adb"]["ok"]:
        device_lines = checks["quest_adb"]["detail"].splitlines()[1:]
        checks["quest_adb"]["ok"] = any(len(line.split()) > 1 and line.split()[1] == "device" for line in device_lines)
    checks["adb_reverse"] = run_command(["adb", "reverse", "--list"])
    root = resolve_path(config["paths"]["dataset_dir"])
    while not root.exists() and root != root.parent:
        root = root.parent
    free = shutil.disk_usage(root).free
    checks["dataset_disk"] = {"ok": free >= config["recording"]["min_free_bytes"],
                              "detail": {"path": str(resolve_path(config["paths"]["dataset_dir"])), "free_bytes": free}}
    try:
        import pyrealsense2 as rs
        devices = [{"serial": d.get_info(rs.camera_info.serial_number), "model": d.get_info(rs.camera_info.name)}
                   for d in rs.context().query_devices()]
        matching = [d for d in devices if d["serial"] == config["cameras"]["wrist_rgb"]["serial"]]
        checks["wrist_camera"] = {"ok": len(matching) == 1, "detail": devices}
    except Exception as exc:
        checks["wrist_camera"] = {"ok": False, "detail": str(exc)}
    try:
        status = request(resolve_path(config["runtime"]["socket"]), "status")
        checks["session"] = {"ok": status["state_age_s"] is not None and status["state_age_s"] < .5,
                             "detail": status}
    except (OSError, RuntimeError) as exc:
        checks["session"] = {"ok": None, "detail": f"No running session: {exc}"}
    return {"python": sys.executable, "checks": checks,
            "note": "Read-only enumeration; CAN link counters/SDK clocks do not prove per-motor feedback freshness"}


def main(argv=None):
    from embodied.core.project import load_config
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", help="Optional JSON report path")
    args = parser.parse_args(argv)
    report = diagnose(load_config())
    if args.output:
        from embodied.core.storage import save_json
        save_json(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if any(value["ok"] is False for value in report["checks"].values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
