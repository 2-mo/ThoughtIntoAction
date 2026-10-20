#!/usr/bin/env python3
"""Resolve a command without importing XR, camera or arm SDKs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import tomllib

# The bootstrap interpreter only needs the standard-library project module.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from embodied.core.project import PROJECT_ROOT, load_config, resolve_path
from embodied.workstation.catalog import GROUPS

PYTHON_COMMANDS = {
    "quest-discovery": "quest/discovery.py",
    "setup-quest-usb": "quest/transport.py",
    "quest-transport": "quest/transport.py",
    "quest-console": "quest/console.py",
    "check-xr": "quest/check.py",
    "check-arm": "diagnostics/check_arm.py",
    "test-joint": "diagnostics/joint_tracking.py",
    "test-joint-2025": "diagnostics/joint_tracking.py",
}
SHELL_COMMANDS = {
    "quest-wake": "quest/simulate_wearing.sh",
    "quest-battery": "quest/battery.sh",
    "start-xr-service": "quest/start_service.sh",
    "can-up": "setup/can_up.sh",
    "can-status": "setup/can_up.sh",
    "setup-arm-device": "setup/setup_arm_device.sh",
    "build-x5-2025-sdk": "setup/build_x5_2025_sdk.sh",
    "prepare-connections": "setup/prepare_connections.sh",
}
MAINTENANCE_COMMANDS = {
    "test": "check.py",
    "check-all": "check_all.py",
    "check-dependencies": "setup/dependencies.py",
    "fetch-dependencies": "setup/dependencies.py",
    "rebuild-xr-sdk": "setup/rebuild_xr_sdk.py",
    "check-camera": "diagnostics/check_camera.py",
}
MODULE_COMMANDS = {
    "collect": "embodied.data.capture.capture",
    "workbench": "embodied.workstation.workbench",
    "component-source": "embodied.workstation.source",
    "phone": "embodied.inputs.phone",
    "motion": "embodied.operations.cli",
    "runtime-data": "embodied.data.export.runtime_v2",
    "quest-calibrate": "legacy.quest_teleop.frame_calibration",
    "camera-service": "embodied.sensors.service",
    "viewer-control": "embodied.monitor.control",
    "teleop-official-x5": "legacy.quest_teleop.cli",
    "teleop-presets-x5": "legacy.quest_teleop.cli",
    "teleop": "legacy.quest_teleop.legacy.controller",
    "teleop-dry-run": "legacy.quest_teleop.legacy.controller",
    "teleop-x5-2025": "legacy.quest_teleop.experimental.arx2025",
    "plugins": "embodied.core.plugins",
    "visualize-xr": "legacy.quest_teleop.xr_visualizer.cli",
    "view-arm": "embodied.monitor.cli",
    "arx-preview": "embodied.monitor.arx_preview",
    "control-arm": "legacy.robot_control.cli",
    "arm-service": "legacy.robot_control.service",
    "handeye": "embodied.calibration.wrist.cli",
    "session": "legacy.quest_teleop.runtime.cli",
    "record": "legacy.quest_teleop.runtime.cli",
    "dataset": "embodied.data.datasets",
    "doctor": "legacy.quest_teleop.diagnostics",
    "monitor": "legacy.quest_teleop.runtime.monitor",
}
BOOTSTRAP_COMMANDS = {"control": "embodied.workstation.control", "components": "embodied.workstation.components.cli"}
HARDWARE_COMMANDS = {
    "test-joint", "test-joint-2025", "teleop", "teleop-dry-run",
    "teleop-x5-2025", "arm-service",
}
MODEL_COMMANDS = {"teleop-official-x5", "teleop-presets-x5", "teleop-x5-2025"}


def argument_tokens(arguments: dict) -> list[str]:
    result = []
    for name, value in arguments.items():
        flag = "--" + name.replace("_", "-")
        if isinstance(value, bool):
            result.append(flag if value else "--no-" + name.replace("_", "-"))
        elif isinstance(value, list):
            result.extend([flag, *map(str, value)])
        else:
            result.extend([flag, str(value)])
    return result


def resolve_python(config: dict) -> Path:
    value = os.environ.get("QUEST_TELEOP_PYTHON", config["environment"]["teleop_python"])
    if "/" in value or value.startswith("~"):
        python = resolve_path(value)
        if not python.is_file() or not os.access(python, os.X_OK):
            raise ValueError(f"Python executable not found: {python}. Run uv sync --locked in {PROJECT_ROOT}")
        return python
    found = shutil.which(value)
    if not found:
        raise ValueError(f"Python executable not found: {value}")
    return Path(found)


def build_command(command: str, arguments: list[str], config: dict) -> tuple[list[str], dict]:
    env = os.environ.copy()
    env["QUEST_TELEOP_LOG_DIR"] = str(resolve_path(config["paths"]["log_dir"]))
    if command in BOOTSTRAP_COMMANDS:
        # This dispatcher uses only the standard library. It selects the control
        # environment before the legacy launcher can inject Python 3.12 SDK paths.
        env["PYTHONPATH"] = str(PROJECT_ROOT / "src")
        return [sys.executable, "-B", "-m", BOOTSTRAP_COMMANDS[command], *arguments], env
    env["QUEST_CAN_PORT"] = config["robot"]["can_port"]
    env["QUEST_CAN_DEVICE"] = config["robot"]["can_device"]
    env["QUEST_CAMERA_SERIAL"] = config["cameras"]["wrist_rgb"]["serial"]
    if command in SHELL_COMMANDS and command != "build-x5-2025-sdk":
        extra = ["--status"] if command == "can-status" else []
        return ["bash", str(PROJECT_ROOT / "scripts" / SHELL_COMMANDS[command]), *extra, *arguments], env

    # Source checkout must be possible before uv creates .venv.
    if command in {"check-dependencies", "fetch-dependencies"}:
        extra = ["--fetch"] if command == "fetch-dependencies" else []
        return [sys.executable, "-B", str(PROJECT_ROOT / "scripts/setup/dependencies.py"), *extra, *arguments], env
    if command == "check-all":
        # Every group re-enters run.sh, which selects that group's interpreter and paths.
        return [sys.executable, "-B", str(PROJECT_ROOT / "scripts/check_all.py"), *arguments], env

    python = resolve_python(config)
    if (command in {"collect", "runtime-data", "workbench", "phone", "motion", "component-source"}
            or command == "handeye" and any(arg == '--runtime' or arg.startswith('--runtime=') for arg in arguments)):
        # Independent components use the gateway; they do not load legacy SDK extensions.
        env["PYTHONPATH"] = str(PROJECT_ROOT / "src")
        for key in ('PYTHONHOME', 'LD_LIBRARY_PATH'):
            env.pop(key, None)
        return [str(python), "-B", "-m", MODULE_COMMANDS[command], *arguments], env
    prefix = python.parent.parent
    # The multi-robot upstream distribution is not installed; load the pinned source.
    paths = [PROJECT_ROOT / "src", PROJECT_ROOT / "third_party/XRoboToolkit-Teleop-Sample-Python"]
    if sdk_path := config["environment"].get("arm_sdk_python"):
        sdk_path = resolve_path(sdk_path)
        if not list(sdk_path.glob("arx5_interface*.so")):
            raise RuntimeError("Patched arm SDK missing; see docs/guides/gripper-startup.md")
        paths.insert(0, sdk_path)
    env["PYTHONPATH"] = os.pathsep.join(map(str, paths))
    xr_dependencies = PROJECT_ROOT / "third_party/XRoboToolkit-Teleop-Sample-Python/dependencies"
    libs = [
        str(prefix / "lib"),
        str(xr_dependencies / "XRoboToolkit-PC-Service-Pybind/lib"),
    ]
    official_sdk = command in {"teleop-x5-2025", "test-joint-2025", "build-x5-2025-sdk"}
    official_sdk |= command == "test-joint" and any(
        token == "--backend=arx2025" or
        (token == "--backend" and arguments[i + 1:i + 2] == ["arx2025"])
        for i, token in enumerate(arguments)
    )
    if official_sdk:
        sdk = PROJECT_ROOT / "third_party/X5/py/arx_x5_python"
        env["PYTHONPATH"] += os.pathsep + str(sdk)
        api = sdk / "bimanual/api"
        libs.extend([str(api / "arx_x5_src"), str(api)])
    if env.get("LD_LIBRARY_PATH"):
        libs.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = os.pathsep.join(libs)
    env["QUEST_TELEOP_PYTHON"] = str(python)
    env["QUEST_TELEOP_PREFIX"] = str(prefix)
    defaults = []
    if command in HARDWARE_COMMANDS:
        defaults.extend(["--can-port", config["robot"]["can_port"]])
    if command in MODEL_COMMANDS:
        defaults.extend(["--robot-urdf-path", str(resolve_path(config["robot"]["urdf"]))])
    if command == "test-joint-2025":
        defaults.extend(["--backend", "arx2025"])
    if command == "teleop-dry-run":
        defaults.append("--dry-run")
    if command in MODULE_COMMANDS:
        if command in {"view-arm", "control-arm", "handeye", "arm-service"}:
            defaults.extend(["--urdf", str(resolve_path(config["robot"]["urdf"]))])
        if command in {"session", "record"}:
            defaults = [command]
        if command in {"teleop-official-x5", "teleop-presets-x5"}:
            defaults.insert(0, "presets" if command == "teleop-presets-x5" else "baseline")
        return [str(python), "-B", "-m", MODULE_COMMANDS[command], *defaults, *arguments], env
    if command == "build-x5-2025-sdk":
        return ["bash", str(PROJECT_ROOT / "scripts" / SHELL_COMMANDS[command]), *arguments], env
    if command in MAINTENANCE_COMMANDS:
        extra = ["--fetch"] if command == "fetch-dependencies" else []
        return [str(python), "-B", str(PROJECT_ROOT / "scripts" / MAINTENANCE_COMMANDS[command]), *extra, *arguments], env
    return [str(python), "-B", str(PROJECT_ROOT / "scripts" / PYTHON_COMMANDS[command]), *defaults, *arguments], env


def catalog_text(group: str | None = None) -> str:
    lines = [] if group else ["具身智能平台：./run.sh <功能> <命令> [参数]；./run.sh <功能> 只看该组。", ""]
    for name, (title, commands) in GROUPS.items():
        if group in (None, name):
            lines.append(f"{name:8} {title}")
            lines.extend(f"  {command:17} {description}" for command, (description, _) in commands.items())
            lines.append("")
    if not group:
        lines.append("更多：./run.sh components list 列出全部组件命令；原有命令名（如 arm-service、view-arm）仍可直接使用。")
    return "\n".join(lines)


def resolve_group(argv: list[str]) -> list[str] | int:
    """Translate `<group> <command> args` into a launcher command, or exec a script."""
    title, commands = GROUPS[argv[0]]
    flat = argv[0] in PYTHON_COMMANDS | SHELL_COMMANDS | MAINTENANCE_COMMANDS | MODULE_COMMANDS | BOOTSTRAP_COMMANDS
    if len(argv) == 1 or argv[1] in ("-h", "--help", "help"):
        print(catalog_text(argv[0]))
        return 0
    if argv[1] not in commands:
        if flat:
            return argv  # The single-word command keeps its own arguments, e.g. `phone --run RUN`.
        print(f"未知命令：{argv[0]} {argv[1]}\n\n" + catalog_text(argv[0]), file=sys.stderr)
        return 2
    target, rest = commands[argv[1]][1], argv[2:]
    if target[0] == "run":
        return [*target[1], *rest]
    if target[0] == "component":
        return ["components", "run", target[1], *rest]
    os.chdir(PROJECT_ROOT)
    if target[0] == "bash":
        os.execvp("bash", ["bash", str(PROJECT_ROOT / target[1]), *target[2], *rest])
    python = os.environ.get("QUEST_BOOTSTRAP_PYTHON", "/usr/bin/python3")
    os.execv(python, [python, "-B", str(PROJECT_ROOT / "scripts/workspace/tmux_workspace.py"), *target[1], *rest])
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("help", "-h", "--help"):
        print(catalog_text())
        return 0
    # Launcher options (--config X, --profile X, --print-command) may precede a group.
    start = 0
    while start < len(argv) and argv[start] in ("--config", "--profile", "--print-command"):
        start += 1 if argv[start] == "--print-command" else 2
    if start < len(argv) and argv[start] in GROUPS:
        resolved = resolve_group(argv[start:])
        if isinstance(resolved, int):
            return resolved
        argv = argv[:start] + resolved
    parser = argparse.ArgumentParser(description=__doc__, epilog="Use COMMAND --help for command-specific options.")
    parser.add_argument("--config", help="Local TOML override (relative to the repository)")
    parser.add_argument("--profile", help="Profile name in configs/profiles, or TOML path")
    parser.add_argument("--print-command", action="store_true", help="Print resolved arguments; do not execute")
    parser.add_argument("command", nargs="?", choices=sorted(PYTHON_COMMANDS | SHELL_COMMANDS | MAINTENANCE_COMMANDS | MODULE_COMMANDS | BOOTSTRAP_COMMANDS))
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    config = load_config(args.config)
    profile = {}
    if args.profile:
        path = resolve_path(args.profile) if "/" in args.profile or args.profile.endswith(".toml") else PROJECT_ROOT / "configs/profiles" / (args.profile + ".toml")
        with path.open("rb") as stream:
            profile = tomllib.load(stream)
    command = args.command or profile.get("command") or config["default_command"]
    if profile.get("command", command) != command:
        raise ValueError(f"Profile is for {profile['command']}, not {command}")
    if command not in PYTHON_COMMANDS | SHELL_COMMANDS | MAINTENANCE_COMMANDS | MODULE_COMMANDS | BOOTSTRAP_COMMANDS:
        raise ValueError(f"Unknown command: {command}")
    if "actuate" in profile.get("arguments", {}):
        raise ValueError("Profiles must not enable actuation; pass --actuate explicitly")
    arguments = argument_tokens(profile.get("arguments", {})) + args.arguments
    # The explicit dry-run mode must remain read-only.
    if command == "teleop-dry-run" and "--no-dry-run" in arguments:
        raise ValueError("teleop-dry-run cannot disable dry-run")
    cmd, env = build_command(command, arguments, config)
    if args.print_command:
        print(json.dumps({"command": command, "argv": cmd, "log_dir": env["QUEST_TELEOP_LOG_DIR"]}, ensure_ascii=False, indent=2))
        return 0
    if args.config:
        env["QUEST_TELEOP_CONFIG"] = str(resolve_path(args.config))
    os.chdir(PROJECT_ROOT)
    os.execvpe(cmd[0], cmd, env)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as exc:
        print(f"Launch error: {exc}", file=sys.stderr)
        raise SystemExit(2)
