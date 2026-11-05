#!/usr/bin/env python3
"""Report the VR collection preconditions and the running kernel's teleop state.

Read-only: never opens the XR SDK, never acquires a control lease.
See docs/guides/vr-collection-runbook.md for the full procedure.
"""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / "outputs/runtime/control"
sys.path.insert(0, str(ROOT / "src"))


def line(label, ok, detail=""):
    mark = "OK  " if ok else "FAIL"
    print(f"{mark} {label:<16} {detail}")


def can_state():
    result = subprocess.run(["ip", "-br", "link", "show", "can0"], capture_output=True, text=True)
    return result.returncode == 0 and "UP" in result.stdout


def xr_connection():
    result = subprocess.run(
        [str(ROOT / "run.sh"), "quest-transport", "status"], capture_output=True, text=True,
        cwd=ROOT, env={**__import__("os").environ, "QUEST_BOOTSTRAP_PYTHON": str(ROOT / ".venv/bin/python")},
    )
    for text in result.stdout.splitlines():
        if "当前连接" in text:
            return not text.rstrip().endswith("无"), text.strip("│ ")
    return False, "未能读取"


def live_run():
    from embodied.contracts.runtime.control import live_runs

    runs = live_runs(RUNS)
    return runs[-1] if runs else None


def main() -> int:
    line("can0", can_state(), "UP" if can_state() else "未建立，先运行 ./run.sh can-up")
    connected, detail = xr_connection()
    line("Quest 客户端", connected, detail)

    run = live_run()
    line("内核", run is not None, run.name if run else "未运行")
    if run is None:
        print("\n结论：前置条件未全部满足，先按 runbook 启动，不要启动内核试。")
        return 1

    from embodied.contracts.runtime.control import ControlClient

    status = ControlClient(run).status()
    ingress = (status.get("ingresses") or {}).get("native") or {}
    log = run / "coordinator.log"
    transitions = [
        entry[entry.index("entering "):][:90]
        for entry in log.read_text(errors="replace").splitlines() if "entering " in entry
    ] if log.exists() else []
    disabled = [entry for entry in transitions if "Disabled" in entry]
    faulted = [entry for entry in disabled if "RouteFault" in entry]

    line("协调器", not disabled, disabled[-1] if disabled else "无 Disabled 记录")
    line("路由健康", not faulted, faulted[-1] if faulted else "无 RouteFault")
    arms = status.get("ingress_arms") or (12 if len(status.get("q") or []) == 12 else 6)
    line("机械臂关节", len(status.get("q") or []) > 0, f"{len(status.get('q') or [])} 关节")
    line("ingress.ready", bool(ingress.get("ready")), str(ingress.get("reason") or ""))
    line("ingress.armed", bool(ingress.get("armed")), "按住 Grip 后应为 true")
    line("ingress.permit", bool(ingress.get("permit")), "协调器处于 CommandFollowing 时才为 true")
    line("phase", status.get("phase") == "teleop", str(status.get("phase")))
    line("owner", (status.get("owner") or {}).get("source") == "quest", json.dumps(status.get("owner"), ensure_ascii=False))

    if disabled:
        print("\n结论：协调器处于 Disabled。先短按右手柄 A（约 0.2 s，不要碰 B）切到 FreeDrive；"
              "若 A 无反应再重启内核。")
        return 1
    if not ingress.get("permit"):
        print("\n结论：等待操作者握持 Grip；若握持后仍为 false，重启内核。")
        return 1
    print("\n结论：遥操链路就绪。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
