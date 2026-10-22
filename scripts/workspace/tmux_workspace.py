#!/usr/bin/env python3
"""One tmux session for the existing interactive consoles; no device SDK imports."""

import argparse
from contextlib import closing
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from embodied.contracts.device_lease import DeviceLease
from embodied.core.project import load_config, resolve_path
from embodied.core.session_protocol import request
from embodied.applications.monitor.mode import MODES, selected_mode, save_mode

SESSION = "questarm-" + hashlib.sha256(str(ROOT).encode()).hexdigest()[:8]
ROLES = {"arm": "机械臂", "teleop": "遥操", "web": "网页", "home": "总控"}


def tmux(*args, check=True):
    result = subprocess.run(["tmux", *args], text=True, capture_output=True)
    if check and result.returncode:
        raise RuntimeError(result.stderr.strip() or "tmux 命令失败")
    return result


def exists():
    return tmux("has-session", "-t", SESSION, check=False).returncode == 0


def arm_info():
    path = resolve_path(load_config()["arm_service"]["socket"])
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
            client.settimeout(.5)
            client.connect(str(path))
            client.sendall(json.dumps({"op": "describe", "sent_monotonic_ns": time.monotonic_ns()}).encode())
            raw, _, flags, _ = client.recvmsg(65536)
        if not raw:
            return None
        if flags & socket.MSG_TRUNC:
            raise RuntimeError("机械臂状态响应不完整")
        response = json.loads(raw)
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "机械臂状态读取失败"))
        return response["result"]
    except (FileNotFoundError, ConnectionRefusedError, ConnectionResetError, TimeoutError):
        return None


def teleop_running():
    try:
        request(resolve_path(load_config()["runtime"]["socket"]), "status", timeout=1)
        return True
    except (FileNotFoundError, ConnectionRefusedError):
        return False


def can_connected():
    config = load_config()["robot"]
    return ((Path("/sys/class/net") / config["can_port"]).exists()
            or resolve_path(config["can_device"]).is_char_device())


def quest_wait_reason():
    """Read-only discovery; an absent/offline headset is not an execution error."""
    result = subprocess.run(["adb", "devices", "-l"], text=True, capture_output=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "ADB 状态读取失败")
    devices = [line.split() for line in result.stdout.splitlines()
               if len(line.split()) >= 2 and not line.startswith("List ")
               and ("usb:" in line or ":" not in line.split()[0])]
    if not devices:
        return "等待 Quest 开机并连接 USB…"
    if len(devices) > 1:
        raise RuntimeError("检测到多台 USB ADB 设备，请仅保留需要遥操的 Quest。")
    serial, state = devices[0][:2]
    if state == "unauthorized":
        return "等待 Quest USB 调试授权：请在头显内允许此主机。"
    if state == "offline":
        return "等待 Quest USB 连接就绪…"
    if state != "device":
        raise RuntimeError(f"Quest ADB 状态异常：{state}")
    boot = subprocess.run(["adb", "-s", serial, "shell", "getprop", "sys.boot_completed"],
                          text=True, capture_output=True, timeout=5)
    if boot.returncode:
        if any(value in boot.stderr.lower() for value in ("offline", "not found", "no devices", "device still authorizing")):
            return "等待 Quest USB 连接恢复…"
        raise RuntimeError(boot.stderr.strip() or "Quest 启动状态读取失败")
    return None if boot.stdout.strip() == "1" else "等待 Quest 完成开机…"


def panes():
    rows = tmux("list-panes", "-s", "-t", SESSION, "-F",
                "#{pane_id}|#{@questarm_role}|#{pane_dead}|#{@questarm_phase}").stdout.splitlines()
    return {role: (pane, dead == "1", phase) for pane, role, dead, phase in
            (row.split("|", 3) for row in rows) if role in ROLES}


def phase(value):
    tmux("set-option", "-p", "-t", os.environ["TMUX_PANE"], "@questarm_phase", value)


def check_cancelled():
    if tmux("show-options", "-pqv", "-t", os.environ["TMUX_PANE"], "@questarm_cancel").stdout.strip() == "1":
        raise SystemExit("已取消等待启动。")


def execute(script):
    check_cancelled()
    phase("console")
    os.execvp("bash", ["bash", str(ROOT / script)])


def run_pane(role):
    pane = os.environ["TMUX_PANE"]
    tmux("set-option", "-w", "-t", pane, "remain-on-exit", "on")
    tmux("set-option", "-w", "-t", pane, "automatic-rename", "off")
    tmux("set-option", "-p", "-t", pane, "@questarm_role", role)
    tmux("set-option", "-p", "-t", pane, "@questarm_cancel", "0")
    phase("waiting")
    # Wait until the launcher has created all windows, including on slow terminals.
    while tmux("show-options", "-qv", "-t", SESSION, "@questarm_ready").stdout.strip() != "1":
        check_cancelled()
        time.sleep(.1)
    check_cancelled()
    print("Ctrl+B 再按 4 返回总控；1/2/3 查看各控制台；Ctrl+B 再按 D 离开。", flush=True)
    if role in ('arm', 'teleop') and selected_mode() == 'simulation':
        execute_details(role)
        return
    if role == "arm":
        announced = False
        while True:
            check_cancelled()
            if arm_info() is not None:
                print("复用已有机械臂服务；控制台仍在原终端，本工作台不负责关闭它。", flush=True)
                return
            if can_connected():
                break
            if not announced:
                print("等待 USB-CAN 连接；连接后自动继续，可用工作台 stop 取消。", flush=True)
                announced = True
            time.sleep(2)
        print("准备 CAN；如需 sudo 密码，请在本窗口输入。启动将使能机械臂并进入阻尼待机。", flush=True)
        execute("scripts/shortcuts/启动机械臂服务.sh")
    elif role == "teleop":
        if teleop_running():
            print("复用已有遥操会话，请在原终端操作；未重复启动。", flush=True)
            return
        last_message = None
        while True:
            check_cancelled()
            if teleop_running():
                print("已有遥操会话启动，直接复用。", flush=True)
                return
            info = arm_info()
            message = "等待机械臂通电、连接并提供反馈…"
            if info is not None:
                if not info.get("hardware"):
                    raise RuntimeError("当前是虚拟机械臂服务，未启动真机遥操。")
                state = info.get("state") or {}
                if 0 <= (time.monotonic_ns() - state.get("host_monotonic_ns", 0)) / 1e9 < .5:
                    message = quest_wait_reason()
                    if message is None:
                        execute("scripts/shortcuts/启动遥操.sh")
                        return
            arm = panes().get("arm")
            if info is None and arm and arm[1]:
                message = "机械臂服务未就绪；请查看窗口 1。修复并重启机械臂窗口后，这里会自动继续等待。"
            if message != last_message:
                print(message, flush=True)
                last_message = message
            time.sleep(2)
    else:
        phase("starting")
        subprocess.run([str(ROOT / "run.sh"), "viewer-control", "start"], check=True)
        execute("scripts/shortcuts/网页控制.sh")


def execute_details(role):
    python = os.environ.get("QUEST_TELEOP_PYTHON", load_config()["environment"]["teleop_python"])
    python = str(resolve_path(python)) if "/" in python else python
    os.execvp(python, [python, "-B", str(ROOT / "scripts/workspace/details.py"), role])


def restore_simulation_panes():
    """Revive informational windows only; never restart hardware consoles."""
    if selected_mode() != 'simulation' or not exists():
        return
    for role, (target, dead, _) in panes().items():
        if role in ('arm', 'teleop') and dead:
            tmux("respawn-pane", "-t", target, "-c", str(ROOT), pane_command(role))


def pane_command(role):
    # Explicitly propagate project overrides even when tmux's server is older.
    variables = [f"{key}={value}" for key, value in os.environ.items()
                 if key.startswith("QUEST_") and key != "QUEST_WORKSPACE_ROLE"]
    variables.append(f"QUEST_WORKSPACE_ROLE={role}")
    if role == "home":
        python = os.environ.get("QUEST_TELEOP_PYTHON", load_config()["environment"]["teleop_python"])
        python = str(resolve_path(python)) if "/" in python else python
        return "exec " + shlex.join(["env", *variables, python, "-B", str(ROOT / "scripts/workspace/dashboard.py")])
    return "exec " + shlex.join(["env", *variables, sys.executable, "-B", str(Path(__file__).resolve()), "_pane", role])


def ensure_home():
    home = panes().get("home")
    if home is None:
        target = tmux("new-window", "-P", "-F", "#{pane_id}", "-t", SESSION,
                      "-n", ROLES["home"], "-c", str(ROOT), pane_command("home")).stdout.strip()
        tmux("set-option", "-p", "-t", target, "@questarm_role", "home")
    else:
        target, dead, _ = home
        if dead:
            tmux("respawn-pane", "-t", target, "-c", str(ROOT), pane_command("home"))
    tmux("select-window", "-t", target)


def start():
    if exists():
        restore_simulation_panes()
        ensure_home()
        return
    if not (ROOT / ".venv/bin/python").exists() and not os.environ.get("QUEST_TELEOP_PYTHON"):
        raise RuntimeError("项目 Python 环境不存在，请先按 README 安装依赖。")
    from embodied.applications.workstation.ports import conflicts
    if found := conflicts(load_config()):
        print("端口冲突：" + "；".join(found) + "。详情：./run.sh check ports", file=sys.stderr)
    tmux("new-session", "-d", "-s", SESSION, "-n", ROLES["arm"], "-c", str(ROOT), pane_command("arm"))
    tmux("set-option", "-t", SESSION, "base-index", "1")
    first = tmux("display-message", "-p", "-t", SESSION + ":^", "#{window_index}").stdout.strip()
    if first != "1":
        tmux("move-window", "-s", SESSION + ":" + first, "-t", SESSION + ":1")
    for role in ("teleop", "web"):
        tmux("new-window", "-t", SESSION, "-n", ROLES[role], "-c", str(ROOT), pane_command(role))
    tmux("set-option", "-t", SESSION, "@questarm_ready", "1")
    ensure_home()


def old_arm_console_exited(target):
    # Migration for consoles opened before the structured restart status existed.
    text = tmux("capture-pane", "-p", "-t", target).stdout
    return "X5 机械臂控制台" in text and "服务已退出 (" in text


def stop_pane(role):
    pane = panes().get(role)
    if not pane or pane[1]:
        return
    target, _, state = pane
    if state in ("waiting", "details"):
        tmux("set-option", "-p", "-t", target, "@questarm_cancel", "1")
    elif state in ("console", "arm-console"):
        if role == "arm":
            if teleop_running():
                raise RuntimeError("仍有遥操会话运行，机械臂服务保留；请先停止遥操。")
            if state != "arm-console" and arm_info() is None and not old_arm_console_exited(target):
                raise RuntimeError("机械臂仍在启动或等待 sudo；按 1 查看机械臂窗口。")
            tmux("send-keys", "-t", target, "q")
        else:
            tmux("send-keys", "-t", target, "C-u", "q", "Enter")
    else:
        raise RuntimeError(f"{ROLES[role]}仍在启动，请稍后重试。")
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if panes()[role][1]:
            return
        time.sleep(.2)
    raise RuntimeError(f"{ROLES[role]}尚未退出，已保留现场；按对应数字查看控制台。")


def start_role(role):
    if role in ('arm', 'teleop') and selected_mode() == 'simulation':
        from embodied.applications.monitor.control import command
        return command('sim-resume')
    pane = panes().get(role)
    if pane is None:
        raise RuntimeError("控制台窗口已被删除，请重新打开工作台。")
    target, dead, state = pane
    if role == "arm" and arm_info() is not None:
        return "机械臂服务已经在线，未重复启动。"
    if not dead and state == "details":
        stop_pane(role)
        dead = True
    if dead:
        tmux("respawn-pane", "-t", target, "-c", str(ROOT), pane_command(role))
        return f"已启动{ROLES[role]}窗口，等待设备就绪。"
    if state == "waiting":
        return f"{ROLES[role]}正在等待设备，开机并连接后会继续。"
    if role == "arm":
        supported = tmux("show-options", "-pqv", "-t", target, "@questarm_restart_key").stdout.strip() == "1"
        if state == "arm-console" and supported:
            tmux("send-keys", "-t", target, "r")
            return "已请求机械臂控制台重连；正在启动的服务不会被重复启动。"
        if old_arm_console_exited(target):
            stop_pane("arm")
            tmux("respawn-pane", "-t", target, "-c", str(ROOT), pane_command("arm"))
            return "已更新旧控制台并重新连接机械臂。"
    elif role == "teleop" and state == "console":
        tmux("send-keys", "-t", target, "C-u", "w", "Enter")
        return "已请求遥操启动 / 恢复。"
    raise RuntimeError(f"{ROLES[role]}正在启动，请稍后重试或查看对应窗口。")


def switch_mode(mode):
    """Called under the workspace lease; never leave real control active in sim mode."""
    if mode not in MODES:
        raise ValueError('请选择 simulation 或 real')
    if mode == 'simulation':
        if exists():
            stop_pane('teleop')
        if teleop_running():
            raise RuntimeError('外部遥操会话仍在运行，请在原控制台退出后再切仿真。')
        # Keep the physical service in its settled hold; no SDK reconnect on return.
        deadline = time.monotonic() + 5
        info = arm_info()
        while info and info.get('mode') == 'stopping' and time.monotonic() < deadline:
            time.sleep(.05)
            info = arm_info()
        if info and (info.get('owner') or info.get('mode') == 'stopping'):
            raise RuntimeError('真机仍被占用或尚未停稳，保留当前模式。')
        if info is None and exists():
            stop_pane('arm')  # Cancel a pending hardware startup, not a connected service.
    from embodied.applications.monitor.control import command
    # The new selection survives viewer errors; retry W without reviving real controllers.
    save_mode(mode)
    result = command('start', mode=mode)
    if mode == 'simulation':
        restore_simulation_panes()
    return result + ('；T 允许遥操，S 暂停保持；松开全部按键后右手 A 长按切换模式。' if mode == 'simulation'
                     else '；A 连接机械臂，T 启动遥操（仍需原启用手势）。')


def stop():
    if not exists():
        print("工作台未运行。")
        return
    # Send each console its normal quit key, never kill a service or cut off a recording.
    for role in ("teleop", "arm", "web"):
        stop_pane(role)
    tmux("kill-session", "-t", SESSION)
    print("工作台已关闭。网页、相机及从原终端复用的服务继续运行。")


def shutdown_services():
    for role in ("teleop", "arm", "web"):
        if exists():
            stop_pane(role)
    if teleop_running() or arm_info() is not None:
        raise RuntimeError("还有外部控制服务运行；请先在其原控制台退出，未关闭网页或相机。")
    from embodied.applications.monitor.control import command
    from embodied.devices.sensors.lifecycle import stop_service
    command("stop")
    stop_service()


def restart_all():
    shutdown_services()
    if not exists():
        start()
        return
    for role in ("arm", "web", "teleop"):
        target = panes()[role][0]
        tmux("respawn-pane", "-t", target, "-c", str(ROOT), pane_command(role))


def main():
    parser = argparse.ArgumentParser(description="统一 tmux 工作台；启动机械臂、遥操和网页。")
    parser.add_argument("action", nargs="?", default="start", choices=["start", "mode", "status", "stop", "restart", "shutdown", "restart-all", "_pane"])
    parser.add_argument("role", nargs="?", choices=[*ROLES, *MODES], help="restart 指定窗口；mode 指定 simulation / real")
    parser.add_argument('--mode', choices=MODES, help='启动时选择仿真或真机；默认沿用上次选择')
    parser.add_argument("--detach", action="store_true", help="创建后不进入；首次需要 sudo 时仍须进入窗口 1 输入密码")
    args = parser.parse_args()
    if not shutil.which("tmux"):
        parser.error("请先安装 tmux：sudo apt install tmux")
    if args.action in ("restart", "_pane") and args.role is None:
        parser.error("请指定 arm / teleop / web")
    if args.action in ('restart', '_pane') and args.role not in ROLES:
        parser.error('请指定工作台窗口')
    if args.action == 'mode' and args.role not in MODES:
        parser.error('请指定 simulation / real')
    if args.mode and args.action != 'start':
        parser.error('--mode 仅用于 start')
    if args.action == "_pane":
        run_pane(args.role)
        return
    with closing(DeviceLease("tmux-workspace:" + str(ROOT))):
        if args.action == 'mode':
            print(switch_mode(args.role), flush=True)
            return
        if args.mode:
            print(switch_mode(args.mode), flush=True)
        if args.action == "shutdown":
            shutdown_services()
            if exists():
                tmux("kill-session", "-t", SESSION)
            print("遥操、机械臂、网页、相机服务及工作台已关闭。")
            return
        if args.action == "restart-all":
            restart_all()
            print("已重新启动服务，设备未就绪时保持等待。")
            return
        if args.action == "stop":
            stop()
            return
        if args.action == "status":
            print(f'当前模式：{MODES[selected_mode()]}')
            if not exists():
                print("工作台未运行。")
            else:
                for role, (_, dead, state) in panes().items():
                    print(f"{ROLES[role]}窗口：{'已退出（可 restart）' if dead else state}；窗口状态不代表设备健康")
            return
        if args.action == "restart":
            if not exists() or args.role not in panes():
                raise RuntimeError("工作台窗口不存在，请先运行 scripts/shortcuts/启动工作台.sh。")
            pane, dead, _ = panes()[args.role]
            if not dead:
                raise RuntimeError("窗口仍在运行，请先在对应控制台正常退出；不会强制重启。")
            tmux("respawn-pane", "-t", pane, "-c", str(ROOT), pane_command(args.role))
            return
        start()
    print(f"工作台：{SESSION}；总控统一操作，1 机械臂 / 2 遥操 / 3 网页 / 4 总控。", flush=True)
    if not args.detach:
        if not sys.stdin.isatty():
            print("当前没有交互终端；请在终端运行同一命令进入工作台。")
            return
        action = "switch-client" if os.environ.get("TMUX") else "attach-session"
        os.execvp("tmux", ["tmux", action, "-t", SESSION])


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"工作台：{exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("已取消等待启动。", flush=True)
