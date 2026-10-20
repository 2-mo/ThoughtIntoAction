"""Rich terminal supervisor; device sampling stays in the service process."""

from collections import deque
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import unicodedata

from robot_core.project import load_config, resolve_path
from robot_control.startup import WAITING_FOR_ARM, NO_MOTOR_FEEDBACK
from robot_core.terminal_keys import terminal_keys, read_key
from rich.console import Console as RichConsole, Group
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.text import Text

KEYS = {"h": "home", "p": "pregrasp", "s": "stop", "i": "status", "d": "damping",
        "b": "discovery-on", "n": "discovery-off", "w": "viewer-start", "e": "viewer-stop", "v": "viewer-status", "c": "camera-on", "x": "camera-off"}
LABELS = {"viewer-start": "启动网页", "viewer-stop": "停止网页", "viewer-status": "网页状态",
          "camera-on": "开启相机", "camera-off": "关闭相机","home": "回零", "pregrasp": "预抓取并张开夹爪", "stop": "停止并保持",
          "status": "查看状态", "damping": "切入阻尼（不锁定位置，请支撑机械臂）",
          "discovery-on": "开启头显发现广播", "discovery-off": "关闭头显发现广播"}


def recovery_command(command, args, hardware):
    if command.startswith(("viewer-", "camera-")):
        return [str(Path(__file__).resolve().parents[2] / "run.sh"), "viewer-control", command.removeprefix("viewer-")]
    if command in {"discovery-on", "discovery-off"}:
        runner = Path(__file__).resolve().parents[2] / "run.sh"
        return [str(runner), "quest-discovery", command.removeprefix("discovery-")]
    script = Path(__file__).resolve().parents[1] / "zero-shot" / "recover.py"
    result = [sys.executable, "-u", "-B", str(script), "--socket", args.socket, "--urdf", args.urdf]
    if not hardware:
        result.append("--virtual")
    if command != "stop":
        result.append("--" + command)
    return result


def fit(text, width):
    """Clip by terminal columns, including Chinese full-width characters."""
    result, used = [], 0
    for char in text:
        if unicodedata.category(char).startswith("C"):
            char = " "
        size = 0 if unicodedata.combining(char) else 2 if unicodedata.east_asian_width(char) in "WF" else 1
        if used + size > width:
            break
        result.append(char)
        used += size
    return "".join(result)


class Console:
    def __init__(self, args, hardware, log):
        self.args, self.hardware, self.log = args, hardware, log
        self.events = queue.SimpleQueue()
        self.lines = deque(maxlen=2000)
        self.readers = []
        self.ready = threading.Event()
        self.service = self.action = None
        self.cancelled = False
        self.activity = "正在启动服务…"
        self.log_lock = threading.Lock()
        self.service_argv = None
        self.service_reader = None
        self.retry_at = None
        self.published_state = None

    def publish_state(self, state):
        if (state != self.published_state and os.environ.get("QUEST_WORKSPACE_ROLE") == "arm"
                and os.environ.get("TMUX_PANE")):
            subprocess.run(["tmux", "set-option", "-p", "-t", os.environ["TMUX_PANE"],
                            "@questarm_service_state", state], check=False, capture_output=True)
            self.published_state = state

    def write(self, line):
        with self.log_lock:
            self.log.write(line.rstrip() + "\n")
            self.log.flush()
        self.events.put(line.rstrip())

    def launch(self, argv, *, service=False):
        child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, errors="replace",
                                 start_new_session=True)
        def read():
            try:
                for line in child.stdout:
                    if service and NO_MOTOR_FEEDBACK in line:
                        line = "等待机械臂通电或恢复连接：尚未收到电机反馈。\n"
                    self.write(line)
                    if service and line.startswith("Arm service ready:"):
                        self.ready.set()
            finally:
                child.stdout.close()
        reader = threading.Thread(target=read, daemon=True)
        self.readers.append(reader)
        if service:
            self.service_reader = reader
        reader.start()
        return child

    def restart_service(self):
        if self.service is not None and self.service.poll() is None:
            self.write("服务仍在运行或启动中，无需重启。")
            return False
        if self.action is not None and self.action.poll() is None:
            self.write("上一项操作还未退出，请等待收尾后再按 R。")
            return False
        if self.service_argv is None:
            self.write("缺少服务启动参数，请重新打开控制台。")
            return False
        if self.service_reader is not None:
            self.service_reader.join(timeout=1)
            if self.service_reader.is_alive():
                self.write("上一服务仍在收尾，请稍后再按 R。")
                return False
        self.ready.clear()
        self.retry_at = None
        self.cancelled = False
        self.action = None
        self.activity = "正在启动服务…"
        self.write("重新连接机械臂，启动后进入阻尼待机；不自动回零。")
        try:
            self.service = self.launch(self.service_argv, service=True)
        except OSError as exc:
            self.write(f"服务启动失败：{exc}；处理后按 R 重试。")
            return False
        return True

    def retry_startup(self):
        if (not self.hardware or self.service_argv is None or self.ready.is_set()
                or self.service.poll() != WAITING_FOR_ARM):
            return False
        self.activity = "等待机械臂通电 / 连接；每 5 秒重试，Q 退出"
        if self.retry_at is None:
            self.retry_at = time.monotonic() + 5
        elif time.monotonic() >= self.retry_at:
            self.service = self.launch(self.service_argv, service=True)
            self.retry_at = None
        return True

    def cancel(self):
        if self.action is not None and self.action.poll() is None:
            if not self.cancelled:
                self.action.send_signal(signal.SIGINT)
                self.cancelled = True
                self.activity = "正在取消动作，等待停稳…"
            return True
        return False

    def command(self, command):
        if self.service.poll() is not None or not self.ready.is_set():
            self.write("服务尚未就绪，未执行操作。")
            return
        if self.action is not None and self.action.poll() is None:
            if command == "stop":
                self.cancel()
            else:
                self.write("当前操作尚未结束；按 S / 空格取消并等待停稳。")
            return
        self.cancelled = False
        self.activity = LABELS[command]
        self.write("> " + self.activity)
        self.action = self.launch(recovery_command(command, self.args, self.hardware))

    def render(self):
        console = RichConsole()
        with terminal_keys(), Live(console=console, screen=True, auto_refresh=False) as live:
            self.render_loop(console, live)

    def render_loop(self, console, live):
        self.write("H 回零保持夹爪；P 预抓取并张开夹爪。动作前停止本项目策略；其他会话占用时拒绝。")
        self.write("S / 空格 / Ctrl+C：取消当前动作，空闲时停止策略并保持。此操作不是硬件急停。")
        self.write("D 进入阻尼，不锁定当前位置，请先支撑机械臂；S 恢复当前位置保持。")
        self.write("B 开启头显发现广播；N 关闭广播（默认关闭，不重连 USB，最多约 5 秒生效）。")
        self.write("W 启动网页；E 停止网页；V 网页状态；C 开相机；X 关相机（录制时拒绝）。")
        self.write("R 重新启动已退出的服务（开机后直接按 R）；Q 关闭服务；↑↓ 日志，End 最新。")
        offset = 0
        while True:
            waiting_for_power = self.retry_startup()
            if self.action is not None and self.action.poll() is not None:
                code = self.action.returncode
                self.activity = "操作完成" if code == 0 else f"操作中断或失败（{code}），按 I 查看状态"
                self.write(self.activity)
                self.action = None
            while not self.events.empty():
                self.lines.append(self.events.get())
            code = self.service.poll()
            if code is not None and not waiting_for_power:
                self.activity = ("SDK 异常退出；确认机械臂已开机，按 R 重新连接" if code == -signal.SIGABRT
                                 else "服务已退出；处理上方提示后按 R 重新连接")
            elif code is None and self.ready.is_set() and self.activity == "正在启动服务…":
                self.activity = "服务已就绪"
            state = ("等待机械臂通电 / 连接" if waiting_for_power else
                     f"服务已退出 ({code})" if code is not None else
                     "服务运行中" if self.ready.is_set() else "服务启动中")
            self.publish_state("waiting" if waiting_for_power else "exited" if code is not None
                               else "ready" if self.ready.is_set() else "starting")
            count = max(1, console.size.height - 10)
            offset = min(offset, max(0, len(self.lines) - count))
            end = len(self.lines) - offset
            color = "yellow" if waiting_for_power or not self.ready.is_set() else "green"
            if code is not None and not waiting_for_power:
                color = "red"
            layout = Layout()
            layout.split_column(Layout(name="keys", size=4), Layout(name="header", size=4), Layout(name="logs"))
            layout["header"].update(Panel(Group(Text(state, style="bold " + color), Text(self.activity)),
                                          title="X5 机械臂控制台", border_style=color))
            lines = [Text(fit(line, max(1, console.size.width - 4)), no_wrap=True)
                     for line in list(self.lines)[max(0, end-count):end]]
            layout["logs"].update(Panel(Group(*lines), title="运行日志", border_style="dim",
                                        subtitle="↑↓ 翻阅 · End 最新"))
            layout["keys"].update(Panel(Group(
                Text("R 重连  H 回零  P 预抓取  S/空格/Ctrl+C 停止  I 状态", style="cyan"),
                Text("D 阻尼  B/N 广播  W/E 网页  V 网页状态  C/X 相机  Q 退出  ? 帮助", style="cyan")),
                title="快捷键 · 直接按键", border_style="cyan"))
            live.update(layout, refresh=True)
            key = read_key()
            if key in ("q", "\x04", "eof"):
                return
            if key == "up":
                offset += 1
            elif key == "down":
                offset = max(0, offset - 1)
            elif key == "end":
                offset = 0
            elif key == "r":
                self.restart_service()
                offset = 0
            elif key == "?":
                self.write("H 回零；P 预抓取；D 阻尼（先支撑机械臂）；S/空格/Ctrl+C 停止并保持；I 状态；Q 退出。")
                self.write("R 重启已退出的服务；B/N 开关广播；W/E 开关网页；C/X 开关腕部相机。")
            elif key in ("\x03", " "):
                self.command("stop")
            elif key in KEYS:
                self.command(KEYS[key])

    def close(self):
        # Recovery finishes its finally/stop-and-hold while the service is alive.
        if self.cancel():
            self.action.wait()
        if self.service is not None:
            if self.service.poll() is None:
                self.service.terminate()
            self.service.wait()
        for reader in self.readers:
            reader.join()


def run_console(args, hardware, service_args):
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise ValueError("交互控制台需要终端；后台/重定向运行请添加 --no-interactive")
    folder = resolve_path(load_config()["paths"]["log_dir"])
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"arm-console-{time.time_ns()}.log"
    def terminate(*_):
        raise SystemExit(143)
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        with path.open("w", encoding="utf-8") as log:
            console = Console(args, hardware, log)
            try:
                console.service_argv = [sys.executable, "-u", "-B", "-m", "robot_control.service",
                                        *service_args, "--no-interactive"]
                console.service = console.launch(console.service_argv, service=True)
                if os.environ.get("QUEST_WORKSPACE_ROLE") == "arm" and os.environ.get("TMUX_PANE"):
                    subprocess.run(["tmux", "set-option", "-p", "-t", os.environ["TMUX_PANE"],
                                    "@questarm_phase", "arm-console"], check=True)
                    subprocess.run(["tmux", "set-option", "-p", "-t", os.environ["TMUX_PANE"],
                                    "@questarm_restart_key", "1"], check=True)
                console.render()
            finally:
                print("正在等待动作停稳并关闭机械臂服务…", flush=True)
                # Repeated terminal interrupts must not cut short stop-and-hold.
                old_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                try:
                    console.close()
                finally:
                    signal.signal(signal.SIGINT, old_int)
    finally:
        signal.signal(signal.SIGTERM, previous)
        print(f"机械臂控制台已退出，日志：{path}", flush=True)
