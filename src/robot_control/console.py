"""Curses supervisor; device sampling stays in the service process."""

from collections import deque
import curses
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import unicodedata

from robot_core.project import load_config, resolve_path

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
                    self.write(line)
                    if service and line.startswith("Arm service ready:"):
                        self.ready.set()
            finally:
                child.stdout.close()
        reader = threading.Thread(target=read, daemon=True)
        self.readers.append(reader)
        reader.start()
        return child

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

    def render(self, screen):
        curses.raw()  # Ctrl+C is a key; it never reaches either child process.
        screen.keypad(True)
        screen.timeout(100)
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        self.write("H 回零保持夹爪；P 预抓取并张开夹爪。动作前停止本项目策略；其他会话占用时拒绝。")
        self.write("S / 空格 / Ctrl+C：取消当前动作，空闲时停止策略并保持。此操作不是硬件急停。")
        self.write("D 进入阻尼，不锁定当前位置，请先支撑机械臂；S 恢复当前位置保持。")
        self.write("B 开启头显发现广播；N 关闭广播（默认关闭，不重连 USB，最多约 5 秒生效）。")
        self.write("W 启动网页；E 停止网页；V 网页状态；C 开相机；X 关相机（录制时拒绝）。")
        self.write("Q 关闭服务；↑↓ 翻阅输出，End 返回最新输出。")
        offset = 0
        while True:
            if self.action is not None and self.action.poll() is not None:
                code = self.action.returncode
                self.activity = "操作完成" if code == 0 else f"操作中断或失败（{code}），按 I 查看状态"
                self.write(self.activity)
                self.action = None
            while not self.events.empty():
                self.lines.append(self.events.get())
            code = self.service.poll()
            state = (f"服务已退出 ({code})" if code is not None else
                     "服务运行中" if self.ready.is_set() else "服务启动中")
            screen.erase()
            height, width = screen.getmaxyx()
            def row(y, text, attr=0):
                if 0 <= y < height:
                    try:
                        screen.addstr(y, 0, fit(text, max(0, width - 1)), attr)
                    except curses.error:
                        pass
            row(0, " X5 机械臂控制台  |  " + state, curses.A_REVERSE | curses.A_BOLD)
            row(1, " 当前操作：" + self.activity)
            row(2, " W 开网页 E 关网页 V 状态 | C 开相机 X 关相机", curses.A_DIM)
            count = max(0, height - 7)
            offset = min(offset, max(0, len(self.lines) - count))
            end = len(self.lines) - offset
            for y, line in enumerate(list(self.lines)[max(0, end - count):end], 4):
                row(y, line)
            row(height - 2, " H 回零  P 预抓取  S/空格 停止  I 状态", curses.A_REVERSE)
            row(height - 1, " D 阻尼  B 开广播 N 关广播  Q 退出  ↑↓ 日志  End 最新  ? 帮助", curses.A_REVERSE)
            screen.refresh()
            key = screen.getch()
            if key in (ord("q"), ord("Q"), 4):
                return
            if key == curses.KEY_UP:
                offset += 1
            elif key == curses.KEY_DOWN:
                offset = max(0, offset - 1)
            elif key == curses.KEY_END:
                offset = 0
            elif key in (ord("?"), curses.KEY_F1):
                self.write("H 回零；P 预抓取（张开夹爪）；D 阻尼（先支撑机械臂）；S/空格/Ctrl+C 停止并保持；I 状态；Q 关闭服务。无需回车。")
                self.write("B 开广播；N 关广播；只改变 XR 服务发现广播，不断开 USB 追踪。")
            elif key in (3, ord(" ")):
                self.command("stop")
            elif 0 <= key < 256 and chr(key).lower() in KEYS:
                self.command(KEYS[chr(key).lower()])

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
                console.service = console.launch(
                    [sys.executable, "-u", "-B", "-m", "robot_control.service",
                     *service_args, "--no-interactive"], service=True)
                curses.wrapper(console.render)
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
