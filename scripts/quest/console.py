#!/usr/bin/env python3
"""Interactive lifecycle owner for a teleop child; no device SDK imports."""
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading

from rich.console import Console as RichConsole

rich_console = RichConsole()
print = rich_console.print

ROOT = Path(__file__).resolve().parents[2]


class Console:
    def __init__(self, runner, arguments, log_dir):
        self.runner, self.arguments = str(runner), arguments
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"teleop-console-{time.time_ns()}.log"
        self.log = self.log_path.open("ab", buffering=0)
        self.child = None

    def stop(self):
        if self.child is None or self.child.poll() is not None:
            return True
        self.child.send_signal(signal.SIGINT)
        try:
            self.child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("遥操尚未退出，未切换头显电源。请查看日志并处理当前进程。")
            return False
        return True

    def power(self, command):
        try:
            result = subprocess.run([self.runner, "quest-wake", command],
                                    stdout=self.log, stderr=self.log, timeout=20)
            if result.returncode == 0:
                return True
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"头显操作失败：{exc}")
        print(f"头显操作未成功，请查看 {self.log_path}")
        return False

    def transport(self, action):
        try:
            return subprocess.run([self.runner, "quest-transport", action], timeout=35).returncode == 0
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"USB 检查失败：{exc}")
            return False

    def wake(self):
        if self.child is not None and self.child.poll() is None:
            if self.session("pause") and self.transport("prepare") and self.power("start") and self.transport("wait") and self.session("resume"):
                print("已模拟佩戴，等待新帧后自动恢复；松开 Grip/A/B/右摇杆按键后按正常方式遥操。")
            return
        if self.transport("prepare") and self.power("start") and self.transport("wait"):
            self.child = subprocess.Popen([self.runner, *self.arguments], stdin=subprocess.DEVNULL,
                                          stdout=self.log, stderr=self.log, start_new_session=True)
            print("已请求模拟佩戴，遥操正在启动；按状态查看进程是否仍运行。")

    def session(self, command):
        try:
            result = subprocess.run([self.runner, "session", command],
                                    stdout=self.log, stderr=self.log, timeout=20)
            if result.returncode == 0:
                return True
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"遥操会话操作失败：{exc}")
        print(f"未完成 {command}；请检查采集会话和日志：{self.log_path}")
        return False

    def unwear(self):
        if self.child is not None and self.child.poll() is None:
            if not self.session("pause"):
                return
        if self.power("unwear"):
            print("已取消模拟佩戴，由 Quest 自行息屏；程序保留，机械臂控制已暂停。")

    def sleep(self):
        if self.stop() and self.power("stop"):
            print("遥操已停止，已发送取消模拟佩戴和休眠请求。")

    def status(self):
        self.transport("status")
        code = None if self.child is None else self.child.poll()
        print("遥操进程运行中（不代表追踪正常）" if self.child is not None and code is None
              else f"遥操未运行，退出码：{code}")
        print(f"日志：{self.log_path}")

    def watch_child(self, stop):
        reported = None
        while not stop.wait(.2):
            child = self.child
            if child is None or child is reported:
                continue
            code = child.poll()
            if code is None:
                continue
            reported = child
            print(f"\n遥操进程已退出（退出码 {code}）；当前没有手柄数据。")
            if code:
                try:
                    with self.log_path.open("rb") as log:
                        log.seek(0, 2)
                        log.seek(max(0, log.tell() - 8192))
                        lines = log.read().decode("utf-8", errors="replace").splitlines()
                    errors = [line for line in lines if "Error:" in line or "Exception:" in line]
                    if errors:
                        rich_console.print(errors[-1][:600], markup=False)
                except OSError:
                    pass
            print(f"日志：{self.log_path}；处理后按 w 重新启动。")

    def calibrate(self, reuse=False):
        if self.child is None or self.child.poll() is not None:
            print("请先输入 w 启动遥操，再在待机状态输入 c 标定。")
            return
        subprocess.run([self.runner, "quest-calibrate", *(["--reuse"] if reuse else [])], check=False)

    def run(self):
        print("w / 1 模拟佩戴并启动遥操 | s / 2 停止遥操并休眠 | u / 3 取消模拟佩戴（保留程序） | c 快速方向对齐 | cr 复用上次方向（Quest 未重定位） | p 状态 | q 退出")
        print(f"启动日志与异常：{self.log_path}")
        watcher_stop = threading.Event()
        watcher = threading.Thread(target=self.watch_child, args=(watcher_stop,), daemon=True)
        watcher.start()
        try:
            self.wake()
            while True:
                try:
                    command = input("遥操> ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    command = "q"
                    print()
                if command in ("w", "1"):
                    self.wake()
                elif command in ("s", "2"):
                    self.sleep()
                elif command in ("u", "3"):
                    self.unwear()
                elif command == "p":
                    self.status()
                elif command in ("c", "cr"):
                    self.calibrate(reuse=command == "cr")
                elif command == "q":
                    if self.stop():
                        break
                else:
                    print("输入 w / u / s / c / cr / p / q 后回车。")
        finally:
            watcher_stop.set()
            watcher.join(timeout=1)
            self.stop()
            self.log.close()


if __name__ == "__main__":
    if any(arg in ("-h", "--help") for arg in sys.argv[1:]):
        print("Usage: quest-console TELEOP_ARGUMENTS; daily entry: ./启动遥操.sh")
        raise SystemExit(0)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    Console(ROOT / "run.sh", sys.argv[1:], ROOT / "outputs/logs").run()
