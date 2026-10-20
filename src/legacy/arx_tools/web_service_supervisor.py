#!/usr/bin/env python3
"""Supervise one detached web service and reap it when the launcher dies."""

from __future__ import annotations

import argparse
import ctypes
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


PR_SET_PDEATHSIG = 1
EXEC_GUARD = Path(__file__).with_name("web_exec_guard.py")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--name", required=True)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command:
        parser.error("missing service command")
    if not 0.5 <= args.timeout <= 60.0:
        parser.error("--timeout must be in 0.5..60 seconds")
    return args


def set_parent_death_signal() -> int:
    parent_pid = os.getppid()
    if parent_pid == 1:
        raise RuntimeError("web launcher already exited")
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_PDEATHSIG, signal.SIGUSR1, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    return parent_pid


def signal_group(process: subprocess.Popen[bytes], signal_number: int) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal_number)
    except ProcessLookupError:
        pass


def stop_process(
    process: subprocess.Popen[bytes],
    *,
    name: str,
    timeout_s: float,
    reason: str,
) -> int:
    signal_group(process, signal.SIGTERM)
    deadline = time.monotonic() + timeout_s
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if process.poll() is None:
        print(
            f"{name} 服务未在 {timeout_s:.0f} 秒内退出；正在结束服务进程组。",
            file=sys.stderr,
            flush=True,
        )
        signal_group(process, signal.SIGKILL)
    try:
        return process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        print(
            f"{name} 服务进程组仍未退出（{reason}）。",
            file=sys.stderr,
            flush=True,
        )
        return 1


def main() -> int:
    args = parse_args()
    parent_pid = set_parent_death_signal()
    parent_died = False
    stop_signal: int | None = None

    def request_stop(signal_number: int, _frame: object) -> None:
        nonlocal parent_died, stop_signal
        if signal_number == signal.SIGUSR1:
            parent_died = True
        else:
            stop_signal = signal_number

    signal.signal(signal.SIGUSR1, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    if os.getppid() != parent_pid:
        parent_died = True

    process = subprocess.Popen(
        [sys.executable, "-B", str(EXEC_GUARD), "--", *args.command],
        start_new_session=True,
    )
    while process.poll() is None and not parent_died and stop_signal is None:
        time.sleep(0.05)
    if parent_died:
        return stop_process(
            process,
            name=args.name,
            timeout_s=args.timeout,
            reason="launcher exited",
        )
    if stop_signal is not None:
        return stop_process(
            process,
            name=args.name,
            timeout_s=args.timeout,
            reason=f"signal {stop_signal}",
        )
    return int(process.returncode or 0)


if __name__ == "__main__":
    raise SystemExit(main())
