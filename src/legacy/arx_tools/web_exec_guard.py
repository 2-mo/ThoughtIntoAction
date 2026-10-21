#!/usr/bin/env python3
"""Exec one service and make it die if its direct supervisor disappears."""

from __future__ import annotations

import ctypes
import os
import signal
import sys


PR_SET_PDEATHSIG = 1


def main(argv: list[str]) -> int:
    command = argv[1:]
    if command[:1] == ["--"]:
        command = command[1:]
    if not command:
        raise SystemExit("missing service command")
    parent_pid = os.getppid()
    if parent_pid == 1:
        raise SystemExit("service supervisor already exited")
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_PDEATHSIG, signal.SIGKILL, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    if os.getppid() != parent_pid:
        os.kill(os.getpid(), signal.SIGKILL)
    os.execvp(command[0], command)
    return 127


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
