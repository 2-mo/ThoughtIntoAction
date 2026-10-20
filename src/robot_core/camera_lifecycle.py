"""Gracefully stop this project's camera owner, including older service versions."""
from contextlib import closing
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys

from robot_core.camera_client import socket_path
from robot_core.camera_control import check_legacy_session, recording_guard
from embodied_contracts.device_lease import DeviceLease


def stop_service():
    # Some uv Python builds omit pidfd APIs; Ubuntu's bootstrap Python provides them.
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        python = '/usr/bin/python3'
        if os.path.realpath(sys.executable) == os.path.realpath(python):
            raise RuntimeError('系统 Python 不支持安全的进程退出等待。')
        env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1]))
        try:
            result = subprocess.run([python, '-B', '-m', 'robot_core.camera_lifecycle'],
                                    env=env, text=True, capture_output=True, timeout=20)
        except subprocess.TimeoutExpired:
            raise RuntimeError('等待相机服务退出超时，未强制终止相机。') from None
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or '相机服务关闭失败')
        return
    path = socket_path()
    # Match ensure_service's launch lock, preventing a simultaneous cooperating launch.
    with closing(DeviceLease('camera-launch:' + str(path))), closing(recording_guard()):
        check_legacy_session()
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            try:
                client.settimeout(1)
                client.connect(str(path))
            except (FileNotFoundError, ConnectionRefusedError):
                return
            pid, uid, _ = struct.unpack('3i', client.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
            if uid != os.getuid():
                raise RuntimeError('相机服务不属于当前用户，未停止。')
            # pidfd ties the signal and completion wait to the actual process, not a reused PID.
            try:
                fd = os.pidfd_open(pid)
            except (ProcessLookupError, FileNotFoundError):
                return
            try:
                argv = (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0')
                if b'robot_core.camera_service' not in argv:
                    raise RuntimeError('相机 socket 的进程身份不匹配，未停止。')
                signal.pidfd_send_signal(fd, signal.SIGTERM)
                if not select.select([fd], [], [], 15)[0]:
                    raise RuntimeError('相机服务尚未退出；已保留现场，未强制终止。')
            except (ProcessLookupError, FileNotFoundError):
                return
            finally:
                os.close(fd)


if __name__ == '__main__':
    try:
        stop_service()
    except (OSError, ValueError, RuntimeError) as exc:
        raise SystemExit(str(exc))
