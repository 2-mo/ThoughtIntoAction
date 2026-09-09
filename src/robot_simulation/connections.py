"""Read-only RTT probes. No control token, SDK initialization or headset wake."""
import json
import os
import select
import socket
import subprocess
import threading
import time


class UsbEcho:
    def __init__(self, serial):
        self.process = subprocess.Popen(
            ["adb", "-s", serial, "shell", "-T", "sh", "-c",
             "'while IFS= read -r line; do printf \"%s\\n\" \"$line\"; done'"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        try:
            self.measure(timeout=5)  # Exclude adb/shell startup from measured RTT.
        except BaseException:
            self.close()
            raise

    def measure(self, timeout=.8):
        token = str(time.monotonic_ns()).encode()
        started = time.monotonic_ns()
        self.process.stdin.write(token + b"\n")
        deadline = time.monotonic() + timeout
        data = b""
        while time.monotonic() < deadline:
            if not select.select([self.process.stdout], [], [], max(0., deadline - time.monotonic()))[0]:
                break
            chunk = os.read(self.process.stdout.fileno(), 4096)
            if not chunk:
                raise RuntimeError("USB echo disconnected")
            data += chunk
            if token in [line.strip() for line in data.split(b"\n")[:-1]]:
                return (time.monotonic_ns() - started) / 1e6
            if len(data) > 8192:
                raise RuntimeError("Unexpected USB echo response")
        raise TimeoutError("USB echo timed out")

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)
        self.process.stdin.close()
        self.process.stdout.close()


def arm_rtt(path):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as sock:
        sock.settimeout(.3)
        started = time.monotonic_ns()
        sock.connect(str(path))
        sock.sendall(json.dumps({"op": "describe", "sent_monotonic_ns": started}).encode())
        raw, _, flags, _ = sock.recvmsg(1_000_000)
        finished = time.monotonic_ns()
        if flags & socket.MSG_TRUNC or not json.loads(raw).get("ok"):
            raise RuntimeError("Arm service probe failed")
    return (finished - started) / 1e6


class ConnectionProbe:
    def __init__(self, arm_socket):
        self.arm_socket = arm_socket
        self.values = {}
        self.stop = threading.Event()
        self.threads = [threading.Thread(target=target, daemon=True)
                        for target in (self._arm, self._quest)]
        for thread in self.threads:
            thread.start()

    def store(self, key, duration=None):
        self.values[key] = {"duration_ms": duration, "sample_monotonic_ns": time.monotonic_ns()}

    def snapshot(self):
        now = time.monotonic_ns()
        return {key: {"duration_ms": value["duration_ms"]}
                for key, value in self.values.copy().items()
                if now - value["sample_monotonic_ns"] < 3_000_000_000}

    def _arm(self):
        while not self.stop.is_set():
            try:
                self.store("arm", arm_rtt(self.arm_socket))
            except (OSError, RuntimeError, ValueError):
                self.store("arm")
            self.stop.wait(1)

    def _quest(self):
        echo = None
        try:
            while not self.stop.is_set():
                try:
                    if echo is None:
                        result = subprocess.run(["adb", "devices", "-l"], capture_output=True,
                                                text=True, timeout=2, check=True)
                        devices = [line.split()[0] for line in result.stdout.splitlines()
                                   if len(line.split()) > 1 and line.split()[1] == "device" and "usb:" in line]
                        if len(devices) != 1:
                            raise RuntimeError("No unique authorized USB headset")
                        echo = UsbEcho(devices[0])
                    self.store("quest_usb", echo.measure())
                except (OSError, RuntimeError, subprocess.SubprocessError):
                    self.store("quest_usb")
                    if echo is not None:
                        echo.close()
                        echo = None
                self.stop.wait(1)
        finally:
            if echo is not None:
                echo.close()

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=9)
