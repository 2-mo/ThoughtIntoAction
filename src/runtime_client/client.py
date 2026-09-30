"""Versioned framed JSON, with explicit stream identity and loss detection."""

import json
from pathlib import Path
import socket
import struct
from component_api.rpc import connect

MAX_MESSAGE = 4_000_000


def read_message(sock):
    def exact(size):
        data = bytearray()
        while len(data) < size:
            part = sock.recv(size - len(data))
            if not part:
                raise EOFError("Runtime stream ended without another complete message")
            data.extend(part)
        return data
    length, = struct.unpack("!I", exact(4))
    if not 0 < length <= MAX_MESSAGE:
        raise ValueError("Invalid runtime message length")
    return json.loads(exact(length))


class RuntimeReader:
    def __init__(self, directory, mode="latest", timeout=3.):
        if mode not in ("latest", "full"):
            raise ValueError("Mode must be latest or full")
        self.directory, self.mode = Path(directory), mode
        self.sequence = None
        self.terminal = False
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        try:
            connect(self.sock, self.directory / "observer.sock")
            data = json.dumps({"version": 1, "mode": mode}).encode()
            self.sock.sendall(struct.pack("!I", len(data)) + data)
            self.descriptor = read_message(self.sock)
            if (not isinstance(self.descriptor, dict) or self.descriptor.get("type") != "descriptor"
                    or self.descriptor.get("protocol_version") != 1 or self.descriptor.get("mode") != mode):
                raise ValueError("Unsupported runtime descriptor")
            expected = json.loads((self.directory / "runtime.json").read_text())
            self.runtime_id = expected["runtime_id"]
            if self.descriptor.get("runtime_id") != self.runtime_id:
                raise ValueError("Runtime identity changed")
            if (self.descriptor.get("boot_id") != expected.get("boot_id")
                    or expected.get("boot_id") != Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
                raise ValueError("Runtime belongs to a different host boot")
            self.gaps = self.descriptor["observer_gaps"]
            self.dropped = None
        except BaseException:
            self.close()
            raise

    def read(self):
        if self.terminal:
            raise EOFError("Runtime terminal already received")
        message = read_message(self.sock)
        if (not isinstance(message, dict) or message.get("type") != "tick" or message.get("version") not in (1, 2)
                or message.get("runtime_id") != self.runtime_id):
            raise ValueError("Runtime protocol or identity changed")
        sequence = message["sequence"]
        if type(sequence) is not int or sequence < 0:
            raise ValueError("Invalid runtime sequence")
        for name in ('producer_dropped', 'observer_gaps'):
            if type(message.get(name)) is not int or message[name] < 0:
                raise ValueError(f"Invalid runtime {name}")
        if type(message.get('terminal')) is not bool:
            raise ValueError("Invalid runtime terminal flag")
        if self.sequence is not None and sequence <= self.sequence:
            raise ValueError("Repeated or regressed runtime sequence")
        if self.mode == "full":
            if self.sequence is not None and sequence != self.sequence + 1:
                raise ValueError("Robot stream sequence gap")
            if message["observer_gaps"] != self.gaps:
                raise ValueError("Observer reported a stream gap")
            if self.dropped is not None and message["producer_dropped"] != self.dropped:
                raise ValueError("Producer dropped telemetry")
        self.sequence = sequence
        self.dropped = message["producer_dropped"]
        self.terminal = bool(message["terminal"])
        return message

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
