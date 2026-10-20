"""Local session JSON client; does not import or initialize devices."""

import json
import socket

MAX_REQUEST = 16384
MAX_RESPONSE = 8_000_000


def request(path, command, *, timeout=15, **arguments):
    payload = json.dumps({"command": command, **arguments}, allow_nan=False).encode() + b"\n"
    if len(payload) > MAX_REQUEST:
        raise ValueError("Request too large")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(str(path))
        sock.sendall(payload)
        with sock.makefile("rb") as stream:
            data = stream.readline(MAX_RESPONSE + 1)
    if len(data) > MAX_RESPONSE or not data.endswith(b"\n"):
        raise RuntimeError("Invalid session response")
    result = json.loads(data)
    if not result["ok"]:
        raise RuntimeError(result["error"])
    return result["result"]
