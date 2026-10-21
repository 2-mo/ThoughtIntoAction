"""Bounded length-prefixed JSON for a local command socket, separate from telemetry."""
import json
import os
from pathlib import Path
import socket
import struct

MAX_MESSAGE = 1_000_000


def receive(sock):
    def exact(size):
        data = bytearray()
        while len(data) < size:
            piece = sock.recv(size - len(data))
            if not piece:
                raise EOFError('Incomplete component message')
            data.extend(piece)
        return data
    size, = struct.unpack('!I', exact(4))
    if not 0 < size <= MAX_MESSAGE:
        raise ValueError('Invalid component message size')
    value = json.loads(exact(size))
    if not isinstance(value, dict):
        raise ValueError('Component messages must be objects')
    return value


def send(sock, value):
    data = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
    if len(data) > MAX_MESSAGE:
        raise ValueError('Component message too large')
    sock.sendall(struct.pack('!I', len(data)) + data)


def request(path, payload, timeout=3.):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        connect(sock, path)
        send(sock, payload)
        reply = receive(sock)
    if reply.get('ok') is not True:
        raise RuntimeError(reply.get('error', 'Component request rejected'))
    return reply['result']


def connect(sock, path):
    """Linux directory-fd address also works in deeply nested checkout paths."""
    path = Path(path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        sock.connect(f'/proc/self/fd/{descriptor}/{path.name}')
    finally:
        os.close(descriptor)
