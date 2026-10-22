"""Read-only IPC to Unix-socket bridge; each reader has a bounded queue."""

import argparse
import json
import os
from pathlib import Path
import queue
import signal
import socket
import struct
import threading
import time

from embodied.control.ticks import CAPACITY, TickSubscriber, layout, to_dict

MAX_MESSAGE = 4_000_000


def encode(value):
    raw = json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
    if len(raw) > MAX_MESSAGE:
        raise ValueError("Observer message too large")
    return struct.pack("!I", len(raw)) + raw


def receive(sock):
    def exact(size):
        chunks = bytearray()
        while len(chunks) < size:
            data = sock.recv(size - len(chunks))
            if not data:
                raise EOFError("Observer peer closed")
            chunks.extend(data)
        return chunks
    size, = struct.unpack("!I", exact(4))
    if not 0 < size <= MAX_MESSAGE:
        raise ValueError("Invalid observer frame size")
    return json.loads(exact(size))


class Peer:
    def __init__(self, sock, descriptor, latest):
        self.sock, self.latest = sock, latest
        self.queue = queue.Queue(maxsize=1 if latest else CAPACITY)
        self.closed = threading.Event()
        self.descriptor = descriptor
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def offer(self, packet):
        if self.closed.is_set():
            return
        try:
            self.queue.put_nowait(packet)
        except queue.Full:
            if self.latest:
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.queue.put_nowait(packet)
                except queue.Full:
                    pass
            else:
                # A full-stream reader must see EOF, never a silently repaired gap.
                self.close()

    def _write(self):
        try:
            self.sock.sendall(encode(self.descriptor))
            while not self.closed.is_set():
                try:
                    packet = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                if packet is None:
                    return
                self.sock.sendall(packet)
        except (OSError, ValueError):
            pass
        finally:
            self.close()

    def close(self):
        self.closed.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


def run(directory):
    directory = Path(directory)
    identity = json.loads((directory / "runtime.json").read_text())
    subscriber = TickSubscriber(identity["runtime_id"])
    path = directory / "observer.sock"
    stopping = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stopping.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    peers = []
    last_sequence = None
    latest_packet = None
    gaps = 0
    terminal = False
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        # Directory fd avoids AF_UNIX's 108-byte absolute-path limit after relocation.
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            server.bind(f'/proc/self/fd/{descriptor}/{path.name}')
        finally:
            os.close(descriptor)
        path.chmod(0o600)
        server.listen(8)
        server.setblocking(False)
        (directory / "observer.ready").write_text("ready\n")
        while not terminal:
            # Limit work per iteration, but drain in order; IPC overflow is detectable.
            for _ in range(1024):
                packet = subscriber.receive()
                if packet is None:
                    break
                if last_sequence is not None and packet.sequence != last_sequence + 1:
                    gaps += 1
                last_sequence = packet.sequence
                value = to_dict(packet)
                value["source"] = value["source"] or ("quest" if identity["input"] == "vr" else identity["input"])
                value["observer_gaps"] = gaps
                value["observer_received_ns"] = time.monotonic_ns()
                latest_packet = encode(value)
                for peer in peers:
                    peer.offer(latest_packet)
                if value["terminal"]:
                    terminal = True
                    break
            peers = [peer for peer in peers if not peer.closed.is_set()]
            if stopping.is_set() or terminal:
                break
            try:
                sock, _ = server.accept()
            except BlockingIOError:
                sock = None
            if sock is not None:
                # Handshake is off the control path, bounded to avoid stalled clients.
                sock.settimeout(.2)
                try:
                    request = receive(sock)
                    if not isinstance(request, dict) or request.get("version") != 1 or request.get("mode") not in ("latest", "full"):
                        raise ValueError("Unsupported observer subscription")
                    if len(peers) >= 8:
                        raise ValueError("Observer reader limit reached")
                    parameters = json.loads((directory / "parameters.json").read_text())
                    backend_file = directory / 'backend-parameters.json'
                    descriptor = {"type": "descriptor", "protocol_version": 1, **identity,
                                  "directory": str(directory),
                                  "layout": layout(), "parameters": parameters, "observer_gaps": gaps,
                                  "joint_count": len(parameters["effective_robot_config"]["park_q"]),
                                  "backend_parameters": json.loads(backend_file.read_text()) if backend_file.exists() else None,
                                  "mode": request["mode"]}
                    peer = Peer(sock, descriptor, request["mode"] == "latest")
                    peers.append(peer)
                    if latest_packet is not None:
                        peer.offer(latest_packet)
                except (OSError, ValueError, EOFError):
                    sock.close()
            stopping.wait(.001)
        deadline = time.monotonic() + 1.5
        for peer in peers:
            # Enqueued after the terminal message; no extra tick is synthesized.
            try:
                peer.queue.put_nowait(None)
            except queue.Full:
                if peer.latest:
                    continue
                peer.close()
        for peer in peers:
            peer.thread.join(max(0, deadline - time.monotonic()))
    finally:
        for peer in peers:
            peer.close()
        server.close()
        if path.exists():
            path.unlink()
        subscriber.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        (directory / "observer-report.json").write_text(json.dumps({
            "last_sequence": last_sequence, "gaps": gaps, "terminal_received": terminal}, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    run(parser.parse_args().directory)
