"""Restricted local JSON protocol. No arbitrary Python, paths or motor commands."""

import json
import os
from pathlib import Path
import socket
import socketserver
import stat
import threading

from robot_core.device_lease import DeviceLease

from robot_core.session_protocol import MAX_REQUEST


class LocalServer:
    def __init__(self, path, dispatch):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lease = DeviceLease("session:" + str(self.path.resolve()))
        self.server = None
        self.thread = None
        try:
            if self.path.exists():
                if not stat.S_ISSOCK(self.path.lstat().st_mode):
                    raise ValueError("Session path exists and is not a socket")
                self.path.unlink()  # Lease proves there is no cooperating live owner.
            class Handler(socketserver.StreamRequestHandler):
                def handle(handler):
                    handler.connection.settimeout(2)
                    try:
                        raw = handler.rfile.readline(MAX_REQUEST + 1)
                        if len(raw) > MAX_REQUEST or not raw.endswith(b"\n"):
                            raise ValueError("Invalid request size/framing")
                        result = dispatch(json.loads(raw))
                        response = {"ok": True, "result": result}
                    except Exception as exc:
                        response = {"ok": False, "error": str(exc)}
                    try:
                        handler.wfile.write(json.dumps(response, allow_nan=False).encode() + b"\n")
                    except (BrokenPipeError, ConnectionResetError, socket.timeout):
                        pass
            class Server(socketserver.ThreadingUnixStreamServer):
                daemon_threads = True
            self.server = Server(str(self.path), Handler)
            os.chmod(self.path, 0o600)
            self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": .1},
                                           name="session-ipc", daemon=True)
            self.thread.start()
        except BaseException:
            if self.server is not None:
                self.server.server_close()
            self.lease.close()
            raise

    def close(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.thread.join(timeout=2)
            self.path.unlink(missing_ok=True)
            self.server = None
        self.lease.close()
