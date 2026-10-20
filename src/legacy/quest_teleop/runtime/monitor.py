"""Local read-only monitor. Closing a browser never changes teleop or recording."""

import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path

from embodied.core.session_protocol import request


def make_server(socket_path, port):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            try:
                if self.path == "/":
                    data = Path(__file__).with_name("monitor.html").read_bytes()
                    content_type = "text/html; charset=utf-8"
                elif self.path == "/status":
                    data = json.dumps(request(socket_path, "status"), allow_nan=False).encode()
                    content_type = "application/json"
                elif self.path.startswith("/camera.jpg"):
                    preview = request(socket_path, "preview")
                    if preview["age_s"] is None or preview["age_s"] > .5 or not preview["jpeg_base64"]:
                        raise RuntimeError("Camera unavailable/stale")
                    data = base64.b64decode(preview["jpeg_base64"])
                    content_type = "image/jpeg"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except (OSError, ValueError, RuntimeError) as exc:
                self.send_error(503, str(exc))

        def log_message(self, *args):
            pass
    # No command endpoint, cross-origin headers or network-wide binding.
    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main(argv=None):
    from embodied.core.project import load_config, resolve_path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args(argv)
    with make_server(resolve_path(load_config()["runtime"]["socket"]), args.port) as server:
        print(f"Read-only monitor: http://127.0.0.1:{server.server_port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
