from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn

from embodied.console.app import create_app
from embodied.console.core.settings import BackendSettings


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARX5 FastAPI backend")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--origin", action="append", dest="origins")
    parser.add_argument("--token")
    parser.add_argument("--camera-url")
    parser.add_argument("--camera-ca", type=Path)
    parser.add_argument("--camera-origin")
    parser.add_argument("--frontend-directory", type=Path)
    parser.add_argument("--ca", type=Path, dest="ca_cert")
    parser.add_argument("--auto-connect-arm", action="store_true")
    args = parser.parse_args()
    if (args.cert is None) != (args.key is None):
        parser.error("--cert and --key must be provided together")
    return args


def main() -> int:
    args = parse_args()
    settings = BackendSettings(
        host=args.host,
        port=args.port,
        interface=args.interface,
        cert=args.cert,
        key=args.key,
        origins=tuple(args.origins or ()),
        token=args.token,
        camera_url=args.camera_url,
        camera_ca=args.camera_ca,
        camera_origin=args.camera_origin,
        frontend_directory=args.frontend_directory,
        ca_cert=args.ca_cert,
        auto_connect_arm=args.auto_connect_arm,
    )
    uvicorn.run(
        create_app(settings),
        host=settings.host,
        port=settings.port,
        ssl_certfile=str(settings.cert) if settings.cert else None,
        ssl_keyfile=str(settings.key) if settings.key else None,
        ws_max_size=16 * 1024,
        ws_ping_interval=5.0,
        ws_ping_timeout=5.0,
        log_level="info",
        access_log=False,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
