#!/usr/bin/env python3
"""Rebuild the pinned Quest binding after a source change or checkout relocation."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    argparse.ArgumentParser(description=__doc__).parse_args()
    if Path(sys.prefix) != ROOT / ".venv":
        raise SystemExit("XR rebuild targets this project's .venv; use the default uv environment.")
    uv = shutil.which("uv")
    if not uv:
        raise SystemExit("uv is required; see docs/setup.md")
    subprocess.run([sys.executable, "-B", str(ROOT / "scripts/setup/dependencies.py")], check=True)
    build = ROOT / "third_party/XRoboToolkit-Teleop-Sample-Python/dependencies/XRoboToolkit-PC-Service-Pybind/build"
    if build.exists():
        saved = ROOT / "outputs/build-cache" / f"xr-sdk-{time.time_ns()}"
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(build), str(saved))
    env = os.environ.copy()
    env.pop("UV_PROJECT_ENVIRONMENT", None)
    subprocess.run([uv, "sync", "--locked", "--managed-python", "--no-cache",
                    "--reinstall-package", "xrobotoolkit-sdk"], cwd=ROOT, env=env, check=True)


if __name__ == "__main__":
    main()
