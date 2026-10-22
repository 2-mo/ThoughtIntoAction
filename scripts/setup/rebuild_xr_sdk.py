#!/usr/bin/env python3
"""Install the pinned XR binding and its upstream wheel's missing companion library."""
import argparse
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from embodied.core.dependencies import dependency_root


def main() -> None:
    argparse.ArgumentParser(description=__doc__).parse_args()
    if Path(sys.prefix) != ROOT / ".venv":
        raise SystemExit("XR rebuild targets this project's .venv; use the default uv environment.")
    uv = shutil.which("uv")
    if not uv:
        raise SystemExit("uv is required; see docs/guides/setup.md")
    subprocess.run([sys.executable, "-B", str(ROOT / "scripts/setup/dependencies.py"),
                    "--fetch", "--name", "xr-sdk"], check=True)
    source = dependency_root("xr-sdk", root=ROOT) / "lib"
    if platform.machine() == "aarch64":
        source /= "aarch64"
    native = source / "libPXREARobotSDK.so"
    if not native.is_file():
        raise SystemExit(f"Pinned XR companion library missing: {native}")
    target = ROOT / ".venv/lib"
    target.mkdir(parents=True, exist_ok=True)
    shutil.copy2(native, target / native.name)
    env = os.environ.copy()
    env.pop("UV_PROJECT_ENVIRONMENT", None)
    env.pop("PYTHONHOME", None)
    env["LD_LIBRARY_PATH"] = str(target)
    subprocess.run([uv, "sync", "--locked", "--managed-python", "--no-cache",
                    "--reinstall-package", "xrobotoolkit-sdk"], cwd=ROOT, env=env, check=True)
    # Importing the extension checks its ABI and native linkage; it does not call init().
    subprocess.run([str(ROOT / ".venv/bin/python"), "-c", "import xrobotoolkit_sdk"],
                   cwd=ROOT, env=env, check=True)
    print(f"XR binding and companion library installed in {ROOT / '.venv'}")


if __name__ == "__main__":
    main()
