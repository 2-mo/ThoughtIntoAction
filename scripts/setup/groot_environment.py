#!/usr/bin/env python3
"""Install the pinned, patched GR00T package into its explicit policy environment."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from embodied.core.dependencies import dependency_python, dependency_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tensorrt", action="store_true")
    args = parser.parse_args()
    source = dependency_root("groot")
    if not (source / "uv.lock").is_file():
        raise SystemExit("Prepare the pinned source first: python3 scripts/setup/groot_dependency.py")
    environment = os.environ.copy()
    environment["UV_PROJECT_ENVIRONMENT"] = str(dependency_python("groot").parent.parent)
    command = ["uv", "sync", "--project", str(source), "--locked", "--no-editable", "--python", "3.12"]
    if args.tensorrt:
        command += ["--extra", "tensorrt"]
    subprocess.run(command, cwd=ROOT, env=environment, check=True)


if __name__ == "__main__":
    main()
