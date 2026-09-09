#!/usr/bin/env python3
"""Check pinned source repositories, or fetch missing repositories without replacing local work."""

import argparse
from pathlib import Path
import subprocess
import sys
import tomllib

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch", action="store_true")
    parser.add_argument("--all", action="store_true", help="Include optional SDK, policy and reference repositories")
    args = parser.parse_args()
    with (ROOT / "dependencies.lock.toml").open("rb") as stream:
        entries = tomllib.load(stream)["repositories"]
    errors = 0
    for entry in entries:
        if not entry["required"] and not args.all:
            continue
        path = ROOT / entry["path"]
        if not path.exists() and args.fetch:
            path.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(["git", "clone", "--no-checkout", entry["url"], str(path)], check=True)
            subprocess.run(["git", "-C", str(path), "checkout", "--detach", entry["commit"]], check=True)
        if not (path / ".git").exists():
            print(f"MISSING {entry['path']}")
            errors += 1
            continue
        revision = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
        status = subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True).strip()
        matches = revision == entry["commit"]
        print(f"{'OK' if matches else 'REVISION MISMATCH'} {entry['path']} {revision[:12]}" + (" (local changes preserved)" if status else ""))
        if not matches:
            errors += 1
    return int(errors > 0)


if __name__ == "__main__":
    sys.exit(main())
