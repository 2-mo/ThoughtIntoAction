#!/usr/bin/env python3
"""Check or fetch pinned installation sources; never read or modify reference checkouts."""
import argparse
from pathlib import Path
import subprocess
import sys
import tomllib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from embodied.core.dependencies import reject_reference_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch", action="store_true")
    parser.add_argument("--all", action="store_true", help="Include optional native SDK and model dependencies")
    parser.add_argument("--name", action="append", help="Select a managed dependency by name (repeatable)")
    args = parser.parse_args()
    with (ROOT / "dependencies.lock.toml").open("rb") as stream:
        entries = tomllib.load(stream)["managed"]
    known = {entry["name"] for entry in entries}
    if unknown := set(args.name or ()) - known:
        parser.error("Unknown dependency: " + ", ".join(sorted(unknown)))
    errors = 0
    for entry in entries:
        if args.name and entry["name"] not in args.name:
            continue
        if not args.name and not entry["required"] and not args.all:
            continue
        path = reject_reference_path(ROOT / entry["path"])
        if not path.exists() and args.fetch:
            path.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(["git", "clone", "--no-checkout", entry["url"], str(path)], check=True)
            subprocess.run(["git", "-C", str(path), "checkout", "--detach", entry["commit"]], check=True)
        if not (path / ".git").exists():
            print(f"MISSING {entry['path']} — ./run.sh fetch-dependencies --name {entry['name']}")
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
