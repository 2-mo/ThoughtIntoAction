#!/usr/bin/env python3
"""Restore the pinned GR00T dependency and its recorded optional-TensorRT patch."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from embodied.core.dependencies import dependency_root


def main():
    record = json.loads((ROOT / 'configs/dependencies/groot.json').read_text())
    path = dependency_root('groot')
    subprocess.run([sys.executable, str(ROOT / 'scripts/setup/dependencies.py'),
                    '--fetch', '--name', 'groot'], check=True)
    revision = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != record['source_commit']:
        raise SystemExit('GR00T revision differs; existing checkout was preserved')
    patches = record['patches']
    def digest(name):
        return hashlib.sha256((path / name).read_bytes()).hexdigest()
    if all(digest(name) == entry['sha256'] for name, entry in patches.items()):
        print('Pinned GR00T source and optional TensorRT patch verified')
        return
    if not all(digest(name) == entry['source_sha256'] for name, entry in patches.items()):
        raise SystemExit('GR00T dependency files have unrecorded local changes; preserved without overwrite')
    patch = ROOT / 'configs/dependencies/groot-tensorrt.patch'
    subprocess.run(['git', '-C', str(path), 'apply', '--check', str(patch)], check=True)
    subprocess.run(['git', '-C', str(path), 'apply', str(patch)], check=True)
    if not all(digest(name) == entry['sha256'] for name, entry in patches.items()):
        raise SystemExit('GR00T patch digest mismatch')
    print('Pinned GR00T dependency prepared')


if __name__ == '__main__':
    main()
