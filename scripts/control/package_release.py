#!/usr/bin/env python3
"""Package the control runtime in the repository layout, without local environments or data."""
import argparse
import hashlib
from io import BytesIO
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[2]
DIRECTORIES = (
    'src/embodied/control', 'src/embodied/contracts', 'src/embodied/motion',
    'src/embodied/devices/robots/arx5', 'src/embodied/devices/inputs/quest',
    'configs/control', 'configs/robots/arx5', 'configs/inputs',
    'assets/robots/x5/runtime', 'scripts/control', 'docs/provenance/arx-televr',
)
FILES = (
    'src/embodied/__init__.py', 'src/embodied/devices/__init__.py',
    'src/embodied/devices/robots/__init__.py', 'src/embodied/devices/inputs/__init__.py',
    'configs/plugins.json', 'configs/environments/control/pyproject.toml',
    'configs/environments/control/uv.lock', 'configs/environments/control/.python-version',
    'scripts/setup/build_gripper_sdk.py', 'docs/guides/control-runtime.md',
    'docs/guides/gripper-startup.md',
)
EXCLUDED = {'.git', '.venv', '__pycache__', '.pytest_cache', '.ruff_cache',
            '.env', '.DS_Store', 'outputs', 'third_party'}
ARCHIVE_ROOT = 'embodied-control'


def release_files(root=ROOT):
    """List runtime sources/assets without freezing platform source hashes."""
    root = Path(root)
    required = {root / name for name in FILES}
    candidates = set(required)
    for name in DIRECTORIES:
        directory = root / name
        if not directory.is_dir():
            raise FileNotFoundError(f'Required release directory missing: {name}')
        candidates.update(directory.rglob('*'))
    result = []
    for path in candidates:
        relative = path.relative_to(root)
        if (EXCLUDED.intersection(relative.parts) or path.suffix in {'.pyc', '.pyo'}
                or path.name.endswith('.local.json') or path.name == 'local.toml'):
            continue
        if path.is_symlink():
            target = path.readlink()
            if target.is_absolute() or not path.resolve().is_relative_to(root.resolve()):
                raise ValueError(f'External symlink cannot be packaged: {relative}')
            if path.is_dir():
                raise ValueError(f'Directory symlink cannot be packaged: {relative}')
        if path.is_file():
            result.append(path)
        elif path in required:
            raise FileNotFoundError(f'Required release file missing: {relative}')
    included = {path.resolve() for path in result if not path.is_symlink()}
    for path in result:
        if path.is_symlink() and path.resolve() not in included:
            raise ValueError(f'Symlink target is not included in release: {path.relative_to(root)}')
    return sorted(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/releases/embodied-control.tar.gz')
    args = parser.parse_args()
    paths = release_files()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(args.output, 'w:gz') as archive:
        checksums = []
        for path in paths:
            relative = path.relative_to(ROOT)
            archive.add(path, arcname=str(Path(ARCHIVE_ROOT) / relative), recursive=False)
            checksums.append(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {relative.as_posix()}\n')
        data = ''.join(checksums).encode()
        info = tarfile.TarInfo(f'{ARCHIVE_ROOT}/MANIFEST.sha256')
        info.size = len(data)
        archive.addfile(info, BytesIO(data))
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_name(args.output.name + '.sha256').write_text(
        f'{digest}  {args.output.name}\n', encoding='utf-8')
    print(f'{args.output.resolve()} ({args.output.stat().st_size / 1024 / 1024:.1f} MiB, {len(paths)} files)')
    print(f'SHA256 {digest}')


if __name__ == '__main__':
    main()
