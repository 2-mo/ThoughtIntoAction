#!/usr/bin/env python3
"""Package portable source assets, excluding environments and local recordings."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]
DIRECTORIES = ('arx5_vr', 'adapters', 'models', 'config', 'native', 'vendor', 'scripts', 'docs')
FILES = ('README.md', 'LICENSE', 'pyproject.toml', 'uv.lock', '.python-version',
         '.gitignore', '.env.example',
         'setup.sh', 'check.sh', 'doctor.sh', 'start_sim.sh', 'start_demo.sh',
         'start_real.sh', 'UPSTREAM.toml', 'UPSTREAM_FILES.json', 'LOCAL_PATCHES.json')
EXCLUDED = {'.git', '.venv', '__pycache__', '.pytest_cache', '.ruff_cache',
            '.env', '.DS_Store'}


def release_files(root=ROOT):
    required = {root / name for name in FILES}
    candidates = list(required)
    for name in DIRECTORIES:
        candidates.extend((root / name).rglob('*'))
    result = []
    for path in candidates:
        relative = path.relative_to(root)
        if (EXCLUDED.intersection(relative.parts) or path.suffix in {'.pyc', '.pyo'}
                or path.name.endswith('.local.json')):
            continue
        if path.is_symlink():
            target = path.readlink()
            if target.is_absolute() or not path.resolve().is_relative_to(root.resolve()):
                raise ValueError(f'External symlink cannot be migrated: {relative}')
        if path.is_file():
            result.append(path)
        elif path in required:
            raise FileNotFoundError(f'Required release file missing: {relative}')
    return sorted(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'dist/arx-televr.tar.gz')
    args = parser.parse_args()
    paths = release_files()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(args.output, 'w:gz') as archive:
        for path in paths:
            archive.add(path, arcname=str(Path('arx-televr') / path.relative_to(ROOT)), recursive=False)
        shared_hashes = {}
        for module in ('embodied_contracts',):
            origin = ROOT.parent / 'src' / module
            if not origin.is_dir():
                origin = ROOT / 'shared' / module
            for path in sorted(origin.rglob('*')):
                if (not path.is_file() or '__pycache__' in path.parts
                        or path.suffix not in ('.py', '.json') and path.name != 'LICENSE'):
                    continue
                relative = Path('shared') / module / path.relative_to(origin)
                archive.add(path, arcname=str(Path('arx-televr') / relative), recursive=False)
                shared_hashes[str(relative)] = hashlib.sha256(path.read_bytes()).hexdigest()
        if not shared_hashes:
            raise FileNotFoundError('Managed control shared modules missing')
        from io import BytesIO
        data = json.dumps(shared_hashes, indent=2).encode()
        info = tarfile.TarInfo('arx-televr/SHARED_SOURCES.json')
        info.size = len(data)
        archive.addfile(info, BytesIO(data))
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_name(args.output.name + '.sha256').write_text(
        f'{digest}  {args.output.name}\n', encoding='utf-8')
    print(f'{args.output.resolve()} ({args.output.stat().st_size / 1024 / 1024:.1f} MiB, {len(paths)} files)')
    print(f'SHA256 {digest}')


if __name__ == '__main__':
    main()
