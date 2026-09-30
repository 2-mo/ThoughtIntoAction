#!/usr/bin/env python3
"""Import working-tree components without overwriting existing local implementations.

The per-file ledger records original bytes, destinations, and preserved local
implementations. Repository snapshots are retained separately from active code.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
REPOS = ('arx5_test', 'reBotArm_control_py', 'robot-teleop')
LEDGER = ROOT / 'third_party/migration/manifest.json'
SNAPSHOTS = ROOT / 'archives/migration/originals'
SKIP = {'.git', '.venv', '.venv-web', '.venv-handeye', 'node_modules',
        '__pycache__', '.pytest_cache', '.uv-cache', 'dist', 'test-results'}


def digest(path):
    with path.open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def destination(repo, name):
    mappings = {
        'arx5_test': [
            ('src/backend/arx5_backend/', 'src/arx5_backend/'),
            ('src/frontend/', 'apps/console/'),
            ('src/gr-zeroshot/gr_zeroshot/', 'src/gr_zeroshot/'),
            ('src/gr-zeroshot/main.py', 'src/gr_zeroshot/cli.py'),
            ('src/scripts/', 'src/arx_tools/'),
            ('src/', 'src/'),
            ('configs/', 'configs/arx_motion/'),
            ('assets/', 'assets/arx_motion/'),
            ('weights/', 'assets/arx_motion/weights/'),
            ('tests/', 'tests/migrated/arx_motion/'),
            ('calibrations/', 'calibrations/arx_motion/'),
            ('pyproject.toml', 'environments/arx_motion/pyproject.toml'),
            ('uv.lock', 'environments/arx_motion/uv.lock'),
        ],
        'reBotArm_control_py': [
            ('reBotArm_control_py/', 'src/reBotArm_control_py/'),
            ('config/', 'configs/rebot/'),
            ('urdf/', 'assets/robots/rebot/'),
            ('example/', 'examples/rebot/'),
            ('pyproject.toml', 'environments/rebot/pyproject.toml'),
            ('uv.lock', 'environments/rebot/uv.lock'),
        ],
        'robot-teleop': [
            ('src/', 'src/'),
            ('apps/', 'apps/'),
            ('assets/', 'assets/phone/'),
            ('configs/', 'configs/phone/'),
            ('tests/', 'tests/migrated/phone/'),
            ('tools/', 'scripts/phone/'),
            ('pyproject.toml', 'environments/phone/pyproject.toml'),
            ('uv.lock', 'environments/phone/uv.lock'),
        ],
    }
    for old, new in mappings[repo]:
        if (old.endswith('/') and name.startswith(old)) or name == old:
            return new + name[len(old):]
    return f'docs/imported/{repo}/' + ('AGENTS.source.md' if name == 'AGENTS.md' else name)


def git(source, *args):
    return subprocess.check_output(['git', '-C', str(source), *args])


def import_sources():
    if LEDGER.exists():
        raise SystemExit(f'Import already recorded: {LEDGER}; edit migrated sources directly')
    result = {'version': 1, 'repositories': {}, 'files': []}
    for repo in REPOS:
        source = ROOT.parent / repo
        result['repositories'][repo] = {
            'head': git(source, 'rev-parse', 'HEAD').decode().strip(),
            'status': git(source, 'status', '--porcelain=v1').decode(),
            'original': str(source), 'snapshot': str(SNAPSHOTS.relative_to(ROOT) / repo),
        }
        names = set(git(source, 'ls-files', '-z', '--cached', '--others', '--exclude-standard').decode().split('\0'))
        # User assets and calibration captures may be ignored by the source repo.
        for folder in (('assets', 'weights', 'calibrations') if repo == 'arx5_test' else ('assets', 'urdf')):
            base = source / folder
            if base.exists():
                names.update(str(p.relative_to(source)) for p in base.rglob('*') if p.is_file())
        for name in sorted(names - {''}):
            p = source / name
            if not p.exists() or not p.is_file() or any(v in SKIP for v in Path(name).parts):
                continue
            target_name = destination(repo, name)
            target = ROOT / target_name
            original_hash = digest(p)
            existing = target.exists()
            if not existing:
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, target, follow_symlinks=True)
            state = 'imported' if digest(target) == original_hash else 'existing-local-implementation'
            result['files'].append({'repository': repo, 'source': name, 'destination': target_name,
                                    'source_sha256': original_hash, 'import_sha256': digest(target),
                                    'executable': bool(p.stat().st_mode & 0o111), 'state': state})
        print(repo, 'imported', flush=True)
    LEDGER.parent.mkdir(parents=True, exist_ok=True)
    LEDGER.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print('Recorded', len(result['files']), 'source files')


def verify():
    data = json.loads(LEDGER.read_text())
    patches_path = LEDGER.with_name('patches.json')
    patches = json.loads(patches_path.read_text()) if patches_path.exists() else {}
    failures = []
    for row in data['files']:
        target = ROOT / row['destination']
        expected = patches.get(row['destination'], {}).get('sha256', row['import_sha256'])
        if not target.is_file() or digest(target) != expected:
            failures.append(row['destination'])
    if failures:
        raise SystemExit('Unrecorded or missing migrated files: ' + ', '.join(failures))
    print(f"Verified {len(data['files'])} mappings; {len(patches)} documented local changes")


def archive():
    verify()
    data = json.loads(LEDGER.read_text())
    # Verify source bytes again before the reversible, same-filesystem rename.
    for repo in REPOS:
        source, target = ROOT.parent / repo, SNAPSHOTS / repo
        if not source.exists():
            if target.is_dir():
                continue
            raise SystemExit(f'Missing source and snapshot: {repo}')
        if target.exists():
            raise SystemExit(f'Snapshot already exists: {target}')
        for row in data['files']:
            if row['repository'] == repo and digest(source / row['source']) != row['source_sha256']:
                raise SystemExit(f'Source changed since import: {repo}/{row["source"]}')
        target.parent.mkdir(parents=True, exist_ok=True)
        os.rename(source, target)
        print(f'Preserved entire original tree: {target}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('import', 'verify', 'archive'))
    action = parser.parse_args().action
    {'import': import_sources, 'verify': verify, 'archive': archive}[action]()
