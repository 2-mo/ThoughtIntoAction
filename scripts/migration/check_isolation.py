#!/usr/bin/env python3
"""Rebuild migrated environments and run contracts from a separate source tree."""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    target = args.output.resolve()
    target.mkdir(parents=True, exist_ok=False)
    ignore = shutil.ignore_patterns('__pycache__', '.venv*', 'node_modules', 'dist', '.git')
    for folder in ('src', 'configs', 'assets', 'environments', 'tests/migrated'):
        shutil.copytree(ROOT / folder, target / folder, ignore=ignore)
    (target / 'scripts').mkdir()
    shutil.copy2(ROOT / 'scripts/check_rebot_components.py', target / 'scripts/check_rebot_components.py')
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    env.pop('PYTHONHOME', None)
    env.pop('LD_LIBRARY_PATH', None)
    for name, version, extra in [('arx_motion', '3.12', ['--only-group', 'handeye']), ('rebot', '3.10', [])]:
        with (target / f'{name}-setup.log').open('w') as log:
            subprocess.run(['uv', 'sync', '--offline', '--locked', '--no-install-project',
                            '--project', str(target / 'environments' / name), '--python', version, *extra],
                           cwd=target, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    # Audit-hook rejection catches dynamic attempts to reopen original source,
    # snapshots, or the live project's source/config/models during this check.
    audit = target / 'audit'
    audit.mkdir()
    blocked = [str(ROOT / part) for part in ('archives', 'src', 'configs', 'assets')]
    blocked += [str(ROOT.parent / name) for name in ('arx5_test', 'reBotArm_control_py', 'robot-teleop')]
    (audit / 'sitecustomize.py').write_text('''import os, sys
BLOCKED = ''' + repr(blocked) + '''
def guard(event, args):
    if event != 'open' or not args or not isinstance(args[0], (str, bytes)):
        return
    value = os.path.realpath(os.fsdecode(args[0]))
    if any(value == p or value.startswith(p + os.sep) for p in BLOCKED):
        raise PermissionError('Migration isolation blocked external project source: ' + value)
sys.addaudithook(guard)
''')
    env['PYTHONPATH'] = str(audit) + os.pathsep + str(target / 'src')
    checks = [
        ('imported-tests', [str(target / 'environments/arx_motion/.venv/bin/python'), '-m', 'pytest',
                            'tests/migrated/arx_motion', 'tests/migrated/phone', '-q', '--import-mode=importlib']),
        ('rebot', [str(target / 'environments/rebot/.venv/bin/python'), 'scripts/check_rebot_components.py']),
    ]
    for name, cmd in checks:
        with (target / f'{name}.log').open('w') as log:
            subprocess.run(cmd, cwd=target, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    result = {'source_tree': str(target), 'blocked_paths': blocked,
              'clean_environments': ['arx_motion', 'rebot'], 'checks': [n for n, _ in checks]}
    (target / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
