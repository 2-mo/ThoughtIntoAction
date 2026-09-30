"""Single-repository entrypoint for imported operations, tools and environments."""
from __future__ import annotations

import argparse
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / 'configs/components/commands.json'


def command(name, arguments):
    catalog = json.loads(CATALOG.read_text())
    if name not in catalog['commands']:
        raise ValueError(f'Unknown component command: {name}')
    spec = catalog['commands'][name]
    environment = catalog['environments'][spec['environment']]
    python = ROOT / environment['python']
    env = os.environ.copy()
    env['PYTHONPATH'] = str(ROOT / 'src')
    env.pop('PYTHONHOME', None)
    env.pop('LD_LIBRARY_PATH', None)
    cwd = ROOT / spec.get('cwd', '.')
    if 'module' in spec:
        if not python.is_file():
            raise ValueError(f'Environment missing; run ./run.sh components setup {spec["environment"]}')
        argv = [str(python), '-B', '-m', spec['module']]
    elif 'script' in spec:
        if not python.is_file():
            raise ValueError(f'Environment missing; run ./run.sh components setup {spec["environment"]}')
        argv = [str(python), '-B', str(ROOT / spec['script'])]
    elif 'shell' in spec:
        argv = ['bash', str(ROOT / spec['shell'])]
    else:
        argv = list(spec['argv'])
    return argv + spec.get('arguments', []) + arguments, cwd, env


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('list')
    setup = sub.add_parser('setup'); setup.add_argument('environment')
    run = sub.add_parser('run'); run.add_argument('name'); run.add_argument('arguments', nargs=argparse.REMAINDER)
    sub.add_parser('verify')
    args = parser.parse_args(argv)
    try:
        catalog = json.loads(CATALOG.read_text())
        if args.action == 'list':
            from component_api.registry import Registry
            print(json.dumps({'plugins': Registry().entries, **catalog}, ensure_ascii=False, indent=2))
            return 0
        if args.action == 'setup':
            if args.environment not in catalog['environments']:
                raise ValueError(f'Unknown environment: {args.environment}')
            spec = catalog['environments'][args.environment]
            if spec.get('prepare'):
                subprocess.run(spec['prepare'], cwd=ROOT, check=True)
            return subprocess.call(spec['setup'], cwd=ROOT)
        if args.action == 'verify':
            return subprocess.call([sys.executable, str(ROOT / 'scripts/migration/consolidate.py'), 'verify'])
        arguments = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
        cmd, cwd, env = command(args.name, arguments)
        os.chdir(cwd)
        os.execvpe(cmd[0], cmd, env)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f'{exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
