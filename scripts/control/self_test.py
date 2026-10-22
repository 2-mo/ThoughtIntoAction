#!/usr/bin/env python3
"""Run offline migration checks and optional complete simulation routes."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description='离线验收；不连接机械臂，不运行 pytest')
    parser.add_argument('--simulation', action='store_true', help='另运行 X5 原生输入/路由/MuJoCo 链路')
    args = parser.parse_args()
    env = os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
               PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1', PYTHONPATH=str(ROOT / 'src'))

    def run(*command):
        subprocess.run([sys.executable, *map(str, command)], cwd=ROOT, env=env, check=True)

    run(ROOT / 'scripts/control/check_portable.py')
    run(ROOT / 'scripts/control/check_integration.py')
    run(ROOT / 'scripts/control/check_commands.py')
    for module, script in [('arx5_interface', 'check_real_offline.py')]:
        if importlib.util.find_spec(module) is None:
            print(f'SKIP: {script}（未安装 {module}；./scripts/control/setup.sh arx 可安装 SDK）', flush=True)
        else:
            run(ROOT / 'scripts/control' / script)
    if args.simulation:
        for robot, config in [('x5', 'configs/robots/arx5/robot.json')]:
            report_path = ROOT / 'outputs/runtime/control' / f'check-{robot}.json'
            run('-m', 'embodied.control', 'run', '--backend', 'sim', '--input', 'demo',
                '--config', ROOT / config, '--headless', '--duration', '12',
                '--record-tuning', '--report', report_path)
            report = json.loads(report_path.read_text())
            checks = {
                'frequency 245..255 Hz': 245 <= report['achieved_hz'] <= 255,
                'arm commands >100': report['pose_commands'] > 100,
                'arm motion >0.01 m': report['max_cartesian_motion_m'] > .01,
                'all three modes': {x['arm'] for x in report['mode_changes']}
                    >= {'disabled', 'free-drive', 'command-following'},
                'clean parking': report['park_completed'] and report['error'] is None,
                'complete records': report['tuning_recording']['complete']
                    and report['tuning_recording']['dropped'] == 0,
                'gripper route': report['gripper_commands'] > 100
                    and report['gripper_max_m'] - report['gripper_min_m'] > .02,
            }
            for name, passed in checks.items():
                print(f'{"PASS" if passed else "FAIL"}: {robot} {name}', flush=True)
            if not all(checks.values()):
                return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
