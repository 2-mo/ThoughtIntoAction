"""运行统一内核往返动作，保存原始反馈及速度、周期、跟踪报告。"""
import argparse
import json
import math
from pathlib import Path

from embodied import Robot
from embodied.applications.operations.benchmark import benchmark, compare_reports
from embodied.core.storage import save_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', help='连接已经启动的内核；省略时自动新建仿真')
    parser.add_argument('--backend', choices=('sim', 'real'), default='sim')
    parser.add_argument('--config', help='新建仿真的机器人配置')
    parser.add_argument('--joint', type=int, default=1, help='关节编号，从 1 开始')
    parser.add_argument('--degrees', type=float, default=10.)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--speed', type=float, help='rad/s，0 使用后端每轴上限；省略沿用内核配置')
    parser.add_argument('--output', default='outputs/benchmarks')
    parser.add_argument('--baseline', type=Path, help='同一模型、动作与速度设置的历史 report.json')
    args = parser.parse_args(argv)
    if args.run and args.config:
        parser.error('--config 只用于新建仿真')
    if args.backend == 'real' and (not args.run or args.config):
        parser.error('真机必须显式 --backend real --run <RUN>，不能传 --config')
    baseline = json.loads(args.baseline.read_text()) if args.baseline else None
    robot = Robot.real(args.run) if args.backend == 'real' else Robot.sim(run=args.run, config=args.config)
    with robot:
        path = benchmark(robot, joint=args.joint - 1, delta_rad=math.radians(args.degrees),
                         repeats=args.repeats, speed=args.speed, output=args.output)
    if baseline:
        comparison = compare_reports(json.loads((path / 'report.json').read_text()), baseline)
        save_json(path / 'comparison.json', comparison)
        print(json.dumps(comparison, ensure_ascii=False, indent=2))
    print((path / 'report.md').read_text())
    print(f'完整报告：{path / "report.json"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
