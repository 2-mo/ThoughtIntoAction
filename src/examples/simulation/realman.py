"""RM65 MuJoCo demo using the platform API, without hardware or Quest."""
import argparse
import json
import math
import sys
import time

from embodied import Robot
from embodied.contracts.paths import PROJECT_ROOT

CONFIG = PROJECT_ROOT / 'configs/robots/realman/sim.json'


def run(robot, pause=print):
    # Let the kernel accumulate its continuous standstill window before acquiring.
    time.sleep(.5)
    initial = robot.simulation.snapshot()
    pause('MoveJ: 关节 1 转动 0.1 rad')
    q = robot.q
    q[0] += .1
    robot.movej(q, speed=.2)
    pause('MoveL: TCP 沿基座 X 轴移动 10 mm')
    robot.movel(offset=[.01, 0, 0], speed=.2, tcp_speed=.02)
    pause('简化夹爪闭合到 20 mm，然后张开到假定的 40 mm')
    robot.gripper.move(.02, timeout=5)
    robot.gripper.open(timeout=5)
    moved = robot.state
    robot.hold()
    robot.simulation.restore(initial)
    restored = robot.simulation.observe()
    time.sleep(.5)
    robot.simulation.reset()
    return {'model': 'RM65 nominal', 'simplified_gripper': True, 'moved_q': moved['q'],
            'restored_q': restored['q'], 'restored_gripper_width_m': restored['gripper_width']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='无窗口连续执行后退出')
    parser.add_argument('--check-kinematics', action='store_true', help='只做官方离线 FK 对照，不启动内核或 VR')
    parser.add_argument('--quest', action='store_true', help='用 Quest 控制新建的 RM65 仿真，不执行脚本演示')
    parser.add_argument('--duration', type=float, default=30., help='Quest 秒数，0 持续至 B/Ctrl+C')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if args.quest and args.check_kinematics:
        parser.error('--quest and --check-kinematics are separate actions')
    if args.check_kinematics:
        from embodied.devices.robots.realman.simulation_check import check
        return check()
    if not args.headless and not sys.stdin.isatty():
        parser.error('可视化示例需要交互终端；无人值守使用 --headless')
    try:
        with Robot.sim(config=CONFIG, viewer=not args.headless) as robot:
            print(f'RM65 仿真 · {robot.run}', flush=True)
            if args.quest:
                from embodied.devices.robots.realman.simulation_quest import connect
                time.sleep(.5)
                print(json.dumps(connect(robot.run, args.duration)), flush=True)
                return 0
            pause = print if args.headless else lambda step: input(f'回车：{step}')
            print(json.dumps(run(robot, pause), ensure_ascii=False), flush=True)
            if not args.headless:
                input('回车关闭仿真；Ctrl+C 也可退出。')
    except (KeyboardInterrupt, EOFError):
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
