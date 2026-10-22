"""同一 Robot 接口控制独立七轴无夹爪 MuJoCo 模型。"""
import argparse

from embodied import Robot
from embodied.core.project import PROJECT_ROOT


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args(argv)
    config = PROJECT_ROOT / 'configs/robots/reference7/robot.json'
    with Robot.sim(config=config, viewer=args.viewer) as robot:
        initial = robot.simulation.snapshot()
        q = robot.q
        q[-1] += .15
        robot.movej(q, speed=.5)
        robot.movel(offset=[.005, 0., 0.], tcp_speed=.03)
        print({'joints': len(robot.q), 'pose': robot.pose, 'gripper': 'absent'})
        robot.simulation.restore(initial)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
