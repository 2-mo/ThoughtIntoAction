"""记点、移动、返回：组合简单动作；每段停稳，不是连续圆角路径。"""
import math

from embodied import Robot
from ..common import run_example, save_point


def run(robot: Robot, *, output="outputs/examples/teach-point.json"):
    """teach() 保存当前姿态快照，不是自由拖动模式；Python 关节编号从 0 开始。"""
    start = robot.teach()
    path = save_point(start, output)
    robot.jog(joint=0, delta=math.radians(5))
    robot.jog(joint=0, delta=math.radians(-10))
    robot.return_to(start)
    return {"saved_point": str(path), "returned_q_rad": robot.q}


def main(argv=None):
    return run_example("teach_return", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
