"""MoveJ：给定关节目标，一行调用完成规划、执行与等待。"""
import math

from embodied import Robot
from ..common import run_example


def run(robot: Robot, *, joint=1, degrees=10., q=None, preview_only=False):
    """默认让第一个关节增加 10°；q 为绝对关节位置，单位 rad。"""
    target = list(q) if q is not None else robot.q
    if q is None:
        if not 1 <= joint <= len(target):
            raise ValueError(f"joint must be in 1..{len(target)}")
        target[joint - 1] += math.radians(degrees)
    if preview_only:
        return robot.plan_movej(target)
    return robot.movej(target)  # 默认等待到位；末端路径不保证是直线。


def main(argv=None):
    return run_example("movej", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
