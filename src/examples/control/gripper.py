"""夹爪宽度控制：也可调用 robot.gripper.open() / close()。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot, *, width=.04):
    """默认开口 40 mm；位置到位不代表成功抓住物体。"""
    actual = robot.gripper.move(width)  # 自动等待宽度到位。
    return {"target_m": width, "actual_m": actual}


def main(argv=None):
    return run_example("gripper", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
