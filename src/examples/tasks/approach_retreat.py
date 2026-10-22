"""组合接近、移动、退离；阶段事件自动保存到运行目录 tasks/。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot):
    """以当前 TCP 为目标，在其上方 20 mm 接近，再沿基座 +X 退离 10 mm。"""
    with robot.task("approach and retreat") as task:
        task.approach(xyz=robot.xyz, offset=[0., 0., .02])
        task.retreat([.01, 0., 0.])
    return task.result


def main(argv=None):
    return run_example("approach_retreat", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
