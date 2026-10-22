"""只读采集内核状态与动作流；无相机时记录诊断轨迹。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot, *, duration=2., output="outputs/examples/episodes"):
    """记录指定秒数，返回 episode 目录；本例不触发运动。"""
    path = robot.record(duration, output=output, task="example-runtime-trace")
    return {"episode": str(path), "duration_s": duration}


def main(argv=None):
    return run_example("record", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
