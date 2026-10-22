"""接触抓取：空夹或超时明确返回 confirmed=False，不伪造抓取成功。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot, *, effort):
    """effort 单位见 describe.robot.gripper：ARX motor_Nm，仿真每爪 N。"""
    robot.gripper.open()
    return robot.gripper.grasp(effort=effort, minimum_width=0., timeout=5.)


def main(argv=None):
    return run_example("grasp", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
