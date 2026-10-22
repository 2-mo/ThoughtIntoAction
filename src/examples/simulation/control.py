"""可视化 MuJoCo 仿真：按回车逐步执行；--headless 连续运行后退出。"""
import argparse
import sys

from embodied import Robot


def run(robot: Robot, *, pause=lambda message: None):
    """同一套接口演示关节控制、XYZ 直线控制、夹爪与返回。"""
    start = robot.teach()
    pause("第一个关节转动 0.2 rad")
    q = robot.q
    q[0] += .2
    robot.movej(q)

    pause("末端沿基座 Z 轴上移 20 mm")
    x, y, z = robot.xyz
    robot.movel(xyz=[x, y, z + .02])

    pause("打开夹爪，再回到开始点")
    robot.gripper.open()
    robot.return_to(start)
    return robot.state


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="不打开窗口、不等待回车，适合 SSH / 自动化")
    args = parser.parse_args(argv)
    if not args.headless and not sys.stdin.isatty():
        parser.error("可视化示例需要交互终端；无人值守使用 --headless")
    try:
        with Robot.sim(viewer=not args.headless, speed=.5) as robot:
            print(f"MuJoCo 仿真 · {robot.run}", flush=True)
            if not args.headless:
                print("窗口仅显示反馈；按终端 Ctrl+C 结束本次仿真。", flush=True)
            state = run(robot, pause=print if args.headless else lambda step: input(f"回车：{step}"))
            print(f"动作完成，当前关节：{state['q']}", flush=True)
            if not args.headless:
                input("回车关闭本次仿真；也可继续查看窗口。")
    except (KeyboardInterrupt, EOFError):
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
