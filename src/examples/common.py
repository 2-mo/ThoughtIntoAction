"""示例共用的命令行与输出；机器人连接、规划、等待由 Robot 封装。"""
import argparse
import json
from pathlib import Path
import sys

from embodied import Robot


def save_point(point, output):
    """保存 teach() 的结果，供之后的 return_to() 使用。"""
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(point, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def _parser(name, description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--backend", choices=("sim", "real"), default="sim", help="默认 sim；real 连接真机，必须同时指定 --run")
    parser.add_argument("--run", type=Path, help="连接指定后端的已有内核；省略则创建仿真")
    parser.add_argument("--viewer", action="store_true", help="自建仿真时打开 MuJoCo 窗口，需要桌面 DISPLAY")
    parser.add_argument("--keep-open", action="store_true", help="动作完成后等待回车再退出，需要交互终端")
    if name in ("movej", "movel", "move_path", "teach_return"):
        parser.add_argument("--speed", type=float, default=.5, help="关节速度 rad/s；0 使用后端每轴上限（默认 .5）")
        parser.add_argument("--acceleration", type=float, default=2., help="关节加速度 rad/s²（默认 2）")
        parser.add_argument("--jerk", type=float, default=10., help="关节加加速度 rad/s³（默认 10）")
    if name in ("movej", "movel", "move_path"):
        parser.add_argument("--preview-only", action="store_true", help="只规划，不执行动作")
    if name in ("movel", "move_path"):
        parser.add_argument("--tcp-speed", type=float, default=.1, help="末端线速度上限 m/s（默认 .1）")
        parser.add_argument("--angular-speed", type=float, default=.5, help="末端角速度上限 rad/s（默认 .5）")
    if name == "grasp":
        parser.add_argument("--effort", type=float, required=True, help="闭合 effort；单位由设备能力声明，ARX 为 motor_Nm，仿真为每爪 N")
    if name == "movej":
        parser.add_argument("--joint", type=int, default=1, help="关节编号从 1 开始，范围由连接的机器人决定")
        parser.add_argument("--degrees", type=float, default=10., help="相对转动角度，默认 10°")
        parser.add_argument("--q", type=float, nargs="+", help="绝对关节目标，单位 rad，个数与机器人一致")
    elif name == "movel":
        target = parser.add_mutually_exclusive_group()
        target.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"), help="基座系绝对位置 m，保持当前朝向")
        target.add_argument("--offset", type=float, nargs=3, metavar=("DX", "DY", "DZ"), help="相对平移 m，保持当前朝向")
        target.add_argument("--pose", type=float, nargs="+", help="绝对位姿：6值 xyz + 旋转向量(rad)，或7值 xyz + xyzw")
        parser.add_argument("--frame", choices=("base", "tool"), default="base", help="偏移的坐标系；绝对位姿只支持 base")
    elif name == "gripper":
        parser.add_argument("--width", type=float, default=.04, help="夹爪宽度 m，默认 .04")
    elif name == "teach_return":
        parser.add_argument("--output", type=Path, default=Path("outputs/examples/teach-point.json"))
    elif name == "record":
        parser.add_argument("--duration", type=float, default=2., help="采集秒数，必须有限且大于 0")
        parser.add_argument("--output", type=Path, default=Path("outputs/examples/episodes"))
    return parser


def _show(result):
    if hasattr(result, "metrics"):
        result = {"result": result.result, "metrics": result.metrics}
    elif isinstance(result, dict) and "trajectory" in result:
        result = {key: result[key] for key in ("kind", "limits", "metrics", "collision_check")}
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))


def run_example(name, demo, argv=None):
    """CLI 外壳；demo(robot, ...) 也可以直接在自己的程序中调用。"""
    parser = _parser(name, demo.__doc__)
    args = vars(parser.parse_args(argv))
    backend = args.pop("backend")
    directory = args.pop("run")
    viewer, keep_open = args.pop("viewer"), args.pop("keep_open")
    if backend == "real" and directory is None:
        parser.error("--backend real 必须指定 --run；示例不会启动或使能真机")
    if viewer and (backend != "sim" or directory is not None):
        parser.error("--viewer 仅用于自建仿真，不能与 --run 或 --backend real 同用")
    if keep_open and not sys.stdin.isatty():
        parser.error("--keep-open 需要交互终端；自动化运行时省略")
    options = {key: args.pop(key) for key in ("speed", "acceleration", "jerk") if key in args}
    connection = (Robot.sim(run=directory, viewer=viewer, **options) if backend == "sim"
                  else Robot.real(directory, **options))
    try:
        with connection as robot:
            print(f"{name} · {robot.backend} · {robot.run}", flush=True)
            _show(demo(robot, **args))
            if keep_open:
                input("动作已完成。按回车退出；关闭显示窗口不会停止脚本。")
    except (KeyboardInterrupt, EOFError):
        return 130
    return 0
