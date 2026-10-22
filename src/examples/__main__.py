"""列出示例，或运行一个示例：./run.sh examples movej --help。"""
import argparse
from importlib import import_module


EXAMPLES = {
    "sim_control": ("simulation.control", "可视化仿真控制：关节、XYZ、夹爪与返回；无需真机或 VR"),
    "sim_task": ("simulation.task_reset", "保存/恢复仿真场景，重复执行任务并检查成功条件"),
    "read_state": ("control.read_state", "读取关节、TCP、夹爪和内核能力（不申请控制权）"),
    "movej": ("control.movej", "robot.movej：给定关节目标，等待到位"),
    "movel": ("control.movel", "TCP 直线运动：绝对 XYZ、相对偏移或完整位姿"),
    "gripper": ("control.gripper", "robot.gripper.move：设置夹爪宽度"),
    "teach_return": ("control.teach_return", "teach → jog → return_to：记点、移动、返回"),
    "approach_retreat": ("tasks.approach_retreat", "任务阶段：接近、退离，自动保存时间轴"),
    "pick_place": ("tasks.pick_place", "二次观测与抓放模板；默认仿真空抓失败后停止"),
    "record": ("data.record", "robot.record：采集一段无相机诊断数据"),
    "annotate": ("data.annotate", "用任务事件生成技能区间标注，在网页复核"),
    "offline_trajectory": ("planning.offline_trajectory", "离线生成关节轨迹，查看 q / dq / ddq"),
    "cartesian_blend": ("planning.cartesian_blend", "离线生成带圆角的笛卡尔几何路径（不是已实现的 MoveP）"),
}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, epilog="控制例子默认仿真；真机显式使用 --backend real --run <RUN>。")
    parser.add_argument("example", nargs="?", choices=EXAMPLES)
    parser.add_argument("arguments", nargs=argparse.REMAINDER, help="传给该示例的参数")
    args = parser.parse_args(argv)
    if args.example is None:
        print("从 src/examples/README.md 开始；每个文件只演示一项功能。\n")
        for name, (module, description) in EXAMPLES.items():
            print(f"  {name:20} {description}  [{module.replace('.', '/')}.py]")
        print("\n可视化仿真：./run.sh examples sim_control\n单项动作：./run.sh examples movej --viewer --keep-open")
        return 0
    module, _ = EXAMPLES[args.example]
    return import_module(f"examples.{module}").main(args.arguments)


if __name__ == "__main__":
    raise SystemExit(main())
