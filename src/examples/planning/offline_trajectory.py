"""离线学习 MoveJ 的轨迹生成：六轴同步、限制速度/加速度/加加速度。

运行：PYTHONPATH=src outputs/envs/control/bin/python -m examples.planning.offline_trajectory
本例只生成 JSON，不连接内核或机械臂；修改 target 即可改变关节目标。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from embodied.motion.offline import plan_ruckig_joint_trajectory


def main(argv: list[str] | None = None) -> int:
    """规划基座转动 10°，观察同一条 Ruckig 曲线的 q、dq、ddq。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--degrees", type=float, default=10.0, help="第 1 轴位移，度")
    parser.add_argument("--speed", type=float, default=0.5, help="正数，rad/s")
    parser.add_argument("--acceleration", type=float, default=2.0, help="正数，rad/s²")
    parser.add_argument("--jerk", type=float, default=10.0, help="正数，rad/s³")
    parser.add_argument("--output", type=Path, default=Path("outputs/examples/offline_trajectory.json"))
    args = parser.parse_args(argv)

    # 单位统一用弧度；这里是教学用的六轴零位，不是实机当前状态。
    start = np.zeros(6)
    target = start.copy()
    target[0] += np.deg2rad(args.degrees)
    # ±π 是本例自定义的位置范围，不代表 ARX/其他机械臂的真实限位。
    lower, upper = np.full(6, -np.pi), np.full(6, np.pi)
    try:
        trajectory = plan_ruckig_joint_trajectory(
            start_position=start,
            target_position=target,
            current_velocity=np.zeros(6),
            min_position=lower,
            max_position=upper,
            max_velocity=args.speed,
            max_acceleration=args.acceleration,
            max_jerk=args.jerk,
            sample_rate_hz=250.0,
            minimum_duration_s=1.0 / 250.0,
        )
    except ValueError as exc:
        parser.error(str(exc))

    # 每个时刻都向原始曲线同时取 q/dq/ddq，不对位置另做插值。
    samples = []
    for time_s in np.concatenate(([0.0], trajectory.sample_times_s)):
        q, dq, ddq = trajectory.at_time(float(time_s))
        samples.append({"time_s": float(time_s), "q_rad": q.tolist(),
                        "dq_rad_s": dq.tolist(), "ddq_rad_s2": ddq.tolist()})
    result = {
        "description": "MoveJ 的离线关节轨迹；q、dq、ddq 来自同一条 Ruckig 曲线。",
        "scope": "只规划，不执行；所有输入限位均为教学配置，未读取硬件。speed 必须为正数。",
        "duration_s": trajectory.duration_s,
        "start_q_rad": start.tolist(),
        "target_q_rad": target.tolist(),
        "limits": {"min_position_rad": lower.tolist(), "max_position_rad": upper.tolist(),
                   "max_velocity_rad_s": args.speed, "max_acceleration_rad_s2": args.acceleration,
                   "max_jerk_rad_s3": args.jerk},
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"MoveJ 离线轨迹：{trajectory.duration_s:.6f} s，{len(samples)} 个样本 → {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
