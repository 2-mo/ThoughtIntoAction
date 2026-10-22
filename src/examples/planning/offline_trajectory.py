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
from embodied.control.motion_limits import MotionLimits
from embodied.control.settings import load_settings


def main(argv: list[str] | None = None) -> int:
    """规划基座转动 10°，观察同一条 Ruckig 曲线的 q、dq、ddq。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--degrees", type=float, default=10.0, help="第 1 轴位移，度")
    parser.add_argument("--speed", type=float, help="rad/s；0 使用模型配置上限；省略沿用 motion.json")
    parser.add_argument("--acceleration", type=float, help="rad/s²；省略沿用 motion.json")
    parser.add_argument("--jerk", type=float, help="rad/s³；省略沿用 motion.json")
    parser.add_argument("--output", type=Path, default=Path("outputs/examples/offline_trajectory.json"))
    args = parser.parse_args(argv)

    config = load_settings().robot
    policy = MotionLimits.load(config)
    # 离线使用机器人配置的名义能力，不连接硬件。
    start = np.asarray(config.park_q).copy()
    target = start.copy()
    target[0] += np.deg2rad(args.degrees)
    lower, upper = np.asarray(config.joint_min), np.asarray(config.joint_max)
    try:
        limits = policy.resolve({key: getattr(args, key) for key in ('speed', 'acceleration', 'jerk')
                                 if getattr(args, key) is not None})
        trajectory = plan_ruckig_joint_trajectory(
            start_position=start,
            target_position=target,
            current_velocity=np.zeros_like(start),
            min_position=lower,
            max_position=upper,
            max_velocity=limits['joint_velocity_rad_s'],
            max_acceleration=limits['acceleration'],
            max_jerk=limits['jerk'],
            sample_rate_hz=config.control_hz,
            minimum_duration_s=1.0 / config.control_hz,
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
        "scope": "只规划，不执行；使用 motion.json 和机器人模型的名义能力，未读取硬件。",
        "motion_configuration": policy.describe(),
        "duration_s": trajectory.duration_s,
        "start_q_rad": start.tolist(),
        "target_q_rad": target.tolist(),
        "limits": {"min_position_rad": lower.tolist(), "max_position_rad": upper.tolist(),
                   "max_velocity_rad_s": limits['joint_velocity_rad_s'], "max_acceleration_rad_s2": limits['acceleration'],
                   "max_jerk_rad_s3": limits['jerk']},
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"MoveJ 离线轨迹：{trajectory.duration_s:.6f} s，{len(samples)} 个样本 → {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
