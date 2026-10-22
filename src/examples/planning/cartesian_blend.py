"""离线学习笛卡尔圆角路径：直线段连接圆弧，姿态使用 XYZW 四元数。

运行：PYTHONPATH=src outputs/envs/control/bin/python -m examples.planning.cartesian_blend
本例只构造几何路径，不包含 IK、时间规划、恒速控制或执行；不是完整 UR movep。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from embodied.motion.geometry import CartesianWaypoint, build_cartesian_path


def main(argv: list[str] | None = None) -> int:
    """用四个路点说明中间圆角半径、路径弧长和姿态的写法。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius", type=float, default=0.025, help="中间路点圆角半径，m")
    parser.add_argument("--step", type=float, default=0.003, help="最大几何采样间距，m")
    parser.add_argument("--output", type=Path, default=Path("outputs/examples/cartesian_blend.json"))
    args = parser.parse_args(argv)

    # XYZ 单位为米，坐标相对于基座；这些教学坐标未做机械臂可达性验证。
    # 首尾半径必须为 0；中间有圆角时，路径绕过角点，不会精确经过角点。
    # XYZW=[0, 0, 0, 1] 是单位姿态；后续省略姿态时继承前一个路点。
    try:
        waypoints = [
            CartesianWaypoint(np.array([0.25, -0.10, 0.30]),
                              np.array([0.0, 0.0, 0.0, 1.0]), name="起点"),
            CartesianWaypoint(np.array([0.40, -0.10, 0.30]),
                              blend_radius_m=args.radius, name="圆角 1"),
            CartesianWaypoint(np.array([0.40, 0.10, 0.30]),
                              blend_radius_m=args.radius, name="圆角 2"),
            CartesianWaypoint(np.array([0.25, 0.10, 0.30]), name="终点"),
        ]
        path = build_cartesian_path(waypoints, max_step_m=args.step)
    except ValueError as exc:
        parser.error(str(exc))

    # distance 是从起点累计的弧长；它不是时间，不能直接当成恒速指令执行。
    samples = [
        {"distance_m": float(distance), "position_xyz_m": position.tolist(),
         "orientation_xyzw": orientation.tolist()}
        for distance, position, orientation in zip(
            path.distance_m, path.positions_xyz_m, path.orientations_xyzw)
    ]
    result = {
        "description": "四个笛卡尔路点组成的直线 + 圆角几何路径，姿态为 XYZW 四元数。",
        "scope": "仅几何；无 IK、无时间规划、无恒速控制、无执行，不等同于完整 UR movep。",
        "length_m": path.length_m,
        "has_blends": path.has_blends,
        "waypoints": [
            {"name": point.name, "position_xyz_m": point.position_xyz_m.tolist(),
             "orientation_xyzw": None if point.orientation_xyzw is None else point.orientation_xyzw.tolist(),
             "blend_radius_m": point.blend_radius_m}
            for point in waypoints
        ],
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"笛卡尔圆角路径：{path.length_m:.6f} m，{len(samples)} 个样本 → {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
