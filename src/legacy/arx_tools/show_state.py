#!/usr/bin/env python3
"""Read one validated ARX X5 snapshot without active motion."""

from __future__ import annotations

import argparse

import numpy as np

from legacy.arx5_control import ArmController
from legacy.arx5_control.gripper_calibration import load_gripper_open_readout


def main() -> int:
    parser = argparse.ArgumentParser(description="读取 ARX X5 状态（不会回零）")
    parser.add_argument("--interface", default="can0")
    args = parser.parse_args()

    controller = ArmController(
        interface=args.interface,
        gripper_open_readout=load_gripper_open_readout(interface=args.interface),
    )
    try:
        state = controller.connect()
        np.set_printoptions(precision=3, suppress=True)
        print("模式：damping")
        print("关节位置 (deg)：", np.degrees(state.joint_position))
        print("关节速度 (rad/s)：", state.joint_velocity)
        print("关节扭矩 (N·m)：", state.joint_torque)
        print(f"夹爪位置：{1000.0 * state.gripper_position:.2f} mm")
        print(f"夹爪速度：{state.gripper_velocity:.4f} m/s")
        print(f"夹爪扭矩：{state.gripper_torque:.3f} N·m")
        print("末端 pose_6d：", state.eef_pose_6d)
        return 0
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
