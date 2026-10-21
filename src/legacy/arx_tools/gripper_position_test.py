#!/usr/bin/env python3
"""Conservative position-controlled test for the ARX X5 gripper.

The script never homes or calibrates the arm.  It connects once, softly
enables gains, opens to 40 mm by default, and slowly closes to 0 mm.
"""

from __future__ import annotations

import argparse
import math
import sys

import arx5_interface as arx5

from legacy.arx5_control import ArmController, ControllerMode, GripperMotionResult
from legacy.arx5_control.gripper_calibration import load_gripper_open_readout


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARX X5 夹爪低速开闭测试")
    parser.add_argument("--model", default="X5")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--open-mm", type=float, default=40.0)
    parser.add_argument("--open-duration", type=float, default=2.0)
    parser.add_argument("--close-duration", type=float, default=2.0)
    parser.add_argument("--hold", type=float, default=0.5)
    parser.add_argument("--gain-scale", type=float, default=0.5)
    parser.add_argument("--gain-ramp", type=float, default=1.5)
    parser.add_argument("--gripper-gain-scale", type=float, default=0.5)
    parser.add_argument("--gripper-gain-ramp", type=float, default=1.0)
    parser.add_argument(
        "--gripper-open-readout",
        type=float,
        default=None,
        help="仅本进程覆盖夹爪全开电机读数；默认读取持久化夹爪标定",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只校验参数和打印计划，不连接机械臂",
    )
    return parser.parse_args()


def validate_plan(args: argparse.Namespace, controller: ArmController) -> None:
    numeric = (
        args.open_mm,
        args.open_duration,
        args.close_duration,
        args.hold,
    )
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError("夹爪测试参数必须是有限数值")
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(args.model)
    max_width_mm = 1000.0 * robot_config.gripper_width
    if not 0.0 <= args.open_mm <= max_width_mm:
        raise ValueError(f"打开宽度必须在 [0, {max_width_mm:.1f}] mm")
    if not 0.0 <= args.hold <= 30.0:
        raise ValueError("停留时间必须在 [0, 30] s")

    open_required = controller.minimum_gripper_duration(args.open_mm / 1000.0)
    close_required = controller.minimum_gripper_duration(-args.open_mm / 1000.0)
    if args.open_duration < open_required:
        raise ValueError(f"打开时间至少需要 {open_required:.2f}s")
    if args.close_duration < close_required:
        raise ValueError(f"关闭时间至少需要 {close_required:.2f}s")


def print_result(result: GripperMotionResult) -> None:
    print("夹爪开闭完成：")
    print(f"  起点       {1000.0 * result.start_width_m:8.2f} mm")
    print(f"  最大目标   {1000.0 * result.excursion_width_m:8.2f} mm")
    print(f"  最终实测   {1000.0 * result.final_width_m:8.2f} mm")
    print(f"  最终误差   {1000.0 * result.final_error_m:+8.2f} mm")
    print(
        "  实测范围   "
        f"{1000.0 * result.min_measured_width_m:8.2f} .. "
        f"{1000.0 * result.max_measured_width_m:8.2f} mm"
    )
    print(f"  最大速度   {result.max_abs_velocity_m_s:8.3f} m/s")
    print(f"  最大扭矩   {result.max_abs_torque_nm:8.3f} N·m")


def main() -> int:
    args = parse_args()
    gripper_open_readout = (
        load_gripper_open_readout(interface=args.interface)
        if args.gripper_open_readout is None
        else args.gripper_open_readout
    )
    controller = ArmController(
        model=args.model,
        interface=args.interface,
        gain_scale=args.gain_scale,
        gain_ramp_s=args.gain_ramp,
        gripper_gain_scale=args.gripper_gain_scale,
        gripper_gain_ramp_s=args.gripper_gain_ramp,
        gripper_open_readout=gripper_open_readout,
    )
    try:
        validate_plan(args, controller)
    except (ValueError, RuntimeError) as exc:
        print(f"参数错误：{exc}", file=sys.stderr)
        return 2

    print(
        f"计划：夹爪打开到 {args.open_mm:.1f} mm / {args.open_duration:.1f}s，"
        f"停留 {args.hold:.1f}s，再关闭到 0 mm / {args.close_duration:.1f}s。"
    )
    print(
        "本进程临时使用 gripper_open_readout="
        f"{gripper_open_readout:+.3f}；不会写入电机或 SDK 配置。"
    )
    print("不会回零或校准机械臂；异常时立即进入阻尼。")
    if args.dry_run:
        print("dry-run 通过；没有连接机械臂，也没有发送命令。")
        return 0

    try:
        initial = controller.connect()
        print(
            "已连接，初始夹爪："
            f"{1000.0 * initial.gripper_position:.2f} mm；正在软启动。"
        )
        result = controller.open_close_gripper(
            args.open_mm / 1000.0,
            open_duration_s=args.open_duration,
            hold_s=args.hold,
            close_duration_s=args.close_duration,
            close_width_m=0.0,
        )
        print_result(result)
        return 0
    except KeyboardInterrupt:
        controller.emergency_damping("keyboard interrupt")
        print("\n收到 Ctrl+C，已立即进入故障阻尼。", file=sys.stderr)
        return 130
    except Exception as exc:
        controller.emergency_damping(f"gripper test failed: {exc}")
        print(f"夹爪测试失败，已进入故障阻尼：{exc}", file=sys.stderr)
        return 1
    finally:
        controller.close(emergency=controller.mode == ControllerMode.FAULT)


if __name__ == "__main__":
    raise SystemExit(main())
