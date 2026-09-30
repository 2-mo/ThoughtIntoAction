#!/usr/bin/env python3
"""Open by position, then close with bounded motor torque."""

from __future__ import annotations

import argparse
import math
import sys

from arx5_control import ArmController, ControllerMode
from arx5_control.gripper_calibration import load_gripper_open_readout


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARX X5 夹爪混合控制测试")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--open-mm", type=float, default=20.0)
    parser.add_argument("--open-duration", type=float, default=2.0)
    parser.add_argument("--closing-torque", type=float, default=0.15)
    parser.add_argument("--minimum-mm", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--contact-hold", type=float, default=0.5)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controller = ArmController(
        interface=args.interface,
        gripper_open_readout=load_gripper_open_readout(interface=args.interface),
    )
    print(
        f"计划：位置控制张开到 {args.open_mm:.1f} mm，随后用 "
        f"{args.closing_torque:.3f} N·m 限扭矩闭合，"
        f"最小开度 {args.minimum_mm:.1f} mm。"
    )
    print("该数值是电机估算扭矩，不是经过标定的指尖夹持力。")
    values = (
        args.open_mm,
        args.open_duration,
        args.closing_torque,
        args.minimum_mm,
        args.timeout,
        args.contact_hold,
    )
    if not all(math.isfinite(value) for value in values):
        print("参数必须是有限数值。", file=sys.stderr)
        return 2
    if not 0.0 <= args.minimum_mm <= args.open_mm <= 88.0:
        print("需要满足 0 <= minimum-mm <= open-mm <= 88。", file=sys.stderr)
        return 2
    if not 0.02 <= args.closing_torque <= 0.30:
        print("closing-torque 必须在 [0.02, 0.30] N·m。", file=sys.stderr)
        return 2
    if not 0.5 <= args.timeout <= 10.0:
        print("timeout 必须在 [0.5, 10] s。", file=sys.stderr)
        return 2
    if not 0.0 <= args.contact_hold <= 5.0:
        print("contact-hold 必须在 [0, 5] s。", file=sys.stderr)
        return 2
    if args.dry_run:
        required = controller.minimum_gripper_duration(args.open_mm / 1000.0)
        if args.open_duration < required:
            print(f"打开时间至少需要 {required:.2f}s", file=sys.stderr)
            return 2
        print("dry-run 通过；没有连接机械臂。")
        return 0

    try:
        initial = controller.connect()
        print(f"已连接，初始开度 {1000.0 * initial.gripper_position:.2f} mm。")
        opened = controller.move_gripper(
            args.open_mm / 1000.0, args.open_duration
        )
        print(f"位置张开完成：{1000.0 * opened.final_width_m:.2f} mm。")
        result = controller.force_close_gripper(
            args.closing_torque,
            minimum_width_m=args.minimum_mm / 1000.0,
            timeout_s=args.timeout,
            contact_hold_s=args.contact_hold,
        )
        print(
            f"限力闭合完成：{1000.0 * result.final_width_m:.2f} mm，"
            f"原因={result.stop_reason}，"
            f"最大实测扭矩={result.max_abs_torque_nm:.3f} N·m。"
        )
        return 0
    except KeyboardInterrupt:
        controller.emergency_damping("keyboard interrupt")
        print("收到 Ctrl+C，已立即阻尼。", file=sys.stderr)
        return 130
    except Exception as exc:
        controller.emergency_damping(f"force gripper test failed: {exc}")
        print(f"测试失败，已进入故障阻尼：{exc}", file=sys.stderr)
        return 1
    finally:
        controller.close(emergency=controller.mode == ControllerMode.FAULT)


if __name__ == "__main__":
    raise SystemExit(main())
