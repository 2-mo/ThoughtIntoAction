#!/usr/bin/env python3
"""Interactive, persistent command line controller for one ARX X5 arm."""

from __future__ import annotations

import argparse
import math
import shlex
import sys
import time

import numpy as np

from arx5_control import (
    ArmController,
    ArmSnapshot,
    BusyError,
    ControlError,
    ControllerMode,
    GripperForceResult,
    GripperMotionResult,
    MotionResult,
    SafetyError,
)
from arx5_control.gripper_calibration import load_gripper_open_readout


HELP_TEXT = """
命令：
  state
      读取关节、夹爪和末端状态。

  move <关节1-6> <相对角度deg> [时间s]
      移动到新位置并保持。例如：move 6 10 3

  pulse <关节1-6> <相对角度deg> [前往s] [返回s] [停留s]
      移动后缓慢返回起点。例如：pulse 6 30 5 5 0.5

  grip <宽度mm> [时间s]
      将夹爪平滑移动到绝对开口宽度。例如：grip 40 2

  grip-pulse [打开宽度mm] [打开s] [关闭s] [停留s]
      打开后缓慢关闭；默认：40 mm、2 s、2 s、停留 0.5 s。

  grip-force [闭合扭矩Nm] [最小宽度mm] [超时s] [接触保持s]
      限扭矩闭合，位置只作行程下限；默认：0.15 Nm、0 mm、5 s、0.5 s。

  hold
      从当前实测姿态软启动位置保持。

  damping
      软释放位置增益并进入阻尼模式。

  clear
      安全故障后重新检查反馈，清除锁存故障；不会主动运动。

  mode
      显示当前控制模式和增益比例。

  help
      显示本帮助。

  quit
      软释放增益、进入阻尼并退出。
""".strip()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ARX X5 单臂长驻底层控制终端（不会自动回零）"
    )
    parser.add_argument("--model", default="X5", help="机械臂型号（默认：X5）")
    parser.add_argument("--interface", default="can0", help="CAN 接口（默认：can0）")
    parser.add_argument(
        "--gain-scale",
        type=float,
        default=0.5,
        help="位置增益占 SDK 默认值的比例（默认：0.5）",
    )
    parser.add_argument(
        "--gain-ramp",
        type=float,
        default=1.5,
        help="软启动/软停止时间，单位秒（默认：1.5）",
    )
    parser.add_argument(
        "--gripper-gain-scale",
        type=float,
        default=0.5,
        help="夹爪位置增益占 SDK 默认值的比例（默认：0.5）",
    )
    parser.add_argument(
        "--gripper-gain-ramp",
        type=float,
        default=1.0,
        help="夹爪增益软启动时间，单位秒（默认：1.0）",
    )
    parser.add_argument(
        "--gripper-open-readout",
        type=float,
        default=None,
        help=(
            "仅本进程覆盖夹爪全开电机读数；默认读取持久化夹爪标定"
        ),
    )
    return parser.parse_args()


def print_state(snapshot: ArmSnapshot) -> None:
    np.set_printoptions(precision=3, suppress=True)
    print("关节位置 (deg)：", np.degrees(snapshot.joint_position))
    print("关节速度 (rad/s)：", snapshot.joint_velocity)
    print("关节扭矩 (N·m)：", snapshot.joint_torque)
    print(
        "夹爪："
        f"pos={snapshot.gripper_position:.6f} m, "
        f"vel={snapshot.gripper_velocity:.6f} m/s, "
        f"torque={snapshot.gripper_torque:.6f} N·m"
    )
    print("末端 pose_6d：", snapshot.eef_pose_6d)
    print(f"控制器时间：{snapshot.timestamp:.3f} s")


def print_result(result: MotionResult) -> None:
    print("动作完成。")
    print("  起点 (deg)：", np.round(np.degrees(result.start_position), 3))
    print("  终点 (deg)：", np.round(np.degrees(result.final_position), 3))
    print("  终点误差 (deg)：", np.round(np.degrees(result.final_error_rad), 3))
    print(
        "  最大实测速度 (rad/s)：",
        np.round(result.max_abs_velocity_rad_s, 3),
    )
    print("  最大实测扭矩 (N·m)：", np.round(result.max_abs_torque_nm, 3))


def print_gripper_result(result: GripperMotionResult) -> None:
    print("夹爪动作完成。")
    print(f"  起点：{1000.0 * result.start_width_m:.2f} mm")
    print(f"  动作最大目标：{1000.0 * result.excursion_width_m:.2f} mm")
    print(f"  最终目标：{1000.0 * result.target_width_m:.2f} mm")
    print(f"  最终实测：{1000.0 * result.final_width_m:.2f} mm")
    print(f"  最终误差：{1000.0 * result.final_error_m:+.2f} mm")
    print(
        "  实测范围："
        f"[{1000.0 * result.min_measured_width_m:.2f}, "
        f"{1000.0 * result.max_measured_width_m:.2f}] mm"
    )
    print(f"  最大实测速度：{result.max_abs_velocity_m_s:.3f} m/s")
    print(f"  最大实测扭矩：{result.max_abs_torque_nm:.3f} N·m")


def print_gripper_force_result(result: GripperForceResult) -> None:
    reasons = {
        "contact": "检测到停转/接触",
        "minimum_width": "到达最小开度",
        "timeout": "到达超时",
    }
    print("限力闭合完成。")
    print(f"  起点：{1000.0 * result.start_width_m:.2f} mm")
    print(f"  终点：{1000.0 * result.final_width_m:.2f} mm")
    print(f"  停止原因：{reasons.get(result.stop_reason, result.stop_reason)}")
    print(f"  设定闭合扭矩：{result.commanded_closing_torque_nm:.3f} N·m")
    print(f"  最大实测扭矩：{result.max_abs_torque_nm:.3f} N·m")
    print(f"  最大实测速度：{result.max_abs_velocity_m_s:.3f} m/s")
    print(
        "  结束状态："
        + ("当前位置保持" if result.position_hold_enabled else "零扭矩")
    )


def parse_joint_and_angle(parts: list[str]) -> tuple[int, float, float]:
    if len(parts) not in (3, 4):
        raise ValueError("用法：move <关节1-6> <相对角度deg> [时间s]")
    joint = int(parts[1])
    degrees = float(parts[2])
    duration = float(parts[3]) if len(parts) == 4 else max(2.0, abs(degrees) / 6.0)
    return joint, degrees, duration


def handle_command(controller: ArmController, line: str) -> bool:
    parts = shlex.split(line)
    if not parts:
        return True
    command = parts[0].lower()

    if command in ("quit", "exit", "q"):
        return False
    if command in ("help", "h", "?"):
        print(HELP_TEXT)
        return True
    if command in ("state", "status"):
        print_state(controller.get_state())
        return True
    if command == "mode":
        print(
            f"模式：{controller.mode.value}，"
            f"机械臂增益：{controller.current_arm_gain_scale:.1%}，"
            f"夹爪增益：{controller.current_gripper_gain_scale:.1%}"
        )
        if controller.fault_reason:
            print(f"故障原因：{controller.fault_reason}")
        return True
    if command in ("hold", "enable", "active"):
        print(f"正在用 {controller.gain_ramp_s:.1f}s 软启动位置保持……")
        snapshot = controller.hold_current()
        print("已保持当前姿态：", np.round(np.degrees(snapshot.joint_position), 3))
        return True
    if command in ("damping", "damp"):
        print(f"正在用 {controller.gain_ramp_s:.1f}s 软释放位置增益……")
        controller.damping()
        print("已进入阻尼模式。")
        return True
    if command == "clear":
        controller.clear_fault()
        print("故障已清除；当前为阻尼模式。")
        return True
    if command == "move":
        joint, degrees, duration = parse_joint_and_angle(parts)
        delta_rad = math.radians(degrees)
        required = controller.minimum_duration(delta_rad)
        if duration < required:
            raise ValueError(
                f"时间太短；当前速度上限要求至少 {required:.2f}s"
            )
        if controller.mode == ControllerMode.DAMPING:
            print(f"首次动作前先用 {controller.gain_ramp_s:.1f}s 软启动……")
        print(f"J{joint} 相对移动 {degrees:+.3f}°，用时 {duration:.3f}s。")
        start = controller.get_state().joint_position
        target = start.copy()
        target[joint - 1] += delta_rad
        velocity_scale = min(1.0, required / duration)
        result = controller.movej(target, velocity_scale=velocity_scale)
        print_result(result)
        print("控制器保持 active；下一条动作不会重新初始化。")
        return True
    if command == "pulse":
        if not 3 <= len(parts) <= 6:
            raise ValueError(
                "用法：pulse <关节1-6> <相对角度deg> "
                "[前往s] [返回s] [停留s]"
            )
        joint = int(parts[1])
        degrees = float(parts[2])
        default_duration = max(2.0, abs(degrees) / 6.0)
        outward = float(parts[3]) if len(parts) >= 4 else default_duration
        returning = float(parts[4]) if len(parts) >= 5 else outward
        hold = float(parts[5]) if len(parts) >= 6 else 0.5
        delta_rad = math.radians(degrees)
        required = controller.minimum_duration(delta_rad)
        if outward < required or returning < required:
            raise ValueError(
                f"前往和返回时间都至少需要 {required:.2f}s"
            )
        if controller.mode == ControllerMode.DAMPING:
            print(f"首次动作前先用 {controller.gain_ramp_s:.1f}s 软启动……")
        print(
            f"J{joint} 往返 {degrees:+.3f}°："
            f"前往 {outward:.3f}s，停留 {hold:.3f}s，返回 {returning:.3f}s。"
        )
        start = controller.get_state().joint_position
        target = start.copy()
        target[joint - 1] += delta_rad
        controller.movej(target, velocity_scale=min(1.0, required / outward))
        time.sleep(hold)
        result = controller.movej(
            start,
            velocity_scale=min(1.0, required / returning),
        )
        print_result(result)
        print("已回到本次起点并继续保持 active。")
        return True
    if command == "grip":
        if len(parts) not in (2, 3):
            raise ValueError("用法：grip <宽度mm> [时间s]")
        target_m = float(parts[1]) / 1000.0
        current_m = controller.get_state().gripper_position
        required = controller.minimum_gripper_duration(target_m - current_m)
        duration = float(parts[2]) if len(parts) == 3 else max(2.0, required)
        if duration < required:
            raise ValueError(
                f"时间太短；当前夹爪速度上限要求至少 {required:.2f}s"
            )
        if controller.mode == ControllerMode.DAMPING:
            print(
                "首次夹爪动作会先软启动机械臂保持和夹爪增益："
                f"{controller.gain_ramp_s:.1f}s + "
                f"{controller.gripper_gain_ramp_s:.1f}s。"
            )
        print(f"夹爪移动到 {1000.0 * target_m:.2f} mm，用时 {duration:.2f}s。")
        print_gripper_result(controller.move_gripper(target_m, duration))
        print("控制器保持 active；夹爪继续保持最终开口。")
        return True
    if command in ("grip-pulse", "grip_pulse"):
        if not 1 <= len(parts) <= 5:
            raise ValueError(
                "用法：grip-pulse [打开宽度mm] [打开s] [关闭s] [停留s]"
            )
        open_m = (float(parts[1]) if len(parts) >= 2 else 40.0) / 1000.0
        open_duration = float(parts[2]) if len(parts) >= 3 else 2.0
        close_duration = float(parts[3]) if len(parts) >= 4 else 2.0
        hold = float(parts[4]) if len(parts) >= 5 else 0.5
        current_m = controller.get_state().gripper_position
        open_required = controller.minimum_gripper_duration(open_m - current_m)
        close_required = controller.minimum_gripper_duration(-open_m)
        if open_duration < open_required or close_duration < close_required:
            raise ValueError(
                "打开/关闭时间太短；分别至少需要 "
                f"{open_required:.2f}s / {close_required:.2f}s"
            )
        if controller.mode == ControllerMode.DAMPING:
            print(
                "首次夹爪动作会先软启动机械臂保持和夹爪增益："
                f"{controller.gain_ramp_s:.1f}s + "
                f"{controller.gripper_gain_ramp_s:.1f}s。"
            )
        print(
            f"夹爪打开到 {1000.0 * open_m:.2f} mm："
            f"打开 {open_duration:.2f}s，停留 {hold:.2f}s，"
            f"关闭 {close_duration:.2f}s。"
        )
        result = controller.open_close_gripper(
            open_m,
            open_duration_s=open_duration,
            hold_s=hold,
            close_duration_s=close_duration,
        )
        print_gripper_result(result)
        print("夹爪已缓慢关闭，控制器继续保持 active。")
        return True
    if command in ("grip-force", "grip_force"):
        if not 1 <= len(parts) <= 5:
            raise ValueError(
                "用法：grip-force [闭合扭矩Nm] [最小宽度mm] "
                "[超时s] [接触保持s]"
            )
        torque = float(parts[1]) if len(parts) >= 2 else 0.15
        minimum_width = (
            float(parts[2]) / 1000.0 if len(parts) >= 3 else 0.0
        )
        timeout = float(parts[3]) if len(parts) >= 4 else 5.0
        contact_hold = float(parts[4]) if len(parts) >= 5 else 0.5
        if controller.mode == ControllerMode.DAMPING:
            print(f"先用 {controller.gain_ramp_s:.1f}s 软启动机械臂保持……")
        print(
            f"夹爪限力闭合：{torque:.3f} N·m，"
            f"最小 {1000.0 * minimum_width:.1f} mm，"
            f"超时 {timeout:.1f}s。"
        )
        result = controller.force_close_gripper(
            torque,
            minimum_width_m=minimum_width,
            timeout_s=timeout,
            contact_hold_s=contact_hold,
        )
        print_gripper_force_result(result)
        return True

    raise ValueError(f"未知命令：{parts[0]}；输入 help 查看命令")


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

    print(f"正在连接 {args.model} / {args.interface}；不会自动回零……")
    try:
        initial = controller.connect()
    except BusyError as exc:
        print(f"无法连接：{exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"连接失败：{exc}", file=sys.stderr)
        return 1

    print("连接成功，当前为阻尼模式。")
    print("当前关节角度 (deg)：", np.round(np.degrees(initial.joint_position), 3))
    print("输入 help 查看命令；Ctrl+C 会立即进入故障阻尼。")

    try:
        keep_running = True
        while keep_running:
            try:
                line = input(f"arx5[{controller.mode.value}]> ")
                keep_running = handle_command(controller, line)
            except KeyboardInterrupt:
                print("\n收到 Ctrl+C，立即进入阻尼并锁存故障。")
                controller.emergency_damping("keyboard interrupt")
            except EOFError:
                print()
                break
            except (ValueError, SafetyError, ControlError) as exc:
                print(f"错误：{exc}", file=sys.stderr)
            except Exception as exc:
                controller.emergency_damping(f"unexpected error: {exc}")
                print(f"未预期错误，已进入故障阻尼：{exc}", file=sys.stderr)
    finally:
        print("正在关闭控制器；若当前 active，将先软释放位置增益……")
        controller.close(emergency=controller.mode == ControllerMode.FAULT)
        print("控制器已关闭。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
