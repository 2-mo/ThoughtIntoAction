#!/usr/bin/env python3
"""Legacy single-run motion test; prefer the persistent control CLI.

The default motion rotates joint 6 by +5 degrees over 3 seconds, holds for
0.5 seconds, and returns to the measured starting pose over 3 seconds.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import arx5_interface as arx5
import numpy as np


MODEL = "X5"
INTERFACE = "can0"
MAX_TEST_ANGLE_DEG = 30.0
TRAJECTORY_RATE_HZ = 50
NON_MOVING_JOINT_TOLERANCE_RAD = math.radians(5.0)
MOVING_JOINT_EXTRA_TOLERANCE_RAD = math.radians(2.0)
VELOCITY_ABORT_THRESHOLD_RAD_S = 0.5
TORQUE_ABORT_RATIO = 0.8
GAIN_UPDATE_RATE_HZ = 100
POST_RETURN_SETTLE_S = 0.5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="让 ARX X5 的一个关节轻微移动后回到起点。"
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=6,
        help="要移动的关节编号，范围 1~6（默认：6）",
    )
    parser.add_argument(
        "--degrees",
        type=float,
        default=5.0,
        help="相对移动角度，负数表示反向（默认：+5.0 度，最大绝对值 30 度）",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="向目标运动的时间，单位秒（默认：3.0，最短 1.0）",
    )
    parser.add_argument(
        "--return-duration",
        type=float,
        default=3.0,
        help="缓慢返回起点的时间，单位秒（默认：3.0，最短 1.0）",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=0.5,
        help="到达目标后的停留时间，单位秒（默认：0.5）",
    )
    parser.add_argument(
        "--gain-scale",
        type=float,
        default=0.5,
        help="位置增益相对 SDK 默认值的比例（默认：0.5）",
    )
    parser.add_argument(
        "--gain-ramp",
        type=float,
        default=1.5,
        help="软启动和软停止的增益渐变时间，单位秒（默认：1.5）",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="运动前要求输入 MOVE；默认不询问",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只显示计划，不连接或驱动机械臂",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 1 <= args.joint <= 6:
        raise ValueError("--joint 必须在 1~6 之间")
    if not math.isfinite(args.degrees) or abs(args.degrees) > MAX_TEST_ANGLE_DEG:
        raise ValueError(
            f"--degrees 必须是有限数，且绝对值不超过 {MAX_TEST_ANGLE_DEG} 度"
        )
    if abs(args.degrees) < 0.1:
        raise ValueError("--degrees 的绝对值至少为 0.1 度")
    if not math.isfinite(args.duration) or not 1.0 <= args.duration <= 10.0:
        raise ValueError("--duration 必须在 1.0~10.0 秒之间")
    if (
        not math.isfinite(args.return_duration)
        or not 1.0 <= args.return_duration <= 10.0
    ):
        raise ValueError("--return-duration 必须在 1.0~10.0 秒之间")
    if not math.isfinite(args.hold) or not 0.0 <= args.hold <= 5.0:
        raise ValueError("--hold 必须在 0.0~5.0 秒之间")
    if not math.isfinite(args.gain_scale) or not 0.1 <= args.gain_scale <= 1.0:
        raise ValueError("--gain-scale 必须在 0.1~1.0 之间")
    if not math.isfinite(args.gain_ramp) or not 0.5 <= args.gain_ramp <= 5.0:
        raise ValueError("--gain-ramp 必须在 0.5~5.0 秒之间")


def make_joint_state(
    position: np.ndarray, gripper_position: float, timestamp: float
) -> arx5.JointState:
    zeros = np.zeros(position.size, dtype=np.float64)
    command = arx5.JointState(
        position.astype(np.float64, copy=True),
        zeros.copy(),
        zeros.copy(),
        float(gripper_position),
        0.0,
        0.0,
    )
    command.timestamp = float(timestamp)
    return command


def cosine_fraction(alpha: float) -> float:
    """Zero-slope interpolation fraction in the range [0, 1]."""
    return 0.5 - 0.5 * math.cos(math.pi * alpha)


def ramp_position_gain(
    controller: arx5.Arx5JointController,
    controller_config: arx5.ControllerConfig,
    start_scale: float,
    end_scale: float,
    duration: float,
) -> None:
    """Smoothly change arm position stiffness while retaining damping."""
    step_count = max(2, int(math.ceil(duration * GAIN_UPDATE_RATE_HZ)))
    start_time = time.monotonic()

    for step in range(1, step_count + 1):
        alpha = cosine_fraction(step / step_count)
        scale = start_scale + (end_scale - start_scale) * alpha
        gain = arx5.Gain(
            controller_config.default_kp * scale,
            controller_config.default_kd,
            0.0,  # Do not stiffen the gripper during an arm-only test.
            controller_config.default_gripper_kd,
        )
        controller.set_gain(gain)

        deadline = start_time + duration * step / step_count
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)


def build_round_trip_trajectory(
    start_position: np.ndarray,
    gripper_position: float,
    joint_index: int,
    delta_rad: float,
    start_timestamp: float,
    duration: float,
    return_duration: float,
    hold: float,
) -> tuple[list[arx5.JointState], np.ndarray, float]:
    target_position = start_position.copy()
    target_position[joint_index] += delta_rad

    step_count = max(2, int(math.ceil(duration * TRAJECTORY_RATE_HZ)))
    trajectory: list[arx5.JointState] = []

    for step in range(1, step_count + 1):
        alpha = step / step_count
        position = start_position.copy()
        position[joint_index] += delta_rad * cosine_fraction(alpha)
        timestamp = start_timestamp + duration * alpha
        trajectory.append(make_joint_state(position, gripper_position, timestamp))

    if hold > 0.0:
        trajectory.append(
            make_joint_state(
                target_position,
                gripper_position,
                start_timestamp + duration + hold,
            )
        )

    return_start = start_timestamp + duration + hold
    return_step_count = max(
        2, int(math.ceil(return_duration * TRAJECTORY_RATE_HZ))
    )
    for step in range(1, return_step_count + 1):
        alpha = step / return_step_count
        position = target_position.copy()
        position[joint_index] -= delta_rad * cosine_fraction(alpha)
        timestamp = return_start + return_duration * alpha
        trajectory.append(make_joint_state(position, gripper_position, timestamp))

    finish_timestamp = return_start + return_duration
    return trajectory, target_position, finish_timestamp


def check_feedback(
    state: arx5.JointState,
    start_position: np.ndarray,
    joint_index: int,
    delta_rad: float,
    torque_limits: np.ndarray,
) -> None:
    position = state.pos()
    velocity = state.vel()
    torque = state.torque()

    if not all(np.all(np.isfinite(values)) for values in (position, velocity, torque)):
        raise RuntimeError("反馈中出现 NaN 或无穷值")

    if np.max(np.abs(velocity)) > VELOCITY_ABORT_THRESHOLD_RAD_S:
        raise RuntimeError(
            f"关节速度超过保护阈值：{np.max(np.abs(velocity)):.3f} rad/s"
        )

    if np.any(np.abs(torque) > torque_limits * TORQUE_ABORT_RATIO):
        bad_joint = int(np.argmax(np.abs(torque) / torque_limits))
        raise RuntimeError(
            f"关节 {bad_joint + 1} 扭矩接近限值：{torque[bad_joint]:.3f} N·m"
        )

    deviation = np.abs(position - start_position)
    non_moving = np.ones(position.size, dtype=bool)
    non_moving[joint_index] = False
    if np.any(deviation[non_moving] > NON_MOVING_JOINT_TOLERANCE_RAD):
        bad_joint = int(np.argmax(deviation * non_moving))
        raise RuntimeError(f"非目标关节 {bad_joint + 1} 发生异常偏移")

    moving_limit = abs(delta_rad) + MOVING_JOINT_EXTRA_TOLERANCE_RAD
    if deviation[joint_index] > moving_limit:
        raise RuntimeError(f"目标关节 {joint_index + 1} 超出预期运动范围")


def print_plan(args: argparse.Namespace) -> None:
    direction = "+" if args.degrees >= 0 else ""
    print("运动计划：")
    print(f"  型号/接口：{MODEL} / {INTERFACE}")
    print(f"  目标关节：J{args.joint}")
    print(f"  相对角度：{direction}{args.degrees:.3f}°")
    print(f"  前往时间：{args.duration:.3f} s")
    print(f"  目标停留：{args.hold:.3f} s")
    print(f"  返回时间：{args.return_duration:.3f} s")
    print(f"  位置增益：SDK 默认值的 {args.gain_scale:.0%}")
    print(f"  软启动/停止：各 {args.gain_ramp:.3f} s")
    print(
        "  总流程时间："
        f"{2 * args.gain_ramp + args.duration + args.hold + args.return_duration + POST_RETURN_SETTLE_S:.3f} s"
    )
    print("  结束状态：返回实测起点并进入阻尼模式")


def run_motion(args: argparse.Namespace) -> None:
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(MODEL)
    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot_config.joint_dof
    )
    controller_config.background_send_recv = True
    controller_config.interpolation_method = "linear"

    controller: arx5.Arx5JointController | None = None
    try:
        print("正在连接机械臂；连接后暂时保持阻尼模式……")
        controller = arx5.Arx5JointController(
            robot_config, controller_config, INTERFACE
        )
        time.sleep(0.5)

        start_state = controller.get_joint_state()
        start_position = start_state.pos().copy()
        gripper_position = float(start_state.gripper_pos)
        joint_index = args.joint - 1
        delta_rad = math.radians(args.degrees)
        target_position = start_position.copy()
        target_position[joint_index] += delta_rad

        if not np.all(np.isfinite(start_position)):
            raise RuntimeError("无法读取有效的初始关节位置")

        lower = robot_config.joint_pos_min[joint_index]
        upper = robot_config.joint_pos_max[joint_index]
        limit_margin = math.radians(1.0)
        if not lower + limit_margin <= target_position[joint_index] <= upper - limit_margin:
            raise RuntimeError(
                f"J{args.joint} 目标 {math.degrees(target_position[joint_index]):.3f}° "
                "距离软件限位太近"
            )

        print("当前关节角度（度）：", np.round(np.degrees(start_position), 3))
        print(
            f"J{args.joint} 将从 {math.degrees(start_position[joint_index]):.3f}° "
            f"移动到 {math.degrees(target_position[joint_index]):.3f}°，然后返回。"
        )

        if args.confirm:
            print("请确认机械臂周围无人、无障碍物，并保持手可触及急停。")
            confirmation = input("输入 MOVE 后按回车开始；其他输入取消：").strip()
            if confirmation != "MOVE":
                print("已取消，机械臂未执行主动运动。")
                return

        # Pin the controller command to the measured pose before enabling
        # position gains, preventing a jump to a stale/default target.
        hold_command = make_joint_state(start_position, gripper_position, 0.0)
        controller.set_joint_cmd(hold_command)
        print(
            f"正在用 {args.gain_ramp:.1f} 秒缓慢增加位置增益至 "
            f"{args.gain_scale:.0%}……"
        )
        ramp_position_gain(
            controller,
            controller_config,
            start_scale=0.0,
            end_scale=args.gain_scale,
            duration=args.gain_ramp,
        )

        trajectory_start = controller.get_timestamp() + 0.2
        trajectory, _, finish_timestamp = build_round_trip_trajectory(
            start_position=start_position,
            gripper_position=gripper_position,
            joint_index=joint_index,
            delta_rad=delta_rad,
            start_timestamp=trajectory_start,
            duration=args.duration,
            return_duration=args.return_duration,
            hold=args.hold,
        )
        controller.set_joint_traj(trajectory)
        print("开始运动；按 Ctrl+C 可立即转入阻尼模式。")

        while controller.get_timestamp() < finish_timestamp + 0.1:
            state = controller.get_joint_state()
            check_feedback(
                state,
                start_position,
                joint_index,
                delta_rad,
                robot_config.joint_torque_max,
            )
            time.sleep(0.02)

        settle_deadline = time.monotonic() + POST_RETURN_SETTLE_S
        while time.monotonic() < settle_deadline:
            state = controller.get_joint_state()
            check_feedback(
                state,
                start_position,
                joint_index,
                delta_rad,
                robot_config.joint_torque_max,
            )
            time.sleep(0.02)

        final_state = controller.get_joint_state()
        final_position = final_state.pos().copy()
        final_error_deg = np.degrees(final_position - start_position)
        print("运动完成，最终关节角度（度）：", np.round(np.degrees(final_position), 3))
        print("相对起点误差（度）：", np.round(final_error_deg, 3))
        print(f"正在用 {args.gain_ramp:.1f} 秒缓慢释放位置增益……")
        ramp_position_gain(
            controller,
            controller_config,
            start_scale=args.gain_scale,
            end_scale=0.0,
            duration=args.gain_ramp,
        )

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在进入阻尼模式……", file=sys.stderr)
    finally:
        if controller is not None:
            try:
                controller.set_to_damping()
            except Exception as exc:
                print(f"设置阻尼模式时发生错误：{exc}", file=sys.stderr)
            del controller


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        print_plan(args)
        if args.dry_run:
            print("dry-run 完成：未连接机械臂，也未发送控制命令。")
            return 0
        run_motion(args)
        return 0
    except (ValueError, RuntimeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
