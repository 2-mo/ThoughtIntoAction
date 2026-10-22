#!/usr/bin/env python3
"""CLI for D455-guided pick, +45 degree rotation, and place."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

from legacy.arx5_control import ArmController, ControllerMode
from legacy.arx5_control.gripper_calibration import load_gripper_open_readout
from embodied.devices.robots.arx5.paths import OUTPUTS_ROOT

from legacy.cube.config import DEFAULT_CONFIG_PATH, load_config, validate_config
from legacy.cube.perception import capture_cube_observation, capture_cube_scene_observation
from legacy.cube.planner import (
    Action,
    build_action_plan,
    execute_action_plan,
    preflight_action_plan,
    validate_initial_pose,
)


CONFIRMATION = "EXECUTE_PICK_ROTATE_PLACE"
DEFAULT_DEBUG_IMAGE = OUTPUTS_ROOT / "cube-pick" / "last-observation.jpg"
DEFAULT_SCENE_JSON = OUTPUTS_ROOT / "cube-pick" / "last-scene.json"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="D455 检测 30 mm 方块，夹取后绕桌面法向旋转 45 度并放回。"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--rgb-only", action="store_true", help="禁用深度流，验证 RGB-only 回退")
    parser.add_argument(
        "--scan-all",
        action="store_true",
        help="扫描全部稳定方块并保存初始三维场景；不会连接机械臂",
    )
    parser.add_argument(
        "--scene-json",
        default=str(DEFAULT_SCENE_JSON),
        help="--scan-all 的场景快照输出；传空字符串可禁用",
    )
    parser.add_argument("--execute", action="store_true", help="连接机械臂并执行；默认仅感知和打印计划")
    parser.add_argument("--confirm", help=f"非交互确认词，必须精确为 {CONFIRMATION}")
    parser.add_argument("--interface", help="覆盖 SocketCAN 接口")
    parser.add_argument("--rotation-deg", type=float, help="覆盖放置旋转角，默认 +45")
    parser.add_argument("--place-offset-x-mm", type=float, help="放置点相对检测点的基座 X 偏移")
    parser.add_argument("--place-offset-y-mm", type=float, help="放置点相对检测点的基座 Y 偏移")
    parser.add_argument("--table-z-mm", type=float, help="覆盖桌面在基座坐标系中的高度")
    parser.add_argument("--tcp-z-offset-mm", type=float, help="实测指尖中心相对 eef_link 的 Z 修正")
    parser.add_argument("--speed-percent", type=float, help="覆盖机械臂速度百分比，范围 1..100")
    parser.add_argument(
        "--debug-image",
        default=str(DEFAULT_DEBUG_IMAGE),
        help="保存最后一帧检测结果；传空字符串可禁用",
    )
    return parser.parse_args(argv)


def _apply_overrides(config, args: argparse.Namespace):
    motion = config.motion
    table = config.table
    if args.rotation_deg is not None:
        if not math.isfinite(args.rotation_deg) or abs(args.rotation_deg) > 360.0:
            raise ValueError("rotation-deg must be finite and within +/-360 degrees")
        motion = replace(motion, rotation_rad=math.radians(args.rotation_deg))
    if args.place_offset_x_mm is not None:
        if not math.isfinite(args.place_offset_x_mm):
            raise ValueError("place-offset-x-mm must be finite")
        motion = replace(motion, place_offset_x_m=args.place_offset_x_mm / 1000.0)
    if args.place_offset_y_mm is not None:
        if not math.isfinite(args.place_offset_y_mm):
            raise ValueError("place-offset-y-mm must be finite")
        motion = replace(motion, place_offset_y_m=args.place_offset_y_mm / 1000.0)
    if args.tcp_z_offset_mm is not None:
        if not math.isfinite(args.tcp_z_offset_mm) or abs(args.tcp_z_offset_mm) > 50.0:
            raise ValueError("tcp-z-offset-mm must be finite and within +/-50 mm")
        motion = replace(motion, tcp_z_offset_m=args.tcp_z_offset_mm / 1000.0)
    if args.table_z_mm is not None:
        if not math.isfinite(args.table_z_mm):
            raise ValueError("table-z-mm must be finite")
        table = replace(table, top_z_m=args.table_z_mm / 1000.0)
    if args.speed_percent is not None:
        if (
            not math.isfinite(args.speed_percent)
            or not 1.0 <= args.speed_percent <= 100.0
        ):
            raise ValueError("speed-percent must be finite and within 1..100")
        motion = replace(motion, velocity_scale=args.speed_percent / 100.0)
    result = replace(
        config,
        interface=config.interface if args.interface is None else args.interface,
        table=table,
        motion=motion,
    )
    validate_config(result)
    return result


def _action_description(action: Action) -> str:
    if action.kind == "move" and action.target_xyz_m is not None:
        x, y, z = action.target_xyz_m
        return f"{action.label}: TCP=({1000*x:.1f}, {1000*y:.1f}, {1000*z:.1f}) mm"
    if action.kind == "gripper" and action.gripper_width_m is not None:
        return f"{action.label}: width={1000*action.gripper_width_m:.1f} mm"
    if action.kind == "wait":
        return f"{action.label}: {float(action.wait_s or 0.0):.1f} s"
    return action.label


def _print_observation(result) -> None:
    observation = result.observation
    x, y, z = observation.base_xyz_m
    print("\n稳定方块观测：")
    print(
        f"  base xyz = ({1000*x:.1f}, {1000*y:.1f}, {1000*z:.1f}) mm\n"
        f"  yaw = {math.degrees(observation.yaw_rad):+.2f} deg (正方形方向按 90 deg 等价)\n"
        f"  measured edge = {1000*observation.size_m:.1f} mm\n"
        f"  source = {observation.position_source}; depth stream = "
        f"{'available' if result.depth_available else 'RGB-only'}\n"
        f"  samples = {observation.accepted_samples}, depth checks = {observation.depth_samples}, "
        f"xy RMS = {1000*observation.xy_rms_m:.2f} mm, "
        f"yaw RMS = {math.degrees(observation.yaw_rms_rad):.2f} deg"
    )
    if result.debug_image_path is not None:
        print(f"  debug image = {result.debug_image_path}")


def _print_scene(result) -> None:
    print(f"\n稳定场景观测：{len(result.observations)} 个方块")
    for index, observation in enumerate(result.observations, start=1):
        x, y, z = observation.base_xyz_m
        name = observation.candidate_id or f"cube-{index}"
        color = "" if observation.color is None else f", color={observation.color}"
        print(
            f"  {name}{color}: base xyz=({1000*x:.1f}, {1000*y:.1f}, {1000*z:.1f}) mm, "
            f"yaw={math.degrees(observation.yaw_rad):+.2f} deg, "
            f"xy RMS={1000*observation.xy_rms_m:.2f} mm, "
            f"depth checks={observation.depth_samples}"
        )
    print(
        f"  frames = {result.frames_captured}; depth stream = "
        f"{'available' if result.depth_available else 'RGB-only'}"
    )
    if result.debug_image_path is not None:
        print(f"  debug image = {result.debug_image_path}")


def _write_scene_snapshot(result, path: Path) -> Path:
    destination = path.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(result.as_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return destination


def _confirmation_value(args: argparse.Namespace) -> str:
    if args.confirm is not None:
        return args.confirm
    if not sys.stdin.isatty():
        return ""
    print("\n确认机械臂运动包络内无人、无障碍，物理急停可用。")
    return input(f"输入 {CONFIRMATION} 开始：").strip()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.scan_all and args.execute:
            raise ValueError("--scan-all cannot be combined with --execute")
        config = _apply_overrides(load_config(args.config), args)
        debug_path = None if not args.debug_image else Path(args.debug_image)
        if args.scan_all:
            print(
                "正在独占 D455 获取全部稳定方块的初始场景"
                + ("（强制 RGB-only）..." if args.rgb_only else "（深度仅做一致性校验）...")
            )
            scene = capture_cube_scene_observation(
                config,
                rgb_only=args.rgb_only,
                debug_image_path=debug_path,
            )
            _print_scene(scene)
            if args.scene_json:
                snapshot_path = _write_scene_snapshot(scene, Path(args.scene_json))
                print(f"  scene snapshot = {snapshot_path}")
            print("\n场景扫描完成，未连接机械臂。")
            return 0
        print(
            "正在独占 D455 获取稳定方块观测"
            + ("（强制 RGB-only）..." if args.rgb_only else "（深度仅做一致性校验）...")
        )
        capture = capture_cube_observation(
            config,
            rgb_only=args.rgb_only,
            debug_image_path=debug_path,
        )
        _print_observation(capture)
        actions = build_action_plan(capture.observation, config)
        print("\n动作计划（放置点默认为原位置）：")
        for index, action in enumerate(actions, start=1):
            print(f"  {index:02d}. {_action_description(action)}")
        print(
            "\n配置摘要："
            + json.dumps(
                {
                    "rotation_deg": math.degrees(config.motion.rotation_rad),
                    "place_offset_mm": [
                        1000.0 * config.motion.place_offset_x_m,
                        1000.0 * config.motion.place_offset_y_m,
                    ],
                    "closing_torque_nm": config.motion.closing_torque_nm,
                    "velocity_scale": config.motion.velocity_scale,
                },
                ensure_ascii=False,
            )
        )
    except (OSError, RuntimeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"实验准备失败：{exc}", file=sys.stderr)
        return 1

    if not args.execute:
        print("\n预览完成，未连接机械臂。确认坐标后添加 --execute 才会真实运动。")
        return 0

    controller = ArmController(
        interface=config.interface,
        gripper_gain_scale=1.0,
        gripper_open_readout=load_gripper_open_readout(interface=config.interface),
    )
    try:
        print(f"\n正在连接 {config.interface} 并保持当前姿态...")
        initial = controller.connect(stabilize=True)
        validate_initial_pose(initial.eef_pose_6d, config)
        preflight = preflight_action_plan(controller, actions)
        print("完整连续 IK 预检通过：")
        for item in preflight:
            print(
                f"  {item.label}: max joint delta "
                f"{math.degrees(item.maximum_joint_delta_rad):.1f} deg, "
                f"min TCP z {1000.0 * item.minimum_tcp_z_m:.1f} mm"
            )
        if _confirmation_value(args) != CONFIRMATION:
            print("确认词不匹配，已取消并进入阻尼。", file=sys.stderr)
            controller.damping()
            return 2
        execute_action_plan(controller, actions, config.motion)
        print("实验完成：方块已旋转并放下，机械臂已进入阻尼。")
        return 0
    except KeyboardInterrupt:
        if controller.is_connected:
            controller.emergency_damping("cube experiment interrupted")
        print("收到 Ctrl+C，已执行紧急阻尼。", file=sys.stderr)
        return 130
    except BaseException as exc:
        if controller.is_connected:
            controller.emergency_damping(f"cube experiment failed: {exc}")
        print(f"实验失败，已执行紧急阻尼：{exc}", file=sys.stderr)
        return 1
    finally:
        controller.close(emergency=controller.mode == ControllerMode.FAULT)


if __name__ == "__main__":
    raise SystemExit(main())
