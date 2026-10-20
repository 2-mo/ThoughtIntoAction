#!/usr/bin/env python3
"""Identify the X5 tool mass or an added TCP payload from static torque."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import arx5_interface as arx5
import numpy as np

from legacy.arx5_control import ArmController, BusyError
from legacy.arx5_control.gripper_calibration import load_gripper_open_readout
from legacy.arx5_control.payload_profiles import (
    list_payload_profiles,
    save_payload_profile,
    select_payload_profile,
)
from embodied.core.arx5_paths import CALIBRATIONS_ROOT


CALIBRATION_ROOT = CALIBRATIONS_ROOT / "payload"
DEFAULT_PLAN_PATH = CALIBRATION_ROOT / "poses.json"
DEFAULT_BASELINE_PATH = CALIBRATION_ROOT / "baseline.json"
DEFAULT_LOADED_PATH = CALIBRATION_ROOT / "loaded.json"
DEFAULT_TOOL_RESULT_PATH = CALIBRATION_ROOT / "tool-result.json"
DEFAULT_PAYLOAD_RESULT_PATH = CALIBRATION_ROOT / "payload-result.json"
TOOL_FIT_JOINT_INDICES = (1, 2, 3)  # J2, J3, J4
PAYLOAD_FIT_JOINT_INDICES = (1, 2, 3, 4, 5)  # J2..J6
MIN_CAPTURE_COUNT = 7
MAX_START_PROXIMAL_ERROR_RAD = math.radians(5.0)
MAX_START_WRIST_ERROR_RAD = math.radians(20.0)


@dataclass(frozen=True)
class StaticCapture:
    label: str
    target_position: np.ndarray
    joint_position: np.ndarray
    joint_torque: np.ndarray
    torque_mad: np.ndarray
    max_abs_velocity: float
    gripper_position_m: float


@dataclass(frozen=True)
class ToolMassFit:
    mass_scale: float
    effective_mass_kg: float
    mass_std_kg: float
    joint_bias_nm: np.ndarray
    rmse_nm: float
    condition_number: float
    used_rows: int
    rejected_rows: int


@dataclass(frozen=True)
class PayloadFit:
    mass_kg: float
    mass_std_kg: float
    center_of_mass_m: np.ndarray
    first_moment_kg_m: np.ndarray
    joint_bias_nm: np.ndarray
    rmse_nm: float
    condition_number: float
    used_rows: int
    rejected_rows: int


def add_capture_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("phase", choices=("baseline", "loaded"))
    parser.add_argument("--interface", default="can0", help="CAN 接口（默认：can0）")
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN_PATH)
    parser.add_argument(
        "--samples",
        type=int,
        default=100,
        help="每个姿态的反馈样本数（默认：100）",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="反馈采样频率 Hz（默认：50）",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=1.0,
        help="到达姿态后的稳定时间 s（默认：1.0）",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.35,
        help="自动回放速度比例（默认：0.35）",
    )
    parser.add_argument(
        "--hold-gripper",
        action="store_true",
        help="采集期间保持开始时的夹爪开度；夹持物体时使用",
    )
    parser.add_argument("--output", type=Path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="示教安全腕部姿态并标定 X5 末端工具或新增 payload"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    teach = commands.add_parser("teach", help="示教一个安全中心并生成腕部激励姿态")
    teach.add_argument("--interface", default="can0", help="CAN 接口（默认：can0）")
    teach.add_argument(
        "--excursion",
        type=float,
        default=15.0,
        help="J4/J5 激励角度 deg（默认：15）",
    )
    teach.add_argument("--output", type=Path, default=DEFAULT_PLAN_PATH)

    capture = commands.add_parser("capture", help="自动回放并采集基线或负载数据")
    add_capture_arguments(capture)

    fit_tool = commands.add_parser(
        "fit-tool",
        help="用 baseline 估计 link6 + 标准夹爪的有效质量",
    )
    fit_tool.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE_PATH)
    fit_tool.add_argument("--output", type=Path, default=DEFAULT_TOOL_RESULT_PATH)

    fit_payload = commands.add_parser(
        "fit-payload",
        help="用 loaded - baseline 估计新增负载的质量与重心",
    )
    fit_payload.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE_PATH)
    fit_payload.add_argument("--loaded", type=Path, default=DEFAULT_LOADED_PATH)
    fit_payload.add_argument("--output", type=Path, default=DEFAULT_PAYLOAD_RESULT_PATH)
    fit_payload.add_argument(
        "--profile",
        default="last",
        help="保存并启用的负载档案名（默认：last）",
    )
    fit_payload.add_argument(
        "--mount-mode",
        choices=("fixed", "grasped"),
        help="负载安装方式；网页标定会写入此来源标记",
    )

    commands.add_parser("profiles", help="列出已缓存的负载档案")
    use_profile = commands.add_parser("use-profile", help="切换当前负载档案")
    use_profile.add_argument("profile", help="档案名；使用 none 表示空载")

    args = parser.parse_args()
    if args.command == "teach":
        if not math.isfinite(args.excursion) or not 8.0 <= args.excursion <= 20.0:
            parser.error("--excursion 必须在 8..20 deg")
    if args.command == "capture":
        if args.samples < 20:
            parser.error("--samples 不能小于 20")
        if not math.isfinite(args.rate) or not 10.0 <= args.rate <= 100.0:
            parser.error("--rate 必须在 10..100 Hz")
        if not math.isfinite(args.settle) or not 0.5 <= args.settle <= 5.0:
            parser.error("--settle 必须在 0.5..5.0 s")
        if not math.isfinite(args.speed) or not 0.2 <= args.speed <= 0.5:
            parser.error("--speed 必须在 0.2..0.5")
        if args.output is None:
            args.output = (
                DEFAULT_BASELINE_PATH
                if args.phase == "baseline"
                else DEFAULT_LOADED_PATH
            )
    return args


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def read_json(path: Path) -> dict:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"文件不存在：{path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"JSON 无效：{path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"JSON 顶层必须是对象：{path}")
    return payload


def joint_vector(value: object, *, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise RuntimeError(f"{name} 必须是六个有限关节角")
    return result


def build_wrist_pose_plan(
    center: np.ndarray,
    joint_minimum: np.ndarray,
    joint_maximum: np.ndarray,
    excursion_deg: float,
) -> list[tuple[str, np.ndarray]]:
    center = joint_vector(center, name="center")
    joint_minimum = joint_vector(joint_minimum, name="joint minimum")
    joint_maximum = joint_vector(joint_maximum, name="joint maximum")
    if np.any(joint_minimum >= joint_maximum):
        raise RuntimeError("关节限位无效")
    if np.any(center < joint_minimum) or np.any(center > joint_maximum):
        raise RuntimeError("中心姿态超出关节限位")
    excursion = math.radians(excursion_deg)
    roll_excursion = math.radians(min(20.0, excursion_deg * 4.0 / 3.0))
    diagonal = excursion * 0.8
    variants = [
        ("center", (0.0, 0.0, 0.0)),
        ("j4_plus", (excursion, 0.0, 0.0)),
        ("j4_minus", (-excursion, 0.0, 0.0)),
        ("j5_plus", (0.0, excursion, 0.0)),
        ("j5_minus", (0.0, -excursion, 0.0)),
        ("j6_plus", (0.0, 0.0, roll_excursion)),
        ("j6_minus", (0.0, 0.0, -roll_excursion)),
        ("diag_ppp", (diagonal, diagonal, diagonal)),
        ("diag_pmm", (diagonal, -diagonal, -diagonal)),
        ("diag_mpm", (-diagonal, diagonal, -diagonal)),
        ("diag_mmp", (-diagonal, -diagonal, diagonal)),
    ]
    margin = math.radians(3.0)
    poses: list[tuple[str, np.ndarray]] = []
    for label, wrist_delta in variants:
        target = center.copy()
        target[3:] += wrist_delta
        if np.any(target < joint_minimum + margin) or np.any(
            target > joint_maximum - margin
        ):
            raise RuntimeError(
                f"中心姿态离关节限位太近，无法生成 {label}；请重新示教"
            )
        poses.append((label, target))
    return poses


def movement_duration(controller: ArmController, delta: np.ndarray, speed: float) -> float:
    minimum = max(controller.minimum_duration(float(value)) for value in delta)
    return max(2.0, minimum / speed)


def _movej_for_duration(
    controller: ArmController,
    target: np.ndarray,
    duration_s: float,
    *,
    allow_large_motion: bool = True,
):
    minimum = max(
        controller.minimum_duration(float(value))
        for value in np.asarray(target, dtype=np.float64)
        - controller.get_state().joint_position
    )
    return controller.movej(
        np.asarray(target, dtype=np.float64),
        velocity_scale=min(1.0, minimum / duration_s),
        allow_large_motion=allow_large_motion,
    )


def preview_wrist_plan(
    controller: ArmController,
    center: np.ndarray,
    poses: list[tuple[str, np.ndarray]],
) -> list[dict]:
    previews: list[dict] = []
    for label, target in poses:
        duration = movement_duration(controller, target - center, 0.35)
        minimum = max(
            controller.minimum_duration(float(value))
            for value in target - center
        )
        preview = controller.plan_movej(
            target,
            velocity_scale=min(1.0, minimum / duration),
            allow_large_motion=True,
        )
        tcp_delta = preview.path_xyz_m - preview.path_xyz_m[0]
        previews.append(
            {
                "label": label,
                "max_joint_delta_deg": float(
                    np.degrees(np.max(np.abs(target - center)))
                ),
                "max_tcp_excursion_mm": float(
                    1000.0 * np.max(np.linalg.norm(tcp_delta, axis=1))
                ),
            }
        )
    return previews


def teach_plan(args: argparse.Namespace) -> Path:
    controller = ArmController(
        model="X5",
        interface=args.interface,
        gripper_open_readout=load_gripper_open_readout(interface=args.interface),
    )
    failed = False
    print("示教阶段不会自动移动机械臂。")
    print("连接会启用官方 X5 重力补偿，机械臂可能轻微改变姿态。")
    input("确认工作区安全后按回车连接：")
    try:
        controller.connect()
        print(
            "请手动摆到远离桌面的中心姿态；腕部留 25°、"
            "其余关节留 3° 余量。"
        )
        input("摆好并松手后按回车锁定中心姿态：")
        controller.activate()
        time.sleep(0.6)
        center = controller.get_state().joint_position.copy()
        controller.damping()
        limits = controller.get_limits()
        poses = build_wrist_pose_plan(
            center,
            limits.joint_position_min,
            limits.joint_position_max,
            args.excursion,
        )
        previews = preview_wrist_plan(controller, center, poses)
        robot_config = arx5.RobotConfigFactory.get_instance().get_config("X5")
        regressor = payload_gravity_regressor(
            make_solver(robot_config, Path(robot_config.urdf_path)),
            np.asarray([position for _, position in poses]),
            np.asarray(robot_config.gravity_vector),
        )
        identification_condition = payload_identification_condition(regressor)
        if identification_condition > 1e5:
            raise RuntimeError("腕部姿态激励条件数过高，请重新示教中心姿态")
    except BaseException:
        failed = True
        raise
    finally:
        controller.close(emergency=failed)

    payload = {
        "version": 2,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "robot_model": "X5",
        "excursion_deg": args.excursion,
        "center_joint_position": center.tolist(),
        "poses": [
            {"label": label, "joint_position": position.tolist()}
            for label, position in poses
        ],
        "previews": previews,
        "payload_identification_condition_number": identification_condition,
    }
    write_json(args.output, payload)
    print("中心关节角 (deg)：", np.round(np.degrees(center), 2))
    print(
        "自动回放最大 TCP 偏移："
        f"{max(item['max_tcp_excursion_mm'] for item in previews):.1f} mm"
    )
    print(f"payload 辨识条件数：{identification_condition:.1f}")
    print(f"姿态计划：{args.output.resolve()}")
    return args.output


def load_pose_plan(path: Path) -> tuple[np.ndarray, list[tuple[str, np.ndarray]]]:
    payload = read_json(path)
    if payload.get("version") != 2 or payload.get("robot_model") != "X5":
        raise RuntimeError("姿态计划版本或机械臂型号不匹配")
    center = joint_vector(payload.get("center_joint_position"), name="center")
    raw_poses = payload.get("poses")
    if not isinstance(raw_poses, list) or len(raw_poses) < MIN_CAPTURE_COUNT:
        raise RuntimeError("姿态计划数量不足")
    poses: list[tuple[str, np.ndarray]] = []
    labels: set[str] = set()
    for index, item in enumerate(raw_poses):
        if not isinstance(item, dict) or not isinstance(item.get("label"), str):
            raise RuntimeError(f"姿态计划第 {index + 1} 项无效")
        if item["label"] in labels:
            raise RuntimeError(f"姿态计划标签重复：{item['label']}")
        labels.add(item["label"])
        poses.append(
            (
                item["label"],
                joint_vector(item.get("joint_position"), name=item["label"]),
            )
        )
    if "center" not in labels:
        raise RuntimeError("姿态计划缺少 center")
    return center, poses


def sample_static_feedback(
    controller: ArmController,
    label: str,
    target: np.ndarray,
    *,
    sample_count: int,
    rate_hz: float,
) -> StaticCapture:
    positions: list[np.ndarray] = []
    velocities: list[np.ndarray] = []
    torques: list[np.ndarray] = []
    gripper_positions: list[float] = []
    period = 1.0 / rate_hz
    next_sample = time.monotonic()
    for _ in range(sample_count):
        snapshot = controller.get_state()
        positions.append(snapshot.joint_position.copy())
        velocities.append(snapshot.joint_velocity.copy())
        torques.append(snapshot.joint_torque.copy())
        gripper_positions.append(snapshot.gripper_position)
        next_sample += period
        remaining = next_sample - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)

    position_array = np.asarray(positions)
    velocity_array = np.asarray(velocities)
    torque_array = np.asarray(torques)
    median_torque = np.median(torque_array, axis=0)
    torque_mad = np.median(np.abs(torque_array - median_torque), axis=0)
    capture = StaticCapture(
        label=label,
        target_position=target.copy(),
        joint_position=np.median(position_array, axis=0),
        joint_torque=median_torque,
        torque_mad=torque_mad,
        max_abs_velocity=float(np.max(np.abs(velocity_array))),
        gripper_position_m=float(np.median(gripper_positions)),
    )
    if capture.max_abs_velocity > 0.08:
        raise RuntimeError(
            f"{label} 采样速度过高：{capture.max_abs_velocity:.3f} rad/s"
        )
    return capture


def validate_capture_start(current: np.ndarray, center: np.ndarray) -> None:
    current = joint_vector(current, name="current")
    center = joint_vector(center, name="center")
    proximal_error = float(np.max(np.abs(current[:3] - center[:3])))
    wrist_error = float(np.max(np.abs(current[3:] - center[3:])))
    if proximal_error > MAX_START_PROXIMAL_ERROR_RAD:
        raise RuntimeError(
            "J1-J3 离示教中心超过 5°，拒绝自动回放；请手动对准中心"
        )
    if wrist_error > MAX_START_WRIST_ERROR_RAD:
        raise RuntimeError(
            "J4-J6 离示教中心超过 20°，拒绝自动回放；请手动靠近中心"
        )


def capture_phase(args: argparse.Namespace) -> Path:
    center, poses = load_pose_plan(args.plan)
    controller = ArmController(
        model="X5",
        interface=args.interface,
        gripper_open_readout=load_gripper_open_readout(interface=args.interface),
    )
    failed = False
    captures: list[StaticCapture] = []
    phase_name = (
        "基线（固定工具，不含待测负载）" if args.phase == "baseline" else "带载"
    )
    print(f"准备采集：{phase_name}")
    print(
        "自动回放只改变已示教中心附近的 J4/J5/J6，并在每个姿态静止采样。"
    )
    print(
        "确认末端配置正确、运动包络内无桌面/相机/人员，"
        "并准备物理急停。"
    )
    if args.hold_gripper:
        print(
            "夹爪将保持开始时的开度；异常阻尼会释放保持，"
            "请始终准备托住物体。"
        )
    input("按回车连接，然后手动把机械臂摆到示教中心附近：")
    try:
        controller.connect()
        input("已摆到中心附近并松手后按回车检查：")
        current = controller.get_state().joint_position
        validate_capture_start(current, center)
        confirmation = input("输入 RUN 后开始低速自动回放：").strip().upper()
        if confirmation != "RUN":
            raise KeyboardInterrupt

        controller.activate()
        if args.hold_gripper:
            controller.activate_gripper()
        center_duration = movement_duration(controller, center - current, args.speed)
        _movej_for_duration(controller, center, center_duration)
        time.sleep(args.settle)
        captures.append(
            sample_static_feedback(
                controller,
                "center",
                center,
                sample_count=args.samples,
                rate_hz=args.rate,
            )
        )
        print(f"已采集 1/{len(poses)}：center")

        capture_poses = [(label, target) for label, target in poses if label != "center"]
        for capture_index, (label, target) in enumerate(capture_poses, start=2):
            outward_duration = movement_duration(controller, target - center, args.speed)
            _movej_for_duration(controller, target, outward_duration)
            time.sleep(args.settle)
            captures.append(
                sample_static_feedback(
                    controller,
                    label,
                    target,
                    sample_count=args.samples,
                    rate_hz=args.rate,
                )
            )
            print(f"已采集 {capture_index}/{len(poses)}：{label}")
            _movej_for_duration(controller, center, outward_duration)
            time.sleep(0.25)
        if args.hold_gripper:
            input("采集完成。请托住物体，按回车进入阻尼并释放夹爪保持：")
        controller.damping()
    except BaseException:
        failed = True
        raise
    finally:
        controller.close(emergency=failed)

    payload = {
        "version": 2,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "robot_model": "X5",
        "phase": args.phase,
        "plan_path": str(args.plan),
        "hold_gripper": args.hold_gripper,
        "captures": [
            {
                "label": item.label,
                "target_joint_position": item.target_position.tolist(),
                "joint_position": item.joint_position.tolist(),
                "joint_torque_nm": item.joint_torque.tolist(),
                "torque_mad_nm": item.torque_mad.tolist(),
                "max_abs_velocity_rad_s": item.max_abs_velocity,
                "gripper_position_m": item.gripper_position_m,
            }
            for item in captures
        ],
    }
    write_json(args.output, payload)
    print(f"{phase_name}数据：{args.output.resolve()}")
    return args.output


def load_captures(path: Path, expected_phase: str) -> list[StaticCapture]:
    payload = read_json(path)
    if (
        payload.get("version") != 2
        or payload.get("robot_model") != "X5"
        or payload.get("phase") != expected_phase
    ):
        raise RuntimeError(f"采集文件版本、型号或阶段不匹配：{path}")
    raw_captures = payload.get("captures")
    if not isinstance(raw_captures, list) or len(raw_captures) < MIN_CAPTURE_COUNT:
        raise RuntimeError(f"采集姿态数量不足：{path}")
    captures: list[StaticCapture] = []
    labels: set[str] = set()
    for index, item in enumerate(raw_captures):
        if not isinstance(item, dict) or not isinstance(item.get("label"), str):
            raise RuntimeError(f"采集文件第 {index + 1} 项无效：{path}")
        if item["label"] in labels:
            raise RuntimeError(f"采集姿态标签重复：{item['label']}")
        labels.add(item["label"])
        max_abs_velocity = float(item.get("max_abs_velocity_rad_s"))
        if not math.isfinite(max_abs_velocity) or max_abs_velocity < 0.0:
            raise RuntimeError(f"采集速度无效：{path}")
        gripper_position = float(item.get("gripper_position_m"))
        if not math.isfinite(gripper_position) or gripper_position < 0.0:
            raise RuntimeError(f"夹爪开度无效：{path}")
        captures.append(
            StaticCapture(
                label=item["label"],
                target_position=joint_vector(
                    item.get("target_joint_position"), name="target"
                ),
                joint_position=joint_vector(item.get("joint_position"), name="position"),
                joint_torque=joint_vector(item.get("joint_torque_nm"), name="torque"),
                torque_mad=joint_vector(item.get("torque_mad_nm"), name="torque MAD"),
                max_abs_velocity=max_abs_velocity,
                gripper_position_m=gripper_position,
            )
        )
    return captures


def link_mass_from_urdf(path: Path, link_name: str = "link6") -> float:
    root = ET.parse(path).getroot()
    link = root.find(f"./link[@name='{link_name}']")
    mass = link.find("./inertial/mass") if link is not None else None
    if mass is None:
        raise RuntimeError(f"URDF 中缺少 {link_name} 的质量")
    value = float(mass.attrib["value"])
    if not math.isfinite(value) or value <= 0.0:
        raise RuntimeError(f"URDF 中 {link_name} 的质量无效：{value}")
    return value


def write_zero_link_mass_urdf(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    link = root.find("./link[@name='link6']")
    inertial = link.find("inertial") if link is not None else None
    mass = inertial.find("mass") if inertial is not None else None
    inertia = inertial.find("inertia") if inertial is not None else None
    if mass is None or inertia is None:
        raise RuntimeError("URDF 中 link6 的惯性参数不完整")
    mass.set("value", "0")
    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        inertia.set(key, "0")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def make_solver(robot_config: arx5.RobotConfig, urdf_path: Path) -> arx5.Arx5Solver:
    return arx5.Arx5Solver(
        str(urdf_path),
        robot_config.joint_dof,
        robot_config.joint_pos_min,
        robot_config.joint_pos_max,
        robot_config.base_link_name,
        robot_config.eef_link_name,
        robot_config.gravity_vector,
    )


def gravity_torques(solver: arx5.Arx5Solver, positions: np.ndarray) -> np.ndarray:
    zeros = np.zeros(positions.shape[1], dtype=np.float64)
    return np.asarray(
        [solver.inverse_dynamics(position, zeros, zeros) for position in positions],
        dtype=np.float64,
    )


def robust_linear_fit(
    design: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, int]:
    design = np.asarray(design, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if design.ndim != 2 or target.shape != (design.shape[0],):
        raise RuntimeError("拟合数据维度无效")
    if design.shape[0] == 0 or not np.all(np.isfinite(design)):
        raise RuntimeError("拟合设计矩阵为空或包含非有限值")
    if not np.all(np.isfinite(target)):
        raise RuntimeError("拟合目标包含非有限值")
    if design.shape[0] <= design.shape[1]:
        raise RuntimeError("有效数据不足，无法完成辨识")
    if int(np.linalg.matrix_rank(design)) < design.shape[1]:
        raise RuntimeError("姿态激励不足，请重新示教更开阔的中心姿态")
    coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)
    residual = target - design @ coefficients
    residual_median = float(np.median(residual))
    residual_mad = float(np.median(np.abs(residual - residual_median)))
    cutoff = max(0.12, 3.5 * 1.4826 * residual_mad)
    keep = np.abs(residual - residual_median) <= cutoff
    rejected = int(np.count_nonzero(~keep))
    if rejected:
        if int(np.count_nonzero(keep)) > design.shape[1]:
            candidate_design = design[keep]
            if int(np.linalg.matrix_rank(candidate_design)) == design.shape[1]:
                design = candidate_design
                target = target[keep]
                coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)
                residual = target - design @ coefficients
            else:
                rejected = 0
        else:
            rejected = 0
    return coefficients, residual, design, float(np.linalg.cond(design)), rejected


def validate_fit_arrays(*arrays: np.ndarray) -> tuple[int, int]:
    converted = [np.asarray(array) for array in arrays]
    if not converted or converted[0].ndim != 2:
        raise RuntimeError("拟合输入必须是二维数组")
    shape = converted[0].shape
    if shape[1] != 6 or any(array.shape != shape for array in converted):
        raise RuntimeError("拟合输入必须是形状一致的 N x 6 数组")
    if any(not np.all(np.isfinite(array)) for array in converted):
        raise RuntimeError("拟合输入包含非有限值")
    return shape


def fit_effective_tool_mass(
    measured: np.ndarray,
    official: np.ndarray,
    baseline: np.ndarray,
    official_mass_kg: float,
) -> ToolMassFit:
    measured = np.asarray(measured, dtype=np.float64)
    official = np.asarray(official, dtype=np.float64)
    baseline = np.asarray(baseline, dtype=np.float64)
    validate_fit_arrays(measured, official, baseline)
    if not math.isfinite(official_mass_kg) or official_mass_kg <= 0.0:
        raise RuntimeError("官方末端质量无效")
    rows: list[list[float]] = []
    targets: list[float] = []
    for pose_index in range(measured.shape[0]):
        for bias_index, joint_index in enumerate(TOOL_FIT_JOINT_INDICES):
            link_signal = official[pose_index, joint_index] - baseline[pose_index, joint_index]
            if abs(link_signal) < 0.03:
                continue
            row = [float(link_signal), *([0.0] * len(TOOL_FIT_JOINT_INDICES))]
            row[1 + bias_index] = 1.0
            rows.append(row)
            targets.append(
                float(
                    measured[pose_index, joint_index]
                    - baseline[pose_index, joint_index]
                )
            )
    design = np.asarray(rows, dtype=np.float64)
    target = np.asarray(targets, dtype=np.float64)
    coefficients, residual, fitted_design, condition, rejected = robust_linear_fit(
        design, target
    )
    mass_scale = float(coefficients[0])
    degrees_of_freedom = fitted_design.shape[0] - fitted_design.shape[1]
    covariance = (
        np.linalg.pinv(fitted_design.T @ fitted_design)
        * float(residual @ residual)
        / max(1, degrees_of_freedom)
    )
    return ToolMassFit(
        mass_scale=mass_scale,
        effective_mass_kg=official_mass_kg * mass_scale,
        mass_std_kg=official_mass_kg * math.sqrt(max(0.0, covariance[0, 0])),
        joint_bias_nm=np.asarray(coefficients[1:]),
        rmse_nm=float(np.sqrt(np.mean(np.square(residual)))),
        condition_number=condition,
        used_rows=int(fitted_design.shape[0]),
        rejected_rows=rejected,
    )


def rotation_matrix_from_rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def payload_gravity_regressor(
    solver: arx5.Arx5Solver,
    joint_positions: np.ndarray,
    gravity_vector: np.ndarray,
    *,
    epsilon: float = 1e-6,
) -> np.ndarray:
    positions = np.asarray(joint_positions, dtype=np.float64)
    gravity_vector = np.asarray(gravity_vector, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 6:
        raise RuntimeError("关节姿态必须是 N x 6 数组")
    if gravity_vector.shape != (3,) or not np.all(np.isfinite(gravity_vector)):
        raise RuntimeError("重力向量必须包含三个有限值")
    if not np.all(np.isfinite(positions)):
        raise RuntimeError("关节姿态包含非有限值")
    if not math.isfinite(epsilon) or epsilon <= 0.0:
        raise RuntimeError("数值微分步长必须为正数")
    regressor = np.zeros((positions.shape[0], positions.shape[1], 4))
    for pose_index, position in enumerate(positions):
        for joint_index in range(positions.shape[1]):
            delta = np.zeros(positions.shape[1])
            delta[joint_index] = epsilon
            plus = np.asarray(solver.forward_kinematics(position + delta))
            minus = np.asarray(solver.forward_kinematics(position - delta))
            plus_rotation = rotation_matrix_from_rpy(plus[3:])
            minus_rotation = rotation_matrix_from_rpy(minus[3:])
            position_derivative = (plus[:3] - minus[:3]) / (2.0 * epsilon)
            rotation_derivative = (plus_rotation - minus_rotation) / (2.0 * epsilon)
            regressor[pose_index, joint_index, 0] = -float(
                position_derivative @ gravity_vector
            )
            for axis in range(3):
                regressor[pose_index, joint_index, 1 + axis] = -float(
                    rotation_derivative[:, axis] @ gravity_vector
                )
    return regressor


def payload_design_matrix(regressor: np.ndarray) -> np.ndarray:
    rows: list[list[float]] = []
    for pose_index in range(regressor.shape[0]):
        for bias_index, joint_index in enumerate(PAYLOAD_FIT_JOINT_INDICES):
            payload_row = regressor[pose_index, joint_index]
            if float(np.linalg.norm(payload_row)) < 0.03:
                continue
            row = [*payload_row.tolist(), *([0.0] * len(PAYLOAD_FIT_JOINT_INDICES))]
            row[4 + bias_index] = 1.0
            rows.append(row)
    return np.asarray(rows, dtype=np.float64)


def payload_identification_condition(regressor: np.ndarray) -> float:
    design = payload_design_matrix(regressor)
    parameter_count = 4 + len(PAYLOAD_FIT_JOINT_INDICES)
    if design.ndim != 2 or design.shape[0] <= parameter_count:
        raise RuntimeError("payload 姿态激励数据不足")
    if int(np.linalg.matrix_rank(design)) < parameter_count:
        raise RuntimeError("payload 姿态激励不满秩，请重新示教中心姿态")
    return float(np.linalg.cond(design))


def fit_added_payload(
    torque_delta: np.ndarray,
    regressor: np.ndarray,
) -> PayloadFit:
    torque_delta = np.asarray(torque_delta, dtype=np.float64)
    regressor = np.asarray(regressor, dtype=np.float64)
    validate_fit_arrays(torque_delta)
    if regressor.shape != (*torque_delta.shape, 4):
        raise RuntimeError("payload 回归矩阵必须是 N x 6 x 4")
    if not np.all(np.isfinite(regressor)):
        raise RuntimeError("payload 回归矩阵包含非有限值")
    design = payload_design_matrix(regressor)
    targets: list[float] = []
    for pose_index in range(torque_delta.shape[0]):
        for joint_index in PAYLOAD_FIT_JOINT_INDICES:
            payload_row = regressor[pose_index, joint_index]
            if float(np.linalg.norm(payload_row)) < 0.03:
                continue
            targets.append(float(torque_delta[pose_index, joint_index]))
    target = np.asarray(targets, dtype=np.float64)
    coefficients, residual, fitted_design, condition, rejected = robust_linear_fit(
        design, target
    )
    mass = float(coefficients[0])
    first_moment = np.asarray(coefficients[1:4], dtype=np.float64)
    center_of_mass = (
        first_moment / mass
        if abs(mass) >= 0.02
        else np.full(3, np.nan, dtype=np.float64)
    )
    degrees_of_freedom = fitted_design.shape[0] - fitted_design.shape[1]
    covariance = (
        np.linalg.pinv(fitted_design.T @ fitted_design)
        * float(residual @ residual)
        / max(1, degrees_of_freedom)
    )
    return PayloadFit(
        mass_kg=mass,
        mass_std_kg=math.sqrt(max(0.0, covariance[0, 0])),
        center_of_mass_m=center_of_mass,
        first_moment_kg_m=first_moment,
        joint_bias_nm=np.asarray(coefficients[4:]),
        rmse_nm=float(np.sqrt(np.mean(np.square(residual)))),
        condition_number=condition,
        used_rows=int(fitted_design.shape[0]),
        rejected_rows=rejected,
    )


def fit_tool_command(args: argparse.Namespace) -> Path:
    captures = load_captures(args.baseline, "baseline")
    robot_config = arx5.RobotConfigFactory.get_instance().get_config("X5")
    official_urdf = Path(robot_config.urdf_path)
    official_mass = link_mass_from_urdf(official_urdf)
    positions = np.asarray([item.joint_position for item in captures])
    measured = np.asarray([item.joint_torque for item in captures])
    with tempfile.TemporaryDirectory(prefix="arx5-tool-fit-") as directory:
        zero_urdf = Path(directory) / "X5_zero_link6.urdf"
        write_zero_link_mass_urdf(official_urdf, zero_urdf)
        official = gravity_torques(make_solver(robot_config, official_urdf), positions)
        zero_link = gravity_torques(make_solver(robot_config, zero_urdf), positions)
    fit = fit_effective_tool_mass(measured, official, zero_link, official_mass)
    write_json(
        args.output,
        {
            "version": 2,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "kind": "tool_mass",
            "official_link6_mass_kg": official_mass,
            "estimated_effective_link6_mass_kg": fit.effective_mass_kg,
            "estimated_mass_std_kg": fit.mass_std_kg,
            "estimated_link6_mass_scale": fit.mass_scale,
            "fit_rmse_nm": fit.rmse_nm,
            "condition_number": fit.condition_number,
            "used_rows": fit.used_rows,
            "rejected_rows": fit.rejected_rows,
            "joint_bias_nm": {
                f"J{joint + 1}": float(fit.joint_bias_nm[index])
                for index, joint in enumerate(TOOL_FIT_JOINT_INDICES)
            },
        },
    )
    print(f"标准末端有效质量：{fit.effective_mass_kg:.3f} ± {fit.mass_std_kg:.3f} kg")
    print(f"相对官方 X5：{100.0 * fit.mass_scale:.1f}%")
    print(f"拟合 RMSE：{fit.rmse_nm:.3f} N·m")
    print(f"结果：{args.output.resolve()}")
    return args.output


def paired_capture_arrays(
    baseline: list[StaticCapture],
    loaded: list[StaticCapture],
) -> tuple[np.ndarray, np.ndarray]:
    if [item.label for item in baseline] != [item.label for item in loaded]:
        raise RuntimeError("baseline 与 loaded 的姿态标签不一致")
    baseline_positions = np.asarray([item.joint_position for item in baseline])
    loaded_positions = np.asarray([item.joint_position for item in loaded])
    mismatch = float(np.max(np.abs(baseline_positions - loaded_positions)))
    if mismatch > math.radians(0.75):
        raise RuntimeError(
            f"baseline 与 loaded 最大姿态偏差 {math.degrees(mismatch):.2f}°，拒绝拟合"
        )
    positions = 0.5 * (baseline_positions + loaded_positions)
    baseline_torque = np.asarray([item.joint_torque for item in baseline])
    loaded_torque = np.asarray([item.joint_torque for item in loaded])
    return positions, loaded_torque - baseline_torque


def fit_payload_command(args: argparse.Namespace) -> Path:
    baseline = load_captures(args.baseline, "baseline")
    loaded = load_captures(args.loaded, "loaded")
    gripper_mismatch = max(
        abs(base.gripper_position_m - load.gripper_position_m)
        for base, load in zip(baseline, loaded)
    )
    if gripper_mismatch > 0.005:
        print(
            "警告：baseline/loaded 夹爪开度最大差异 "
            f"{1000.0 * gripper_mismatch:.1f} mm，差异会增加拟合噪声。"
        )
    positions, torque_delta = paired_capture_arrays(baseline, loaded)
    robot_config = arx5.RobotConfigFactory.get_instance().get_config("X5")
    solver = make_solver(robot_config, Path(robot_config.urdf_path))
    regressor = payload_gravity_regressor(
        solver,
        positions,
        np.asarray(robot_config.gravity_vector),
    )
    fit = fit_added_payload(torque_delta, regressor)
    center_of_mass = (
        fit.center_of_mass_m.tolist()
        if np.all(np.isfinite(fit.center_of_mass_m))
        else None
    )
    result = {
        "version": 2,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "kind": "added_payload",
        "mount_mode": getattr(args, "mount_mode", None),
        "estimated_payload_mass_kg": fit.mass_kg,
        "estimated_mass_std_kg": fit.mass_std_kg,
        "estimated_payload_weight_n": fit.mass_kg * 9.807,
        "center_of_mass_tcp_m": center_of_mass,
        "first_moment_kg_m": fit.first_moment_kg_m.tolist(),
        "max_gripper_position_mismatch": gripper_mismatch,
        "fit_rmse_nm": fit.rmse_nm,
        "condition_number": fit.condition_number,
        "used_rows": fit.used_rows,
        "rejected_rows": fit.rejected_rows,
        "joint_bias_nm": {
            f"J{joint + 1}": float(fit.joint_bias_nm[index])
            for index, joint in enumerate(PAYLOAD_FIT_JOINT_INDICES)
        },
    }
    write_json(args.output, result)
    profile_path = save_payload_profile(args.profile, result)
    print(f"新增负载质量：{fit.mass_kg:.3f} ± {fit.mass_std_kg:.3f} kg")
    if center_of_mass is None:
        print("TCP 重心：质量过小，无法可靠辨识")
    else:
        print("TCP 重心 (mm)：", np.round(1000.0 * fit.center_of_mass_m, 1))
    print(f"拟合 RMSE：{fit.rmse_nm:.3f} N·m")
    print(f"结果：{args.output.resolve()}")
    print(f"已缓存并启用负载档案 {args.profile}：{profile_path.resolve()}")
    if fit.mass_kg <= 0.02 or fit.rmse_nm > 0.25 or fit.condition_number > 1e5:
        print(
            "警告：质量过小、误差较高或姿态激励不足，"
            "请复查安装和重采数据。"
        )
    return args.output


def list_profiles_command() -> None:
    profiles = list_payload_profiles()
    if not profiles:
        print("尚无负载档案。")
        return
    for payload in profiles:
        marker = "*" if payload["active"] else " "
        mass_g = 1000.0 * float(payload["estimated_payload_mass_kg"])
        print(f"{marker} {payload['profile']:<20} {mass_g:8.1f} g")


def use_profile_command(name: str) -> None:
    if name.strip().lower() == "none":
        select_payload_profile(None)
        print("当前负载已切换为空载。")
        return
    select_payload_profile(name)
    print(f"当前负载档案：{name}")


def main() -> int:
    args = parse_args()
    try:
        if args.command == "teach":
            teach_plan(args)
        elif args.command == "capture":
            capture_phase(args)
        elif args.command == "fit-tool":
            fit_tool_command(args)
        elif args.command == "fit-payload":
            fit_payload_command(args)
        elif args.command == "profiles":
            list_profiles_command()
        else:
            use_profile_command(args.profile)
    except BusyError as exc:
        print(f"无法连接：{exc}")
        print("请先断开机械臂并停止占用 can0 的网页控制服务。")
        return 2
    except KeyboardInterrupt:
        print("\n流程已取消，机械臂已释放到阻尼状态。")
        return 130
    except Exception as exc:
        print(f"标定失败：{exc}")
        return 1
    if args.command in {"teach", "capture", "fit-tool", "fit-payload"}:
        print("结果尚未自动应用到正常控制。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
