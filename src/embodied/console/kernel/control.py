"""Hardware-owning control kernel shared by the versioned API features."""

from __future__ import annotations

import asyncio
from dataclasses import replace
import hmac
import json
import math
import os
from pathlib import Path
import queue
import re
import secrets
import signal
import sys
import time
from typing import Any, Callable
from types import SimpleNamespace
from urllib.parse import parse_qs, urlsplit

import numpy as np
from websockets.exceptions import ConnectionClosed

from legacy.arx5_control import (
    ArmController,
    ArmSnapshot,
    BusyError,
    CartesianMotionResult,
    CartesianPlan,
    ControlError,
    ControllerMode,
    GripperForceResult,
    GripperMotionResult,
    GripperServo,
    GripperState,
    MotionPlan,
    MotionResult,
    MotionType,
    Payload,
    Pose,
    SafetyConfig,
    SafetyError,
    TeleoperationConfig,
    TeleoperationServo,
    TeleoperationTarget,
    load_teleoperation_config,
)
from legacy.arx5_control.cartesian import (
    CartesianPlanningError,
    normalize_quaternion_xyzw,
    rpy_to_quaternion_xyzw,
)
from legacy.arx5_control.teleoperation import (
    limit_teleoperation_target_step,
    quaternion_multiply_xyzw,
)
from legacy.arx5_control.gripper_calibration import (
    load_gripper_calibration,
    load_gripper_open_readout,
    save_gripper_open_readout,
)
from legacy.arx5_control.payload_profiles import PayloadProfileReader, validate_profile_name
from embodied.calibration.hand_eye.collector import ManualEyeToHandCollector
from embodied.calibration.hand_eye.__main__ import load_config as load_handeye_config
from embodied.calibration.hand_eye.core.storage import CalibrationSession
from embodied.calibration.hand_eye.core.validation import (
    disable_result,
    enable_validated_result,
    read_json,
    validate_cached_result,
)
from embodied.tasks.cube.config import load_config
from embodied.tasks.cube.planner import (
    Action as CubeAction,
    CubePlanError,
    blended_linear_actions,
    build_pick_actions,
    cartesian_path_for_actions,
    grasp_orientation_candidates,
    maximum_plausible_contact_width_m,
    point_inside_table,
    preflight_action_plan,
    validate_initial_pose,
)
from embodied.data.groot.config import load_collection_config
from embodied.data.groot.home import (
    load_collection_home,
    save_collection_home,
)
from embodied.data.groot.raw_episode import RawEpisodeStore
from embodied.data.groot.recording import ArmStateRecorder
from embodied.data.groot.validator import (
    MINIMUM_USABLE_CAMERA_RATE_FRACTION,
    validate_raw_episode,
)
from embodied.console.features.collection.camera_client import CollectionCameraClient
from embodied.console.features.cube.stack import (
    CUBE_STACK_PREVIEW_TIMEOUT_S,
    CubeStackPlan,
    execute_cube_stack,
    plan_cube_stack,
)
from embodied.console.features.gripper import HOME_GRIPPER_WIDTH_M
from legacy.arx_tools.runtime_identity import SERVICE_STARTED_AT, SOURCE_REVISION
from embodied.core.arx5_paths import CALIBRATIONS_ROOT, CONFIG_ROOT, HANDEYE_ROOT, PROJECT_ROOT


WEB_MAX_JOINT_STEP_DEG = 10.0
WEB_MAX_GRIPPER_WIDTH_MM = 88.0
WEB_MAX_GRIPPER_TORQUE_NM = 0.25
WEB_MIN_SPEED_PERCENT = 1.0
WEB_MAX_SPEED_PERCENT = 100.0
WEB_DEFAULT_SPEED_PERCENT = 10.0
WEB_SPEED_TIERS_PERCENT = (1, 2, 3, 4, 5, 10, 25, 50, 75, 100)
WEB_MOTION_PROFILE_REVISION = 6
WEB_SPEED_CONTROL_ACTIONS = frozenset(
    {
        "preview_park",
        "park",
        "preview_xyz",
        "preview_xyz_live",
        "move_xyz",
        "move_joint",
        "move_joint_target",
        "reset_joint",
        "gripper_position",
        "reset_gripper",
        "cube_pick_preview",
        "cube_pick_execute",
        "cube_move_preview",
        "cube_move_execute",
        "cube_place_preview",
        "cube_place_execute",
        "cube_stack_preview",
        "cube_stack_execute",
    }
)
WEB_POST_MOTION_MODE_ACTIONS = frozenset(
    {
        "move_xyz",
        "move_joint",
        "move_joint_target",
        "reset_joint",
        "gripper_position",
        "reset_gripper",
        # Held-object stages must stay stiff. Release is the first safe point
        # where the status-bar mode can be restored.
        "cube_place_execute",
    }
)
WEB_XYZ_PATH_MODES = frozenset({"smooth", "linear"})
WEB_PARK_NOMINAL_TOLERANCE_DEG = 1.0
WEB_PARK_ABORT_TOLERANCE_DEG = 3.0
WEB_CARTESIAN_PREVIEW_JOINT_TOLERANCE_DEG = 1.0
WEB_CARTESIAN_PREVIEW_POSITION_TOLERANCE_MM = 2.0
CUBE_PICK_PREVIEW_TIMEOUT_S = 90.0
# Preferred oblique side grasp; the preview tries this profile before the rest.
CUBE_PREFERRED_SIDE_PROFILE = "_tilt_45_roll_0"
WEB_MOTION_FEEDBACK_PERIOD_S = 0.04
WEB_TRAJECTORY_PREVIEW_MAX_POINTS = 160
PAYLOAD_CALIBRATION_ROOT = CALIBRATIONS_ROOT / "payload"
PAYLOAD_CALIBRATION_MODES = frozenset({"fixed", "grasped"})
PAYLOAD_CALIBRATION_PROGRESS_RE = re.compile(r"已采集\s+(\d+)/(\d+)")
HANDEYE_CONFIG = HANDEYE_ROOT / "configs" / "x5_d455_eye_to_hand.json"
TELEOPERATION_CONFIG_PATH = CONFIG_ROOT / "teleoperation.json"
HANDEYE_OUTPUT_ROOT = CALIBRATIONS_ROOT / "hand-eye" / "eye-to-hand"
HANDEYE_CALIBRATION_MODES = frozenset(
    {"dual", "eye_in_hand", "eye_to_hand"}
)
HANDEYE_VALIDATION_ACTIONS = frozenset(
    {"handeye_validate", "handeye_enable", "handeye_disable"}
)
HANDEYE_PROGRESS_RE = re.compile(r"已采集\s+(\d+)(?:/(\d+))?")
HANDEYE_POSE_RE = re.compile(
    r"^HANDEYE_POSE\s+(\d+)/(\d+)\s+valid=(\d+)(?:\s+target=(\d+))?"
)
HANDEYE_DUAL_POSE_RE = re.compile(
    r"^HANDEYE_DUAL_POSE\s+(eye_in_hand|eye_to_hand)\s+"
    r"(\d+)/(\d+)\s+valid=(\d+)(?:\s+target=(\d+))?"
)
HANDEYE_MANUAL_STAGE_RE = re.compile(
    r"^HANDEYE_STAGE manual\s+valid=(\d+)\s+minimum=(\d+)\s+"
    r"target=(\d+)\s+ready=([01])"
)
HANDEYE_MANUAL_SAMPLE_RE = re.compile(
    r"^HANDEYE_MANUAL_SAMPLE\s+valid=(\d+)\s+target=(\d+)\s+"
    r"accepted=([01])"
)
HANDEYE_FEEDBACK_PREFIX = "HANDEYE_FEEDBACK "
HANDEYE_PREVIEW_PREFIX = "HANDEYE_PREVIEW "
GRIPPER_CALIBRATION_OPEN_READOUT_RE = re.compile(
    r"Fully-open joint position readout:\s*([-+0-9.eE]+)"
)
WEB_FEEDBACK_ACTIONS = frozenset(
    {
        "activate",
        "damping",
        "park",
        "move_joint",
        "move_joint_target",
        "move_xyz",
        "reset_joint",
        "gripper_position",
        "reset_gripper",
        "gripper_force",
        "cube_pick_execute",
        "cube_move_execute",
        "cube_place_execute",
        "cube_stack_execute",
    }
)
# X5 reports joint_vel_max=[5, 5, 5.5, 5.5, 5, 5] rad/s,
# ee_vel_max[:3]=[0.6, 0.6, 0.6] m/s, and gripper_vel_max=0.3 m/s.
# The scalar joint cap is intersected with those per-joint SDK limits.
WEB_MOTION_SAFETY = replace(
    SafetyConfig(),
    max_commanded_velocity_rad_s=5.5,
    max_commanded_cartesian_velocity_m_s=0.60,
    max_commanded_cartesian_acceleration_m_s2=1.20,
    max_commanded_cartesian_jerk_m_s3=12.0,
    cartesian_geometry_step_m=0.006,
    cartesian_orientation_step_rad=math.radians(4.0),
    cartesian_singularity_min_speed_scale=1.0,
    feedback_velocity_limit_rad_s=5.75,
    max_commanded_gripper_velocity_m_s=0.30,
    feedback_gripper_velocity_limit_m_s=0.36,
    gripper_servo_max_acceleration_m_s2=1.20,
    gripper_servo_max_jerk_m_s3=6.0,
    trajectory_rate_hz=100.0,
    minimum_motion_duration_s=0.25,
    settle_time_s=0.65,
    trajectory_final_hold_s=0.30,
    terminal_position_tolerance_rad=math.radians(6.0),
    terminal_velocity_tolerance_rad_s=0.04,
    servo_max_velocity_rad_s=5.5,
    servo_max_acceleration_rad_s2=4.0,
    servo_max_jerk_rad_s3=24.0,
)
ServerConnection = Any


def finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def cube_plan_message(error: CubePlanError) -> str:
    """Localize a shared cube planner rejection for the browser UI.

    The planner keeps English messages so logs and IK failure summaries stay
    greppable; only the codes that reach an operator are translated here.
    """
    context = error.context
    if error.code == "grasp_below_table":
        return "抓取 TCP 高度不高于桌面，请检查桌面高度或 TCP 修正。"
    if error.code == "side_pregrasp_outside_table":
        return "侧抓预接近点超出桌面安全范围。"
    if error.code == "side_missing_approach":
        return "侧抓缺少水平接近方向。"
    if error.code == "place_outside_table":
        return "放置目标超出配置的桌面安全范围。"
    if error.code == "unsupported_grasp_mode":
        return f"不支持的抓取模式：{context.get('grasp_mode')}"
    if error.code == "invalid_tcp_pose":
        return "控制器当前 TCP 姿态无效。"
    if error.code == "initial_tcp_too_low":
        clearance_mm = 1000.0 * float(context.get("clearance_m", 0.0))
        return f"当前 TCP 太低，自动抓取要求至少离桌面 {clearance_mm:.0f} mm。"
    return str(error)


def web_error_message(action: str, error: BaseException) -> str:
    """Translate common planner diagnostics into concise UI-facing Chinese."""
    if isinstance(error, CubePlanError):
        return cube_plan_message(error)
    message = str(error)
    if "Failed to open USB-CAN adapter with interface name" in message:
        return (
            "未检测到 SocketCAN 接口 can0。请在主机终端运行 "
            "./arx5 can-up，看到 ERROR-ACTIVE 后重新连接。"
        )
    if action not in {
        "preview_xyz",
        "preview_xyz_live",
        "move_xyz",
        "cube_pick_preview",
        "cube_pick_execute",
        "cube_move_preview",
        "cube_move_execute",
        "cube_place_preview",
        "cube_place_execute",
    }:
        return message
    if message.startswith("inverse kinematics failed:"):
        detail = message.removeprefix("inverse kinematics failed:").strip()
        if action.startswith(("cube_pick", "cube_move", "cube_place")):
            return f"抓取点暂无可达逆解；已尝试等价俯抓和多方向斜侧抓姿态：{detail}"
        return f"当前末端目标无法求解逆运动学：{detail}"
    return message


def handeye_failure_message(log: list[Any]) -> str:
    lines = [str(line).strip() for line in log if str(line).strip()]
    for line in reversed(lines):
        if line.startswith("手眼标定失败："):
            return line.removeprefix("手眼标定失败：").strip()
    for line in reversed(lines):
        if line.startswith("当前只有"):
            return line
    fallback = "手眼标定未完成。"
    return f"{fallback} 最后输出：{lines[-1]}" if lines else fallback


def velocity_scale_from_params(params: dict[str, Any]) -> float:
    """Parse a browser-supplied arm speed and return a trusted scale."""
    percent = params.get("speed_percent", WEB_DEFAULT_SPEED_PERCENT)
    if not finite_number(percent) or float(percent) not in WEB_SPEED_TIERS_PERCENT:
        raise ValueError(
            "机械臂速度必须是 1%、2%、3%、4%、5%、10%、25%、50%、75% 或 100%。"
        )
    return float(percent) / 100.0


def motion_safety_from_params(params: dict[str, Any]) -> SafetyConfig:
    """Return the one server-trusted envelope for every web motion."""
    if "speed_profile" in params or "hardware_max_confirmed" in params:
        raise ValueError(
            "机械臂速度配置已统一，请只使用 speed_percent。"
        )
    return WEB_MOTION_SAFETY


def post_motion_mode_from_params(params: dict[str, Any]) -> ControllerMode:
    """Parse the status-bar mode restored after one motion."""
    value = params.get("mode_after_motion", ControllerMode.ACTIVE.value)
    if value not in (ControllerMode.DAMPING.value, ControllerMode.ACTIVE.value):
        raise ValueError("动作完成模式必须是 damping 或 active。")
    return ControllerMode(value)


def minimum_motion_duration_s(controller: ArmController) -> float:
    """Return the configured duration floor while supporting lightweight fakes."""
    return float(
        getattr(controller.safety, "minimum_motion_duration_s", 2.0)
    )


def park_derivative_scales(
    velocity_scale: float,
    safety: SafetyConfig | None = None,
) -> tuple[float, float, float]:
    """Scale the one return trajectory profile by the operator's time scale."""
    del safety
    if not math.isfinite(velocity_scale) or not 0.0 < velocity_scale <= 1.0:
        raise ValueError("velocity_scale must be in (0, 1]")
    return velocity_scale, velocity_scale**2, velocity_scale**3


def xyz_path_mode_from_params(params: dict[str, Any]) -> str:
    """Accept only the two explicitly presented XYZ path geometries."""
    mode = params.get("path_mode", "smooth")
    if not isinstance(mode, str) or mode not in WEB_XYZ_PATH_MODES:
        raise ValueError("XYZ 轨迹模式必须是 smooth 或 linear。")
    return mode


def cartesian_delta_from_params(params: dict[str, Any]) -> np.ndarray:
    """Parse a base-frame relative XYZ request expressed in millimetres."""
    values = [params.get(name) for name in ("x_mm", "y_mm", "z_mm")]
    if not all(finite_number(value) for value in values):
        raise ValueError("X、Y、Z 必须都是有限数值。")
    delta_mm = np.asarray(values, dtype=np.float64)
    distance_mm = math.hypot(*(float(value) for value in delta_mm))
    if distance_mm < 0.1 and params.get("orientation_xyzw") is None:
        raise ValueError("XYZ 位移太小，请至少设置 0.1 mm。")
    return delta_mm / 1000.0


def target_orientation_from_params(
    params: dict[str, Any],
) -> np.ndarray | None:
    """Parse an optional browser target quaternion in normalized XYZW order."""
    value = params.get("orientation_xyzw")
    if value is None:
        return None
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 4
        or not all(finite_number(component) for component in value)
    ):
        raise ValueError("目标姿态必须是四个有限数值组成的 XYZW quaternion。")
    try:
        return normalize_quaternion_xyzw(np.asarray(value, dtype=np.float64))
    except CartesianPlanningError as exc:
        raise ValueError(str(exc)) from exc


def quaternion_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    """Return shortest angular distance between two XYZW quaternions."""
    a = normalize_quaternion_xyzw(first)
    b = normalize_quaternion_xyzw(second)
    dot = max(-1.0, min(1.0, abs(float(np.dot(a, b)))))
    return 2.0 * math.acos(dot)


def cartesian_servo_delta_from_params(params: dict[str, Any]) -> np.ndarray:
    """Parse one live-servo target; unlike a move, zero is a valid hold target."""
    values = [params.get(name) for name in ("x_mm", "y_mm", "z_mm")]
    if not all(finite_number(value) for value in values):
        raise ValueError("伺服 X、Y、Z 必须都是有限数值。")
    delta_mm = np.asarray(values, dtype=np.float64)
    return delta_mm / 1000.0


def teleoperation_target_from_params(params: dict[str, Any]) -> TeleoperationTarget:
    if not isinstance(params, dict):
        raise ValueError("遥操作 params 必须是对象。")
    try:
        return TeleoperationTarget(
            delta_xyz_m=np.asarray(params["delta_xyz_m"], dtype=np.float64),
            delta_orientation_xyzw=np.asarray(
                params["delta_orientation_xyzw"], dtype=np.float64
            ),
            gripper_state=GripperState(params["gripper_state"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"遥操作目标无效：{exc}") from exc


class ControlKernel:
    def __init__(
        self,
        interface: str,
        token: str | None = None,
        payload_root: Path | None = None,
        teleoperation_config: TeleoperationConfig | None = None,
        collection_camera_url: str | None = None,
        collection_camera_ca: Path | None = None,
        collection_camera_origin: str | None = None,
        persistent_controller: bool = False,
    ) -> None:
        self.interface = interface
        self.token = token
        self.controller: ArmController | None = None
        self.persistent_controller = persistent_controller
        self.controller_connect_lock = asyncio.Lock()
        self.controller_maintain_task: asyncio.Task[None] | None = None
        self.auto_connect_suspended = False
        self.shutting_down = False
        self.owner: ServerConnection | None = None
        self.motion_task: asyncio.Task[None] | None = None
        self.send_lock = asyncio.Lock()
        self.last_snapshot: ArmSnapshot | None = None
        self.last_snapshot_wall_time = 0.0
        self.reference_joint_position: tuple[float, ...] | None = None
        self.reference_gripper_position: float | None = None
        self.cartesian_preview: MotionPlan | None = None
        self.cartesian_preview_velocity_scale: float | None = None
        self.cartesian_preview_path_mode: str | None = None
        self.park_preview: MotionPlan | None = None
        self.park_preview_velocity_scale: float | None = None
        self.cartesian_servo_origin: Pose | None = None
        self.cartesian_servo_velocity_scale = 1.0
        self.cartesian_servo_sequence = -1
        self.cartesian_servo_mode_after_motion = ControllerMode.ACTIVE
        self.gripper_servo: GripperServo | None = None
        self.gripper_servo_sequence = -1
        self.gripper_servo_mode_after_motion = ControllerMode.ACTIVE
        self.teleoperation_config = (
            load_teleoperation_config(TELEOPERATION_CONFIG_PATH)
            if teleoperation_config is None
            else teleoperation_config
        )
        self.teleoperation_servo: TeleoperationServo | None = None
        self.teleoperation_sequence = -1
        self.teleoperation_session_id: str | None = None
        self.teleoperation_profile_name: str | None = None
        self.teleoperation_last_message_at = 0.0
        self.teleoperation_last_update_at = 0.0
        self.teleoperation_preview_only = False
        self.teleoperation_preview_start_pose: np.ndarray | None = None
        self.teleoperation_preview_start_joint: np.ndarray | None = None
        self.teleoperation_preview_seed: np.ndarray | None = None
        self.teleoperation_preview_target: TeleoperationTarget | None = None
        self.teleoperation_preview_processed_at = 0.0
        self.teleoperation_preview_generation = 0
        self.teleoperation_preview_last_payload: dict[str, Any] | None = None
        self.payload_profiles = (
            PayloadProfileReader()
            if payload_root is None
            else PayloadProfileReader(root=payload_root)
        )
        self._payload_context_profile: str | None = None
        self.payload_calibration: dict[str, Any] | None = None
        self.payload_calibration_process: asyncio.subprocess.Process | None = None
        self.payload_calibration_task: asyncio.Task[None] | None = None
        self.gripper_calibration: dict[str, Any] | None = None
        self.gripper_calibration_process: asyncio.subprocess.Process | None = None
        self.gripper_calibration_task: asyncio.Task[None] | None = None
        self.handeye_calibration: dict[str, Any] | None = None
        self.handeye_calibration_process: asyncio.subprocess.Process | None = None
        self.handeye_calibration_task: asyncio.Task[None] | None = None
        self.handeye_commands: queue.Queue[str] | None = None
        self.handeye_collector: ManualEyeToHandCollector | None = None
        self.cube_pick_preview: dict[str, Any] | None = None
        self.cube_stage_preview: dict[str, Any] | None = None
        self.cube_stack_preview: CubeStackPlan | None = None
        self.cube_manipulation: dict[str, Any] | None = None
        self.collection_config = load_collection_config()
        self.raw_episode_store = RawEpisodeStore(config=self.collection_config)
        self.collection_recorder: ArmStateRecorder | None = None
        self.collection_session_id: str | None = None
        self.collection_stage = "idle"
        self.collection_validation: dict[str, Any] | None = None
        self.collection_home: dict[str, Any] | None = None
        self.collection_timeout_task: asyncio.Task[None] | None = None
        self.collection_health_task: asyncio.Task[None] | None = None
        self.collection_camera = (
            None
            if collection_camera_url is None
            else CollectionCameraClient(
                collection_camera_url,
                token=token,
                origin=collection_camera_origin,
                ca_cert=collection_camera_ca,
            )
        )
        self.collection_camera_state: dict[str, Any] | None = None
        self.collection_camera_refreshed_at = 0.0
        self.collection_counts = self.raw_episode_store.counts()
        self.recover_collection_review()

    @property
    def is_busy(self) -> bool:
        return self.motion_task is not None and not self.motion_task.done()

    @property
    def servo_active(self) -> bool:
        cartesian = (
            None if self.controller is None else self.controller.servop_status()
        )
        return bool(
            (cartesian is not None and cartesian.active)
            or (
                self.gripper_servo is not None
                and self.gripper_servo.status().active
            )
            or (
                self.teleoperation_servo is not None
                and self.teleoperation_servo.status().active
            )
        )

    @property
    def teleoperation_session_active(self) -> bool:
        return self.teleoperation_session_id is not None

    @property
    def payload_calibration_active(self) -> bool:
        task = self.payload_calibration_task
        return task is not None and not task.done()

    @property
    def gripper_calibration_active(self) -> bool:
        task = self.gripper_calibration_task
        return task is not None and not task.done()

    def gripper_open_readout(self) -> float:
        return load_gripper_open_readout(interface=self.interface)

    @property
    def handeye_calibration_active(self) -> bool:
        state = self.handeye_calibration
        if state is not None and state.get("stage") in {
            "done",
            "cancelled",
            "incomplete",
            "error",
        }:
            return False
        task = self.handeye_calibration_task
        return task is not None and not task.done()

    def handeye_result_summary(
        self,
        result: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        if result is None:
            result_path = HANDEYE_OUTPUT_ROOT / "current.json"
            try:
                with result_path.open("r", encoding="utf-8") as stream:
                    result = json.load(stream)
            except (OSError, json.JSONDecodeError):
                return None
        if not isinstance(result, dict):
            return None
        residual = result.get("residual") or {}
        stability = result.get("leave_one_out_stability") or {}
        if not isinstance(residual, dict) or not isinstance(stability, dict):
            return None
        hand = result.get("eye_in_hand") or {}
        hand_residual = hand.get("residual") if isinstance(hand, dict) else {}
        hand_stability = (
            hand.get("leave_one_out_stability") if isinstance(hand, dict) else {}
        )
        external_relocalization = (
            result.get("eye_to_hand", {}).get("independent_validation")
            if isinstance(result.get("eye_to_hand"), dict)
            else {}
        )
        hand_relocalization = (
            hand.get("independent_validation") if isinstance(hand, dict) else {}
        )
        dual = result.get("calibration_type") == "dual_camera_hand_eye"
        return {
            "sample_count": result.get("sample_count"),
            "best_method": result.get("best_method"),
            "created_at": result.get("created_at"),
            "closure_translation_mm": (
                1000.0 * residual["closure_translation_rms_m"]
                if isinstance(residual.get("closure_translation_rms_m"), (int, float))
                else None
            ),
            "closure_rotation_deg": residual.get("closure_rotation_rms_deg"),
            "stability_translation_mm": (
                1000.0 * stability["translation_rms_m"]
                if isinstance(stability.get("translation_rms_m"), (int, float))
                else None
            ),
            "stability_rotation_deg": stability.get("rotation_rms_deg"),
            "dual": dual,
            "sample_counts": result.get("sample_counts"),
            "hand_closure_translation_mm": (
                1000.0 * hand_residual["closure_translation_rms_m"]
                if isinstance(hand_residual, dict)
                and isinstance(
                    hand_residual.get("closure_translation_rms_m"), (int, float)
                )
                else None
            ),
            "hand_closure_rotation_deg": (
                hand_residual.get("closure_rotation_rms_deg")
                if isinstance(hand_residual, dict)
                else None
            ),
            "hand_stability_translation_mm": (
                1000.0 * hand_stability["translation_rms_m"]
                if isinstance(hand_stability, dict)
                and isinstance(hand_stability.get("translation_rms_m"), (int, float))
                else None
            ),
            "relocalization_translation_mm": (
                1000.0 * external_relocalization["closure_translation_rms_m"]
                if isinstance(external_relocalization, dict)
                and isinstance(
                    external_relocalization.get("closure_translation_rms_m"),
                    (int, float),
                )
                else None
            ),
            "relocalization_rotation_deg": (
                external_relocalization.get("closure_rotation_rms_deg")
                if isinstance(external_relocalization, dict)
                else None
            ),
            "hand_relocalization_translation_mm": (
                1000.0 * hand_relocalization["closure_translation_rms_m"]
                if isinstance(hand_relocalization, dict)
                and isinstance(
                    hand_relocalization.get("closure_translation_rms_m"),
                    (int, float),
                )
                else None
            ),
            "hand_relocalization_rotation_deg": (
                hand_relocalization.get("closure_rotation_rms_deg")
                if isinstance(hand_relocalization, dict)
                else None
            ),
            "session": result.get("session"),
            "phase": result.get("calibration_role"),
            "combined_profile_updated": result.get("combined_profile_updated"),
            "standalone_profile_updated": result.get("standalone_profile_updated"),
        }

    def handeye_calibration_state(self) -> dict[str, Any] | None:
        state = self.handeye_calibration
        if state is None:
            result = self.handeye_result_summary()
            if result is None:
                return None
            return {
                "active": False,
                "stage": "cached",
                "message": "已有手眼标定结果，可在采集窗口重新标定。",
                "progress": None,
                "quality": None,
                "return_to_reference": True,
                "return_reference_label": "网页参考位",
                "result": result,
                "error": None,
            }
        return {
            "active": self.handeye_calibration_active,
            "stage": state.get("stage"),
            "message": state.get("message"),
            "progress": state.get("progress"),
            "quality": state.get("quality"),
            "result": state.get("result"),
            "error": state.get("error"),
            "feedback": state.get("feedback"),
            "manual_ready": state.get("manual_ready", False),
            "mode": state.get("mode", "dual"),
            "return_to_reference": bool(state.get("return_to_reference", True)),
            "return_reference_label": state.get("return_reference_label"),
        }

    def handeye_validation_state(self) -> dict[str, Any]:
        """Expose solve validation and activation independently of collection."""
        root = HANDEYE_OUTPUT_ROOT
        result_path = root / "current.json"
        current = read_json(result_path)
        validation = read_json(root / "validation.json")
        enabled = read_json(root / "enabled.json")
        enabled_requested = enabled is not None and enabled.get("enabled") is True
        current_stamp = current.get("created_at") if current else None
        enabled_stamp = (
            enabled.get("validation", {}).get("source_created_at")
            if enabled_requested and isinstance(enabled.get("validation"), dict)
            else None
        )
        enabled_matches = bool(
            enabled_requested
            and (
                (current_stamp is not None and enabled_stamp == current_stamp)
                or (
                    current_stamp is None
                    and current is not None
                    and enabled.get("T_base_camera") == current.get("T_base_camera")
                )
            )
        )
        raw_reviews = current.get("operator_reviews", {}) if current else {}
        operator_reviewed = bool(
            isinstance(raw_reviews, dict)
            and all(
                isinstance(raw_reviews.get(role), dict)
                and raw_reviews[role].get("accepted") is True
                and isinstance(raw_reviews[role].get("fit_sample_count"), int)
                and raw_reviews[role]["fit_sample_count"] >= 20
                for role in ("eye_in_hand", "eye_to_hand")
            )
        )
        return {
            "available": result_path.is_file(),
            "validated": validation is not None,
            "passed": validation.get("passed") is True if validation else False,
            "enabled": enabled_matches,
            "stale": enabled_requested and not enabled_matches,
            "validation": validation,
            "operator_reviewed": operator_reviewed,
            "operator_override": bool(
                enabled_matches and enabled.get("operator_override") is True
            ),
            "enabled_at": enabled.get("enabled_at") if enabled else None,
            "T_base_camera": enabled.get("T_base_camera")
            if enabled_matches
            else None,
            "T_eef_camera": enabled.get("T_eef_camera")
            if enabled_matches
            else None,
            "profile": (
                enabled.get("profile", "eye-to-hand")
                if enabled_matches
                else (
                    current.get("calibration_type", "eye-to-hand")
                    if current
                    else "eye-to-hand"
                )
            ),
        }

    async def send_handeye_validation_state(
        self,
        websocket: ServerConnection,
        *,
        request_id: str | int | None = None,
        action: str | None = None,
        ok: bool | None = None,
        error: str | None = None,
    ) -> None:
        payload = {
            "type": "handeye_validation",
            "validation": self.handeye_validation_state(),
        }
        await self.send(websocket, payload)
        if request_id is not None:
            await self.send(
                websocket,
                {
                    "type": "command_result",
                    "id": request_id,
                    "action": action,
                    "ok": bool(ok),
                    "error": error,
                    "result": self.handeye_validation_state(),
                    "state": self.state_payload(),
                },
            )

    async def validate_handeye_result(
        self,
        websocket: ServerConnection,
        request_id: str | int,
    ) -> None:
        if (
            self.handeye_calibration_active
            or self.payload_calibration_active
            or self.gripper_calibration_active
        ):
            await self.send_error(
                websocket, request_id, "标定流程正在进行，请完成或取消后再验证。",
                action="handeye_validate",
            )
            return
        root = HANDEYE_OUTPUT_ROOT
        try:
            validate_cached_result(root)
        except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
            await self.send_error(
                websocket, request_id, str(exc), action="handeye_validate"
            )
            return
        await self.send_handeye_validation_state(
            websocket, request_id=request_id, action="handeye_validate", ok=True
        )

    async def enable_handeye_result(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        *,
        operator_override: bool = False,
    ) -> None:
        root = HANDEYE_OUTPUT_ROOT
        try:
            enable_validated_result(root, operator_override=operator_override)
        except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
            await self.send_error(
                websocket, request_id, str(exc), action="handeye_enable"
            )
            return
        await self.send_handeye_validation_state(
            websocket, request_id=request_id, action="handeye_enable", ok=True
        )

    async def disable_handeye_result(
        self,
        websocket: ServerConnection,
        request_id: str | int,
    ) -> None:
        root = HANDEYE_OUTPUT_ROOT
        try:
            disable_result(root)
        except (OSError, ValueError) as exc:
            await self.send_error(
                websocket, request_id, str(exc), action="handeye_disable"
            )
            return
        await self.send_handeye_validation_state(
            websocket, request_id=request_id, action="handeye_disable", ok=True
        )

    @staticmethod
    def _cube_pick_observation(params: dict[str, Any], config: Any) -> Any:
        status = params.get("status", "tracking")
        if status != "tracking":
            raise ControlError("方块当前没有稳定跟踪结果，请等待检测稳定后重试。")
        position = params.get("base_xyz_m")
        if (
            not isinstance(position, (list, tuple))
            or len(position) != 3
            or not all(finite_number(value) for value in position)
        ):
            raise ValueError("方块基座坐标无效，请等待 RGB 检测结果。")
        x_m, y_m, z_m = (float(value) for value in position)
        confidence = params.get("confidence", 0.0)
        size_m = params.get("size_m", params.get("size_estimate_m", config.cube_edge_m))
        yaw_rad = params.get("yaw_rad", 0.0)
        if not finite_number(confidence) or float(confidence) < config.perception.minimum_confidence:
            raise ControlError(
                f"方块置信度不足（需要至少 {config.perception.minimum_confidence:.2f}）。"
            )
        if not finite_number(size_m) or not (
            config.perception.minimum_size_m <= float(size_m) <= config.perception.maximum_size_m
        ):
            raise ControlError(
                "方块尺寸不在安全范围内："
                f"{1000.0 * config.perception.minimum_size_m:.0f}.."
                f"{1000.0 * config.perception.maximum_size_m:.0f} mm。"
            )
        if not finite_number(yaw_rad):
            raise ValueError("方块 yaw 无效。")
        expected_z = config.table.top_z_m + config.cube_edge_m / 2.0
        if abs(z_m - expected_z) > 0.035:
            raise ControlError("方块高度与桌面模型不一致，请重新检查桌面标定。")
        if not point_inside_table(config, x_m, y_m):
            raise ControlError("方块中心不在配置的桌面安全范围内。")
        return SimpleNamespace(
            base_xyz_m=(x_m, y_m, z_m),
            yaw_rad=float(yaw_rad),
            size_m=float(size_m),
            confidence=float(confidence),
            position_source=str(params.get("position_source", "web-rgb")),
            accepted_samples=max(2, int(params.get("accepted_samples", 2))),
            depth_samples=max(0, int(params.get("depth_samples", 0))),
            xy_rms_m=0.0,
            yaw_rms_rad=0.0,
            timestamp=float(params.get("timestamp", time.time())),
        )

    @staticmethod
    def _cube_pick_execute(controller: Any, actions: tuple[Any, ...], config: Any) -> None:
        """Execute a preflighted cube plan for the browser.

        This intentionally does not reuse ``planner.execute_action_plan``. The
        web path scales gripper timing by the operator's speed tier, enforces a
        plausible contact width before lifting, and emits machine-readable
        ``CUBE_GRIPPER`` telemetry; the terminal experiment uses fixed gripper
        durations and human-readable progress. Unifying them would change real
        robot timing on the experiment path, so only the geometry and preflight
        are shared.
        """

        def gripper_duration(delta_m: float) -> float:
            required = controller.minimum_gripper_duration(delta_m)
            velocity_scale = max(0.2, float(config.motion.velocity_scale))
            return max(0.5, required / velocity_scale)

        def move_to(action: Any) -> None:
            target = Pose(
                np.asarray(action.target_xyz_m, dtype=np.float64),
                np.asarray(action.orientation_xyzw, dtype=np.float64),
            )
            if action.path_mode == "smooth":
                controller.movej(
                    target,
                    velocity_scale=config.motion.velocity_scale,
                    allow_large_motion=True,
                )
            elif action.path_mode == "linear":
                controller.movel(
                    target,
                    velocity_scale=config.motion.velocity_scale,
                    unrestricted=True,
                )
            else:
                raise ControlError(
                    f"方块动作 {action.label!r} 缺少有效轨迹模式。"
                )

        def force_close() -> Any:
            return controller.force_close_gripper(
                config.motion.closing_torque_nm,
                minimum_width_m=config.motion.minimum_gripper_width_m,
                timeout_s=config.motion.gripper_timeout_s,
                contact_hold_s=config.motion.contact_hold_s,
                position_hold_after=True,
                torque_hold_after=True,
            )

        index = 0
        while index < len(actions):
            action = actions[index]
            if action.kind == "move":
                blended = blended_linear_actions(actions, index)
                if blended:
                    controller.move_path(
                        cartesian_path_for_actions(blended),
                        velocity_scale=config.motion.velocity_scale,
                        unrestricted=True,
                    )
                    index += len(blended)
                    continue
                move_to(action)
            elif action.kind == "gripper":
                state = controller.get_state()
                duration = gripper_duration(
                    action.gripper_width_m - state.gripper_position
                )
                controller.move_gripper(action.gripper_width_m, duration)
            elif action.kind == "force_close":
                result = force_close()
                maximum_contact_width_m = maximum_plausible_contact_width_m(config)
                plausible_contact = (
                    result.stop_reason == "contact"
                    and result.final_width_m <= maximum_contact_width_m
                )
                print(
                    "CUBE_GRIPPER "
                    f"stop={result.stop_reason} "
                    f"width_mm={1000.0 * result.final_width_m:.1f} "
                    f"torque_nm={result.max_abs_torque_nm:.3f} "
                    f"elapsed_s={result.elapsed_s:.2f} "
                    f"plausible={int(plausible_contact)}",
                    flush=True,
                )
                if not plausible_contact:
                    final_width_mm = 1000.0 * float(
                        getattr(result, "final_width_m", float("nan"))
                    )
                    max_torque_nm = float(
                        getattr(result, "max_abs_torque_nm", float("nan"))
                    )
                    elapsed_s = float(getattr(result, "elapsed_s", float("nan")))
                    raise RuntimeError(
                        "未确认有效夹取接触，拒绝抬升"
                        f"（{result.stop_reason}；末端开度 {final_width_mm:.1f} mm；"
                        f"峰值扭矩 {max_torque_nm:.2f} N·m；耗时 {elapsed_s:.1f} s）。"
                    )
            elif action.kind == "wait":
                time.sleep(float(action.wait_s or 0.0))
            elif action.kind == "damping":
                controller.damping()
            index += 1

    @staticmethod
    def _hold_after_cube_failure(controller: Any, reason: str) -> None:
        if controller.mode != ControllerMode.ACTIVE:
            return
        try:
            controller.hold_current()
        except BaseException:
            controller.emergency_damping(reason)

    def prepare_cube_pick_preview(self, params: dict[str, Any]) -> dict[str, Any]:
        controller = self.controller
        self.cube_pick_preview = None
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        if controller.mode not in (ControllerMode.DAMPING, ControllerMode.ACTIVE):
            raise ControlError("当前模式不能预览抓取动作。")
        if self.handeye_validation_state().get("enabled") is not True:
            raise ControlError("固定相机手眼标定结果尚未验证并启用，不能自动抓取。")
        config = load_config()
        scene_table = params.get("scene_table")
        if isinstance(scene_table, dict):
            position_mm = scene_table.get("position_mm")
            size_mm = scene_table.get("size_mm")
            yaw_deg = scene_table.get("yaw_deg")
            if (
                not isinstance(position_mm, (list, tuple))
                or len(position_mm) != 3
                or not all(finite_number(value) for value in position_mm)
                or not isinstance(size_mm, (list, tuple))
                or len(size_mm) != 2
                or not all(finite_number(value) for value in size_mm)
                or not finite_number(yaw_deg)
            ):
                raise ValueError("网页桌面标定参数无效。")
            width_m, depth_m = (float(value) / 1000.0 for value in size_mm)
            if not 0.2 <= width_m <= 4.0 or not 0.2 <= depth_m <= 3.0:
                raise ValueError("网页桌面尺寸超出抓取安全范围。")
            config = replace(
                config,
                table=replace(
                    config.table,
                    center_x_m=float(position_mm[0]) / 1000.0,
                    center_y_m=float(position_mm[1]) / 1000.0,
                    top_z_m=float(position_mm[2]) / 1000.0,
                    width_m=width_m,
                    depth_m=depth_m,
                    yaw_rad=math.radians(float(yaw_deg)),
                ),
            )
        speed_percent = params.get("speed_percent", WEB_DEFAULT_SPEED_PERCENT)
        if not finite_number(speed_percent) or float(speed_percent) not in WEB_SPEED_TIERS_PERCENT:
            raise ValueError(
                "抓取速度必须是 1%、2%、3%、4%、5%、10%、25%、50%、75% 或 100%。"
            )
        config = replace(
            config,
            motion=replace(config.motion, velocity_scale=float(speed_percent) / 100.0),
        )
        observation = self._cube_pick_observation(params, config)
        initial = controller.get_state()
        validate_initial_pose(initial.eef_pose_6d, config)
        selected_plan: tuple[
            float,
            str,
            str,
            tuple[Any, ...],
            tuple[Any, ...],
        ] | None = None
        failures: list[str] = []
        candidates = grasp_orientation_candidates(observation, initial.eef_pose_6d)
        preferred = tuple(
            candidate
            for candidate in candidates
            if CUBE_PREFERRED_SIDE_PROFILE in candidate.name
        )
        preferred_names = {candidate.name for candidate in preferred}
        fallback = tuple(
            candidate for candidate in candidates if candidate.name not in preferred_names
        )
        for candidate_group in (preferred, fallback):
            group_plan = None
            for candidate in candidate_group:
                try:
                    actions = build_pick_actions(
                        observation,
                        config,
                        candidate.orientation_xyzw,
                        grasp_mode=candidate.grasp_mode,
                        approach_xy=candidate.approach_xy,
                    )
                    preflight = preflight_action_plan(controller, actions)
                except (RuntimeError, ValueError, CartesianPlanningError) as exc:
                    failures.append(str(exc))
                    continue
                score = max(
                    (item.maximum_joint_delta_rad for item in preflight),
                    default=float("inf"),
                )
                if group_plan is None or score < group_plan[0]:
                    group_plan = (
                        score,
                        candidate.name,
                        candidate.grasp_mode,
                        actions,
                        preflight,
                    )
            if group_plan is not None:
                selected_plan = group_plan
                break
        if selected_plan is None:
            failure_counts: dict[str, int] = {}
            for failure in failures:
                detail = failure.removeprefix("inverse kinematics failed:").strip()
                failure_counts[detail] = failure_counts.get(detail, 0) + 1
            summary = "；".join(
                f"{detail} x{count}"
                for detail, count in sorted(
                    failure_counts.items(),
                    key=lambda item: (-item[1], item[0]),
                )[:3]
            ) or "inverse kinematics returned no candidate"
            x_m, y_m, z_m = observation.base_xyz_m
            raise ControlError(
                "抓取点暂无可达逆解；已尝试等价俯抓及多个方向、倾角的侧抓姿态。"
                f"方块基座坐标 ({1000.0 * x_m:.0f}, {1000.0 * y_m:.0f}, "
                f"{1000.0 * z_m:.0f}) mm；失败汇总：{summary}"
            )
        _, orientation_name, grasp_mode, actions, preflight = selected_plan
        tilt_match = re.search(r"_tilt_(-?\d+)_", orientation_name)
        grasp_tilt_deg = int(tilt_match.group(1)) if tilt_match else 90
        minimum_tcp_z = min(
            (item.minimum_tcp_z_m for item in preflight),
            default=float("inf"),
        )
        if minimum_tcp_z < config.table.top_z_m + 0.005:
            raise SafetyError("抓取预检轨迹过于接近或穿过桌面，已拒绝执行。")
        self.cube_pick_preview = {
            "observation": observation,
            "actions": actions,
            "config": config,
            "start_joint_position": np.asarray(initial.joint_position, dtype=np.float64).copy(),
            "start_pose": np.asarray(initial.eef_pose_6d, dtype=np.float64).copy(),
            "created_at": time.time(),
        }
        steps = []
        grasp_action = None
        for action in actions:
            item: dict[str, Any] = {"label": action.label, "kind": action.kind}
            if action.target_xyz_m is not None:
                item["target_xyz_mm"] = [1000.0 * float(value) for value in action.target_xyz_m]
            if action.orientation_xyzw is not None:
                item["orientation_xyzw"] = [
                    float(value) for value in action.orientation_xyzw
                ]
            if action.gripper_width_m is not None:
                item["gripper_width_mm"] = 1000.0 * float(action.gripper_width_m)
            if action.wait_s is not None:
                item["wait_s"] = float(action.wait_s)
            if action.label in {"horizontal side grasp", "vertical grasp descent"}:
                grasp_action = action
            steps.append(item)
        path_xyz_m = [np.asarray(initial.eef_pose_6d[:3], dtype=np.float64)]
        for item in preflight:
            path_xyz_m.extend(item.path_xyz_m)
        return {
            "kind": "cube_pick_preview",
            "observation": {
                "base_xyz_m": list(observation.base_xyz_m),
                "yaw_rad": observation.yaw_rad,
                "size_m": observation.size_m,
                "confidence": observation.confidence,
                "position_source": observation.position_source,
            },
            "steps": steps,
            "preflight": [
                {
                    "label": item.label,
                    "max_joint_delta_deg": math.degrees(item.maximum_joint_delta_rad),
                    "minimum_tcp_z_mm": 1000.0 * item.minimum_tcp_z_m,
                }
                for item in preflight
            ],
            "path_xyz_mm": [
                [1000.0 * float(value) for value in point]
                for point in path_xyz_m
            ],
            "velocity_percent": 100.0 * config.motion.velocity_scale,
            "grasp_mode": grasp_mode,
            "grasp_tilt_deg": grasp_tilt_deg,
            "grasp_orientation": orientation_name,
            "grasp_pose_mm": (
                [1000.0 * float(value) for value in grasp_action.target_xyz_m]
                if grasp_action is not None
                else None
            ),
            "grasp_orientation_xyzw": (
                [float(value) for value in grasp_action.orientation_xyzw]
                if grasp_action is not None
                else None
            ),
            "grasp_width_mm": 1000.0 * config.cube_edge_m,
            "valid_for_s": CUBE_PICK_PREVIEW_TIMEOUT_S,
            "requires_confirmation": True,
        }

    @staticmethod
    def _cube_stage_preview_payload(
        kind: str,
        initial: Any,
        actions: tuple[Any, ...],
        preflight: tuple[Any, ...],
        config: Any,
    ) -> dict[str, Any]:
        path_xyz_m = [np.asarray(initial.eef_pose_6d[:3], dtype=np.float64)]
        for item in preflight:
            path_xyz_m.extend(item.path_xyz_m)
        preview_move = next(
            (action for action in reversed(actions) if action.kind == "move"),
            None,
        )
        return {
            "kind": kind,
            "steps": [
                {
                    "label": action.label,
                    "kind": action.kind,
                    **(
                        {
                            "target_xyz_mm": [
                                1000.0 * float(value)
                                for value in action.target_xyz_m
                            ]
                        }
                        if action.target_xyz_m is not None
                        else {}
                    ),
                }
                for action in actions
            ],
            "path_xyz_mm": [
                [1000.0 * float(value) for value in point]
                for point in path_xyz_m
            ],
            "preview_tcp_pose": (
                {
                    "target_xyz_mm": [
                        1000.0 * float(value)
                        for value in preview_move.target_xyz_m
                    ],
                    "orientation_xyzw": [
                        float(value)
                        for value in preview_move.orientation_xyzw
                    ],
                }
                if preview_move is not None
                else None
            ),
            "velocity_percent": 100.0 * config.motion.velocity_scale,
            "valid_for_s": CUBE_PICK_PREVIEW_TIMEOUT_S,
            "requires_confirmation": True,
        }

    def prepare_cube_move_preview(self, params: dict[str, Any]) -> dict[str, Any]:
        controller = self.controller
        state = self.cube_manipulation
        self.cube_stage_preview = None
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        if controller.mode not in (ControllerMode.DAMPING, ControllerMode.ACTIVE):
            raise ControlError("当前模式不能预览持物移动。")
        if state is None or state.get("stage") != "held":
            raise ControlError("请先抓取并抬升一个方块。")
        config = state["config"]
        config = replace(
            config,
            motion=replace(
                config.motion,
                velocity_scale=velocity_scale_from_params(params),
            ),
        )
        target = self._cube_pick_observation(params, config)
        source_xy = np.asarray(state["source_xy_m"], dtype=np.float64)
        target_xy = np.asarray(target.base_xyz_m[:2], dtype=np.float64)
        if float(np.linalg.norm(target_xy - source_xy)) < config.cube_edge_m * 1.25:
            raise ControlError("请在 RGB 画面中点选另一个方块作为堆叠目标。")
        place_z = (
            config.table.top_z_m
            + 1.5 * config.cube_edge_m
            + config.motion.tcp_z_offset_m
        )
        hover_z = place_z + config.motion.lift_height_m
        orientation = tuple(float(value) for value in state["orientation_xyzw"])
        blend_radius_m = min(
            config.cube_edge_m / 2.0,
            config.motion.lift_height_m / 4.0,
        )
        actions = (
            CubeAction(
                kind="move",
                label="transfer held cube above target",
                target_xyz_m=(float(target_xy[0]), float(target_xy[1]), hover_z),
                orientation_xyzw=orientation,
                path_mode="linear",
                blend_radius_m=blend_radius_m,
            ),
            CubeAction(
                kind="move",
                label="vertical stack descent",
                target_xyz_m=(float(target_xy[0]), float(target_xy[1]), place_z),
                orientation_xyzw=orientation,
                path_mode="linear",
            ),
        )
        initial = controller.get_state()
        preflight = preflight_action_plan(controller, actions)
        minimum_tcp_z = min(
            (item.minimum_tcp_z_m for item in preflight),
            default=float("inf"),
        )
        if minimum_tcp_z < config.table.top_z_m + 0.005:
            raise SafetyError("持物移动轨迹过于接近桌面，已拒绝执行。")
        self.cube_stage_preview = {
            "kind": "move",
            "actions": actions,
            "config": config,
            "target": target,
            "target_xy_m": tuple(float(value) for value in target_xy),
            "place_z_m": place_z,
            "hover_z_m": hover_z,
            "orientation_xyzw": orientation,
            "start_joint_position": np.asarray(initial.joint_position, dtype=np.float64).copy(),
            "start_pose": np.asarray(initial.eef_pose_6d, dtype=np.float64).copy(),
            "created_at": time.time(),
        }
        return self._cube_stage_preview_payload(
            "cube_move_preview", initial, actions, preflight, config
        )

    def prepare_cube_place_preview(self, params: dict[str, Any]) -> dict[str, Any]:
        controller = self.controller
        state = self.cube_manipulation
        self.cube_stage_preview = None
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        if controller.mode not in (ControllerMode.DAMPING, ControllerMode.ACTIVE):
            raise ControlError("当前模式不能预览放置动作。")
        if state is None or state.get("stage") != "over_target":
            raise ControlError("请先将持有的方块移动到目标方块上方。")
        config = state["config"]
        config = replace(
            config,
            motion=replace(
                config.motion,
                velocity_scale=velocity_scale_from_params(params),
            ),
        )
        target_x, target_y = state["target_xy_m"]
        hover_z = float(state["hover_z_m"])
        orientation = tuple(float(value) for value in state["orientation_xyzw"])
        actions = (
            CubeAction(
                kind="gripper",
                label="release stacked cube",
                gripper_width_m=config.motion.release_width_m,
            ),
            CubeAction(
                kind="wait",
                label="release settle",
                wait_s=config.motion.settle_s,
            ),
            CubeAction(
                kind="move",
                label="vertical retreat after stack",
                target_xyz_m=(target_x, target_y, hover_z),
                orientation_xyzw=orientation,
                path_mode="linear",
            ),
        )
        initial = controller.get_state()
        preflight = preflight_action_plan(controller, actions)
        minimum_tcp_z = min(
            (item.minimum_tcp_z_m for item in preflight),
            default=float("inf"),
        )
        if minimum_tcp_z < config.table.top_z_m + config.cube_edge_m + 0.001:
            raise SafetyError("堆叠下降轨迹低于目标方块顶面，已拒绝执行。")
        self.cube_stage_preview = {
            "kind": "place",
            "actions": actions,
            "config": config,
            "start_joint_position": np.asarray(initial.joint_position, dtype=np.float64).copy(),
            "start_pose": np.asarray(initial.eef_pose_6d, dtype=np.float64).copy(),
            "created_at": time.time(),
        }
        return self._cube_stage_preview_payload(
            "cube_place_preview", initial, actions, preflight, config
        )

    def prepare_cube_stack_preview(self, params: dict[str, Any]) -> dict[str, Any]:
        controller = self.controller
        self.cube_stack_preview = None
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        if controller.mode not in (ControllerMode.DAMPING, ControllerMode.ACTIVE):
            raise ControlError("当前模式不能预览连续叠放。")
        source_params = params.get("source")
        target_params = params.get("target")
        if not isinstance(source_params, dict) or not isinstance(target_params, dict):
            raise ValueError("连续叠放需要有效的源方块和目标方块。")
        speed_percent = params.get("speed_percent", WEB_DEFAULT_SPEED_PERCENT)
        pick_payload = self.prepare_cube_pick_preview(
            {**source_params, "speed_percent": speed_percent}
        )
        pick_preview = self.cube_pick_preview
        if pick_preview is None:
            raise ControlError("抓取子计划未能生成。")
        config = pick_preview["config"]
        target = self._cube_pick_observation(target_params, config)
        source = pick_preview["observation"]
        if math.hypot(
            target.base_xyz_m[0] - source.base_xyz_m[0],
            target.base_xyz_m[1] - source.base_xyz_m[1],
        ) < config.cube_edge_m * 1.25:
            raise ControlError("源方块和目标方块距离过近。")
        initial = controller.get_state()
        reference = self.reference_joint_position
        if reference is None:
            raise ControlError("尚未记录任务起始参考状态。")
        if math.degrees(
            float(
                np.max(
                    np.abs(
                        initial.joint_position
                        - np.asarray(reference, dtype=np.float64)
                    )
                )
            )
        ) > WEB_CARTESIAN_PREVIEW_JOINT_TOLERANCE_DEG:
            raise ControlError("机械臂已离开任务起始参考位，请重新开始一键叠放。")

        self.cube_stack_preview = plan_cube_stack(
            controller,
            initial=initial,
            pick_actions=pick_preview["actions"],
            target=target,
            config=config,
            reference_joint_position=tuple(reference),
        )
        self.cube_pick_preview = None
        return self.cube_stack_preview.payload()

    def payload_calibration_state(self) -> dict[str, Any] | None:
        state = self.payload_calibration
        if state is None:
            return None
        return {
            "active": self.payload_calibration_active,
            "mode": state.get("mode"),
            "profile": state.get("profile"),
            "stage": state.get("stage"),
            "progress": state.get("progress"),
            "message": state.get("message"),
            "error": state.get("error"),
        }

    async def send(self, websocket: ServerConnection, payload: dict[str, Any]) -> None:
        try:
            async with self.send_lock:
                await websocket.send(json.dumps(payload, separators=(",", ":")))
        except ConnectionClosed:
            pass

    async def send_payload_calibration_state(
        self,
        websocket: ServerConnection,
        *,
        message: str | None = None,
    ) -> None:
        calibration = self.payload_calibration_state()
        if calibration is None:
            return
        if message is not None:
            if self.payload_calibration is not None:
                self.payload_calibration["message"] = message
            calibration["message"] = message
        await self.send(
            websocket,
            {
                "type": "payload_calibration",
                "calibration": calibration,
                "state": self.state_payload(),
            },
        )

    async def _write_payload_calibration_input(self, value: str) -> None:
        process = self.payload_calibration_process
        if process is None or process.stdin is None or process.returncode is not None:
            raise ControlError("负载标定进程已经结束，请重新开始。")
        process.stdin.write(value.encode("utf-8"))
        await process.stdin.drain()

    async def start_payload_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        params: dict[str, Any],
    ) -> None:
        if self.controller is not None:
            await self.send_error(
                websocket,
                request_id,
                "请先断开机械臂，再开始负载标定。",
                action="payload_calibration_start",
            )
            return
        if (
            self.payload_calibration_active
            or self.gripper_calibration_active
            or self.handeye_calibration_active
        ):
            await self.send_error(
                websocket,
                request_id,
                "已有标定正在进行。",
                action="payload_calibration_start",
            )
            return
        if params.get("safety_confirmed") is not True:
            await self.send_error(
                websocket,
                request_id,
                "请先在网页确认工作区、实体急停和负载承托均已准备好。",
                action="payload_calibration_start",
            )
            return
        mode = params.get("mode")
        if mode not in PAYLOAD_CALIBRATION_MODES:
            await self.send_error(
                websocket,
                request_id,
                "标定类型必须是 fixed 或 grasped。",
                action="payload_calibration_start",
            )
            return
        try:
            profile = validate_profile_name(params.get("profile", "last"))
        except (TypeError, RuntimeError) as exc:
            await self.send_error(
                websocket,
                request_id,
                str(exc),
                action="payload_calibration_start",
            )
            return
        root = PROJECT_ROOT
        required = (
            PAYLOAD_CALIBRATION_ROOT / "poses.json",
            PAYLOAD_CALIBRATION_ROOT / "baseline.json",
        )
        missing = [str(path) for path in required if not path.exists()]
        if missing:
            await self.send_error(
                websocket,
                request_id,
                "缺少示教计划或空载基线，请先在终端完成 calibrate-payload teach/capture baseline。",
                action="payload_calibration_start",
            )
            return

        command = [
            sys.executable,
            "-B",
            "-m",
            "legacy.arx_tools.calibrate_payload",
            "capture",
            "loaded",
        ]
        if mode == "grasped":
            command.append("--hold-gripper")
        self.payload_calibration = {
            "mode": mode,
            "profile": profile,
            "stage": "starting",
            "progress": {"current": 0, "total": 11},
            "message": "正在准备标定进程……",
            "error": None,
            "request_id": request_id,
        }
        await self.send(
            websocket,
            {
                "type": "command_started",
                "id": request_id,
                "action": "payload_calibration_start",
            },
        )
        try:
            self.payload_calibration_process = await asyncio.create_subprocess_exec(
                *command,
                cwd=root,
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
            )
        except OSError as exc:
            self.payload_calibration = None
            await self.send_error(
                websocket,
                request_id,
                f"无法启动标定进程：{exc}",
                action="payload_calibration_start",
            )
            return
        self.payload_calibration_task = asyncio.create_task(
            self.run_payload_calibration(websocket, request_id)
        )
        await self.send_payload_calibration_state(
            websocket,
            message="标定进程已启动；正在连接控制器。",
        )

    async def confirm_payload_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        params: dict[str, Any],
    ) -> None:
        if not self.payload_calibration_active or self.payload_calibration is None:
            await self.send_error(
                websocket,
                request_id,
                "当前没有等待确认的负载标定。",
                action="payload_calibration_confirm",
            )
            return
        stage = params.get("stage")
        expected = self.payload_calibration.get("stage")
        if stage != expected or stage not in {"center", "run", "releasing"}:
            await self.send_error(
                websocket,
                request_id,
                "标定流程阶段已变化，请按网页当前提示操作。",
                action="payload_calibration_confirm",
            )
            return
        if stage == "center":
            value = "\n"
            message = "正在检查中心姿态；通过后会等待最终 RUN 确认。"
        elif stage == "run":
            value = "RUN\n"
            message = "正在低速回放 11 个姿态并采样。"
        else:
            value = "\n"
            message = "正在释放夹爪保持并完成采集。"
        self.payload_calibration["stage"] = "connecting"
        self.payload_calibration["message"] = message
        await self._write_payload_calibration_input(value)
        await self.send(
            websocket,
            {
                "type": "command_result",
                "id": request_id,
                "action": "payload_calibration_confirm",
                "ok": True,
                "result": {"stage": stage},
                "state": self.state_payload(),
            },
        )
        await self.send_payload_calibration_state(websocket)

    async def cancel_payload_calibration(self, reason: str = "用户取消") -> None:
        task = self.payload_calibration_task
        process = self.payload_calibration_process
        if task is None or task.done():
            return
        if self.payload_calibration is not None:
            self.payload_calibration["message"] = reason
        if process is not None and process.returncode is None:
            try:
                process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            await asyncio.wait_for(task, timeout=8.0)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            if process is not None and process.returncode is None:
                process.kill()
            if not task.done():
                task.cancel()
        self.payload_calibration_process = None

    async def _consume_payload_output(
        self,
        websocket: ServerConnection,
        process: asyncio.subprocess.Process,
    ) -> int:
        if process.stdout is None:
            raise RuntimeError("标定进程没有输出管道。")
        buffer = ""
        while True:
            chunk = await process.stdout.read(1)
            if not chunk:
                break
            buffer += chunk.decode("utf-8", errors="replace")
            state = self.payload_calibration
            if state is None:
                continue
            if state.get("stage") == "starting" and "按回车连接" in buffer:
                state["stage"] = "connecting"
                state["message"] = "正在连接机械臂；连接后请按网页提示摆到中心。"
                await self._write_payload_calibration_input("\n")
                await self.send_payload_calibration_state(websocket)
            if (
                state.get("stage") not in {"center", "run", "releasing"}
                and "已摆到中心附近并松手后按回车检查" in buffer
            ):
                state["stage"] = "center"
                state["message"] = "请手动摆到示教中心附近，松手后点击“中心已就位”。"
                await self.send_payload_calibration_state(websocket)
            if (
                state.get("stage") not in {"run", "releasing"}
                and "输入 RUN 后开始低速自动回放" in buffer
            ):
                state["stage"] = "run"
                state["message"] = "中心姿态检查通过；确认范围安全后点击“开始采集”。"
                await self.send_payload_calibration_state(websocket)
            if (
                state.get("stage") != "releasing"
                and "采集完成。请托住物体" in buffer
            ):
                state["stage"] = "releasing"
                state["message"] = "请托住物体后点击“已托住，释放夹爪”。"
                await self.send_payload_calibration_state(websocket)
            if "\n" in buffer:
                lines = buffer.split("\n")
                buffer = lines.pop()
                for line in lines:
                    progress = PAYLOAD_CALIBRATION_PROGRESS_RE.search(line)
                    if progress:
                        state["progress"] = {
                            "current": int(progress.group(1)),
                            "total": int(progress.group(2)),
                        }
                        state["message"] = (
                            f"正在采样 {progress.group(1)}/{progress.group(2)}。"
                        )
                        await self.send_payload_calibration_state(websocket)
        return await process.wait()

    async def run_payload_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
    ) -> None:
        state = self.payload_calibration
        process = self.payload_calibration_process
        if state is None or process is None:
            return
        try:
            return_code = await self._consume_payload_output(websocket, process)
            if return_code != 0:
                raise RuntimeError(
                    "标定采集未完成；请检查控制器输出并确认机械臂已回到阻尼。"
                )
            state["stage"] = "fitting"
            state["message"] = "采样完成，正在拟合质量与 TCP 重心并写入缓存。"
            await self.send_payload_calibration_state(websocket)
            root = PROJECT_ROOT
            fit_command = [
                sys.executable,
                "-B",
                "-m",
                "legacy.arx_tools.calibrate_payload",
                "fit-payload",
                "--profile",
                str(state["profile"]),
                "--mount-mode",
                str(state["mode"]),
            ]
            fit_process = await asyncio.create_subprocess_exec(
                *fit_command,
                cwd=root,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
            )
            self.payload_calibration_process = fit_process
            fit_code = await self._consume_payload_output(websocket, fit_process)
            if fit_code != 0:
                raise RuntimeError("负载拟合失败，请检查 baseline/loaded 数据。")
            self.payload_profiles._next_refresh = 0.0
            state["stage"] = "done"
            state["message"] = "负载已拟合并缓存；网页状态将显示估计质量。"
            await self.send_payload_calibration_state(websocket)
            await self.send(
                websocket,
                {
                    "type": "command_result",
                    "id": request_id,
                    "action": "payload_calibration_start",
                    "ok": True,
                    "result": {
                        "profile": state["profile"],
                        "payload": self.payload_profiles.web_state(),
                    },
                    "state": self.state_payload(),
                },
            )
        except (ControlError, RuntimeError, OSError) as exc:
            if self.payload_calibration is not None:
                self.payload_calibration["stage"] = "error"
                self.payload_calibration["error"] = str(exc)
                self.payload_calibration["message"] = str(exc)
            await self.send_payload_calibration_state(websocket)
            await self.send_error(
                websocket,
                request_id,
                str(exc),
                action="payload_calibration_start",
            )
        finally:
            self.payload_calibration_process = None

    def gripper_calibration_state(self) -> dict[str, Any] | None:
        state = self.gripper_calibration
        if state is None:
            cached = load_gripper_calibration(interface=self.interface)
            if cached is None:
                return None
            return {
                "active": False,
                "stage": "cached",
                "message": (
                    "夹爪闭合零点已完成一次性校准；"
                    "当前采集会直接复用该结果。仅更换夹爪或发现反馈异常时重校准。"
                ),
                "error": None,
                "gripper_open_readout": cached["gripper_open_readout"],
                "calibrated_at": cached.get("calibrated_at"),
                "zero_calibrated": cached.get("zero_calibrated", True),
            }
        return {
            "active": self.gripper_calibration_active,
            "stage": state.get("stage"),
            "message": state.get("message"),
            "error": state.get("error"),
            "gripper_open_readout": state.get("gripper_open_readout"),
            "calibrated_at": state.get("calibrated_at"),
            "zero_calibrated": state.get("zero_calibrated"),
        }

    async def send_gripper_calibration_state(
        self,
        websocket: ServerConnection,
    ) -> None:
        calibration = self.gripper_calibration_state()
        if calibration is None:
            return
        await self.send(
            websocket,
            {
                "type": "gripper_calibration",
                "calibration": calibration,
                "state": self.state_payload(),
            },
        )

    async def _write_gripper_calibration_input(self, value: str) -> None:
        process = self.gripper_calibration_process
        if process is None or process.stdin is None or process.returncode is not None:
            raise ControlError("夹爪标定进程已经结束，请重新开始。")
        process.stdin.write(value.encode("utf-8"))
        await process.stdin.drain()

    async def start_gripper_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        params: dict[str, Any],
    ) -> None:
        if self.controller is not None:
            await self.send_error(
                websocket,
                request_id,
                "请先断开机械臂；夹爪零点标定必须独占 CAN。",
                action="gripper_calibration_start",
            )
            return
        if (
            self.gripper_calibration_active
            or self.payload_calibration_active
            or self.handeye_calibration_active
        ):
            await self.send_error(
                websocket,
                request_id,
                "已有标定正在进行。",
                action="gripper_calibration_start",
            )
            return
        if params.get("safety_confirmed") is not True:
            await self.send_error(
                websocket,
                request_id,
                "请先确认六轴已有支撑、夹爪周围无物体或手指，并准备实体急停。",
                action="gripper_calibration_start",
            )
            return
        root = PROJECT_ROOT
        self.gripper_calibration = {
            "stage": "starting",
            "message": "正在独占 CAN 并准备夹爪零点标定。",
            "error": None,
            "request_id": request_id,
            "gripper_open_readout": None,
        }
        await self.send(
            websocket,
            {
                "type": "command_started",
                "id": request_id,
                "action": "gripper_calibration_start",
            },
        )
        try:
            self.gripper_calibration_process = await asyncio.create_subprocess_exec(
                sys.executable,
                "-B",
                "-m",
                "legacy.arx_tools.calibrate_gripper_guided",
                "--interface",
                self.interface,
                cwd=root,
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
            )
        except OSError as exc:
            self.gripper_calibration = None
            await self.send_error(
                websocket,
                request_id,
                f"无法启动夹爪标定：{exc}",
                action="gripper_calibration_start",
            )
            return
        self.gripper_calibration_task = asyncio.create_task(
            self.run_gripper_calibration(websocket, request_id)
        )
        await self.send_gripper_calibration_state(websocket)

    async def confirm_gripper_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        params: dict[str, Any],
    ) -> None:
        state = self.gripper_calibration
        if not self.gripper_calibration_active or state is None:
            await self.send_error(
                websocket,
                request_id,
                "当前没有等待确认的夹爪标定。",
                action="gripper_calibration_confirm",
            )
            return
        stage = params.get("stage")
        if stage != state.get("stage") or stage not in {"await_closed", "await_open"}:
            await self.send_error(
                websocket,
                request_id,
                "夹爪标定阶段已变化，请按网页当前提示操作。",
                action="gripper_calibration_confirm",
            )
            return
        if stage == "await_closed":
            state["stage"] = "zeroing"
            state["message"] = "正在以当前完全闭合位置写入夹爪零点。"
        else:
            state["stage"] = "measuring_open"
            state["message"] = "正在读取完全张开时的电机读数。"
        await self._write_gripper_calibration_input("\n")
        await self.send_result(
            websocket,
            request_id,
            "gripper_calibration_confirm",
            {"stage": stage},
        )
        await self.send_gripper_calibration_state(websocket)

    async def cancel_gripper_calibration(self, reason: str = "用户取消") -> None:
        task = self.gripper_calibration_task
        process = self.gripper_calibration_process
        if task is None or task.done():
            return
        if self.gripper_calibration is not None:
            self.gripper_calibration["stage"] = "cancelled"
            self.gripper_calibration["message"] = reason
        if process is not None and process.returncode is None:
            try:
                process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            await asyncio.wait_for(task, timeout=5.0)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            if process is not None and process.returncode is None:
                process.kill()
            if not task.done():
                task.cancel()
        self.gripper_calibration_process = None

    async def _consume_gripper_calibration_output(
        self,
        websocket: ServerConnection,
        process: asyncio.subprocess.Process,
    ) -> tuple[int, float | None]:
        if process.stdout is None:
            raise RuntimeError("夹爪标定进程没有输出管道。")
        buffer = ""
        open_readout: float | None = None
        while True:
            chunk = await process.stdout.read(1)
            if not chunk:
                break
            buffer += chunk.decode("utf-8", errors="replace")
            state = self.gripper_calibration
            if state is None:
                continue
            if (
                state.get("stage") == "starting"
                and "GRIPPER_CALIBRATION_READY" in buffer
            ):
                state["stage"] = "await_closed"
                state["message"] = (
                    "请完全闭合夹爪，确认夹爪周围无人和无物体后"
                    "点击“已完全闭合”。"
                )
                await self.send_gripper_calibration_state(websocket)
            if (
                state.get("stage") == "zeroing"
                and "Finish setting zero point. Please fully open the gripper" in buffer
            ):
                state["stage"] = "await_open"
                state["message"] = (
                    "零点已写入；请完全张开夹爪后点击“已完全张开”。"
                )
                await self.send_gripper_calibration_state(websocket)
            if "\n" not in buffer:
                continue
            lines = buffer.split("\n")
            buffer = lines.pop()
            for line in lines:
                match = GRIPPER_CALIBRATION_OPEN_READOUT_RE.search(line)
                if match is not None:
                    try:
                        open_readout = float(match.group(1))
                    except ValueError:
                        pass
        return await process.wait(), open_readout

    async def _reconnect_after_gripper_calibration(self) -> str | None:
        controller = ArmController(
            interface=self.interface,
            gain_scale=1.0,
            gripper_open_readout=self.gripper_open_readout(),
            safety=WEB_MOTION_SAFETY,
        )
        try:
            snapshot = await asyncio.to_thread(controller.connect, stabilize=True)
        except (
            TypeError,
            ValueError,
            BusyError,
            ControlError,
            SafetyError,
            RuntimeError,
            OSError,
        ) as exc:
            try:
                await asyncio.to_thread(controller.close, emergency=True)
            except BaseException:
                pass
            return web_error_message("connect", exc)
        self.controller = controller
        self.remember(snapshot)
        self.capture_reference(snapshot)
        return None

    async def run_gripper_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
    ) -> None:
        state = self.gripper_calibration
        process = self.gripper_calibration_process
        if state is None or process is None:
            return
        try:
            return_code, open_readout = await self._consume_gripper_calibration_output(
                websocket, process
            )
            if state.get("stage") == "cancelled":
                return
            if return_code != 0:
                raise RuntimeError(
                    "夹爪零点标定未完成；请重新检查闭合和张开位置。"
                )
            if open_readout is None:
                raise RuntimeError("未收到夹爪全开电机读数；结果没有保存。")
            readout = save_gripper_open_readout(
                open_readout,
                interface=self.interface,
            )
            state["gripper_open_readout"] = readout
            state["stage"] = "reconnecting"
            state["message"] = "已保存全开读数；正在重新连接并校验夹爪开度。"
            await self.send_gripper_calibration_state(websocket)
            reconnect_error = await self._reconnect_after_gripper_calibration()
            if reconnect_error is not None:
                raise RuntimeError(f"零点已保存，但自动重连失败：{reconnect_error}")
            state["stage"] = "done"
            state["message"] = (
                "夹爪零点与全开读数已保存，机械臂已重新连接并保持当前位置。"
            )
            await self.send_gripper_calibration_state(websocket)
            await self.send_result(
                websocket,
                request_id,
                "gripper_calibration_start",
                {"gripper_open_readout": readout},
            )
        except (ControlError, RuntimeError, OSError, ValueError) as exc:
            if self.gripper_calibration is not None:
                self.gripper_calibration["stage"] = "error"
                self.gripper_calibration["error"] = str(exc)
                self.gripper_calibration["message"] = str(exc)
            await self.send_gripper_calibration_state(websocket)
            await self.send_error(
                websocket,
                request_id,
                str(exc),
                action="gripper_calibration_start",
            )
        finally:
            self.gripper_calibration_process = None

    async def send_handeye_calibration_state(
        self,
        websocket: ServerConnection,
        *,
        message: str | None = None,
    ) -> None:
        calibration = self.handeye_calibration_state()
        if calibration is None:
            return
        if message is not None and self.handeye_calibration is not None:
            self.handeye_calibration["message"] = message
            calibration["message"] = message
        await self.send(
            websocket,
            {
                "type": "handeye_calibration",
                "calibration": calibration,
                "state": self.state_payload(),
            },
        )

    async def start_handeye_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        params: dict[str, Any],
    ) -> None:
        mode = params.get("mode", "dual")
        if not isinstance(mode, str) or mode not in HANDEYE_CALIBRATION_MODES:
            await self.send_error(
                websocket,
                request_id,
                "手眼标定相机模式无效。",
                action="handeye_calibration_start",
            )
            return
        if self.is_busy or self.servo_active or self.teleoperation_session_active:
            await self.send_error(websocket, request_id, "机械臂仍在运动或实时控制中，不能开始手眼标定。", action="handeye_calibration_start")
            return
        if (
            self.handeye_calibration_active
            or self.payload_calibration_active
            or self.gripper_calibration_active
        ):
            await self.send_error(websocket, request_id, "已有标定正在进行。", action="handeye_calibration_start")
            return
        if params.get("safety_confirmed") is not True:
            await self.send_error(websocket, request_id, "请先确认标定板、所选相机视野、工作区和实体急停均已准备好。", action="handeye_calibration_start")
            return
        controller = self.controller
        if controller is None:
            await self.send_error(websocket, request_id, "请先连接机械臂；标定将复用当前控制器。", action="handeye_calibration_start")
            return
        root = PROJECT_ROOT
        config_path = HANDEYE_CONFIG
        try:
            config = load_handeye_config(config_path)
            scene_hint = params.get("scene_hint")
            if isinstance(scene_hint, dict):
                config["scene_hint"] = scene_hint
            automatic = config.setdefault("automatic", {})
            return_to_reference = params.get(
                "return_to_reference_after_complete",
                automatic.get("return_to_reference_after_complete", True),
            )
            if not isinstance(return_to_reference, bool):
                raise ValueError("自动回参考位设置无效")
            automatic["return_to_reference_after_complete"] = return_to_reference
            if return_to_reference and self.reference_joint_position is not None:
                automatic["return_reference_joint_position_rad"] = list(
                    self.reference_joint_position
                )
                automatic["return_reference_label"] = "网页参考位"
            output_root = Path(config["collection"]["output_root"])
            session = CalibrationSession((output_root / "sessions" / time.strftime("%Y%m%d-%H%M%S")).resolve())
        except (OSError, RuntimeError, ValueError) as exc:
            await self.send_error(websocket, request_id, f"无法准备手眼标定：{exc}", action="handeye_calibration_start")
            return
        commands: queue.Queue[str] = queue.Queue()
        # JPEG guidance and feedback are snapshots, not a history. Keep at
        # most one unsent item of each so a slow browser never receives stale
        # arm positions after newer samples have been produced.
        stage_events: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        latest_events: dict[str, dict[str, Any]] = {}
        scheduled_latest_types: set[str] = set()
        loop = asyncio.get_running_loop()
        def enqueue(event: dict[str, Any]) -> None:
            event_type = str(event.get("type", ""))
            if event_type in {"preview", "feedback"}:
                latest_events[event_type] = event
                if event_type in scheduled_latest_types:
                    return
                scheduled_latest_types.add(event_type)
                stage_events.put_nowait({"type": f"latest_{event_type}"})
                return
            stage_events.put_nowait(event)
        def on_stage(stage: str) -> None:
            loop.call_soon_threadsafe(enqueue, {"type": "stage", "stage": stage})
        def on_event(event: dict[str, Any]) -> None:
            loop.call_soon_threadsafe(enqueue, event)
        self.handeye_commands = commands
        self.handeye_collector = ManualEyeToHandCollector(
            config, session, controller=controller, commands=commands,
            on_stage=on_stage, on_event=on_event,
        )
        minimum_samples = int(config["quality"].get("min_samples", 10))
        target_samples = max(
            minimum_samples,
            int(config.get("automatic", {}).get(
                "target_valid_samples", minimum_samples + 3
            )),
        )
        maximum_attempts = max(
            1,
            int(config.get("automatic", {}).get("max_pose_attempts", 72)),
        )
        initial_stage = "board" if mode == "eye_to_hand" else "table_check"
        initial_phase = "eye_to_hand" if mode == "eye_to_hand" else "eye_in_hand"
        initial_message = {
            "dual": "D405 桌面板阶段将进入自由阻尼；手动调整到画面可见后会自动锁定并采样。",
            "eye_in_hand": "D405 桌面板阶段将进入自由阻尼；手动调整到画面可见后会自动锁定并采样。",
            "eye_to_hand": "D455 独立标定：请将标定板放入夹爪；确认后会夹紧并进入 D455 安全中心规划。",
        }[mode]
        self.handeye_calibration = {
            "stage": initial_stage,
            "progress": {
                "phase": initial_phase,
                "stage": "preparing",
                "attempted": 0,
                "attempt_limit": maximum_attempts,
                "current": 0,
                "total": maximum_attempts,
                "valid": 0,
                "target": target_samples,
                "elapsed_s": 0.0,
                "estimated_remaining_s": None,
                "maximum_remaining_s": None,
            },
            "message": initial_message, "error": None,
            "result": None, "request_id": request_id, "cancel_requested": False,
            "log": [], "feedback": None, "quality": None,
            "return_to_reference": return_to_reference,
            "return_reference_label": (
                "网页参考位" if self.reference_joint_position is not None else "六关节零位"
            ),
            "controller_handoff": False,
            "mode": mode,
        }
        await self.send(websocket, {"type":"command_started", "id":request_id, "action":"handeye_calibration_start"})
        self.handeye_calibration_task = asyncio.create_task(
            self.run_handeye_calibration(
                websocket, request_id, stage_events, latest_events,
                scheduled_latest_types,
            )
        )
        await self.send_handeye_calibration_state(websocket)

    async def confirm_handeye_calibration(self, websocket: ServerConnection, request_id: str | int, params: dict[str, Any]) -> None:
        state = self.handeye_calibration
        commands = self.handeye_commands
        if not self.handeye_calibration_active or state is None or commands is None:
            await self.send_error(websocket, request_id, "当前没有等待确认的手眼标定。", action="handeye_calibration_confirm")
            return
        stage = params.get("stage")
        values = {"ready":"READY", "board":"BOARD", "center":"CENTER", "plan":"RUN", "remove":"REMOVE", "cancel_remove":"REMOVE"}
        if stage == "manual" and state.get("stage") == "manual":
            manual_action = params.get("manual_action")
            manual_values = {
                "sample": "SAMPLE",
                "done": "DONE",
                "home": "HOME",
            }
            if manual_action not in manual_values:
                await self.send_error(
                    websocket,
                    request_id,
                    "手动补采操作无效。",
                    action="handeye_calibration_confirm",
                )
                return
            state["stage"] = "manual_working"
            state["message"] = {
                "sample": "正在保持当前位置并采集手动样本。",
                "done": "正在检查手动补采质量并进入独立验证。",
                "home": "正在安全回零；完成后继续手动补采。",
            }[manual_action]
            commands.put(manual_values[manual_action])
            await self.send_result(
                websocket,
                request_id,
                "handeye_calibration_confirm",
                {"stage": stage, "manual_action": manual_action},
            )
            await self.send_handeye_calibration_state(websocket)
            return
        if stage == "review" and state.get("stage") == "review":
            review_action = params.get("review_action")
            review_values = {
                "more": "MORE",
                "accept": "ACCEPT",
            }
            if review_action not in review_values:
                await self.send_error(
                    websocket,
                    request_id,
                    "标定评审操作无效。",
                    action="handeye_calibration_confirm",
                )
                return
            state["stage"] = "review_working"
            state["message"] = {
                "more": "正在将本轮独立验证样本纳入拟合，补采 4 个高信息差异化姿态；随后使用全新独立姿态重新评估。",
                "accept": "已采用当前人工评审结果，正在继续下一阶段。",
            }[review_action]
            commands.put(review_values[review_action])
            await self.send_result(
                websocket,
                request_id,
                "handeye_calibration_confirm",
                {"stage": stage, "review_action": review_action},
            )
            await self.send_handeye_calibration_state(websocket)
            return
        if stage not in values or stage != state.get("stage"):
            await self.send_error(websocket, request_id, "标定流程阶段已变化，请按当前提示操作。", action="handeye_calibration_confirm")
            return
        state["stage"] = "working"
        state["message"] = "已确认，标定继续执行并保持当前控制器。"
        commands.put(values[stage])
        await self.send_result(websocket, request_id, "handeye_calibration_confirm", {"stage":stage})
        await self.send_handeye_calibration_state(websocket)

    async def cancel_handeye_calibration(self, reason: str = "用户取消", *, return_home: bool = False) -> None:
        task = self.handeye_calibration_task
        if task is None or task.done(): return
        state = self.handeye_calibration
        if state is not None:
            state["cancel_requested"] = True
            state["stage"] = "cancelling"
            state["message"] = (
                "正在安全取消并回零。"
                if return_home
                else "正在由同一控制器安全回到阶段起点。"
            )
        if self.handeye_commands is not None:
            self.handeye_commands.put("HOME" if return_home else "CANCEL")
        try:
            await asyncio.wait_for(task, timeout=30.0)
        except asyncio.TimeoutError:
            # Never cancel the worker or close CAN while a physical move may run.
            if state is not None:
                state["message"] = "取消仍在等待安全收尾；控制器保持当前位置。"

    async def run_handeye_calibration(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        stage_events: asyncio.Queue[dict[str, Any]],
        latest_events: dict[str, dict[str, Any]],
        scheduled_latest_types: set[str],
    ) -> None:
        state = self.handeye_calibration
        collector = self.handeye_collector
        if state is None or collector is None: return
        mode = str(state.get("mode", "dual"))
        worker = asyncio.create_task(
            asyncio.to_thread(
                collector.run_automatic
                if mode == "dual"
                else collector.run_automatic_phase,
                *(() if mode == "dual" else (mode,)),
            )
        )
        messages = {
            "ready": "D405 姿态已对齐桌面标定板；点击确认后生成自动采样计划。",
            "board": "请把标定板放入夹爪后确认；系统将夹紧它并直接开始 D455 标定。",
            "center": "夹爪已保持标定板；调整 D455 安全中心后确认。",
            "plan": "自动采样计划已生成并显示在仿真中；确认后机械臂才会执行。",
            "manual": "自动采样未达精度目标；已进入自由阻尼手动补采。",
            "review": (
                "已完成目标有效姿态及独立重定位评估。误差指标仅供评审；"
                "可采用当前结果，或补采 4 组新的联合平移/旋转姿态。"
            ),
            "motion_hold": (
                "轨迹未完全到位，控制器正保持当前位置；"
                "请检查工作区，可使用“取消标定并回零”或实体急停。"
            ),
            "remove": (
                "采样完成；托住标定板后确认张开夹爪，随后自动回"
                f"{state.get('return_reference_label', '参考位')}。"
                if state.get("return_to_reference", True)
                else "采样完成；托住标定板后确认张开夹爪，随后保持当前位置。"
            ),
            "return_reference": (
                "标定板已移除，正在自动回"
                f"{state.get('return_reference_label', '参考位')}；夹爪保持张开。"
            ),
        }
        try:
            while not worker.done():
                try:
                    event = await asyncio.wait_for(stage_events.get(), timeout=0.2)
                except asyncio.TimeoutError:
                    continue
                event_type = str(event.get("type", ""))
                if event_type.startswith("latest_"):
                    latest_type = event_type.removeprefix("latest_")
                    scheduled_latest_types.discard(latest_type)
                    event = latest_events.pop(latest_type, None)
                    if event is None:
                        continue
                    event_type = latest_type
                if event_type == "preview":
                    await self.send(websocket, {"type": "handeye_preview", "preview": event["preview"]})
                    continue
                if event_type == "feedback":
                    state["feedback"] = event["feedback"]
                    await self.send_handeye_calibration_state(websocket)
                    continue
                if event_type == "progress":
                    progress = event.get("progress")
                    if isinstance(progress, dict):
                        state["progress"] = progress
                        await self.send_handeye_calibration_state(websocket)
                    continue
                if event_type == "quality":
                    quality = event.get("quality")
                    if isinstance(quality, dict):
                        state["quality"] = quality
                        await self.send_handeye_calibration_state(websocket)
                    continue
                if event_type == "manual_status":
                    manual = event.get("manual")
                    if isinstance(manual, dict):
                        state["manual_ready"] = bool(manual.get("ready"))
                        message = manual.get("message")
                        if isinstance(message, str) and message:
                            state["message"] = message
                        await self.send_handeye_calibration_state(websocket)
                    continue
                if event_type == "plan":
                    state["plan"] = event["plan"]
                    await self.send(websocket, {"type": "handeye_plan", "plan": event["plan"]})
                    continue
                stage = event.get("stage")
                if stage in messages:
                    state["stage"] = stage
                    state["message"] = messages[stage]
                    await self.send_handeye_calibration_state(websocket)
            result = await worker
            if state.get("cancel_requested"):
                state["stage"] = "cancelled"; state["message"] = "手眼标定已取消；控制器保持当前位置。"
            elif result is None:
                state["stage"] = "incomplete"; state["message"] = "采样结束但未达到精度要求。"
            else:
                state["result"] = self.handeye_result_summary(result)
                state["stage"] = "done"
                state["message"] = {
                    "dual": "双相机手眼标定完成；机械臂已回参考位并保持。",
                    "eye_in_hand": "D405 眼在手标定已独立保存；D455 无需参与本次流程。",
                    "eye_to_hand": "D455 外部相机标定已独立保存；D405 无需参与本次流程。",
                }.get(mode, "手眼标定完成；机械臂已回参考位并保持。")
            ok = state["stage"] == "done"
            await self.send_handeye_calibration_state(websocket)
            await self.send(websocket, {"type":"command_result","id":request_id,"action":"handeye_calibration_start","ok":ok,"result":state.get("result"),"error":None if ok else state["message"],"state":self.state_payload()})
        except BaseException as exc:
            state["stage"] = "error"; state["error"] = str(exc); state["message"] = f"手眼标定失败：{exc}；控制器保持当前位置。"
            await self.send_handeye_calibration_state(websocket)
            await self.send_error(websocket, request_id, state["message"], action="handeye_calibration_start")
        finally:
            self.handeye_commands = None
            self.handeye_collector = None
            self.handeye_calibration_process = None

    async def _restore_controller_after_handeye(self) -> str | None:
        """Reacquire CAN in damping after the calibration child releases it."""
        if self.controller is not None:
            return None
        last_error: BaseException | None = None
        for attempt in range(3):
            controller = ArmController(
                interface=self.interface,
                gain_scale=1.0,
                gripper_open_readout=self.gripper_open_readout(),
                safety=WEB_MOTION_SAFETY,
            )
            try:
                snapshot = await asyncio.to_thread(
                    controller.connect,
                    stabilize=True,
                )
                if controller.mode != ControllerMode.ACTIVE:
                    raise ControlError("网页重新接管后未保持当前位置")
            except (BusyError, ControlError, SafetyError, RuntimeError, OSError) as exc:
                last_error = exc
                try:
                    await asyncio.to_thread(controller.close, emergency=True)
                except BaseException:
                    pass
                if attempt < 2:
                    await asyncio.sleep(0.15)
                continue
            self.controller = controller
            self.remember(snapshot)
            self.capture_reference(snapshot)
            return None
        return web_error_message(
            "connect",
            last_error or RuntimeError("未知 CAN 接管错误"),
        )

    def state_payload(self, snapshot: ArmSnapshot | None = None) -> dict[str, Any]:
        controller = self.controller
        snapshot = snapshot or self.last_snapshot
        payload_profile = self.payload_profiles.web_state()
        payload_calibration = self.payload_calibration_state()
        gripper_calibration = self.gripper_calibration_state()
        handeye_calibration = self.handeye_calibration_state()
        handeye_validation = self.handeye_validation_state()
        cube_manipulation = {
            "stage": self.cube_manipulation.get("stage", "idle")
        } if self.cube_manipulation is not None else {"stage": "idle"}
        collection = self.collection_state()
        if controller is None:
            return {
                "connected": False,
                "mode": "disconnected",
                "busy": (
                    self.is_busy
                    or self.payload_calibration_active
                    or self.gripper_calibration_active
                    or self.handeye_calibration_active
                ),
                "servo_active": False,
                "servo_kind": None,
                "teleoperation_config": self.teleoperation_config.web_payload(),
                "teleoperation": {
                    "active": False,
                    "preview_only": False,
                    "stopping": False,
                    "target_age_ms": None,
                    "servo_period_ms": None,
                    "gripper_contact_latched": False,
                },
                "payload_calibration": payload_calibration,
                "gripper_calibration": gripper_calibration,
                "handeye_calibration": handeye_calibration,
                "handeye_validation": handeye_validation,
                "cube_manipulation": cube_manipulation,
                "collection": collection,
                **payload_profile,
            }
        servo_status = controller.servop_status()
        gripper_servo_status = (
            None if self.gripper_servo is None else self.gripper_servo.status()
        )
        teleoperation_status = (
            None
            if self.teleoperation_servo is None
            else self.teleoperation_servo.status()
        )
        payload: dict[str, Any] = {
            "connected": True,
            "mode": controller.mode.value,
            "busy": (
                self.is_busy
                or self.payload_calibration_active
                or self.gripper_calibration_active
                or self.handeye_calibration_active
            ),
            "servo_active": bool(
                (servo_status and servo_status.active)
                or (gripper_servo_status and gripper_servo_status.active)
                or (teleoperation_status and teleoperation_status.active)
            ),
            "servo_stopping": bool(
                (servo_status and servo_status.stopping)
                or (gripper_servo_status and gripper_servo_status.stopping)
                or (teleoperation_status and teleoperation_status.stopping)
            ),
            "servo_kind": (
                "teleoperation"
                if teleoperation_status and teleoperation_status.active
                else "teleoperation_preview"
                if (
                    self.teleoperation_preview_only
                    and self.teleoperation_session_id is not None
                )
                else "gripper"
                if gripper_servo_status and gripper_servo_status.active
                else "cartesian"
                if servo_status and servo_status.active
                else None
            ),
            "fault_reason": controller.fault_reason,
            "arm_gain_percent": 100.0 * controller.current_arm_gain_scale,
            "gripper_gain_percent": (
                100.0 * controller.current_gripper_gain_scale
            ),
            "payload_calibration": payload_calibration,
            "gripper_calibration": gripper_calibration,
            "handeye_calibration": handeye_calibration,
            "handeye_validation": handeye_validation,
            "cube_manipulation": cube_manipulation,
            "collection": collection,
            "teleoperation_config": self.teleoperation_config.web_payload(),
            "teleoperation": {
                "active": bool(
                    teleoperation_status and teleoperation_status.active
                ),
                "preview_only": bool(
                    self.teleoperation_preview_only
                    and self.teleoperation_session_id is not None
                ),
                "stopping": bool(
                    teleoperation_status and teleoperation_status.stopping
                ),
                "target_age_ms": (
                    1000.0 * teleoperation_status.update_age_s
                    if teleoperation_status
                    else 1000.0
                    * max(
                        0.0,
                        time.monotonic() - self.teleoperation_last_message_at,
                    )
                    if (
                        self.teleoperation_preview_only
                        and self.teleoperation_last_message_at > 0.0
                    )
                    else None
                ),
                "servo_period_ms": (
                    1000.0 * teleoperation_status.cycle_period_s
                    if teleoperation_status
                    else None
                ),
                "gripper_contact_latched": bool(
                    teleoperation_status
                    and teleoperation_status.gripper_contact_latched
                ),
            },
            **payload_profile,
        }
        if servo_status is not None and servo_status.error:
            payload["servo_error"] = servo_status.error
        if gripper_servo_status is not None and gripper_servo_status.error:
            payload["servo_error"] = gripper_servo_status.error
        if teleoperation_status is not None and teleoperation_status.error:
            payload["servo_error"] = teleoperation_status.error
        if snapshot is not None:
            limits = controller.get_limits()
            payload.update(
                {
                    "joint_position_deg": [
                        math.degrees(float(value))
                        for value in snapshot.joint_position
                    ],
                    "joint_velocity_rad_s": [
                        float(value) for value in snapshot.joint_velocity
                    ],
                    "joint_torque_nm": [
                        float(value) for value in snapshot.joint_torque
                    ],
                    "joint_min_deg": [
                        math.degrees(float(value))
                        for value in limits.joint_position_min
                    ],
                    "joint_max_deg": [
                        math.degrees(float(value))
                        for value in limits.joint_position_max
                    ],
                    "gripper_position_mm": 1000.0 * snapshot.gripper_position,
                    "gripper_velocity_m_s": snapshot.gripper_velocity,
                    "gripper_torque_nm": snapshot.gripper_torque,
                    "gripper_max_mm": min(
                        WEB_MAX_GRIPPER_WIDTH_MM,
                        1000.0 * limits.gripper_width_m,
                    ),
                    "eef_pose_6d": [float(value) for value in snapshot.eef_pose_6d],
                    "state_age_ms": max(
                        0.0,
                        1000.0 * (time.monotonic() - self.last_snapshot_wall_time),
                    ),
                }
            )
        if self.reference_joint_position is not None:
            payload["reference_joint_position_deg"] = [
                math.degrees(value) for value in self.reference_joint_position
            ]
        if self.reference_gripper_position is not None:
            payload["reference_gripper_position_mm"] = (
                1000.0 * self.reference_gripper_position
            )
        return payload

    def collection_state(self, *, refresh_counts: bool = False) -> dict[str, Any]:
        if refresh_counts:
            self.collection_counts = self.raw_episode_store.counts()
        recorder = self.collection_recorder
        accepted = int(self.collection_counts.get("accepted", 0))
        target = self.collection_config.target_accepted_episodes
        return {
            "stage": self.collection_stage,
            "active": recorder is not None,
            "session_id": self.collection_session_id,
            "task_instruction": self.collection_config.task_instruction,
            "task_display": self.collection_config.task_display,
            "source_color": self.collection_config.source_color,
            "target_color": self.collection_config.target_color,
            "target_accepted_episodes": (
                target
            ),
            "remaining_episodes": max(0, target - accepted),
            "counts": dict(self.collection_counts),
            "state_summary": None if recorder is None else recorder.summary(),
            "camera_state": self.collection_camera_state,
            "home_calibration": (
                None
                if self.collection_home is None
                else {
                    "fixed": True,
                    "calibrated_at": self.collection_home.get("calibrated_at"),
                    "method": self.collection_home.get("method"),
                }
            ),
            "validation": self.collection_validation,
            "dataset": {
                "id": self.collection_config.dataset_id,
                "batch_id": self.collection_config.batch_id,
                "format": "GR00T-flavored LeRobot v2.1",
                "groot_release": "N1.7",
                "raw_root": str(self.raw_episode_store.root.resolve()),
                "state_dimension": 16,
                "action_dimension": 16,
                "action_label": "next synchronized state",
            },
            "recording_spec": {
                "arm_rate_hz": self.collection_config.sampling.arm_rate_hz,
                "camera_fps": self.collection_config.camera.fps,
                "camera_timeline": {
                    "mode": "native_rate_shared_guard",
                    "fps": self.collection_config.camera.fps,
                    "master_video_key": "wrist_rgb",
                },
                "cameras": [
                    {
                        "video_key": camera.video_key,
                        "width": camera.width,
                        "height": camera.height,
                        "fps": camera.fps,
                        "source_pixel_format": "rgb8",
                        "source_bit_depth": 8,
                        "color_control_mode": "auto_warmup_then_fixed",
                        "warmup_frames": 30,
                        "exposure_us": camera.exposure_us,
                        "gain": camera.gain,
                        "white_balance_k": camera.white_balance_k,
                        "storage": "H.264 Matroska + per-frame timestamps",
                    }
                    for camera in self.collection_config.cameras
                ],
                "minimum_episode_seconds": (
                    self.collection_config.sampling.minimum_episode_seconds
                ),
                "maximum_episode_seconds": (
                    self.collection_config.sampling.maximum_episode_seconds
                ),
                "minimum_still_seconds": (
                    self.collection_config.validation.minimum_still_seconds
                ),
                "action_horizon": self.collection_config.sampling.action_horizon,
                "maximum_frame_drop_fraction": (
                    self.collection_config.validation.maximum_frame_drop_fraction
                ),
                "maximum_state_alignment_p99_ms": (
                    self.collection_config.validation.maximum_state_alignment_p99_ms
                ),
                "maximum_camera_alignment_p99_ms": (
                    self.collection_config.validation.maximum_camera_alignment_p99_ms
                ),
                "maximum_video_gap_ms": (
                    self.collection_config.validation.maximum_video_gap_ms
                ),
                "minimum_usable_camera_rate_fraction": (
                    MINIMUM_USABLE_CAMERA_RATE_FRACTION
                ),
            },
        }

    def remember_collection_camera_state(
        self, response: dict[str, Any]
    ) -> dict[str, Any]:
        state = {key: value for key, value in response.items() if key != "id"}
        state["ready"] = state.get("ok") is not False
        self.collection_camera_state = state
        self.collection_camera_refreshed_at = time.monotonic()
        return state

    async def refresh_collection_camera_state(
        self, *, force: bool = False
    ) -> dict[str, Any]:
        camera = self.collection_camera
        if camera is None:
            self.collection_camera_state = {
                "ok": False,
                "ready": False,
                "active": False,
                "error": "主机未配置双相机采集服务",
            }
            return self.collection_camera_state
        interval_s = 0.45 if self.collection_recorder is not None else 3.0
        if (
            not force
            and self.collection_camera_state is not None
            and time.monotonic() - self.collection_camera_refreshed_at < interval_s
        ):
            return self.collection_camera_state
        try:
            response = await camera.request(
                "collection_camera_status", timeout_s=15.0 if force else 2.0
            )
        except RuntimeError as exc:
            self.collection_camera_state = {
                "ok": False,
                "ready": False,
                "active": False,
                "error": str(exc),
            }
            self.collection_camera_refreshed_at = time.monotonic()
            return self.collection_camera_state
        state = self.remember_collection_camera_state(response)
        session_id = self.collection_session_id
        if self.collection_recorder is not None and (
            not state.get("active") or state.get("session_id") != session_id
        ):
            state["ok"] = False
            state["ready"] = False
            state["error"] = "双相机录制已中断，当前 episode 不可保存"
        return state

    def recover_collection_review(self) -> None:
        """Expose the newest unfinished episode so a restarted page can reject it."""
        candidates = self.raw_episode_store.paths_for_stage("inprogress")
        if not candidates:
            self.collection_session_id = None
            self.collection_stage = "idle"
            self.collection_validation = None
            return
        episode_path = candidates[-1]
        try:
            with (episode_path / "episode.json").open(
                "r", encoding="utf-8"
            ) as stream:
                manifest = json.load(stream)
            session_id = str(manifest["session_id"])
            validation_path = episode_path / "validation.json"
            if validation_path.is_file():
                with validation_path.open("r", encoding="utf-8") as stream:
                    validation = json.load(stream)
            else:
                validation = {
                    "ok": False,
                    "errors": [
                        "episode was interrupted before validation; reject and redo"
                    ],
                    "warnings": [],
                    "metrics": {},
                }
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        self.collection_session_id = session_id
        self.collection_stage = "pending_review"
        self.collection_validation = validation

    async def handle_collection_command(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        action: str,
        params: dict[str, Any],
    ) -> None:
        try:
            if action == "collection_prepare":
                result = await self.prepare_collection()
            elif action == "collection_status":
                await self.refresh_collection_camera_state(force=True)
                result = self.collection_state(refresh_counts=True)
            elif action == "collection_start":
                result = await self.start_collection(params)
            elif action == "collection_stop":
                result = await self.stop_collection()
            elif action == "collection_accept":
                result = await self.accept_collection(params)
            elif action == "collection_reject":
                result = await self.reject_collection(params)
            else:
                raise ValueError(f"unsupported collection action: {action}")
        except (ControlError, ValueError, RuntimeError, OSError) as exc:
            await self.send_error(
                websocket,
                request_id,
                str(exc),
                action=action,
            )
            return
        await self.send_result(websocket, request_id, action, result)

    def apply_collection_home(self, snapshot: ArmSnapshot) -> None:
        controller = self.controller
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        limits = controller.get_limits()
        margin = controller.safety.joint_limit_margin_rad
        minimum = np.asarray(limits.joint_position_min, dtype=np.float64) + margin
        maximum = np.asarray(limits.joint_position_max, dtype=np.float64) - margin
        home = load_collection_home(interface=self.interface)
        if home is None:
            joints = np.clip(
                np.asarray(snapshot.joint_position, dtype=np.float64),
                minimum,
                maximum,
            )
            gripper_max = min(
                WEB_MAX_GRIPPER_WIDTH_MM / 1000.0,
                float(limits.gripper_width_m),
            )
            home = save_collection_home(
                joints,
                max(0.0, min(gripper_max, HOME_GRIPPER_WIDTH_M)),
                interface=self.interface,
            )
        joints = np.asarray(home["joint_position_rad"], dtype=np.float64)
        if np.any(joints < minimum) or np.any(joints > maximum):
            raise ControlError("已保存的采集零位超出当前控制器安全范围")
        gripper_max = min(
            WEB_MAX_GRIPPER_WIDTH_MM / 1000.0,
            float(limits.gripper_width_m),
        )
        gripper = float(home["gripper_position_m"])
        if not 0.0 <= gripper <= gripper_max:
            raise ControlError("已保存的采集零位夹爪位置超出安全范围")
        self.collection_home = home
        self.reference_joint_position = tuple(float(value) for value in joints)
        self.reference_gripper_position = gripper
        self.park_preview = None
        self.park_preview_velocity_scale = None

    async def prepare_collection(self) -> dict[str, Any]:
        self.auto_connect_suspended = False
        snapshot = await self.ensure_controller_connected()
        self.apply_collection_home(snapshot)
        controller = self.controller
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        if controller.mode == ControllerMode.FAULT:
            raise ControlError("机械臂处于故障阻尼状态，请检查实体急停和控制器")
        if controller.mode != ControllerMode.DAMPING:
            snapshot = await asyncio.to_thread(controller.damping)
            self.remember(snapshot)
        return self.collection_state(refresh_counts=True)

    async def start_collection(self, _params: dict[str, Any]) -> dict[str, Any]:
        controller = self.controller
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        if controller.mode != ControllerMode.DAMPING:
            raise ControlError("开始采集前必须切换到阻尼示教模式。")
        if self.collection_home is None:
            raise ControlError("固定采集零位尚未准备完成。")
        if self.is_busy or self.servo_active:
            raise ControlError("机械臂仍在运动或实时伺服中，不能开始采集。")
        if self.collection_recorder is not None:
            raise RuntimeError("已有采集正在进行。")
        if self.collection_session_id is not None:
            raise RuntimeError("请先接受或重做当前 episode。")
        if (
            self.collection_counts["accepted"]
            >= self.collection_config.target_accepted_episodes
        ):
            raise RuntimeError("已达到目标 episode 数量，请先导出数据集。")
        gripper_calibration = load_gripper_calibration(interface=self.interface)
        location = self.raw_episode_store.begin()
        recorder: ArmStateRecorder | None = None
        try:
            snapshot = await asyncio.to_thread(controller.get_state)
            limits = controller.get_limits()
            self.raw_episode_store.update(
                location.session_id,
                stage="recording",
                started_host_monotonic_ns=time.monotonic_ns(),
                teaching_mode="full_damping_kinesthetic",
                collection_home={
                    "fixed": True,
                    "calibrated_at": self.collection_home.get("calibrated_at"),
                    "method": self.collection_home.get("method"),
                    "joint_position_rad": list(
                        self.collection_home["joint_position_rad"]
                    ),
                    "gripper_position_m": self.collection_home[
                        "gripper_position_m"
                    ],
                },
                gripper_calibration={
                    "schema_version": (
                        gripper_calibration.get("schema_version")
                        if gripper_calibration is not None
                        else None
                    ),
                    "calibrated_at": (
                        gripper_calibration.get("calibrated_at")
                        if gripper_calibration is not None
                        else None
                    ),
                    "gripper_open_readout": (
                        gripper_calibration.get("gripper_open_readout")
                        if gripper_calibration is not None
                        else self.gripper_open_readout()
                    ),
                    "zero_calibrated": bool(
                        gripper_calibration is not None
                        and gripper_calibration.get("zero_calibrated", True)
                    ),
                    "status": (
                        "calibrated"
                        if gripper_calibration is not None
                        else "fallback_readout"
                    ),
                },
                initial_state=ArmStateRecorder.snapshot_payload(
                    snapshot, time.monotonic_ns()
                ),
                limits={
                    "joint_position_min_rad": [
                        float(value) for value in limits.joint_position_min
                    ],
                    "joint_position_max_rad": [
                        float(value) for value in limits.joint_position_max
                    ],
                    "gripper_width_m": float(limits.gripper_width_m),
                    "gripper_position_tolerance_m": float(
                        controller.safety.gripper_position_tolerance_m
                    ),
                },
            )
            recorder = ArmStateRecorder(
                location.path,
                controller,
                rate_hz=self.collection_config.sampling.arm_rate_hz,
            )
            recorder.start()
            camera = self.collection_camera
            if camera is None:
                raise RuntimeError("主机未配置双相机采集服务")
            camera_response = await camera.request(
                "collection_camera_start",
                session_id=location.session_id,
                timeout_s=20.0,
            )
            self.remember_collection_camera_state(camera_response)
        except BaseException:
            if recorder is not None:
                try:
                    await asyncio.to_thread(recorder.stop)
                except (OSError, RuntimeError, ValueError):
                    pass
            if self.collection_camera is not None:
                try:
                    await self.collection_camera.request(
                        "collection_camera_abort",
                        session_id=location.session_id,
                        timeout_s=10.0,
                    )
                except RuntimeError:
                    pass
            try:
                self.raw_episode_store.reject(
                    location.session_id, "episode recorder failed to start"
                )
            except (OSError, ValueError):
                pass
            raise
        self.collection_recorder = recorder
        self.collection_session_id = location.session_id
        self.collection_stage = "recording"
        self.collection_validation = None
        self.collection_counts = self.raw_episode_store.counts()
        self.collection_timeout_task = asyncio.create_task(
            self.enforce_collection_duration(location.session_id)
        )
        self.collection_health_task = asyncio.create_task(
            self.monitor_collection_health(location.session_id)
        )
        return self.collection_state()

    def cancel_collection_timeout(self) -> None:
        task = self.collection_timeout_task
        self.collection_timeout_task = None
        if task is not None and task is not asyncio.current_task():
            task.cancel()

    def cancel_collection_health(self) -> None:
        task = self.collection_health_task
        self.collection_health_task = None
        if task is not None and task is not asyncio.current_task():
            task.cancel()

    async def monitor_collection_health(self, session_id: str) -> None:
        consecutive_failures = 0
        failure_reason = ""
        try:
            while (
                self.collection_session_id == session_id
                and self.collection_recorder is not None
            ):
                await asyncio.sleep(0.5)
                recorder = self.collection_recorder
                if recorder is None:
                    return
                state_error = recorder.summary().get("error")
                if state_error:
                    consecutive_failures = 3
                    failure_reason = f"机械臂状态记录失败：{state_error}"
                else:
                    camera = self.collection_camera
                    if camera is None:
                        consecutive_failures += 1
                        failure_reason = "主机未配置双相机采集服务"
                    else:
                        try:
                            response = await camera.request(
                                "collection_camera_status",
                                timeout_s=3.0,
                            )
                            state = self.remember_collection_camera_state(response)
                            if (
                                not state.get("active")
                                or state.get("session_id") != session_id
                            ):
                                raise RuntimeError("双相机录制会话已中断")
                        except RuntimeError as exc:
                            consecutive_failures += 1
                            failure_reason = str(exc)
                        else:
                            consecutive_failures = 0
                            failure_reason = ""
                if consecutive_failures >= 3:
                    self.collection_health_task = None
                    await self.abort_collection(
                        f"后台实时质量监测：{failure_reason}"
                    )
                    print(
                        f"Episode {session_id} 已提前终止：{failure_reason}"
                    )
                    return
        except asyncio.CancelledError:
            pass

    async def enforce_collection_duration(self, session_id: str) -> None:
        try:
            await asyncio.sleep(
                self.collection_config.sampling.maximum_episode_seconds
            )
            if (
                self.collection_session_id == session_id
                and self.collection_recorder is not None
            ):
                self.collection_timeout_task = None
                await self.stop_collection()
                print(f"Episode {session_id} 已由后台按时长上限自动停止。")
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"Episode {session_id} 后台自动停止失败：{exc}")

    async def stop_collection(self) -> dict[str, Any]:
        self.cancel_collection_timeout()
        self.cancel_collection_health()
        recorder = self.collection_recorder
        session_id = self.collection_session_id
        if recorder is None or session_id is None:
            raise RuntimeError("当前没有正在进行的采集。")
        camera_error: str | None = None
        camera = self.collection_camera
        try:
            if camera is None:
                raise RuntimeError("主机未配置双相机采集服务")
            camera_response = await camera.request(
                "collection_camera_stop",
                session_id=session_id,
                timeout_s=20.0,
            )
            self.remember_collection_camera_state(camera_response)
        except RuntimeError as exc:
            camera_error = str(exc)
            if camera is not None:
                try:
                    await camera.request(
                        "collection_camera_abort",
                        session_id=session_id,
                        timeout_s=5.0,
                    )
                except RuntimeError:
                    pass
            self.collection_camera_state = {
                "ok": False,
                "ready": False,
                "active": False,
                "error": camera_error,
            }
        summary = await asyncio.to_thread(recorder.stop)
        self.collection_recorder = None
        self.raw_episode_store.finish_recording(session_id, summary)
        location = self.raw_episode_store.locate(session_id)
        report = await asyncio.to_thread(
            validate_raw_episode,
            location.path,
            self.collection_config,
        )
        if camera_error is not None:
            report = dict(report)
            report["ok"] = False
            report["errors"] = [
                *[str(error) for error in report.get("errors", [])],
                f"双相机停止异常：{camera_error}",
            ]
        self.collection_stage = "pending_review"
        self.collection_validation = report
        return self.collection_state()

    def _review_session_id(self, params: dict[str, Any]) -> str:
        requested = params.get("session_id", self.collection_session_id)
        if not isinstance(requested, str) or not requested:
            raise ValueError("缺少待审核的 episode session_id。")
        return requested

    async def accept_collection(self, params: dict[str, Any]) -> dict[str, Any]:
        if self.collection_recorder is not None:
            raise RuntimeError("请先停止采集。")
        session_id = self._review_session_id(params)
        location = self.raw_episode_store.locate(session_id)
        manifest_path = location.path / "episode.json"
        with manifest_path.open("r", encoding="utf-8") as stream:
            manifest = json.load(stream)
        if manifest.get("stage") != "pending_review":
            raise RuntimeError("只有已停止且待审核的 episode 可以接受。")
        report = await asyncio.to_thread(
            validate_raw_episode,
            location.path,
            self.collection_config,
        )
        if not report["ok"]:
            raise RuntimeError(
                "数据完整性检查未通过：" + "；".join(report["errors"])
            )
        self.raw_episode_store.accept(session_id, report)
        self.collection_session_id = None
        self.collection_stage = "idle"
        self.collection_validation = None
        self.collection_counts = self.raw_episode_store.counts()
        self.recover_collection_review()
        return self.collection_state()

    async def reject_collection(self, params: dict[str, Any]) -> dict[str, Any]:
        if self.collection_recorder is not None:
            raise RuntimeError("请先停止采集。")
        session_id = self._review_session_id(params)
        reason = str(params.get("reason") or "operator requested redo")
        self.raw_episode_store.reject(session_id, reason)
        self.collection_session_id = None
        self.collection_stage = "idle"
        self.collection_validation = None
        self.collection_counts = self.raw_episode_store.counts()
        self.recover_collection_review()
        return self.collection_state()

    async def abort_collection(self, reason: str) -> None:
        self.cancel_collection_timeout()
        self.cancel_collection_health()
        recorder = self.collection_recorder
        session_id = self.collection_session_id
        if recorder is None or session_id is None:
            return
        camera = self.collection_camera
        if camera is not None:
            try:
                response = await camera.request(
                    "collection_camera_abort",
                    session_id=session_id,
                    timeout_s=10.0,
                )
                self.remember_collection_camera_state(response)
            except RuntimeError as exc:
                self.collection_camera_state = {
                    "ok": False,
                    "ready": False,
                    "active": False,
                    "error": str(exc),
                }
        try:
            summary = await asyncio.to_thread(recorder.stop)
            self.raw_episode_store.update(
                session_id,
                stage="aborted",
                abort_reason=reason,
                state=summary,
            )
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"GR00T state recording abort failed: {exc}")
        finally:
            self.collection_recorder = None
            self.collection_stage = "pending_review"
            self.collection_validation = {
                "ok": False,
                "errors": [f"episode interrupted: {reason}"],
                "warnings": [],
                "metrics": {},
            }

    def remember(self, snapshot: ArmSnapshot) -> ArmSnapshot:
        self.last_snapshot = snapshot
        self.last_snapshot_wall_time = time.monotonic()
        return snapshot

    def capture_reference(self, snapshot: ArmSnapshot) -> None:
        controller = self.controller
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        limits = controller.get_limits()
        margin = controller.safety.joint_limit_margin_rad
        self.reference_joint_position = tuple(
            max(
                float(limits.joint_position_min[index] + margin),
                min(
                    float(limits.joint_position_max[index] - margin),
                    float(value),
                ),
            )
            for index, value in enumerate(snapshot.joint_position)
        )
        gripper_max = min(
            WEB_MAX_GRIPPER_WIDTH_MM / 1000.0,
            float(limits.gripper_width_m),
        )
        self.reference_gripper_position = max(
            0.0,
            min(gripper_max, float(snapshot.gripper_position)),
        )
        self.park_preview = None
        self.park_preview_velocity_scale = None
        self.cube_stack_preview = None

    async def handle(self, websocket: ServerConnection) -> None:
        if not self.authorized(websocket):
            await self.send(
                websocket,
                {"type": "fatal", "message": "缺少或配对令牌不正确。"},
            )
            await websocket.close(code=1008, reason="invalid pairing token")
            return
        if self.owner is not None:
            if not self.takeover_requested(websocket):
                await self.send(
                    websocket,
                    {
                        "type": "fatal",
                        "message": "已有一个网页客户端占用控制服务。",
                    },
                )
                await websocket.close(
                    code=1013,
                    reason="control client already connected",
                )
                return
            if self.is_busy or self.servo_active:
                await self.send(
                    websocket,
                    {
                        "type": "fatal",
                        "message": "机械臂正在运动，暂不能接管控制连接。",
                    },
                )
                await websocket.close(code=1013, reason="control session is busy")
                return
            previous_owner = self.owner
            self.owner = websocket
            await previous_owner.close(
                code=1012,
                reason="control session handed over",
            )

        self.owner = websocket
        self.teleoperation_sequence = -1
        print(f"网页控制客户端已连接：{websocket.remote_address}")
        await self.send(
            websocket,
            {
                "type": "hello",
                "protocol": 1,
                "source_revision": SOURCE_REVISION,
                "service_started_at": SERVICE_STARTED_AT,
                "limits": {
                    "motion_profile_revision": WEB_MOTION_PROFILE_REVISION,
                    "max_joint_step_deg": WEB_MAX_JOINT_STEP_DEG,
                    "max_gripper_width_mm": WEB_MAX_GRIPPER_WIDTH_MM,
                    "max_gripper_torque_nm": WEB_MAX_GRIPPER_TORQUE_NM,
                    "min_speed_percent": WEB_MIN_SPEED_PERCENT,
                    "max_speed_percent": WEB_MAX_SPEED_PERCENT,
                    "default_speed_percent": WEB_DEFAULT_SPEED_PERCENT,
                    "speed_tiers_percent": list(WEB_SPEED_TIERS_PERCENT),
                },
                "state": self.state_payload(),
            },
        )
        try:
            async for raw_message in websocket:
                await self.handle_message(websocket, raw_message)
        finally:
            if self.owner is websocket:
                print("网页控制客户端断开；正在安全收尾客户端操作。")
                await self.cancel_payload_calibration("网页控制连接已断开，标定已取消。")
                await self.cancel_gripper_calibration("网页控制连接已断开，夹爪标定已取消。")
                if self.handeye_calibration_active:
                    # Calibration child owns CAN. Keep it alive through a
                    # browser reconnect; cancelling it here creates a CAN
                    # handoff gap and can release the arm unexpectedly.
                    print("网页控制客户端断开；手眼标定继续保持 CAN 控制。")
                elif self.persistent_controller:
                    await self.retain_controller_after_browser_disconnect()
                else:
                    await self.close_controller(browser_lost=True)
                self.owner = None

    def authorized(self, websocket: ServerConnection) -> bool:
        if self.token is None:
            return True
        request = websocket.request
        if request is None:
            return False
        values = parse_qs(urlsplit(request.path).query).get("token", [])
        return len(values) == 1 and hmac.compare_digest(values[0], self.token)

    @staticmethod
    def takeover_requested(websocket: ServerConnection) -> bool:
        request = websocket.request
        if request is None:
            return False
        values = parse_qs(urlsplit(request.path).query).get("takeover", [])
        return values == ["1"]

    async def handle_message(
        self, websocket: ServerConnection, raw_message: str | bytes
    ) -> None:
        try:
            message = json.loads(raw_message)
        except (json.JSONDecodeError, TypeError):
            await self.send_error(websocket, None, "消息不是有效 JSON。")
            return
        if not isinstance(message, dict):
            await self.send_error(websocket, None, "消息必须是对象。")
            return
        if message.get("type") == "teleop":
            await self.handle_teleoperation_message(websocket, message)
            return
        if message.get("type") == "servo":
            if message.get("kind") == "gripper":
                await self.handle_gripper_servo_message(websocket, message)
            else:
                await self.handle_servo_message(websocket, message)
            return
        if message.get("type") != "command":
            await self.send_error(
                websocket,
                message.get("id"),
                "消息类型必须是 command、servo 或 teleop。",
            )
            return
        request_id = message.get("id")
        action = message.get("action")
        if (
            isinstance(request_id, bool)
            or not isinstance(request_id, (str, int))
            or not isinstance(action, str)
        ):
            await self.send_error(websocket, request_id, "缺少有效的 id 或 action。")
            return

        if action == "connect":
            self.auto_connect_suspended = False
            await self.connect_controller(websocket, request_id)
            return
        if action == "emergency":
            if self.gripper_calibration_active:
                await self.cancel_gripper_calibration("网页急停：夹爪标定已取消。")
                await self.send_result(websocket, request_id, action, None)
                return
            if self.handeye_calibration_active:
                state = self.handeye_calibration
                if state is not None:
                    state["cancel_requested"] = True
                    state["stage"] = "emergency"
                    state["message"] = "急停已触发；标定不会再执行任何运动。"
                if self.handeye_commands is not None:
                    self.handeye_commands.put("EMERGENCY")
                # Do not await cooperative cancellation here: it can be inside
                # a move or camera sample. Damping must be commanded now.
                await self.emergency(websocket, request_id)
                return
            await self.emergency(websocket, request_id)
            return
        if action == "state":
            await self.send_state(websocket, request_id)
            return
        if action == "disconnect":
            self.auto_connect_suspended = True
            await self.close_controller(browser_lost=False)
            await self.send_result(websocket, request_id, action, None)
            return

        params = message.get("params")
        if params is None:
            params = {}
        if not isinstance(params, dict):
            await self.send_error(
                websocket, request_id, "params 必须是对象。", action=action
            )
            return
        if action in {
            "collection_prepare",
            "collection_status",
            "collection_start",
            "collection_stop",
            "collection_accept",
            "collection_reject",
        }:
            await self.handle_collection_command(
                websocket, request_id, action, params
            )
            return
        if self.teleoperation_session_active:
            if self.teleoperation_preview_only:
                await self.stop_teleoperation_servo(
                    "another control operation started",
                    emergency=False,
                )
            else:
                await self.send_error(
                    websocket,
                    request_id,
                    "手势遥操作正在运行；请先松开离合手。",
                    action=action,
                )
                return
        if action == "payload_calibration_start":
            await self.start_payload_calibration(websocket, request_id, params)
            return
        if action == "payload_calibration_confirm":
            try:
                await self.confirm_payload_calibration(
                    websocket, request_id, params
                )
            except (ControlError, RuntimeError, OSError) as exc:
                await self.send_error(
                    websocket,
                    request_id,
                    str(exc),
                    action=action,
                )
            return
        if action == "payload_calibration_cancel":
            await self.cancel_payload_calibration()
            await self.send_result(websocket, request_id, action, None)
            return
        if action == "gripper_calibration_start":
            await self.start_gripper_calibration(websocket, request_id, params)
            return
        if action == "gripper_calibration_confirm":
            try:
                await self.confirm_gripper_calibration(websocket, request_id, params)
            except (ControlError, RuntimeError, OSError) as exc:
                await self.send_error(websocket, request_id, str(exc), action=action)
            return
        if action == "gripper_calibration_cancel":
            await self.cancel_gripper_calibration()
            await self.send_result(websocket, request_id, action, None)
            return
        if action == "handeye_calibration_start":
            await self.start_handeye_calibration(websocket, request_id, params)
            return
        if action == "handeye_calibration_confirm":
            try:
                await self.confirm_handeye_calibration(websocket, request_id, params)
            except (ControlError, RuntimeError, OSError) as exc:
                await self.send_error(websocket, request_id, str(exc), action=action)
            return
        if action == "handeye_calibration_cancel":
            await self.cancel_handeye_calibration(return_home=True)
            await self.send_result(websocket, request_id, action, None)
            return
        if action == "handeye_validate":
            await self.validate_handeye_result(websocket, request_id)
            return
        if action == "handeye_enable":
            operator_override = params.get("operator_override", False)
            if not isinstance(operator_override, bool):
                await self.send_error(
                    websocket,
                    request_id,
                    "人工采用标记必须是布尔值。",
                    action=action,
                )
                return
            await self.enable_handeye_result(
                websocket,
                request_id,
                operator_override=operator_override,
            )
            return
        if action == "handeye_disable":
            await self.disable_handeye_result(websocket, request_id)
            return

        controller = self.controller
        if controller is None:
            await self.send_error(
                websocket, request_id, "机械臂尚未连接。", action=action
            )
            return
        if self.collection_recorder is not None:
            await self.send_error(
                websocket,
                request_id,
                "数据采集中只允许停止采集或急停。",
                action=action,
            )
            return
        if self.servo_active:
            await self.send_error(
                websocket,
                request_id,
                "实时伺服正在运行；请先松开拖动。",
                action=action,
            )
            return
        if self.is_busy:
            await self.send_error(
                websocket, request_id, "上一动作尚未完成。", action=action
            )
            return

        mode_after_motion: ControllerMode | None = None
        if action in WEB_SPEED_CONTROL_ACTIONS:
            try:
                controller.safety = motion_safety_from_params(params)
                velocity_scale_from_params(params)
                if action in WEB_POST_MOTION_MODE_ACTIONS:
                    mode_after_motion = post_motion_mode_from_params(params)
            except ValueError as exc:
                await self.send_error(
                    websocket, request_id, str(exc), action=action
                )
                return

        if action in {
            "cube_pick_preview",
            "cube_move_preview",
            "cube_place_preview",
            "cube_stack_preview",
        }:
            await self.send(
                websocket,
                {"type": "command_started", "id": request_id, "action": action},
            )
            try:
                if action == "cube_pick_preview":
                    result = await asyncio.to_thread(
                        self.prepare_cube_pick_preview, params
                    )
                elif action == "cube_move_preview":
                    result = await asyncio.to_thread(
                        self.prepare_cube_move_preview, params
                    )
                elif action == "cube_place_preview":
                    result = await asyncio.to_thread(
                        self.prepare_cube_place_preview, params
                    )
                else:
                    result = await asyncio.to_thread(
                        self.prepare_cube_stack_preview, params
                    )
            except (ValueError, ControlError, SafetyError, RuntimeError) as exc:
                await self.send_error(websocket, request_id, str(exc), action=action)
            else:
                await self.send_result(websocket, request_id, action, result)
            return

        # Any manual motion invalidates a previously prepared pick plan.
        if action != "cube_pick_execute":
            self.cube_pick_preview = None
        if action not in {"cube_move_execute", "cube_place_execute"}:
            self.cube_stage_preview = None
        if action != "cube_stack_execute":
            self.cube_stack_preview = None

        if action == "preview_xyz_live":
            await self.preview_xyz_live(websocket, request_id, params)
            return
        try:
            operation = self.make_operation(action, params)
        except (ValueError, ControlError, SafetyError) as exc:
            await self.send_error(websocket, request_id, str(exc), action=action)
            return

        await self.send(
            websocket,
            {"type": "command_started", "id": request_id, "action": action},
        )
        task = asyncio.create_task(
            self.run_operation(
                websocket,
                request_id,
                action,
                operation,
                mode_after_motion=mode_after_motion,
            )
        )
        self.motion_task = task

    async def handle_teleoperation_message(
        self,
        websocket: ServerConnection,
        message: dict[str, Any],
    ) -> None:
        sequence = message.get("seq")
        action = message.get("action")
        if (
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence < 0
            or action not in {"start", "update", "stop"}
        ):
            await self.send_teleoperation_state(
                websocket,
                sequence,
                str(action),
                False,
                session_id=self.teleoperation_session_id,
                error="遥操作消息缺少有效 seq 或 action。",
            )
            return
        if sequence <= self.teleoperation_sequence:
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=self.teleoperation_session_id,
                error="遥操作消息 seq 必须严格递增。",
            )
            return
        self.teleoperation_sequence = sequence

        controller = self.controller
        if controller is None:
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=self.teleoperation_session_id,
                error="机械臂尚未连接。",
            )
            return
        params = message.get("params", {})
        if not isinstance(params, dict):
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=self.teleoperation_session_id,
                error="遥操作 params 必须是对象。",
            )
            return

        if action == "start":
            self.reap_teleoperation_session()
            if (
                self.is_busy
                or self.servo_active
                or self.teleoperation_session_active
                or self.payload_calibration_active
                or self.handeye_calibration_active
            ):
                await self.send_teleoperation_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    session_id=self.teleoperation_session_id,
                    error="控制器正在执行其他动作。",
                )
                return
            if controller.mode not in (
                ControllerMode.DAMPING,
                ControllerMode.ACTIVE,
            ):
                await self.send_teleoperation_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    session_id=None,
                    error="当前模式不能启动遥操作。",
                )
                return
            try:
                profile_name = str(params.get("profile", "fine"))
                profile = self.teleoperation_config.profiles[profile_name]
                preview_only = params.get("preview_only", False)
                if not isinstance(preview_only, bool):
                    raise ValueError("preview_only 必须是布尔值。")
                control_side = params.get("control_side", "right")
                if control_side not in {"left", "right"}:
                    raise ValueError("控制手必须是 left 或 right。")
                open_width_mm = params.get("open_width_mm", 40.0)
                closing_torque_nm = params.get("closing_torque_nm", 0.15)
                if not finite_number(open_width_mm) or not (
                    0.0 <= float(open_width_mm) <= 88.0
                ):
                    raise ValueError("夹爪张开宽度必须在 0..88 mm。")
                if not finite_number(closing_torque_nm) or not (
                    0.05 <= float(closing_torque_nm) <= 0.25
                ):
                    raise ValueError(
                        "夹爪闭合扭矩必须在 0.05..0.25 N·m。"
                    )
                current = await asyncio.to_thread(controller.get_state)
                if not self.teleoperation_config.workspace.contains(
                    current.eef_pose_6d[:3]
                ):
                    raise SafetyError("当前 TCP 位于遥操作安全盒之外。")
                now = time.monotonic()
                session_id = secrets.token_urlsafe(18)
                if preview_only:
                    self.teleoperation_session_id = session_id
                    self.teleoperation_profile_name = profile_name
                    self.teleoperation_preview_only = True
                    self.teleoperation_preview_start_pose = (
                        current.eef_pose_6d.copy()
                    )
                    self.teleoperation_preview_start_joint = (
                        current.joint_position.copy()
                    )
                    self.teleoperation_preview_seed = current.joint_position.copy()
                    self.teleoperation_preview_target = TeleoperationTarget(
                        np.zeros(3),
                        np.array([0.0, 0.0, 0.0, 1.0]),
                        GripperState.OPEN,
                    )
                    self.teleoperation_preview_processed_at = now
                    self.teleoperation_last_message_at = now
                    self.teleoperation_last_update_at = 0.0
                    self.teleoperation_preview_generation = 0
                    self.teleoperation_preview_last_payload = None
                    self.remember(current)
                    await self.send_teleoperation_state(
                        websocket,
                        sequence,
                        action,
                        True,
                        session_id=session_id,
                    )
                    return
                controller.safety = replace(
                    WEB_SAFE_MOTION_SAFETY,
                    max_commanded_velocity_rad_s=max(
                        0.35, profile.joint_velocity_rad_s
                    ),
                    max_commanded_cartesian_velocity_m_s=(
                        profile.tcp_linear_velocity_m_s
                    ),
                    servo_max_velocity_rad_s=profile.joint_velocity_rad_s,
                    servo_max_acceleration_rad_s2=(
                        profile.joint_acceleration_rad_s2
                    ),
                    servo_max_jerk_rad_s3=profile.joint_jerk_rad_s3,
                )
                if controller.mode == ControllerMode.DAMPING:
                    self.remember(
                        await asyncio.to_thread(controller.hold_current)
                    )
                servo = TeleoperationServo(
                    controller,
                    config=self.teleoperation_config,
                    profile_name=profile_name,
                    rate_hz=controller.safety.servo_rate_hz,
                    watchdog_timeout_s=(
                        controller.safety.servo_watchdog_timeout_s
                    ),
                    state_stale_timeout_s=(
                        controller.safety.state_stale_timeout_s
                    ),
                    gripper_velocity_m_s=(
                        controller.safety.max_commanded_gripper_velocity_m_s
                    ),
                    gripper_acceleration_m_s2=(
                        controller.safety.gripper_servo_max_acceleration_m_s2
                    ),
                    gripper_jerk_m_s3=(
                        controller.safety.gripper_servo_max_jerk_m_s3
                    ),
                )
                snapshot = await asyncio.to_thread(
                    servo.start,
                    open_width_m=float(open_width_mm) / 1000.0,
                    closing_torque_nm=float(closing_torque_nm),
                )
            except (
                KeyError,
                TypeError,
                ValueError,
                ControlError,
                SafetyError,
                RuntimeError,
            ) as exc:
                await self.send_teleoperation_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    session_id=self.teleoperation_session_id,
                    error=str(exc),
                )
                return
            self.teleoperation_servo = servo
            self.teleoperation_session_id = session_id
            self.teleoperation_profile_name = profile_name
            self.teleoperation_preview_only = False
            self.teleoperation_last_message_at = now
            self.teleoperation_last_update_at = 0.0
            self.remember(snapshot)
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                True,
                session_id=session_id,
            )
            return

        session_id = message.get("session_id")
        if (
            not isinstance(session_id, str)
            or session_id != self.teleoperation_session_id
        ):
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=self.teleoperation_session_id,
                error="遥操作 session_id 不匹配。",
            )
            return

        if action == "stop":
            stopped_id = self.teleoperation_session_id
            await self.stop_teleoperation_servo(
                "clutch released",
                emergency=False,
            )
            try:
                self.remember(await asyncio.to_thread(controller.get_state))
            except (ControlError, SafetyError, RuntimeError):
                pass
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                True,
                session_id=stopped_id,
            )
            return

        now = time.monotonic()
        if self.teleoperation_preview_only:
            self.teleoperation_last_message_at = now
            await self._handle_teleoperation_preview_update(
                websocket,
                sequence,
                action,
                params,
                now,
            )
            return

        servo = self.teleoperation_servo
        if servo is None or not servo.status().active:
            stale_session_id = self.teleoperation_session_id
            error = None if servo is None else servo.status().error
            self.reap_teleoperation_session()
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=stale_session_id,
                error=error or "遥操作伺服未运行或已由看门狗停止。",
            )
            return
        try:
            target = teleoperation_target_from_params(params)
            generation = await asyncio.to_thread(servo.update_target, target)
            self.teleoperation_last_update_at = now
            self.teleoperation_last_message_at = now
            status = await asyncio.to_thread(
                servo.wait_for_target,
                generation,
                0.045,
            )
        except (TypeError, ValueError, ControlError, SafetyError, RuntimeError) as exc:
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=self.teleoperation_session_id,
                error=str(exc),
            )
            return
        if status.error:
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=self.teleoperation_session_id,
                error=status.error,
            )
            return
        if status.feedback_snapshot is not None:
            self.remember(status.feedback_snapshot)
        preview = {
            "generation": status.processed_generation,
            "target_generation": status.target_generation,
            "reachable": (
                status.target_error is None
                and status.target_generation >= generation
            ),
            "target_joint_position_deg": (
                None
                if status.target_joint_position is None
                else np.degrees(status.target_joint_position).tolist()
            ),
            "delta_xyz_mm": (
                None
                if status.target_delta_xyz_m is None
                else (1000.0 * status.target_delta_xyz_m).tolist()
            ),
            "target_orientation_xyzw": (
                None
                if status.target_orientation_xyzw is None
                else status.target_orientation_xyzw.tolist()
            ),
            "workspace_speed_scale": status.workspace_speed_scale,
            "gripper_state": status.gripper_state.value,
        }
        if status.target_error:
            preview["error"] = status.target_error
        await self.send_teleoperation_state(
            websocket,
            sequence,
            action,
            True,
            session_id=self.teleoperation_session_id,
            preview=preview,
        )

    async def _handle_teleoperation_preview_update(
        self,
        websocket: ServerConnection,
        sequence: int,
        action: str,
        params: dict[str, Any],
        now: float,
    ) -> None:
        try:
            target = teleoperation_target_from_params(params)
            if self.teleoperation_profile_name is None:
                raise ControlError("遥操作档位状态缺失。")
            profile = self.teleoperation_config.profiles[
                self.teleoperation_profile_name
            ]
            start_pose = self.teleoperation_preview_start_pose
            start_joint = self.teleoperation_preview_start_joint
            seed = self.teleoperation_preview_seed
            previous = self.teleoperation_preview_target
            if any(
                value is None
                for value in (start_pose, start_joint, seed, previous)
            ):
                raise ControlError("只读遥操作预览状态不完整。")
            requested_position = start_pose[:3] + target.delta_xyz_m
            if not self.teleoperation_config.workspace.contains(requested_position):
                raise SafetyError("遥操作目标位于安全盒之外。")
            bounded = limit_teleoperation_target_step(
                previous,
                target,
                profile,
                now - self.teleoperation_preview_processed_at,
            )
            target_position = start_pose[:3] + bounded.delta_xyz_m
            workspace_scale = self.teleoperation_config.workspace.speed_scale(
                target_position
            )
            target_orientation = quaternion_multiply_xyzw(
                bounded.delta_orientation_xyzw,
                rpy_to_quaternion_xyzw(start_pose[3:]),
            )
            plan = await asyncio.to_thread(
                self.controller._plan_cartesian_servo_delta,
                start_pose,
                start_joint,
                bounded.delta_xyz_m,
                seed,
                target_orientation_xyzw=target_orientation,
            )
            next_generation = self.teleoperation_preview_generation + 1
            preview = {
                "generation": next_generation,
                "target_generation": next_generation,
                "reachable": True,
                "target_joint_position_deg": np.degrees(
                    plan.target_joint_position
                ).tolist(),
                "delta_xyz_mm": (1000.0 * bounded.delta_xyz_m).tolist(),
                "target_orientation_xyzw": target_orientation.tolist(),
                "workspace_speed_scale": min(
                    workspace_scale,
                    float(getattr(plan, "singularity_speed_scale", 1.0)),
                ),
                "gripper_state": bounded.gripper_state.value,
            }
        except (TypeError, ValueError, ControlError, SafetyError, RuntimeError) as exc:
            previous_payload = self.teleoperation_preview_last_payload
            preview = (
                {"reachable": False, "error": str(exc)}
                if previous_payload is None
                else {**previous_payload, "reachable": False, "error": str(exc)}
            )
            await self.send_teleoperation_state(
                websocket,
                sequence,
                action,
                False,
                session_id=self.teleoperation_session_id,
                error=str(exc),
                preview=preview,
            )
            return
        self.teleoperation_preview_generation = next_generation
        self.teleoperation_preview_target = bounded
        self.teleoperation_preview_seed = plan.target_joint_position.copy()
        self.teleoperation_preview_processed_at = now
        self.teleoperation_last_message_at = now
        self.teleoperation_last_update_at = now
        self.teleoperation_preview_last_payload = preview
        await self.send_teleoperation_state(
            websocket,
            sequence,
            action,
            True,
            session_id=self.teleoperation_session_id,
            preview=preview,
        )

    async def send_teleoperation_state(
        self,
        websocket: ServerConnection,
        sequence: Any,
        action: str,
        ok: bool,
        *,
        session_id: str | None,
        error: str | None = None,
        preview: dict[str, Any] | None = None,
    ) -> None:
        payload: dict[str, Any] = {
            "type": "teleop_state",
            "seq": sequence,
            "action": action,
            "ok": ok,
            "session_id": session_id,
            "state": self.state_payload(),
        }
        if error is not None:
            payload["error"] = error
        if preview is not None:
            payload["preview"] = preview
        await self.send(websocket, payload)

    def clear_teleoperation_session(self) -> None:
        self.teleoperation_session_id = None
        self.teleoperation_profile_name = None
        self.teleoperation_preview_only = False
        self.teleoperation_preview_start_pose = None
        self.teleoperation_preview_start_joint = None
        self.teleoperation_preview_seed = None
        self.teleoperation_preview_target = None
        self.teleoperation_preview_processed_at = 0.0
        self.teleoperation_preview_generation = 0
        self.teleoperation_preview_last_payload = None
        self.teleoperation_last_message_at = 0.0
        self.teleoperation_last_update_at = 0.0

    def reap_teleoperation_session(self) -> None:
        """Release ownership after a worker has already stopped itself."""
        servo = self.teleoperation_servo
        if servo is None:
            if self.teleoperation_session_id is not None:
                self.clear_teleoperation_session()
            return
        status = servo.status()
        if status.active or status.stopping:
            return
        self.teleoperation_servo = None
        self.clear_teleoperation_session()

    async def handle_servo_message(
        self, websocket: ServerConnection, message: dict[str, Any]
    ) -> None:
        sequence = message.get("seq")
        action = message.get("action")
        if (
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence < 0
            or action not in {"start", "update", "stop"}
        ):
            await self.send_servo_state(
                websocket,
                sequence,
                str(action),
                False,
                "伺服消息缺少有效 seq 或 action。",
            )
            return
        controller = self.controller
        if controller is None:
            await self.send_servo_state(
                websocket, sequence, action, False, "机械臂尚未连接。"
            )
            return

        params = message.get("params", {})
        if not isinstance(params, dict):
            await self.send_servo_state(
                websocket, sequence, action, False, "伺服 params 必须是对象。"
            )
            return

        if action == "start":
            if self.teleoperation_preview_only:
                await self.stop_teleoperation_servo(
                    "Cartesian servo started",
                    emergency=False,
                )
            if self.is_busy or self.servo_active:
                await self.send_servo_state(
                    websocket, sequence, action, False, "控制器正在执行其他动作。"
                )
                return
            if controller.mode not in (
                ControllerMode.DAMPING,
                ControllerMode.ACTIVE,
            ):
                await self.send_servo_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    "当前模式不能启动实时伺服。",
                )
                return
            try:
                controller.safety = motion_safety_from_params(params)
                velocity_scale = velocity_scale_from_params(params)
                mode_after_motion = post_motion_mode_from_params(params)
                if controller.mode == ControllerMode.DAMPING:
                    self.remember(
                        await asyncio.to_thread(controller.hold_current)
                    )
                origin = await asyncio.to_thread(controller.get_tcp_pose)
                status = await asyncio.to_thread(
                    controller.servop,
                    origin,
                    velocity_scale=velocity_scale,
                    wait_timeout_s=0.045,
                )
            except (ValueError, ControlError, SafetyError, RuntimeError) as exc:
                await self.send_servo_state(
                    websocket, sequence, action, False, str(exc)
                )
                return
            self.cartesian_preview = None
            self.cartesian_preview_velocity_scale = None
            self.cartesian_preview_path_mode = None
            self.cartesian_servo_origin = origin
            self.cartesian_servo_velocity_scale = velocity_scale
            self.cartesian_servo_sequence = sequence
            self.cartesian_servo_mode_after_motion = mode_after_motion
            if status.feedback_snapshot is not None:
                self.remember(status.feedback_snapshot)
            await self.send_servo_state(websocket, sequence, action, True)
            return

        status = controller.servop_status()
        if (
            self.cartesian_servo_origin is None
            or status is None
            or not status.active
        ):
            error = None if status is None else status.error
            await self.send_servo_state(
                websocket,
                sequence,
                action,
                False,
                error or "实时伺服未运行或已由看门狗停止。",
            )
            return
        if sequence <= self.cartesian_servo_sequence:
            return
        self.cartesian_servo_sequence = sequence

        if action == "update":
            try:
                delta = cartesian_servo_delta_from_params(params)
                origin = self.cartesian_servo_origin
                if origin is None:  # pragma: no cover - checked above
                    raise ControlError("实时伺服原点已丢失。")
                target = Pose(
                    origin.position_xyz_m + delta,
                    origin.orientation_xyzw,
                )
                status = await asyncio.to_thread(
                    controller.servop,
                    target,
                    velocity_scale=self.cartesian_servo_velocity_scale,
                    wait_timeout_s=0.045,
                )
            except (ValueError, ControlError, SafetyError, RuntimeError) as exc:
                await self.send_servo_state(
                    websocket, sequence, action, False, str(exc)
                )
                return
            if status.error:
                await self.send_servo_state(
                    websocket, sequence, action, False, status.error
                )
                return
            if status.feedback_snapshot is not None:
                self.remember(status.feedback_snapshot)
            preview = None
            if (
                status.processed_generation >= status.target_generation
                and status.target_joint_position is not None
            ):
                preview = {
                    "generation": status.processed_generation,
                    "target_generation": status.target_generation,
                    "reachable": (
                        status.target_error is None
                        and status.processed_generation
                        >= status.target_generation
                    ),
                    "target_joint_position_deg": np.degrees(
                        status.target_joint_position
                    ).tolist(),
                    "delta_xyz_mm": (1000.0 * delta).tolist(),
                }
                if status.target_error:
                    preview["error"] = status.target_error
            await self.send_servo_state(
                websocket,
                sequence,
                action,
                True,
                preview=preview,
            )
            return

        await self.stop_cartesian_servo("pointer released", emergency=False)
        try:
            self.remember(await asyncio.to_thread(controller.get_state))
        except (ControlError, SafetyError, RuntimeError):
            pass
        await self.send_servo_state(websocket, sequence, action, True)

    async def handle_gripper_servo_message(
        self, websocket: ServerConnection, message: dict[str, Any]
    ) -> None:
        sequence = message.get("seq")
        action = message.get("action")
        if (
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence < 0
            or action not in {"start", "update", "stop"}
        ):
            await self.send_servo_state(
                websocket,
                sequence,
                str(action),
                False,
                "夹爪伺服消息缺少有效 seq 或 action。",
                kind="gripper",
            )
            return
        controller = self.controller
        if controller is None:
            await self.send_servo_state(
                websocket,
                sequence,
                action,
                False,
                "机械臂尚未连接。",
                kind="gripper",
            )
            return
        params = message.get("params", {})
        if not isinstance(params, dict):
            await self.send_servo_state(
                websocket,
                sequence,
                action,
                False,
                "夹爪伺服 params 必须是对象。",
                kind="gripper",
            )
            return

        if action == "start":
            if self.teleoperation_preview_only:
                await self.stop_teleoperation_servo(
                    "gripper servo started",
                    emergency=False,
                )
            if self.is_busy or self.servo_active:
                await self.send_servo_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    "控制器正在执行其他动作。",
                    kind="gripper",
                )
                return
            if controller.mode not in (ControllerMode.DAMPING, ControllerMode.ACTIVE):
                await self.send_servo_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    "当前模式不能启动夹爪伺服。",
                    kind="gripper",
                )
                return
            try:
                direction_value = params.get("direction")
                if isinstance(direction_value, bool) or direction_value not in (-1, 1):
                    raise ValueError("夹爪方向必须是 -1 或 1。")
                direction = int(direction_value)
                controller.safety = motion_safety_from_params(params)
                velocity_scale = velocity_scale_from_params(params)
                mode_after_motion = post_motion_mode_from_params(params)
                closing_torque_nm = params.get("torque_nm", 0.15)
                minimum_mm = params.get("minimum_mm", 0.0)
                if not finite_number(closing_torque_nm) or not (
                    0.05 <= float(closing_torque_nm) <= WEB_MAX_GRIPPER_TORQUE_NM
                ):
                    raise ValueError(
                        f"夹爪闭合扭矩必须在 0.05..{WEB_MAX_GRIPPER_TORQUE_NM:.2f} N·m。"
                    )
                if not finite_number(minimum_mm) or not (
                    0.0 <= float(minimum_mm) <= WEB_MAX_GRIPPER_WIDTH_MM
                ):
                    raise ValueError(
                        f"最小开度必须在 0..{WEB_MAX_GRIPPER_WIDTH_MM:.0f} mm。"
                    )
                if controller.mode == ControllerMode.DAMPING:
                    self.remember(await asyncio.to_thread(controller.hold_current))
                self.remember(await asyncio.to_thread(controller.activate_gripper))
                safety = controller.safety
                servo = GripperServo(
                    controller,
                    rate_hz=safety.servo_rate_hz,
                    max_velocity_m_s=safety.max_commanded_gripper_velocity_m_s,
                    max_acceleration_m_s2=safety.gripper_servo_max_acceleration_m_s2,
                    max_jerk_m_s3=safety.gripper_servo_max_jerk_m_s3,
                    watchdog_timeout_s=safety.servo_watchdog_timeout_s,
                    state_stale_timeout_s=safety.state_stale_timeout_s,
                    velocity_scale=velocity_scale,
                )
                snapshot = await asyncio.to_thread(
                    servo.start,
                    direction,
                    closing_torque_nm=float(closing_torque_nm),
                    minimum_width_m=float(minimum_mm) / 1000.0,
                )
            except (TypeError, ValueError, ControlError, SafetyError, RuntimeError) as exc:
                await self.send_servo_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    str(exc),
                    kind="gripper",
                )
                return
            self.gripper_servo = servo
            self.gripper_servo_sequence = sequence
            self.gripper_servo_mode_after_motion = mode_after_motion
            self.remember(snapshot)
            await self.send_servo_state(
                websocket, sequence, action, True, kind="gripper"
            )
            return

        servo = self.gripper_servo
        if servo is None or not servo.status().active:
            error = None if servo is None else servo.status().error
            await self.send_servo_state(
                websocket,
                sequence,
                action,
                False,
                error or "夹爪伺服未运行或已停止。",
                kind="gripper",
            )
            return
        if sequence <= self.gripper_servo_sequence:
            return
        self.gripper_servo_sequence = sequence

        if action == "update":
            try:
                direction_value = params.get("direction")
                if isinstance(direction_value, bool) or direction_value not in (-1, 1):
                    raise ValueError("夹爪方向必须是 -1 或 1。")
                direction = int(direction_value)
                await asyncio.to_thread(servo.update_direction, direction)
                status = servo.status()
            except (TypeError, ValueError, ControlError, SafetyError, RuntimeError) as exc:
                await self.send_servo_state(
                    websocket,
                    sequence,
                    action,
                    False,
                    str(exc),
                    kind="gripper",
                )
                return
            if status.feedback_snapshot is not None:
                self.remember(status.feedback_snapshot)
            await self.send_servo_state(
                websocket, sequence, action, True, kind="gripper"
            )
            return

        await self.stop_gripper_servo("key released", emergency=False)
        try:
            self.remember(await asyncio.to_thread(controller.get_state))
        except (ControlError, SafetyError, RuntimeError):
            pass
        await self.send_servo_state(
            websocket, sequence, action, True, kind="gripper"
        )

    async def send_servo_state(
        self,
        websocket: ServerConnection,
        sequence: Any,
        action: str,
        ok: bool,
        error: str | None = None,
        *,
        preview: dict[str, Any] | None = None,
        kind: str = "cartesian",
    ) -> None:
        payload: dict[str, Any] = {
            "type": "servo_state",
            "seq": sequence,
            "action": action,
            "kind": kind,
            "ok": ok,
            "state": self.state_payload(),
        }
        if error:
            payload["error"] = error
        if preview is not None:
            payload["preview"] = preview
        await self.send(websocket, payload)

    async def stop_cartesian_servo(
        self, reason: str, *, emergency: bool
    ) -> None:
        controller = self.controller
        if controller is None or self.cartesian_servo_origin is None:
            return
        mode_after_motion = self.cartesian_servo_mode_after_motion
        if emergency:
            await asyncio.to_thread(controller.emergency_damping, reason)
        else:
            await asyncio.to_thread(controller.stopl)
        self.cartesian_servo_origin = None
        self.cartesian_servo_velocity_scale = 1.0
        self.cartesian_servo_sequence = -1
        self.cartesian_servo_mode_after_motion = ControllerMode.ACTIVE
        controller = self.controller
        if (
            not emergency
            and mode_after_motion == ControllerMode.DAMPING
            and controller is not None
            and controller.mode == ControllerMode.ACTIVE
        ):
            self.remember(await asyncio.to_thread(controller.damping))

    async def stop_gripper_servo(self, reason: str, *, emergency: bool) -> None:
        servo = self.gripper_servo
        if servo is None:
            return
        mode_after_motion = self.gripper_servo_mode_after_motion
        await asyncio.to_thread(servo.stop, reason, emergency=emergency)
        self.gripper_servo = None
        self.gripper_servo_sequence = -1
        self.gripper_servo_mode_after_motion = ControllerMode.ACTIVE
        controller = self.controller
        if (
            not emergency
            and mode_after_motion == ControllerMode.DAMPING
            and controller is not None
            and controller.mode == ControllerMode.ACTIVE
        ):
            self.remember(await asyncio.to_thread(controller.damping))

    async def stop_teleoperation_servo(
        self,
        reason: str,
        *,
        emergency: bool,
    ) -> None:
        servo = self.teleoperation_servo
        try:
            if servo is not None:
                await asyncio.to_thread(
                    servo.stop,
                    reason,
                    emergency=emergency,
                )
        finally:
            self.teleoperation_servo = None
            self.clear_teleoperation_session()

    async def startup(self) -> None:
        if not self.persistent_controller:
            return
        try:
            await self.ensure_controller_connected()
            print("后台机械臂控制器已自动连接。")
        except (
            TypeError,
            ValueError,
            BusyError,
            ControlError,
            SafetyError,
            RuntimeError,
            OSError,
        ) as exc:
            print(f"后台机械臂首次自动连接失败，将继续重试：{exc}")
        self.controller_maintain_task = asyncio.create_task(
            self.maintain_controller_connection()
        )

    async def maintain_controller_connection(self) -> None:
        while not self.shutting_down:
            if self.controller is None and not self.auto_connect_suspended:
                try:
                    await self.ensure_controller_connected()
                    print("后台机械臂控制器已恢复连接。")
                except (
                    TypeError,
                    ValueError,
                    BusyError,
                    ControlError,
                    SafetyError,
                    RuntimeError,
                    OSError,
                ):
                    pass
            await asyncio.sleep(3.0)

    async def ensure_controller_connected(self) -> ArmSnapshot:
        async with self.controller_connect_lock:
            controller = self.controller
            if controller is not None:
                snapshot = await asyncio.to_thread(controller.get_state)
                return self.remember(snapshot)
            if (
                self.is_busy
                or self.payload_calibration_active
                or self.gripper_calibration_active
                or self.handeye_calibration_active
            ):
                raise ControlError("控制服务正忙。")
            candidate = ArmController(
                interface=self.interface,
                gain_scale=1.0,
                gripper_open_readout=self.gripper_open_readout(),
                safety=WEB_MOTION_SAFETY,
            )
            try:
                snapshot = await asyncio.to_thread(candidate.connect, stabilize=True)
                self.controller = candidate
                self._sync_payload_context()
                self.remember(snapshot)
                self.capture_reference(snapshot)
                return snapshot
            except BaseException:
                self.controller = None
                try:
                    candidate.close(emergency=True)
                except BaseException:
                    pass
                raise

    async def connect_controller(
        self, websocket: ServerConnection, request_id: str | int
    ) -> None:
        try:
            snapshot = await self.ensure_controller_connected()
        except (
            TypeError,
            ValueError,
            BusyError,
            ControlError,
            SafetyError,
            RuntimeError,
            OSError,
        ) as exc:
            message = web_error_message("connect", exc)
            print(f"网页连接机械臂失败：{message}")
            await self.send_error(
                websocket,
                request_id,
                f"连接失败：{message}",
                action="connect",
            )
            return
        await self.send_result(websocket, request_id, "connect", None, snapshot)

    def _sync_payload_context(self) -> None:
        controller = self.controller
        if controller is None or not hasattr(controller, "set_payload"):
            return
        profile = self.payload_profiles.web_state()
        name = profile.get("payload_profile")
        if name == self._payload_context_profile:
            return
        center = profile.get("payload_center_of_mass_tcp_m")
        center_xyz = (
            np.zeros(3, dtype=np.float64)
            if center is None
            else np.asarray(center, dtype=np.float64)
        )
        controller.set_payload(
            Payload(
                name=str(name or "web_payload"),
                mass_kg=float(profile.get("payload_mass_kg", 0.0)),
                center_of_mass_tcp_m=center_xyz,
            )
        )
        self._payload_context_profile = name

    async def preview_xyz_live(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        params: dict[str, Any],
    ) -> None:
        """Solve one coalesced drag preview without entering motion-busy state."""
        controller = self.controller
        if controller is None:
            await self.send_error(
                websocket, request_id, "机械臂尚未连接。", action="preview_xyz_live"
            )
            return
        # A live drag result is display-only. It also invalidates any older
        # formal preview so a client cannot execute a stale target.
        self.cartesian_preview = None
        self.cartesian_preview_velocity_scale = None
        self.cartesian_preview_path_mode = None
        if self.is_busy:
            await self.send_error(
                websocket,
                request_id,
                "上一动作尚未完成。",
                action="preview_xyz_live",
            )
            return
        if controller.mode not in (
            ControllerMode.DAMPING,
            ControllerMode.ACTIVE,
        ):
            await self.send_error(
                websocket,
                request_id,
                "当前模式不能预览 XYZ 运动。",
                action="preview_xyz_live",
            )
            return
        try:
            controller.safety = motion_safety_from_params(params)
            velocity_scale_from_params(params)
            delta = cartesian_delta_from_params(params)
            current = controller.get_tcp_pose()
            target = Pose(
                current.position_xyz_m + delta,
                current.orientation_xyzw,
            )
            velocity_scale = velocity_scale_from_params(params)
            if xyz_path_mode_from_params(params) == "smooth":
                plan = await asyncio.to_thread(
                    controller.plan_movej,
                    target,
                    velocity_scale=velocity_scale,
                    allow_large_motion=True,
                )
            else:
                plan = await asyncio.to_thread(
                    controller.plan_movel,
                    target,
                    velocity_scale=velocity_scale,
                    unrestricted=True,
                )
        except (ValueError, ControlError, SafetyError, RuntimeError) as exc:
            message = web_error_message("preview_xyz_live", exc)
            await self.send_error(
                websocket,
                request_id,
                message,
                action="preview_xyz_live",
            )
            return
        await self.send_result(
            websocket,
            request_id,
            "preview_xyz_live",
            plan,
        )

    def make_operation(
        self, action: str, params: dict[str, Any]
    ) -> Callable[[], Any]:
        controller = self.controller
        if controller is None:
            raise ControlError("机械臂尚未连接。")
        if action == "activate":
            if controller.mode not in (
                ControllerMode.DAMPING,
                ControllerMode.ACTIVE,
            ):
                raise ControlError("当前模式不能启用位置保持。")
            return controller.hold_current
        if action == "damping":
            return controller.damping
        if action == "clear_fault":
            return controller.clear_fault
        if action == "set_reference":
            return controller.get_state
        if action == "cube_stack_execute":
            self.require_active(controller)
            if params.get("confirmed") is not True:
                raise ControlError("请先确认完整叠放预览和工作区安全条件。")
            preview = self.cube_stack_preview
            if preview is None:
                raise ControlError("请先生成完整连续叠放预览。")
            if not math.isclose(
                velocity_scale_from_params(params),
                preview.config.motion.velocity_scale,
                abs_tol=1e-9,
            ):
                self.cube_stack_preview = None
                raise ControlError("叠放速度已改变，请重新生成完整预览。")
            if (
                time.time() - preview.created_at
                > CUBE_STACK_PREVIEW_TIMEOUT_S
            ):
                self.cube_stack_preview = None
                raise ControlError("完整叠放预览已超时，请重新生成。")
            expected_joints = np.asarray(
                preview.start_joint_position, dtype=np.float64
            )
            expected_pose = np.asarray(preview.start_pose, dtype=np.float64)

            def execute_complete_stack() -> dict[str, Any]:
                current = controller.get_state()
                joint_change_deg = math.degrees(
                    float(np.max(np.abs(current.joint_position - expected_joints)))
                )
                position_change_mm = 1000.0 * float(
                    np.linalg.norm(current.eef_pose_6d[:3] - expected_pose[:3])
                )
                if (
                    joint_change_deg > WEB_CARTESIAN_PREVIEW_JOINT_TOLERANCE_DEG
                    or position_change_mm
                    > WEB_CARTESIAN_PREVIEW_POSITION_TOLERANCE_MM
                ):
                    self.cube_stack_preview = None
                    raise ControlError("机械臂姿态已变化，请重新预览完整叠放。")
                try:
                    result = execute_cube_stack(controller, preview)
                    self.cube_manipulation = None
                    self.cube_stage_preview = None
                    return result
                except BaseException as exc:
                    self._hold_after_cube_failure(
                        controller,
                        f"连续叠放失败且位置保持失败：{exc}",
                    )
                    raise
                finally:
                    self.cube_stack_preview = None

            return execute_complete_stack
        if action == "cube_pick_execute":
            self.require_active(controller)
            if params.get("confirmed") is not True:
                raise ControlError("请先在网页中确认抓取预览和安全条件。")
            preview = self.cube_pick_preview
            if preview is None:
                raise ControlError("请先点击“预览抓取”，确认动作包后再执行。")
            actions = preview["actions"]
            config = preview["config"]
            if (
                time.time() - float(preview.get("created_at", 0.0))
                > CUBE_PICK_PREVIEW_TIMEOUT_S
            ):
                self.cube_pick_preview = None
                raise ControlError(
                    f"抓取预览已超过 {CUBE_PICK_PREVIEW_TIMEOUT_S:.0f} 秒，"
                    "请重新生成预览。"
                )
            prepared_observation = preview["observation"]
            expected_joints = np.asarray(
                preview["start_joint_position"], dtype=np.float64
            )
            expected_pose = np.asarray(preview["start_pose"], dtype=np.float64)

            def execute_cube_pick() -> None:
                current = controller.get_state()
                joint_change_deg = math.degrees(
                    float(np.max(np.abs(current.joint_position - expected_joints)))
                )
                position_change_mm = 1000.0 * float(
                    np.linalg.norm(current.eef_pose_6d[:3] - expected_pose[:3])
                )
                if (
                    joint_change_deg > WEB_CARTESIAN_PREVIEW_JOINT_TOLERANCE_DEG
                    or position_change_mm > WEB_CARTESIAN_PREVIEW_POSITION_TOLERANCE_MM
                ):
                    raise ControlError("机械臂姿态已变化，请重新预览抓取动作。")
                try:
                    self._cube_pick_execute(controller, actions, config)
                    final_move = next(
                        action for action in reversed(actions)
                        if action.kind == "move"
                    )
                    self.cube_manipulation = {
                        "stage": "held",
                        "config": config,
                        "source_xy_m": tuple(
                            float(value)
                            for value in prepared_observation.base_xyz_m[:2]
                        ),
                        "orientation_xyzw": tuple(
                            float(value) for value in final_move.orientation_xyzw
                        ),
                    }
                    self.cube_stage_preview = None
                except BaseException as exc:
                    self._hold_after_cube_failure(
                        controller,
                        f"网页抓取动作失败且位置保持失败：{exc}",
                    )
                    raise
                finally:
                    self.cube_pick_preview = None

            return execute_cube_pick
        if action in {"cube_move_execute", "cube_place_execute"}:
            self.require_active(controller)
            if params.get("confirmed") is not True:
                raise ControlError("请先预览当前阶段的末端轨迹，再确认执行。")
            preview = self.cube_stage_preview
            expected_kind = "move" if action == "cube_move_execute" else "place"
            if preview is None or preview.get("kind") != expected_kind:
                raise ControlError(
                    "请先生成移动预览。"
                    if expected_kind == "move"
                    else "请先生成放置预览。"
                )
            if (
                time.time() - float(preview.get("created_at", 0.0))
                > CUBE_PICK_PREVIEW_TIMEOUT_S
            ):
                self.cube_stage_preview = None
                raise ControlError("阶段预览已超时，请重新生成。")
            if expected_kind == "move":
                current_target = self._cube_pick_observation(
                    params, preview["config"]
                )
                prepared_target = preview["target"]
                if math.hypot(
                    current_target.base_xyz_m[0] - prepared_target.base_xyz_m[0],
                    current_target.base_xyz_m[1] - prepared_target.base_xyz_m[1],
                ) > 0.015:
                    self.cube_stage_preview = None
                    raise ControlError("目标方块位置已变化，请重新预览移动轨迹。")
            expected_joints = np.asarray(
                preview["start_joint_position"], dtype=np.float64
            )
            expected_pose = np.asarray(preview["start_pose"], dtype=np.float64)

            def execute_cube_stage() -> None:
                current = controller.get_state()
                joint_change_deg = math.degrees(
                    float(np.max(np.abs(current.joint_position - expected_joints)))
                )
                position_change_mm = 1000.0 * float(
                    np.linalg.norm(current.eef_pose_6d[:3] - expected_pose[:3])
                )
                if (
                    joint_change_deg > WEB_CARTESIAN_PREVIEW_JOINT_TOLERANCE_DEG
                    or position_change_mm > WEB_CARTESIAN_PREVIEW_POSITION_TOLERANCE_MM
                ):
                    self.cube_stage_preview = None
                    raise ControlError("机械臂姿态已变化，请重新预览当前阶段。")
                try:
                    self._cube_pick_execute(
                        controller, preview["actions"], preview["config"]
                    )
                except BaseException as exc:
                    self._hold_after_cube_failure(
                        controller,
                        f"网页方块{expected_kind}动作失败且位置保持失败：{exc}",
                    )
                    raise
                if expected_kind == "move":
                    state = self.cube_manipulation or {}
                    self.cube_manipulation = {
                        **state,
                        "stage": "over_target",
                        "config": preview["config"],
                        "target_xy_m": preview["target_xy_m"],
                        "place_z_m": preview["place_z_m"],
                        "hover_z_m": preview["hover_z_m"],
                        "orientation_xyzw": preview["orientation_xyzw"],
                    }
                else:
                    state = self.cube_manipulation or {}
                    self.cube_manipulation = {**state, "stage": "released"}
                self.cube_stage_preview = None

            return execute_cube_stage
        if action == "preview_xyz":
            if controller.mode not in (
                ControllerMode.DAMPING,
                ControllerMode.ACTIVE,
            ):
                raise ControlError("当前模式不能预览 XYZ 运动。")
            velocity_scale = velocity_scale_from_params(params)
            path_mode = xyz_path_mode_from_params(params)
            delta = cartesian_delta_from_params(params)
            target_orientation = target_orientation_from_params(params)

            def preview_xyz() -> MotionPlan:
                self.cartesian_preview = None
                current = controller.get_tcp_pose()
                resolved_orientation = (
                    current.orientation_xyzw
                    if target_orientation is None
                    else target_orientation
                )
                target = Pose(
                    current.position_xyz_m + delta,
                    resolved_orientation,
                )
                if path_mode == "smooth":
                    trajectory_preview = controller.plan_movej(
                        target,
                        velocity_scale=velocity_scale,
                        allow_large_motion=True,
                    )
                else:
                    trajectory_preview = controller.plan_movel(
                        target,
                        velocity_scale=velocity_scale,
                        unrestricted=True,
                    )
                self.cartesian_preview = trajectory_preview
                self.cartesian_preview_velocity_scale = velocity_scale
                self.cartesian_preview_path_mode = path_mode
                return trajectory_preview

            return preview_xyz
        if action == "move_xyz":
            self.require_active(controller)
            velocity_scale = velocity_scale_from_params(params)
            path_mode = xyz_path_mode_from_params(params)
            delta = cartesian_delta_from_params(params)
            target_orientation = target_orientation_from_params(params)
            preview = self.cartesian_preview
            if preview is None:
                raise ControlError("请先点击“预览 XYZ”，确认 3D 目标姿态后再执行。")
            if (
                self.cartesian_preview_velocity_scale is None
                or not math.isclose(
                    velocity_scale,
                    self.cartesian_preview_velocity_scale,
                    abs_tol=1e-9,
                )
            ):
                self.cartesian_preview = None
                self.cartesian_preview_velocity_scale = None
                self.cartesian_preview_path_mode = None
                raise ControlError("运动速度已改变，请重新预览 XYZ 目标。")
            if self.cartesian_preview_path_mode != path_mode:
                self.cartesian_preview = None
                self.cartesian_preview_velocity_scale = None
                self.cartesian_preview_path_mode = None
                raise ControlError("轨迹模式已改变，请重新预览 XYZ 目标。")
            backend = preview.plan
            if backend is None:
                raise ControlError("XYZ 预览数据无效，请重新预览。")
            if not np.allclose(delta, backend.delta_xyz_m, atol=1e-9, rtol=0.0):
                raise ControlError("XYZ 数值已改变，请重新预览后再执行。")
            preview_orientation = rpy_to_quaternion_xyzw(
                backend.target_pose_6d[3:]
            )
            requested_orientation = (
                rpy_to_quaternion_xyzw(backend.start_pose_6d[3:])
                if target_orientation is None
                else target_orientation
            )
            if quaternion_distance_rad(
                requested_orientation,
                preview_orientation,
            ) > 1e-6:
                raise ControlError("目标姿态已改变，请重新预览 XYZ 目标。")

            def move_xyz() -> MotionResult | CartesianMotionResult:
                current = controller.get_state()
                joint_change_deg = math.degrees(
                    float(
                        np.max(
                            np.abs(
                                current.joint_position
                                - preview.start_joint_position
                            )
                        )
                    )
                )
                position_change_mm = 1000.0 * float(
                    np.linalg.norm(
                        current.eef_pose_6d[:3] - backend.start_pose_6d[:3]
                    )
                )
                if (
                    joint_change_deg
                    > WEB_CARTESIAN_PREVIEW_JOINT_TOLERANCE_DEG
                    or position_change_mm
                    > WEB_CARTESIAN_PREVIEW_POSITION_TOLERANCE_MM
                ):
                    self.cartesian_preview = None
                    self.cartesian_preview_velocity_scale = None
                    self.cartesian_preview_path_mode = None
                    raise ControlError("机械臂姿态已变化，请重新预览 XYZ 目标。")
                self.cartesian_preview = None
                self.cartesian_preview_velocity_scale = None
                self.cartesian_preview_path_mode = None
                return controller.execute_motion_plan(preview)

            return move_xyz
        if action == "preview_park":
            if controller.mode not in (
                ControllerMode.DAMPING,
                ControllerMode.ACTIVE,
            ):
                raise ControlError("当前模式不能预览回参考轨迹。")
            velocity_scale = velocity_scale_from_params(params)
            reference = self.reference_joint_position
            if reference is None:
                raise ControlError("尚未记录机械臂参考状态。")

            def preview_park() -> MotionPlan:
                self.park_preview = None
                self.park_preview_velocity_scale = None
                target = np.asarray(reference, dtype=np.float64)
                preview = controller.plan_movej(
                    target,
                    velocity_scale=velocity_scale,
                    allow_large_motion=True,
                )
                self.park_preview = preview
                self.park_preview_velocity_scale = velocity_scale
                return preview

            return preview_park
        if action == "park":
            self.require_active(controller)
            velocity_scale = velocity_scale_from_params(params)
            reference = self.reference_joint_position
            if reference is None:
                raise ControlError("尚未记录机械臂参考状态。")
            preview = self.park_preview
            if preview is None:
                raise ControlError("请先预览回参考姿态和末端轨迹，再确认执行。")
            target = np.asarray(reference, dtype=np.float64)
            if not np.allclose(
                target,
                preview.target_joint_position,
                atol=1e-9,
                rtol=0.0,
            ):
                self.park_preview = None
                self.park_preview_velocity_scale = None
                raise ControlError("参考姿态已改变，请重新预览回参考轨迹。")
            if (
                self.park_preview_velocity_scale is None
                or not math.isclose(
                    velocity_scale,
                    self.park_preview_velocity_scale,
                    abs_tol=1e-9,
                )
            ):
                self.park_preview = None
                self.park_preview_velocity_scale = None
                raise ControlError("运动速度已改变，请重新预览回参考轨迹。")

            def park() -> MotionResult | GripperMotionResult | ArmSnapshot:
                def restore_gripper() -> GripperMotionResult | None:
                    current_gripper = controller.get_state().gripper_position
                    target_gripper = (
                        HOME_GRIPPER_WIDTH_M
                        if self.reference_gripper_position is None
                        else self.reference_gripper_position
                    )
                    if abs(target_gripper - current_gripper) < 0.0001:
                        return None
                    duration_s = max(
                        minimum_motion_duration_s(controller),
                        controller.minimum_gripper_duration(
                            target_gripper - current_gripper
                        ),
                    ) / velocity_scale
                    return controller.move_gripper(
                        target_gripper,
                        duration_s,
                    )

                current = controller.get_state()
                start_change_deg = math.degrees(
                    float(
                        np.max(
                            np.abs(
                                current.joint_position
                                - preview.start_joint_position
                            )
                        )
                    )
                )
                if start_change_deg > WEB_CARTESIAN_PREVIEW_JOINT_TOLERANCE_DEG:
                    self.park_preview = None
                    self.park_preview_velocity_scale = None
                    raise ControlError("机械臂姿态已变化，请重新预览回参考轨迹。")
                self.park_preview = None
                self.park_preview_velocity_scale = None
                remaining = target - current.joint_position
                max_remaining = float(np.max(np.abs(remaining)))
                if max_remaining < math.radians(0.1):
                    gripper_motion = restore_gripper()
                    result = controller.hold_current()
                    self.cube_manipulation = None
                    self.cube_stage_preview = None
                    return gripper_motion or result
                motion = controller.movej(
                    target,
                    velocity_scale=velocity_scale,
                    allow_large_motion=True,
                )
                error_deg = np.degrees(np.abs(motion.final_error_rad))
                max_error_deg = float(np.max(error_deg))
                if max_error_deg > WEB_PARK_ABORT_TOLERANCE_DEG:
                    joint = int(np.argmax(error_deg)) + 1
                    reason = (
                        f"回参考终点误差过大：J{joint} {max_error_deg:.1f}°；"
                        "已进入故障阻尼，请检查姿态和负载。"
                    )
                    controller.emergency_damping(reason)
                    raise SafetyError(reason)
                if max_error_deg > WEB_PARK_NOMINAL_TOLERANCE_DEG:
                    print(
                        "回参考完成但终点残差偏大："
                        f"{max_error_deg:.2f}°；继续保持当前位置。"
                    )
                restore_gripper()
                controller.hold_current()
                self.cube_manipulation = None
                self.cube_stage_preview = None
                return motion

            return park
        if action == "move_joint":
            self.require_active(controller)
            velocity_scale = velocity_scale_from_params(params)
            joint = params.get("joint")
            delta_deg = params.get("delta_deg")
            if (
                isinstance(joint, bool)
                or not isinstance(joint, int)
                or not 1 <= joint <= 6
            ):
                raise ValueError("joint 必须是 1..6。")
            if not finite_number(delta_deg) or not (
                0.1 <= abs(float(delta_deg)) <= WEB_MAX_JOINT_STEP_DEG
            ):
                raise ValueError(
                    f"网页单次关节步长必须在 0.1..{WEB_MAX_JOINT_STEP_DEG:.0f}°。"
                )
            delta_rad = math.radians(float(delta_deg))

            def move_joint() -> MotionResult:
                target = controller.get_state().joint_position.copy()
                target[joint - 1] += delta_rad
                return controller.movej(
                    target,
                    velocity_scale=velocity_scale,
                )

            return move_joint
        if action == "move_joint_target":
            self.require_active(controller)
            velocity_scale = velocity_scale_from_params(params)
            joint = params.get("joint")
            target_deg = params.get("target_deg")
            if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 6:
                raise ValueError("joint 必须是 1..6。")
            if not finite_number(target_deg):
                raise ValueError("关节目标角度必须是有限数值。")
            target_rad = math.radians(float(target_deg))
            limits = controller.get_limits()
            margin = controller.safety.joint_limit_margin_rad
            lower = float(limits.joint_position_min[joint - 1] + margin)
            upper = float(limits.joint_position_max[joint - 1] - margin)
            if not lower <= target_rad <= upper:
                raise ValueError(
                    f"J{joint} 目标必须在 "
                    f"{math.degrees(lower):.1f}..{math.degrees(upper):.1f}°。"
                )

            def move_joint_target() -> MotionResult:
                target = controller.get_state().joint_position.copy()
                delta_deg = math.degrees(target_rad - float(target[joint - 1]))
                if abs(delta_deg) < 0.1:
                    raise ValueError(f"J{joint} 已经位于该目标附近。")
                target[joint - 1] = target_rad
                return controller.movej(
                    target,
                    velocity_scale=velocity_scale,
                    allow_large_motion=True,
                )

            return move_joint_target
        if action == "reset_joint":
            self.require_active(controller)
            velocity_scale = velocity_scale_from_params(params)
            joint = params.get("joint")
            if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 6:
                raise ValueError("joint 必须是 1..6。")
            reference = self.reference_joint_position
            if reference is None:
                raise ControlError("尚未记录参考姿态。")
            reference_rad = reference[joint - 1]

            def reset_joint() -> MotionResult:
                target = controller.get_state().joint_position.copy()
                remaining = reference_rad - float(target[joint - 1])
                if abs(math.degrees(remaining)) < 0.1:
                    raise ValueError(f"J{joint} 已经位于参考角度附近。")
                target[joint - 1] = reference_rad
                return controller.movej(
                    target,
                    velocity_scale=velocity_scale,
                    allow_large_motion=True,
                )

            return reset_joint
        if action == "gripper_position":
            self.require_active(controller)
            width_mm = params.get("width_mm")
            if not finite_number(width_mm) or not (
                0.0 <= float(width_mm) <= WEB_MAX_GRIPPER_WIDTH_MM
            ):
                raise ValueError(
                    f"网页夹爪开度必须在 0..{WEB_MAX_GRIPPER_WIDTH_MM:.0f} mm。"
                )

            def move_gripper() -> GripperMotionResult:
                current = controller.get_state().gripper_position
                target = float(width_mm) / 1000.0
                velocity_scale = velocity_scale_from_params(params)
                duration = max(
                    minimum_motion_duration_s(controller),
                    controller.minimum_gripper_duration(target - current),
                ) / velocity_scale
                return controller.move_gripper(target, duration)

            return move_gripper
        if action == "reset_gripper":
            self.require_active(controller)

            def reset_gripper() -> GripperMotionResult:
                current = controller.get_state().gripper_position
                target = 0.0
                if abs(target - current) < 0.0001:
                    raise ValueError("夹爪已经处于关闭状态。")
                velocity_scale = velocity_scale_from_params(params)
                duration = max(
                    minimum_motion_duration_s(controller),
                    controller.minimum_gripper_duration(target - current),
                ) / velocity_scale
                return controller.move_gripper(target, duration)

            return reset_gripper
        if action == "gripper_force":
            self.require_active(controller)
            torque_nm = params.get("torque_nm", 0.15)
            minimum_mm = params.get("minimum_mm", 0.0)
            if not finite_number(torque_nm) or not (
                0.05 <= float(torque_nm) <= WEB_MAX_GRIPPER_TORQUE_NM
            ):
                raise ValueError(
                    f"网页闭合扭矩必须在 0.05..{WEB_MAX_GRIPPER_TORQUE_NM:.2f} N·m。"
                )
            if not finite_number(minimum_mm) or not (
                0.0 <= float(minimum_mm) <= WEB_MAX_GRIPPER_WIDTH_MM
            ):
                raise ValueError(
                    f"最小开度必须在 0..{WEB_MAX_GRIPPER_WIDTH_MM:.0f} mm。"
                )
            return lambda: controller.force_close_gripper(
                float(torque_nm),
                minimum_width_m=float(minimum_mm) / 1000.0,
                timeout_s=5.0,
                contact_hold_s=0.5,
            )
        raise ValueError(f"不支持的动作：{action}")

    @staticmethod
    def require_active(controller: ArmController) -> None:
        if controller.mode != ControllerMode.ACTIVE:
            raise ControlError("请先点击“启用当前位置保持”。")

    async def run_operation(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        action: str,
        operation: Callable[[], Any],
        *,
        mode_after_motion: ControllerMode | None = None,
    ) -> None:
        worker = asyncio.create_task(asyncio.to_thread(operation))
        stream_feedback = action in WEB_FEEDBACK_ACTIONS
        try:
            while not worker.done():
                done, _ = await asyncio.wait(
                    {worker}, timeout=WEB_MOTION_FEEDBACK_PERIOD_S
                )
                if worker in done or not stream_feedback:
                    continue
                controller = self.controller
                if controller is None:
                    stream_feedback = False
                    continue
                snapshot = controller.get_cached_state()
                if snapshot is None:
                    continue
                self.remember(snapshot)
                await self.send(
                    websocket,
                    {
                        "type": "motion_state",
                        "id": request_id,
                        "action": action,
                        "state": self.state_payload(snapshot),
                    },
                )
            result = await worker
            snapshot = result if isinstance(result, ArmSnapshot) else None
            controller = self.controller
            if (
                mode_after_motion == ControllerMode.DAMPING
                and controller is not None
                and controller.mode == ControllerMode.ACTIVE
            ):
                snapshot = await asyncio.to_thread(controller.damping)
            elif snapshot is None and controller is not None:
                snapshot = await asyncio.to_thread(controller.get_state)
            if snapshot is not None:
                self.remember(snapshot)
                if action == "set_reference":
                    self.capture_reference(snapshot)
            if self.motion_task is asyncio.current_task():
                self.motion_task = None
            await self.send_result(
                websocket, request_id, action, result, snapshot
            )
            print(f"网页命令完成：{action}")
        except (ValueError, ControlError, SafetyError, RuntimeError) as exc:
            if self.motion_task is asyncio.current_task():
                self.motion_task = None
            controller = self.controller
            if controller is not None:
                snapshot = controller.get_cached_state()
                if snapshot is not None:
                    self.remember(snapshot)
            message = web_error_message(action, exc)
            print(f"网页命令失败：{action}: {message}")
            await self.send_error(websocket, request_id, message, action=action)
        except BaseException as exc:
            controller = self.controller
            if controller is not None:
                await asyncio.to_thread(
                    controller.emergency_damping,
                    f"unexpected web control error: {exc}",
                )
            if self.motion_task is asyncio.current_task():
                self.motion_task = None
            await self.send_error(
                websocket,
                request_id,
                f"未预期错误：{exc}",
                action=action,
            )
        finally:
            if self.motion_task is asyncio.current_task():
                self.motion_task = None

    async def send_state(
        self, websocket: ServerConnection, request_id: str | int
    ) -> None:
        controller = self.controller
        if controller is not None and self.collection_recorder is None:
            try:
                cached_only = (
                    self.is_busy
                    or self.servo_active
                    or self.payload_calibration_active
                    or self.gripper_calibration_active
                    or self.handeye_calibration_active
                )
                snapshot = (
                    controller.get_cached_state()
                    if cached_only
                    else await asyncio.to_thread(controller.get_state)
                )
                if snapshot is not None:
                    self.remember(snapshot)
            except (ControlError, SafetyError, RuntimeError) as exc:
                await self.send_error(websocket, request_id, str(exc))
                return
        if self.collection_stage != "pending_review":
            await self.refresh_collection_camera_state()
        await self.send(
            websocket,
            {"type": "state", "id": request_id, "state": self.state_payload()},
        )

    async def emergency(
        self, websocket: ServerConnection, request_id: str | int
    ) -> None:
        controller = self.controller
        if controller is None:
            await self.send_error(websocket, request_id, "机械臂尚未连接。")
            return
        await self.abort_collection("emergency stop")
        self.cartesian_preview = None
        self.cartesian_preview_velocity_scale = None
        self.cartesian_preview_path_mode = None
        self.cube_pick_preview = None
        self.cube_stage_preview = None
        self.cube_stack_preview = None
        self.cube_manipulation = None
        self.park_preview = None
        self.park_preview_velocity_scale = None
        await self.stop_cartesian_servo("web emergency button", emergency=True)
        await self.stop_gripper_servo("web emergency button", emergency=True)
        await self.stop_teleoperation_servo(
            "web emergency button",
            emergency=True,
        )
        await asyncio.to_thread(controller.emergency_damping, "web emergency button")
        active_task = self.motion_task
        if active_task is not None and active_task is not asyncio.current_task():
            try:
                await active_task
            except BaseException:
                pass
        # Re-assert damping after the worker has stopped.  Controller-side
        # mode guards prevent further commands, and this final call also
        # covers an SDK command that was already in flight at button press.
        await asyncio.to_thread(controller.emergency_damping, "web emergency button")
        try:
            self.remember(await asyncio.to_thread(controller.get_state))
        except (ControlError, SafetyError, RuntimeError):
            pass
        await self.send_result(websocket, request_id, "emergency", None)

    async def retain_controller_after_browser_disconnect(self) -> None:
        controller = self.controller
        if controller is None:
            return
        if self.collection_recorder is not None:
            print("采集继续由后台保持；手机可重新打开页面接管。")
            return
        await self.stop_cartesian_servo(
            "browser disconnected during servo", emergency=True
        )
        await self.stop_gripper_servo(
            "browser disconnected during gripper servo", emergency=True
        )
        await self.stop_teleoperation_servo(
            "browser disconnected during teleoperation", emergency=True
        )
        if self.is_busy:
            await asyncio.to_thread(
                controller.emergency_damping,
                "browser disconnected during motion",
            )
            active_task = self.motion_task
            if active_task is not None:
                try:
                    await active_task
                except BaseException:
                    pass
        if controller.mode == ControllerMode.ACTIVE:
            try:
                self.remember(await asyncio.to_thread(controller.damping))
            except (ControlError, SafetyError, RuntimeError):
                await asyncio.to_thread(
                    controller.emergency_damping,
                    "browser disconnected while entering damping",
                )
        print("后台继续持有机械臂，当前保持阻尼安全待机。")

    async def close_controller(self, *, browser_lost: bool) -> None:
        await self.abort_collection(
            "browser disconnected" if browser_lost else "controller disconnected"
        )
        controller = self.controller
        if controller is None:
            self.clear_teleoperation_session()
            return
        await self.stop_cartesian_servo(
            "browser disconnected during servo" if browser_lost else "controller closing",
            emergency=browser_lost,
        )
        await self.stop_gripper_servo(
            "browser disconnected during gripper servo"
            if browser_lost
            else "controller closing",
            emergency=browser_lost,
        )
        await self.stop_teleoperation_servo(
            "browser disconnected during teleoperation"
            if browser_lost
            else "controller closing",
            emergency=browser_lost,
        )
        if self.is_busy:
            await asyncio.to_thread(
                controller.emergency_damping,
                "browser disconnected during motion",
            )
            active_task = self.motion_task
            if active_task is not None:
                try:
                    await active_task
                except BaseException:
                    pass
            await asyncio.to_thread(
                controller.emergency_damping,
                "browser disconnected during motion",
            )
        emergency = controller.mode == ControllerMode.FAULT
        await asyncio.to_thread(controller.close, emergency=emergency)
        self.controller = None
        self.last_snapshot = None
        self.last_snapshot_wall_time = 0.0
        self.reference_joint_position = None
        self.reference_gripper_position = None
        self.cartesian_preview = None
        self.cartesian_preview_velocity_scale = None
        self.cartesian_preview_path_mode = None
        self.cube_pick_preview = None
        self.cube_stage_preview = None
        self.cube_stack_preview = None
        self.cube_manipulation = None
        self.park_preview = None
        self.park_preview_velocity_scale = None
        if browser_lost:
            print("浏览器 watchdog 已释放控制器。")

    async def send_result(
        self,
        websocket: ServerConnection,
        request_id: str | int,
        action: str,
        result: Any,
        snapshot: ArmSnapshot | None = None,
    ) -> None:
        await self.send(
            websocket,
            {
                "type": "command_result",
                "id": request_id,
                "action": action,
                "ok": True,
                "result": self.result_payload(result),
                "state": self.state_payload(snapshot),
            },
        )

    async def send_error(
        self,
        websocket: ServerConnection,
        request_id: str | int | None,
        message: str,
        *,
        action: str | None = None,
    ) -> None:
        await self.send(
            websocket,
            {
                "type": "command_result",
                "id": request_id,
                "action": action,
                "ok": False,
                "error": message,
                "state": self.state_payload(),
            },
        )

    @staticmethod
    def result_payload(result: Any) -> dict[str, Any] | None:
        if isinstance(result, dict):
            return result
        if isinstance(result, MotionResult):
            return {
                "kind": "joint_motion",
                "final_error_deg": [
                    math.degrees(float(value)) for value in result.final_error_rad
                ],
            }
        if isinstance(result, MotionPlan):
            backend = result.plan
            if backend is None:
                path = np.asarray(result.path_xyz_m, dtype=np.float64)
                kind = "park_preview"
                payload: dict[str, Any] = {
                    "kind": kind,
                    "start_joint_position_deg": np.degrees(
                        result.start_joint_position
                    ).tolist(),
                    "target_joint_position_deg": np.degrees(
                        result.target_joint_position
                    ).tolist(),
                    "max_joint_delta_deg": math.degrees(
                        result.max_joint_delta_rad
                    ),
                }
            else:
                payload = ControlKernel.result_payload(backend) or {}
                path = np.asarray(result.path_xyz_m, dtype=np.float64)
                if result.motion_type == MotionType.MOVEL:
                    payload["path_mode"] = "linear"
                elif (
                    result.motion_type == MotionType.MOVEJ
                    and result.start_tcp_pose_in_base is not None
                    and result.target_tcp_pose_in_base is not None
                ):
                    payload["path_mode"] = "smooth"
                else:
                    payload["path_mode"] = result.path_mode
                if (
                    result.start_tcp_pose_in_base is not None
                    and result.target_tcp_pose_in_base is not None
                ):
                    start_tcp = result.start_tcp_pose_in_base
                    target_tcp = result.target_tcp_pose_in_base
                    payload.update(
                        {
                            "start_xyz_mm": (
                                1000.0 * start_tcp.position_xyz_m
                            ).tolist(),
                            "target_xyz_mm": (
                                1000.0 * target_tcp.position_xyz_m
                            ).tolist(),
                            "delta_xyz_mm": (
                                1000.0
                                * (
                                    target_tcp.position_xyz_m
                                    - start_tcp.position_xyz_m
                                )
                            ).tolist(),
                            "target_orientation_xyzw": target_tcp.orientation_xyzw.tolist(),
                        }
                    )
            if path.shape[0] > WEB_TRAJECTORY_PREVIEW_MAX_POINTS:
                indices = np.linspace(
                    0,
                    path.shape[0] - 1,
                    WEB_TRAJECTORY_PREVIEW_MAX_POINTS,
                    dtype=np.int64,
                )
                path = path[indices]
            payload.update(
                {
                    "path_xyz_mm": (1000.0 * path).tolist(),
                    "trajectory_duration_s": result.duration_s,
                }
            )
            return payload
        if isinstance(result, CartesianPlan):
            return {
                "kind": "cartesian_preview",
                "delta_xyz_mm": [
                    1000.0 * float(value) for value in result.delta_xyz_m
                ],
                "start_xyz_mm": [
                    1000.0 * float(value) for value in result.start_pose_6d[:3]
                ],
                "target_xyz_mm": [
                    1000.0 * float(value) for value in result.target_pose_6d[:3]
                ],
                "target_joint_position_deg": [
                    math.degrees(float(value))
                    for value in result.target_joint_position
                ],
                "target_orientation_xyzw": rpy_to_quaternion_xyzw(
                    result.target_pose_6d[3:]
                ).tolist(),
                "ik_position_error_mm": 1000.0 * result.position_error_m,
                "ik_orientation_error_deg": math.degrees(
                    result.orientation_error_rad
                ),
                "max_joint_delta_deg": math.degrees(
                    result.max_joint_delta_rad
                ),
            }
        if isinstance(result, CartesianMotionResult):
            target_pose = (
                result.plan.target_pose_6d
                if result.target_tcp_pose_6d is None
                else result.target_tcp_pose_6d
            )
            return {
                "kind": "cartesian_motion",
                "target_xyz_mm": [
                    1000.0 * float(value)
                    for value in target_pose[:3]
                ],
                "final_xyz_mm": [
                    1000.0 * float(value) for value in result.final_pose_6d[:3]
                ],
                "final_position_error_mm": (
                    1000.0 * result.final_position_error_m
                ),
                "final_joint_error_deg": [
                    math.degrees(float(value))
                    for value in result.joint_motion.final_error_rad
                ],
            }
        if isinstance(result, GripperMotionResult):
            return {
                "kind": "gripper_position",
                "final_width_mm": 1000.0 * result.final_width_m,
                "final_error_mm": 1000.0 * result.final_error_m,
            }
        if isinstance(result, GripperForceResult):
            return {
                "kind": "gripper_force",
                "final_width_mm": 1000.0 * result.final_width_m,
                "stop_reason": result.stop_reason,
                "max_abs_torque_nm": result.max_abs_torque_nm,
            }
        return None

    async def shutdown(self) -> None:
        self.shutting_down = True
        maintain_task = self.controller_maintain_task
        self.controller_maintain_task = None
        if maintain_task is not None:
            maintain_task.cancel()
            await asyncio.gather(maintain_task, return_exceptions=True)
        await self.cancel_payload_calibration("网页服务关闭，标定已取消。")
        await self.cancel_gripper_calibration("网页服务关闭，夹爪标定已取消。")
        await self.cancel_handeye_calibration("网页服务关闭，标定已取消。")
        await self.close_controller(browser_lost=True)
        if self.collection_camera is not None:
            await self.collection_camera.close()
