from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np

from arx5_control import (
    ArmController,
    ControlError,
    ControllerMode,
    GripperState,
    MotionPlan,
    SafetyConfig,
    SafetyError,
    TeleoperationServo,
    TeleoperationTarget,
    WorkspaceBounds,
    load_teleoperation_config,
)
from arx5_control.cartesian import (
    normalize_quaternion_xyzw,
    rpy_to_quaternion_xyzw,
)
from arx5_control.gripper_calibration import load_gripper_open_readout
from arx5_control.teleoperation import (
    quaternion_distance_rad,
    quaternion_multiply_xyzw,
)
from arx5_paths import CONFIG_ROOT
from isaac_groot_data_collection.geometry import pose_6d_to_eef_9d
from isaac_groot_data_collection.home import load_collection_home, load_training_start

from gr_zeroshot.cameras import DualRgbCamera, SynchronizedFrames
from gr_zeroshot.checkpoint import ACTION_HORIZON, LANGUAGE_KEY, STATE_KEYS, VIDEO_KEYS
from gr_zeroshot.policy import PolicyClient, PolicyError, PolicyWorker


EXECUTION_HZ = 30.0
REPLAN_STEPS = ACTION_HORIZON
REPLAN_INTERVAL_S = REPLAN_STEPS / EXECUTION_HZ
RTC_MAX_OVERLAP_STEPS = 8
RTC_RAMP_RATE = 4.0
PREVIEW_ATTEMPTS = 1
ACTION_PERIOD_NS = round(1e9 / EXECUTION_HZ)
MAXIMUM_TRANSLATION_JUMP_M = 0.020
MAXIMUM_ROTATION_JUMP_RAD = math.radians(15.0)
MINIMUM_USABLE_ACTIONS = 3
# Keep a usable tail even when one inference takes longer than a short chunk.
# Re-planning every 8 steps makes the first half of a stale chunk the least
# useful part; dropping the whole 16-step chunk leaves the servo holding home.
MAX_LATENCY_SKIP_STEPS = min(RTC_MAX_OVERLAP_STEPS, ACTION_HORIZON - MINIMUM_USABLE_ACTIONS)
WORKSPACE_PROJECTION_MARGIN_M = 0.005
HOME_VELOCITY_SCALE = 0.75
TRAINING_START_VELOCITY_SCALE = 0.55
TRAINING_OPEN_GRIPPER_WIDTH_M = 0.080


def _latency_skip_steps(latency_s: float) -> int:
    if not math.isfinite(latency_s) or latency_s < 0.0:
        raise ActionRejected("推理延迟无效")
    return min(MAX_LATENCY_SKIP_STEPS, int(math.ceil(latency_s * EXECUTION_HZ)))


class RuntimeFault(RuntimeError):
    """Raised when continuous control must stop and return to damping."""


class ActionRejected(RuntimeFault):
    """Raised when a complete model action chunk is not safe to consume."""


@dataclass(frozen=True)
class ActionTarget:
    xyz_m: np.ndarray
    quaternion_xyzw: np.ndarray
    gripper_state: GripperState
    raw_gripper_m: float

    def __post_init__(self) -> None:
        xyz = np.asarray(self.xyz_m, dtype=np.float64).copy()
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError("action XYZ must contain three finite values")
        quaternion = normalize_quaternion_xyzw(self.quaternion_xyzw)
        if not math.isfinite(self.raw_gripper_m):
            raise ValueError("action gripper value must be finite")
        xyz.setflags(write=False)
        quaternion.setflags(write=False)
        object.__setattr__(self, "xyz_m", xyz)
        object.__setattr__(self, "quaternion_xyzw", quaternion)
        object.__setattr__(self, "gripper_state", GripperState(self.gripper_state))
        object.__setattr__(self, "raw_gripper_m", float(self.raw_gripper_m))


@dataclass(frozen=True)
class ActionPreview:
    usable_actions: int
    latency_ms: float
    first_delta_mm: np.ndarray
    first_rotation_delta_deg: float
    gripper_state: GripperState


@dataclass(frozen=True)
class ScheduledActionTarget:
    execute_at_ns: int
    target: ActionTarget


def _blend_quaternion(
    previous: np.ndarray,
    current: np.ndarray,
    weight: float,
) -> np.ndarray:
    previous_value = normalize_quaternion_xyzw(previous)
    current_value = normalize_quaternion_xyzw(current)
    if float(np.dot(previous_value, current_value)) < 0.0:
        current_value = -current_value
    return normalize_quaternion_xyzw(
        (1.0 - weight) * previous_value + weight * current_value
    )


def blend_action_targets(
    previous: ActionTarget,
    current: ActionTarget,
    weight: float,
) -> ActionTarget:
    if not math.isfinite(weight) or not 0.0 <= weight <= 1.0:
        raise ValueError("action blend weight must lie in [0, 1]")
    raw_gripper = (
        (1.0 - weight) * previous.raw_gripper_m
        + weight * current.raw_gripper_m
    )
    return ActionTarget(
        (1.0 - weight) * previous.xyz_m + weight * current.xyz_m,
        _blend_quaternion(
            previous.quaternion_xyzw,
            current.quaternion_xyzw,
            weight,
        ),
        _next_gripper_state(raw_gripper, previous.gripper_state),
        raw_gripper,
    )


def schedule_action_targets(
    targets: list[ActionTarget],
    *,
    observation_ns: int,
    latency_s: float,
    timeline_origin_ns: int,
) -> list[ScheduledActionTarget]:
    skipped_steps = _latency_skip_steps(latency_s)
    scheduled: list[ScheduledActionTarget] = []
    for offset, target in enumerate(targets):
        action_index = skipped_steps + offset
        desired_ns = observation_ns + (action_index + 1) * ACTION_PERIOD_NS
        timeline_slot = max(
            0,
            int(round((desired_ns - timeline_origin_ns) / ACTION_PERIOD_NS)),
        )
        scheduled.append(
            ScheduledActionTarget(
                timeline_origin_ns + timeline_slot * ACTION_PERIOD_NS,
                target,
            )
        )
    return scheduled


def merge_scheduled_actions(
    queued: deque[ScheduledActionTarget],
    incoming: list[ScheduledActionTarget],
    *,
    now_ns: int,
) -> deque[ScheduledActionTarget]:
    future = {
        item.execute_at_ns: item.target
        for item in queued
        if item.execute_at_ns + ACTION_PERIOD_NS // 2 >= now_ns
    }
    incoming_by_time = {
        item.execute_at_ns: item.target
        for item in incoming
        if item.execute_at_ns + ACTION_PERIOD_NS // 2 >= now_ns
    }
    overlap_times = sorted(set(future) & set(incoming_by_time))
    blended_times = set(overlap_times[:RTC_MAX_OVERLAP_STEPS])
    blend_count = len(blended_times)
    blend_index = 0
    for execute_at_ns in sorted(incoming_by_time):
        incoming_target = incoming_by_time[execute_at_ns]
        if execute_at_ns in blended_times:
            blend_index += 1
            future[execute_at_ns] = blend_action_targets(
                future[execute_at_ns],
                incoming_target,
                blend_index / (blend_count + 1),
            )
        else:
            future[execute_at_ns] = incoming_target
    return deque(
        ScheduledActionTarget(execute_at_ns, future[execute_at_ns])
        for execute_at_ns in sorted(future)
    )


def validate_scheduled_actions(
    scheduled: deque[ScheduledActionTarget],
    *,
    current_pose_6d: np.ndarray,
    workspace: WorkspaceBounds,
) -> None:
    pose = np.asarray(current_pose_6d, dtype=np.float64)
    previous_xyz = pose[:3]
    previous_quaternion = rpy_to_quaternion_xyzw(pose[3:])
    for index, item in enumerate(scheduled):
        target = item.target
        if not workspace.contains(target.xyz_m):
            raise ActionRejected(f"融合动作 {index} 超出工作区")
        if float(np.linalg.norm(target.xyz_m - previous_xyz)) > MAXIMUM_TRANSLATION_JUMP_M:
            raise ActionRejected(f"融合动作 {index} 位移跳变超过 20 mm")
        if (
            quaternion_distance_rad(target.quaternion_xyzw, previous_quaternion)
            > MAXIMUM_ROTATION_JUMP_RAD
        ):
            raise ActionRejected(f"融合动作 {index} 姿态跳变超过 15 度")
        previous_xyz = target.xyz_m
        previous_quaternion = target.quaternion_xyzw


def _limit_action_target(
    target: ActionTarget,
    *,
    previous_xyz: np.ndarray,
    previous_quaternion: np.ndarray,
    workspace: WorkspaceBounds,
) -> ActionTarget:
    xyz = np.clip(
        target.xyz_m,
        workspace.minimum_m + WORKSPACE_PROJECTION_MARGIN_M,
        workspace.maximum_m - WORKSPACE_PROJECTION_MARGIN_M,
    )
    delta = xyz - previous_xyz
    distance = float(np.linalg.norm(delta))
    maximum_translation = MAXIMUM_TRANSLATION_JUMP_M - 1e-6
    if distance > maximum_translation:
        xyz = previous_xyz + delta * (maximum_translation / distance)
    quaternion = previous_quaternion
    return ActionTarget(
        xyz,
        quaternion,
        target.gripper_state,
        target.raw_gripper_m,
    )


def limit_scheduled_actions(
    scheduled: deque[ScheduledActionTarget],
    *,
    current_pose_6d: np.ndarray,
    workspace: WorkspaceBounds,
) -> deque[ScheduledActionTarget]:
    pose = np.asarray(current_pose_6d, dtype=np.float64)
    previous_xyz = pose[:3]
    previous_quaternion = rpy_to_quaternion_xyzw(pose[3:])
    limited: deque[ScheduledActionTarget] = deque()
    for item in scheduled:
        target = _limit_action_target(
            item.target,
            previous_xyz=previous_xyz,
            previous_quaternion=previous_quaternion,
            workspace=workspace,
        )
        limited.append(ScheduledActionTarget(item.execute_at_ns, target))
        previous_xyz = target.xyz_m
        previous_quaternion = target.quaternion_xyzw
    return limited


def rtc_options(
    previous_action: dict[str, np.ndarray] | None,
    *,
    previous_observation_ns: int | None,
    observation_ns: int,
    estimated_latency_s: float,
) -> dict[str, Any] | None:
    if previous_action is None or previous_observation_ns is None:
        return None
    elapsed_steps = max(
        1,
        int(round((observation_ns - previous_observation_ns) / ACTION_PERIOD_NS)),
    )
    overlap_steps = min(
        RTC_MAX_OVERLAP_STEPS,
        max(0, ACTION_HORIZON - elapsed_steps),
    )
    if overlap_steps == 0:
        return None
    frozen_steps = min(
        overlap_steps,
        _latency_skip_steps(estimated_latency_s),
    )
    return {
        "previous_action": previous_action,
        "action_horizon": ACTION_HORIZON,
        "rtc_overlap_steps": overlap_steps,
        "rtc_frozen_steps": frozen_steps,
        "rtc_ramp_rate": RTC_RAMP_RATE,
    }


def build_observation(
    frames: SynchronizedFrames,
    snapshot: Any,
    task: str,
    video_keys: tuple[str, ...] = VIDEO_KEYS,
    state_keys: tuple[str, ...] = STATE_KEYS,
) -> dict[str, Any]:
    instruction = str(task).strip()
    if not instruction:
        raise ValueError("任务描述不能为空")
    pose = np.asarray(snapshot.eef_pose_6d, dtype=np.float64)
    joints = np.asarray(snapshot.joint_position, dtype=np.float64)
    gripper = float(snapshot.gripper_position)
    if (
        pose.shape != (6,)
        or joints.shape != (6,)
        or not np.all(np.isfinite(pose))
        or not np.all(np.isfinite(joints))
        or not math.isfinite(gripper)
    ):
        raise RuntimeFault("机械臂观测状态无效")
    available_frames = {"wrist_rgb": frames.wrist}
    if frames.exterior is not None:
        available_frames["exterior_rgb"] = frames.exterior
    missing = set(video_keys) - set(available_frames)
    if missing:
        raise RuntimeFault(f"缺少模型所需相机帧：{', '.join(sorted(missing))}")
    available_states = {
        "eef_9d": pose_6d_to_eef_9d(pose)[None, None].astype(np.float32),
        "gripper_position": np.asarray([[[gripper]]], dtype=np.float32),
        "joint_position": joints[None, None].astype(np.float32),
    }
    return {
        "video": {
            key: available_frames[key].image[None, None, ...]
            for key in video_keys
        },
        "state": {key: available_states[key] for key in state_keys},
        "language": {LANGUAGE_KEY: [[instruction]]},
    }


def synthetic_observation(
    task: str = "move the object",
    video_keys: tuple[str, ...] = VIDEO_KEYS,
    state_keys: tuple[str, ...] = STATE_KEYS,
) -> dict[str, Any]:
    image = np.zeros((224, 224, 3), dtype=np.uint8)
    eef_9d = np.asarray(
        [0.30, 0.0, 0.20, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        dtype=np.float32,
    )
    available_states = {
        "eef_9d": eef_9d[None, None],
        "gripper_position": np.asarray([[[0.02]]], dtype=np.float32),
        "joint_position": np.zeros((1, 1, 6), dtype=np.float32),
    }
    return {
        "video": {key: image[None, None] for key in video_keys},
        "state": {key: available_states[key] for key in state_keys},
        "language": {LANGUAGE_KEY: [[str(task).strip()]]},
    }


def _action_array(
    actions: dict[str, Any],
    key: str,
    dimension: int,
) -> np.ndarray:
    value = actions.get(key)
    if not isinstance(value, np.ndarray):
        raise ActionRejected(f"动作 {key} 不是 ndarray")
    expected = (1, ACTION_HORIZON, dimension)
    if value.shape != expected:
        raise ActionRejected(f"动作 {key} 形状应为 {expected}，实际为 {value.shape}")
    if value.dtype.kind not in "fc" or not np.all(np.isfinite(value)):
        raise ActionRejected(f"动作 {key} 含无效数值")
    return value[0].astype(np.float64, copy=False)


def _matrix_from_rot6d(value: np.ndarray) -> np.ndarray:
    rows = np.asarray(value, dtype=np.float64)
    if rows.shape != (6,) or not np.all(np.isfinite(rows)):
        raise ActionRejected("旋转 6D 无效")
    first_norm = float(np.linalg.norm(rows[:3]))
    if first_norm < 1e-6:
        raise ActionRejected("旋转 6D 第一行退化")
    first = rows[:3] / first_norm
    second_raw = rows[3:] - float(np.dot(rows[3:], first)) * first
    second_norm = float(np.linalg.norm(second_raw))
    if second_norm < 1e-6:
        raise ActionRejected("旋转 6D 第二行退化")
    second = second_raw / second_norm
    third = np.cross(first, second)
    matrix = np.stack((first, second, third), axis=0)
    if (
        not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-5)
        or not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1e-5)
    ):
        raise ActionRejected("旋转 6D 无法重建正交矩阵")
    return matrix


def _quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    value = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(value))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            [
                (value[2, 1] - value[1, 2]) / scale,
                (value[0, 2] - value[2, 0]) / scale,
                (value[1, 0] - value[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(value)))
        if index == 0:
            scale = math.sqrt(1.0 + value[0, 0] - value[1, 1] - value[2, 2]) * 2.0
            quaternion = np.asarray(
                [
                    0.25 * scale,
                    (value[0, 1] + value[1, 0]) / scale,
                    (value[0, 2] + value[2, 0]) / scale,
                    (value[2, 1] - value[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + value[1, 1] - value[0, 0] - value[2, 2]) * 2.0
            quaternion = np.asarray(
                [
                    (value[0, 1] + value[1, 0]) / scale,
                    0.25 * scale,
                    (value[1, 2] + value[2, 1]) / scale,
                    (value[0, 2] - value[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + value[2, 2] - value[0, 0] - value[1, 1]) * 2.0
            quaternion = np.asarray(
                [
                    (value[0, 2] + value[2, 0]) / scale,
                    (value[1, 2] + value[2, 1]) / scale,
                    0.25 * scale,
                    (value[1, 0] - value[0, 1]) / scale,
                ]
            )
    return normalize_quaternion_xyzw(quaternion)


def _next_gripper_state(
    value_m: float,
    previous: GripperState,
    reference_width_m: float | None = None,
) -> GripperState:
    if reference_width_m is not None:
        if (
            previous == GripperState.OPEN
            and value_m <= reference_width_m - 0.003
        ):
            return GripperState.CLOSE
        if (
            previous == GripperState.CLOSE
            and value_m >= reference_width_m + 0.020
        ):
            return GripperState.OPEN
        return previous
    if value_m >= 0.015:
        return GripperState.OPEN
    if value_m <= 0.005:
        return GripperState.CLOSE
    return previous


def decode_action_chunk(
    actions: dict[str, Any],
    *,
    current_pose_6d: np.ndarray,
    workspace: WorkspaceBounds,
    previous_gripper_state: GripperState,
    latency_s: float,
    current_gripper_m: float | None = None,
) -> list[ActionTarget]:
    action_keys = set(actions)
    if action_keys not in (
        {"eef_9d", "gripper_position"},
        {"eef_9d", "gripper_position", "joint_position"},
    ):
        raise ActionRejected("动作键必须包含 eef_9d、gripper_position，可选 joint_position")
    eef = _action_array(actions, "eef_9d", 9)
    gripper = _action_array(actions, "gripper_position", 1)[:, 0]
    if "joint_position" in actions:
        _action_array(actions, "joint_position", 6)
    pose = np.asarray(current_pose_6d, dtype=np.float64)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise ActionRejected("当前 TCP 状态无效")
    current_xyz = pose[:3]
    current_quaternion = rpy_to_quaternion_xyzw(pose[3:])
    targets: list[ActionTarget] = []
    state = GripperState(previous_gripper_state)
    for index in range(ACTION_HORIZON):
        xyz = eef[index, :3]
        quaternion = _quaternion_from_matrix(_matrix_from_rot6d(eef[index, 3:]))
        state = _next_gripper_state(
            float(gripper[index]),
            state,
            current_gripper_m,
        )
        target = ActionTarget(xyz, quaternion, state, float(gripper[index]))
        targets.append(target)
    skip = _latency_skip_steps(latency_s)
    usable = targets[skip:]
    if len(usable) < MINIMUM_USABLE_ACTIONS:
        raise ActionRejected("推理延迟后不足 3 个可用动作")
    limited: list[ActionTarget] = []
    previous_xyz = current_xyz
    previous_quaternion = current_quaternion
    for target in usable:
        target = _limit_action_target(
            target,
            previous_xyz=previous_xyz,
            previous_quaternion=previous_quaternion,
            workspace=workspace,
        )
        limited.append(target)
        previous_xyz = target.xyz_m
        previous_quaternion = target.quaternion_xyzw
    return limited


def action_preview(
    targets: list[ActionTarget],
    current_pose_6d: np.ndarray,
    latency_s: float,
) -> ActionPreview:
    current = np.asarray(current_pose_6d, dtype=np.float64)
    current_quaternion = rpy_to_quaternion_xyzw(current[3:])
    first = targets[0]
    return ActionPreview(
        usable_actions=len(targets),
        latency_ms=1000.0 * latency_s,
        first_delta_mm=1000.0 * (first.xyz_m - current[:3]),
        first_rotation_delta_deg=math.degrees(
            quaternion_distance_rad(first.quaternion_xyzw, current_quaternion)
        ),
        gripper_state=first.gripper_state,
    )


def _quaternion_inverse(value: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(value)
    return np.asarray([-x, -y, -z, w], dtype=np.float64)


def to_servo_target(
    target: ActionTarget,
    start_pose_6d: np.ndarray,
) -> TeleoperationTarget:
    start = np.asarray(start_pose_6d, dtype=np.float64)
    start_quaternion = rpy_to_quaternion_xyzw(start[3:])
    relative_quaternion = quaternion_multiply_xyzw(
        target.quaternion_xyzw,
        _quaternion_inverse(start_quaternion),
    )
    return TeleoperationTarget(
        target.xyz_m - start[:3],
        relative_quaternion,
        target.gripper_state,
    )


class FineTunedRuntime:
    def __init__(
        self,
        cameras: DualRgbCamera,
        *,
        policy_host: str,
        policy_port: int,
        controller: ArmController | None = None,
        print_fn: Callable[[str], None] = print,
        video_keys: tuple[str, ...] = VIDEO_KEYS,
        state_keys: tuple[str, ...] = STATE_KEYS,
    ) -> None:
        self.cameras = cameras
        self.policy_host = policy_host
        self.policy_port = policy_port
        self.controller = controller or ArmController(
            model="X5",
            interface="can0",
            gain_scale=1.0,
            safety=replace(
                SafetyConfig(),
                cartesian_fk_orientation_tolerance_rad=math.radians(2.0),
                feedback_velocity_limit_rad_s=np.finfo(np.float64).max,
            ),
            gripper_open_readout=load_gripper_open_readout(
                model="X5", interface="can0"
            ),
        )
        self.print = print_fn
        self.video_keys = video_keys
        self.state_keys = state_keys
        self.teleoperation_config = load_teleoperation_config(
            CONFIG_ROOT / "teleoperation.json"
        )
        self._connected = False
        self._return_home_required = False

    def connect(self) -> None:
        if not self._connected:
            self.controller.connect(stabilize=False)
            self._connected = True

    def preview(self, task: str) -> ActionPreview:
        self.connect()
        snapshot = self.controller.get_state()
        observation = build_observation(
            self.cameras.latest(), snapshot, task, self.video_keys, self.state_keys
        )
        state = (
            GripperState.OPEN
            if snapshot.gripper_position >= 0.010
            else GripperState.CLOSE
        )
        with PolicyClient(
            self.policy_host,
            self.policy_port,
            timeout_s=2.0,
        ) as client:
            last_error: ActionRejected | None = None
            for attempt in range(1, PREVIEW_ATTEMPTS + 1):
                started = time.monotonic()
                actions, _ = client.get_action(observation)
                latency = time.monotonic() - started
                try:
                    targets = decode_action_chunk(
                        actions,
                        current_pose_6d=snapshot.eef_pose_6d,
                        workspace=self.teleoperation_config.workspace,
                        previous_gripper_state=state,
                        latency_s=latency,
                        current_gripper_m=snapshot.gripper_position,
                    )
                except ActionRejected as exc:
                    last_error = exc
                    if attempt < PREVIEW_ATTEMPTS:
                        self.print(
                            f"模型预览第 {attempt} 次采样无效，正在自动重采样：{exc}"
                        )
                    continue
                if attempt > 1:
                    self.print(f"模型预览第 {attempt} 次采样通过。")
                return action_preview(targets, snapshot.eef_pose_6d, latency)
        assert last_error is not None
        raise ActionRejected(
            f"连续 {PREVIEW_ATTEMPTS} 次模型预览均无安全动作：{last_error}"
        ) from last_error

    def run_task(self, task: str, poll_command: Callable[[], str | None]) -> str:
        self.connect()
        servo: TeleoperationServo | None = None
        worker: PolicyWorker | None = None
        reason = "任务停止"
        normal_stop = False
        try:
            self.controller.activate()
            servo = TeleoperationServo(
                self.controller,
                config=self.teleoperation_config,
                profile_name="fine",
                rate_hz=100.0,
                watchdog_timeout_s=0.25,
                state_stale_timeout_s=0.20,
                gripper_velocity_m_s=0.04,
                gripper_acceleration_m_s2=0.12,
                gripper_jerk_m_s3=0.60,
                startup_gripper_tolerance_m=float(
                    getattr(
                        getattr(self.controller, "safety", None),
                        "gripper_position_tolerance_m",
                        0.005,
                    )
                ),
            )
            start = servo.start(
                open_width_m=TRAINING_OPEN_GRIPPER_WIDTH_M,
                closing_torque_nm=0.15,
            )
            self._return_home_required = True
            gripper_state = (
                GripperState.OPEN
                if start.gripper_position >= 0.010
                else GripperState.CLOSE
            )
            hold = ActionTarget(
                start.eef_pose_6d[:3],
                rpy_to_quaternion_xyzw(start.eef_pose_6d[3:]),
                gripper_state,
                start.gripper_position,
            )
            queue: deque[ScheduledActionTarget] = deque()
            last_target = hold
            last_servo_target = hold
            next_action_at = time.monotonic()
            inference_pending = False
            worker = PolicyWorker(
                self.policy_host,
                self.policy_port,
                timeout_s=1.0,
            )
            while True:
                command = poll_command()
                if command is not None:
                    reason = f"终端命令 {command}"
                    normal_stop = True
                    return command
                status = servo.status()
                if status.error:
                    raise RuntimeFault(status.error)
                snapshot = status.feedback_snapshot
                if snapshot is None:
                    raise RuntimeFault("实时伺服没有机械臂反馈")
                result = worker.poll()
                if result is not None:
                    inference_pending = False
                    if result.error is not None:
                        pass
                    elif result.actions is None:
                        pass
                    else:
                        try:
                            targets = decode_action_chunk(
                                result.actions,
                                current_pose_6d=snapshot.eef_pose_6d,
                                workspace=self.teleoperation_config.workspace,
                                previous_gripper_state=gripper_state,
                                latency_s=0.0,
                                current_gripper_m=snapshot.gripper_position,
                            )
                            execute_from_ns = time.monotonic_ns()
                            candidate = limit_scheduled_actions(
                                deque(
                                    ScheduledActionTarget(
                                        execute_from_ns
                                        + (index + 1) * ACTION_PERIOD_NS,
                                        target,
                                    )
                                    for index, target in enumerate(
                                        targets[:REPLAN_STEPS]
                                    )
                                ),
                                current_pose_6d=snapshot.eef_pose_6d,
                                workspace=self.teleoperation_config.workspace,
                            )
                        except ActionRejected:
                            pass
                        else:
                            queue = candidate
                if not inference_pending and not queue:
                    observation_ns = time.monotonic_ns()
                    worker.submit(
                        build_observation(
                            self.cameras.latest(), snapshot, task, self.video_keys, self.state_keys
                        ),
                        observation_ns,
                        None,
                    )
                    inference_pending = True
                now = time.monotonic()
                if now >= next_action_at:
                    action_deadline_ns = time.monotonic_ns() + ACTION_PERIOD_NS // 2
                    due_target: ActionTarget | None = None
                    while queue and queue[0].execute_at_ns <= action_deadline_ns:
                        due_target = queue.popleft().target
                    if due_target is not None:
                        last_target = due_target
                        gripper_state = last_target.gripper_state
                    generation = servo.update_target(
                        to_servo_target(last_target, start.eef_pose_6d)
                    )
                    accepted = servo.wait_for_target(generation, timeout_s=0.08)
                    if accepted.error:
                        raise RuntimeFault(accepted.error)
                    if accepted.target_error:
                        last_target = last_servo_target
                    elif accepted.processed_generation >= generation:
                        last_servo_target = last_target
                    next_action_at = max(
                        next_action_at + 1.0 / EXECUTION_HZ,
                        now + 0.001,
                    )
                time.sleep(0.005)
        except BaseException as exc:
            reason = str(exc)
            raise
        finally:
            home_error: RuntimeFault | None = None
            try:
                if worker is not None:
                    worker.close()
            finally:
                try:
                    if servo is not None:
                        servo.stop(reason, emergency=False)
                finally:
                    if self._return_home_required:
                        try:
                            self.return_home()
                        except RuntimeFault as exc:
                            home_error = exc
                            self.print(f"动作结束后自动回零失败：{exc}")
                    if self.controller.mode == ControllerMode.ACTIVE:
                        self.controller.damping()
            if home_error is not None and normal_stop:
                raise home_error

    def preview_home(self) -> tuple[MotionPlan, float]:
        self.connect()
        home = load_collection_home(interface="can0")
        if home is None:
            raise RuntimeFault("断开安全零位不存在或无效")
        snapshot = self.controller.get_state()
        plan = self.controller.plan_movej(
            np.asarray(home["joint_position_rad"], dtype=np.float64),
            velocity_scale=HOME_VELOCITY_SCALE,
            allow_large_motion=True,
            start_snapshot=snapshot,
        )
        return plan, float(home["gripper_position_m"])

    def preview_training_start(self) -> tuple[MotionPlan, float]:
        self.connect()
        training_start = load_training_start(interface="can0")
        if training_start is None:
            raise RuntimeFault("模型训练起点不存在或无效")
        snapshot = self.controller.get_state()
        plan = self.controller.plan_movej(
            np.asarray(training_start["joint_position_rad"], dtype=np.float64),
            velocity_scale=TRAINING_START_VELOCITY_SCALE,
            allow_large_motion=True,
            start_snapshot=snapshot,
        )
        return plan, float(training_start["gripper_position_m"])

    def execute_training_start(
        self,
        plan: MotionPlan,
        gripper_position_m: float,
    ) -> None:
        self.execute_home(
            plan,
            gripper_position_m,
            keep_active=True,
            velocity_scale=TRAINING_START_VELOCITY_SCALE,
        )
        self._return_home_required = True

    def execute_home(
        self,
        plan: MotionPlan,
        gripper_position_m: float,
        *,
        keep_active: bool = False,
        velocity_scale: float = HOME_VELOCITY_SCALE,
    ) -> None:
        self._return_home_required = True
        self.controller.activate()
        try:
            for attempt in range(3):
                snapshot = self.controller.get_state()
                active_plan = self.controller.plan_movej(
                    plan.target_joint_position,
                    velocity_scale=velocity_scale,
                    allow_large_motion=True,
                    start_snapshot=snapshot,
                )
                try:
                    self.controller.execute_motion_plan(active_plan)
                except SafetyError as exc:
                    if (
                        str(exc) == "joint position changed after planning; plan again"
                        and attempt < 2
                    ):
                        continue
                    raise
                break
            current = self.controller.get_state().gripper_position
            gripper_delta = gripper_position_m - current
            if (
                abs(gripper_delta)
                > self.controller.safety.gripper_position_tolerance_m
            ):
                duration = max(
                    self.controller.safety.minimum_motion_duration_s,
                    self.controller.minimum_gripper_duration(gripper_delta) + 0.05,
                )
                self.controller.move_gripper(gripper_position_m, duration)
            self._return_home_required = False
        except (ControlError, SafetyError, ValueError) as exc:
            raise RuntimeFault(f"自动回零失败：{exc}") from exc
        finally:
            if not keep_active and self.controller.mode == ControllerMode.ACTIVE:
                self.controller.damping()

    def return_home(self) -> None:
        if self.controller.mode in {
            ControllerMode.DISCONNECTED,
            ControllerMode.CLOSED,
            ControllerMode.FAULT,
        }:
            raise RuntimeFault(
                f"控制器处于 {self.controller.mode.value}，无法自动回零"
            )
        try:
            plan, gripper = self.preview_home()
        except (ControlError, SafetyError, ValueError) as exc:
            raise RuntimeFault(f"自动回零规划失败：{exc}") from exc
        self.print(
            "动作已停止，正在自动回零："
            f"预计 {plan.duration_s:.1f} s，"
            f"最大关节变化 {math.degrees(plan.max_joint_delta_rad):.1f} deg"
        )
        self.execute_home(plan, gripper)
        self._return_home_required = False
        self.print("已回到原机械零位并进入阻尼。")

    def close(self) -> None:
        if self._connected:
            try:
                if self._return_home_required:
                    try:
                        self.return_home()
                    except RuntimeFault as exc:
                        self.print(f"断开前自动回零失败：{exc}")
            finally:
                self.controller.close(emergency=False)
                self._connected = False
