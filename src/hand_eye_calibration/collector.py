"""Manual, safety-oriented eye-to-hand sample collection."""

from __future__ import annotations

import base64
from collections import deque
from dataclasses import replace
import datetime as dt
import json
import math
import os
from pathlib import Path
import queue
import select
import sys
import threading
import time
from collections.abc import Callable
from typing import Any

import numpy as np

from arx5_control import (
    ArmController,
    ControllerMode,
    MotionSettleError,
    Pose,
    SafetyError,
)

from hand_eye_calibration.core.camera import RealSenseCamera
from hand_eye_calibration.core.schema import CameraIntrinsics, EyeToHandSample
from hand_eye_calibration.core.solver import (
    CalibrationQualityError,
    analyze_motion_diversity,
    analyze_transform_motion_diversity,
    validate_motion_diversity,
)
from hand_eye_calibration.core.storage import (
    CalibrationSession,
    phase_cache_filename,
    read_cached_result,
    write_cached_result,
)
from hand_eye_calibration.core.targets import TargetDetection, build_target_detector
from hand_eye_calibration.core.transforms import (
    average_transforms,
    pose6_to_transform,
    quaternion_xyzw_from_matrix,
    rotation_distance_rad,
    rotation_vector_from_matrix,
    transform_error,
)
from hand_eye_calibration.eye_in_hand import evaluate_eye_in_hand_validation, solve_eye_in_hand
from hand_eye_calibration.eye_to_hand import evaluate_eye_to_hand_validation, solve_eye_to_hand


def _evaluate_independent_validation(
    samples: list[HandEyeSample],
    result: dict[str, Any],
) -> dict[str, float | int]:
    """Dispatch held-out metrics through the matching camera adapter."""
    calibration_type = result.get("calibration_type")
    if calibration_type == "eye_in_hand":
        return evaluate_eye_in_hand_validation(samples, result)
    if calibration_type == "eye_to_hand":
        return evaluate_eye_to_hand_validation(samples, result)
    raise ValueError(
        f"unsupported calibration type for validation: {calibration_type!r}"
    )


class HandEyeCancelled(Exception):
    """Requested cooperative cancellation of the automatic workflow."""


class HandEyeReturnHome(HandEyeCancelled):
    """Cancel calibration, remove the target if needed, then return to zero."""


class HandEyeEmergency(HandEyeCancelled):
    """Stop calibration without issuing any further arm motion."""


class AutomaticPosePlanError(RuntimeError):
    """The taught center cannot produce enough safe automatic poses."""


class ManualEyeToHandCollector:
    def __init__(
        self,
        config: dict[str, Any],
        session: CalibrationSession,
        *,
        controller: ArmController | None = None,
        commands: queue.Queue[str] | None = None,
        on_stage: Callable[[str], None] | None = None,
        on_event: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.config = config
        self.session = session
        # Sampling and browser guidance must never serialize on one detector
        # lock.  Both see frames from the same source, but the sampling
        # detector always has priority-free access to its high-resolution copy.
        self.detector = build_target_detector(config["target"])
        self._preview_detector = build_target_detector(config["target"])
        camera_config = config["camera"]
        self.external_camera = RealSenseCamera(
            width=int(camera_config["width"]),
            height=int(camera_config["height"]),
            fps=int(camera_config["fps"]),
            serial=camera_config.get("serial"),
            warmup_frames=int(camera_config.get("warmup_frames", 30)),
            name="外部 D455",
        )
        hand_camera_config = config.get("hand_camera", {})
        self.hand_camera = RealSenseCamera(
            width=int(hand_camera_config.get("width", 424)),
            height=int(hand_camera_config.get("height", 240)),
            fps=int(hand_camera_config.get("fps", 15)),
            serial=hand_camera_config.get("serial"),
            warmup_frames=int(hand_camera_config.get("warmup_frames", 30)),
            name="眼在手 D405",
        )
        self.camera = self.external_camera
        robot = config["robot"]
        self._owns_controller = controller is None
        self.controller = controller or ArmController(
            model=str(robot.get("model", "X5")),
            interface=str(robot.get("interface", "can0")),
            gripper_gain_scale=float(robot.get("gripper_gain_scale", 1.0)),
            gripper_open_readout=float(robot.get("gripper_open_readout", -3.4)),
        )
        self._commands = commands
        self._on_stage = on_stage
        self._on_event = on_event
        self.intrinsics: CameraIntrinsics | None = None
        self.samples: list[EyeToHandSample] = []
        self.datasets: dict[str, list[EyeToHandSample]] = {
            "eye_in_hand": [],
            "eye_to_hand": [],
        }
        # Kept separate from the solver datasets.  These samples are only
        # collected after a provisional result exists and must never leak into
        # the fit they are evaluating.
        self.validation_datasets: dict[str, list[EyeToHandSample]] = {
            "eye_in_hand": [],
            "eye_to_hand": [],
        }
        self.operator_reviews: dict[str, dict[str, Any]] = {}
        self.camera_intrinsics: dict[str, CameraIntrinsics] = {}
        self._sample_role = "eye_to_hand"
        self._sample_image_prefix = ""
        self._camera_label = "D455"
        self._dual_mode = False
        self._board_clamped = False
        self._feedback_stop = threading.Event()
        self._feedback_thread: threading.Thread | None = None
        self._camera_stream_stop = threading.Event()
        self._camera_stream_thread: threading.Thread | None = None
        self._camera_frame_condition = threading.Condition()
        self._camera_frame: np.ndarray | None = None
        self._camera_frame_sequence = 0
        self._camera_stream_error: BaseException | None = None
        self._automatic_preview_context = "自动标定准备中"
        self._last_preview_emit_at = 0.0
        preview = config.get("web_preview", {})
        self._web_preview_width = int(preview.get("width", 640))
        self._web_preview_fps = float(preview.get("fps", 30.0))
        self._web_preview_jpeg_quality = int(preview.get("jpeg_quality", 55))
        if self._web_preview_width < 160:
            raise ValueError("手眼标定网页预览宽度至少为 160")
        if not math.isfinite(self._web_preview_fps) or not 0.2 <= self._web_preview_fps <= 30.0:
            raise ValueError("手眼标定网页预览 FPS 必须在 0.2..30")
        if not 20 <= self._web_preview_jpeg_quality <= 95:
            raise ValueError("手眼标定网页预览 JPEG 质量必须在 20..95")

    def run(self) -> dict[str, Any] | None:
        cv2 = _cv2()
        window = "ARX5 D455 eye-to-hand calibration"
        try:
            self.intrinsics = self.camera.start()
            self.controller.connect()
            self.session.initialize(
                config=self.config,
                intrinsics=self.intrinsics,
            )
            self.samples, _ = self.session.read_samples()
            print(
                "D455 与机械臂已连接；当前为阻尼模式。"
                "手动改变姿态后松手，"
                "按 Space/S 采样。"
            )
            print("按 D 删除最后一组，Q/Esc 结束并求解。")
            while True:
                frame = self.camera.read()
                detection = self.detector.detect(frame, self.intrinsics)
                display = frame if detection is None else detection.annotated
                self._draw_status(display, detection)
                cv2.imshow(window, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("s"), 32):
                    sample = self.capture_sample()
                    if sample is not None:
                        self.samples.append(sample)
                        self._persist_samples()
                        print(
                            f"已采集 {sample.index}: reprojection="
                            f"{sample.reprojection_error_px:.3f}px, "
                            f"stable={sample.translation_std_m * 1000.0:.2f}mm/"
                            f"{sample.rotation_std_deg:.3f}deg"
                        )
                elif key == ord("d"):
                    if self.samples:
                        removed = self.samples.pop()
                        self._persist_samples()
                        print(f"已删除样本 {removed.index}。")
                elif key in (ord("q"), 27):
                    break
        finally:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            self.camera.stop()
            self.controller.close(emergency=False)

        minimum = int(self.config["quality"].get("min_samples", 10))
        if len(self.samples) < minimum:
            print(
                f"当前只有 {len(self.samples)} 组样本，至少需要 {minimum} 组；"
                f"数据已保存到 {self.session.path}。"
            )
            return None
        return solve_and_store_session(self.session)

    def run_automatic(self) -> dict[str, Any] | None:
        """Run two automatic datasets with the target mounted in both ways."""
        self._dual_mode = True
        settings = {
            "open_width_m": 0.012,
            "pregrasp_width_m": 0.007,
            "closing_torque_nm": 0.30,
            "minimum_width_m": 0.002,
            "settle_s": 0.65,
            "speed": 0.30,
            **self.config.get("automatic", {}),
        }
        failed = False
        board_clamped = False
        open_width = float(settings["open_width_m"])
        self._phase_center: np.ndarray | None = None
        previous_safety = self.controller.safety
        motion_settle_s = float(settings.get("motion_settle_s", 2.0))
        if not math.isfinite(motion_settle_s) or motion_settle_s < previous_safety.settle_time_s:
            raise RuntimeError("自动标定末端稳定时间配置无效")
        planning_margin_deg = float(
            settings.get("handeye_joint_limit_margin_deg", 0.10)
        )
        # This is a planning buffer, not a hard-limit override.  The SDK and
        # Cartesian planner still reject targets at or outside a joint limit.
        if not math.isfinite(planning_margin_deg) or not 0.05 <= planning_margin_deg <= 1.0:
            raise RuntimeError("自动标定关节规划余量必须在 0.05..1 度")
        self._reconfigure_controller_safety(
            replace(
                previous_safety,
                settle_time_s=motion_settle_s,
                joint_limit_margin_rad=math.radians(planning_margin_deg),
            )
        )
        try:
            if os.environ.get("ARX5_HANDEYE_WAIT_FOR_TAKEOVER") == "1":
                print("HANDEYE_STAGE takeover_ready", flush=True)
                self._require_stage_input("TAKEOVER")
            if self._owns_controller:
                self.controller.connect(stabilize=True)
            else:
                self.controller.hold_current()
            self._start_feedback_stream()
            self._load_dual_session_data()

            # The D405 is mounted on the gripper: fully open fingers before
            # free-damping teaching so neither finger occludes its target view.
            d405_open_width = float(self.controller.get_limits().gripper_width_m)
            self._open_gripper(d405_open_width)
            self._run_dual_phase(
                role="eye_in_hand",
                settings=settings,
                guided_confirmation=False,
            )
            self._require_phase_quality("eye_in_hand", settings)

            open_width = self._prepare_gripper_target(settings)
            board_clamped = True
            self._run_dual_phase(
                role="eye_to_hand",
                settings=settings,
                guided_confirmation=True,
            )

            self._require_phase_quality("eye_to_hand", settings)
            print("HANDEYE_STAGE remove", flush=True)
            self._require_stage_input("REMOVE")
            self._open_gripper(open_width)
            board_clamped = False
            self._return_to_reference(settings)
            self._idle_after_motion()
        except HandEyeEmergency:
            failed = True
            self.controller.emergency_damping("hand-eye emergency stop")
            return None
        except HandEyeReturnHome:
            try:
                board_clamped = self._cancel_dual_to_home(
                    settings,
                    board_clamped=self._board_clamped,
                    open_width_m=open_width,
                )
                return None
            except BaseException:
                failed = True
                raise
        except SafetyError as exc:
            # A missed terminal tolerance must not terminate the process and
            # release CAN. Keep active hold until the operator explicitly
            # cancels; cancellation follows the existing safe path.
            self.controller.hold_current()
            self._automatic_preview_context = (
                "轨迹到位恢复失败；机械臂正保持当前位置，可取消或急停"
            )
            self._notify_stage("motion_hold")
            print("HANDEYE_STAGE motion_hold", flush=True)
            print(f"HANDEYE_MOTION_HOLD {exc}", flush=True)
            while True:
                try:
                    self._require_stage_command()
                except HandEyeEmergency:
                    failed = True
                    self.controller.emergency_damping("hand-eye emergency stop")
                    return None
                except HandEyeReturnHome:
                    board_clamped = self._cancel_dual_to_home(
                        settings,
                        board_clamped=self._board_clamped,
                        open_width_m=open_width,
                    )
                    return None
                except HandEyeCancelled:
                    board_clamped = self._cancel_dual_to_center(
                        settings,
                        board_clamped=self._board_clamped,
                        open_width_m=open_width,
                    )
                    return None
                print("HANDEYE_MOTION_HOLD 请发送 CANCEL 后安全退出", flush=True)
        except HandEyeCancelled:
            try:
                board_clamped = self._cancel_dual_to_center(
                    settings,
                    board_clamped=self._board_clamped,
                    open_width_m=open_width,
                )
                return None
            except BaseException:
                failed = True
                raise
        except BaseException:
            failed = True
            raise
        finally:
            self._stop_feedback_stream()
            self._stop_automatic_camera_stream()
            self.hand_camera.stop()
            self.external_camera.stop()
            self._reconfigure_controller_safety(previous_safety)
            if self._owns_controller:
                self.controller.close(emergency=failed)
            elif not failed:
                self.controller.hold_current()
            if board_clamped or self._board_clamped:
                print(
                    "标定中断时夹爪位置保持已释放；取板前请先托住标定板。",
                    flush=True,
                )

        return solve_and_store_dual_session(self.session)

    def run_automatic_phase(self, role: str) -> dict[str, Any] | None:
        """Calibrate one camera role while retaining the other role's cache.

        This is deliberately serial physical control, not parallel collection:
        D405 is taught against the table, while D455 first clamps the board in
        the gripper.  Either result can be saved on its own and the active
        dual profile is refreshed as soon as both phase results are present.
        """
        if role not in {"eye_in_hand", "eye_to_hand"}:
            raise ValueError(f"unknown hand-eye camera role: {role}")
        self._dual_mode = True
        settings = {
            "open_width_m": 0.012,
            "pregrasp_width_m": 0.007,
            "closing_torque_nm": 0.30,
            "minimum_width_m": 0.002,
            "settle_s": 0.65,
            "speed": 0.30,
            **self.config.get("automatic", {}),
        }
        failed = False
        board_clamped = False
        open_width = float(settings["open_width_m"])
        self._phase_center = None
        previous_safety = self.controller.safety
        motion_settle_s = float(settings.get("motion_settle_s", 2.0))
        if (
            not math.isfinite(motion_settle_s)
            or motion_settle_s < previous_safety.settle_time_s
        ):
            raise RuntimeError("自动标定末端稳定时间配置无效")
        planning_margin_deg = float(
            settings.get("handeye_joint_limit_margin_deg", 0.10)
        )
        if (
            not math.isfinite(planning_margin_deg)
            or not 0.05 <= planning_margin_deg <= 1.0
        ):
            raise RuntimeError("自动标定关节规划余量必须在 0.05..1 度")
        self._reconfigure_controller_safety(
            replace(
                previous_safety,
                settle_time_s=motion_settle_s,
                joint_limit_margin_rad=math.radians(planning_margin_deg),
            )
        )
        try:
            if os.environ.get("ARX5_HANDEYE_WAIT_FOR_TAKEOVER") == "1":
                print("HANDEYE_STAGE takeover_ready", flush=True)
                self._require_stage_input("TAKEOVER")
            if self._owns_controller:
                self.controller.connect(stabilize=True)
            else:
                self.controller.hold_current()
            self._start_feedback_stream()
            self._load_dual_session_data()

            if role == "eye_in_hand":
                # Fingers must be open before the user teaches a D405 view,
                # otherwise they can occlude the table target.
                self._open_gripper(float(self.controller.get_limits().gripper_width_m))
                self._run_dual_phase(
                    role=role,
                    settings=settings,
                    guided_confirmation=False,
                )
            else:
                # D455 does not depend on a D405 solve: clamp the board and
                # enter its own safe-center/trajectory workflow directly.
                open_width = self._prepare_gripper_target(settings)
                board_clamped = True
                self._run_dual_phase(
                    role=role,
                    settings=settings,
                    guided_confirmation=True,
                )

            self._require_phase_quality(role, settings)
            if role == "eye_to_hand":
                print("HANDEYE_STAGE remove", flush=True)
                self._require_stage_input("REMOVE")
                self._open_gripper(open_width)
                board_clamped = False
            self._return_to_reference(settings)
            self._idle_after_motion()
        except HandEyeEmergency:
            failed = True
            self.controller.emergency_damping("hand-eye emergency stop")
            return None
        except HandEyeReturnHome:
            try:
                self._cancel_dual_to_home(
                    settings,
                    board_clamped=self._board_clamped,
                    open_width_m=open_width,
                )
                return None
            except BaseException:
                failed = True
                raise
        except SafetyError as exc:
            self.controller.hold_current()
            self._automatic_preview_context = (
                "轨迹到位恢复失败；机械臂正保持当前位置，可取消或急停"
            )
            self._notify_stage("motion_hold")
            print("HANDEYE_STAGE motion_hold", flush=True)
            print(f"HANDEYE_MOTION_HOLD {exc}", flush=True)
            while True:
                try:
                    self._require_stage_command()
                except HandEyeEmergency:
                    failed = True
                    self.controller.emergency_damping("hand-eye emergency stop")
                    return None
                except HandEyeReturnHome:
                    self._cancel_dual_to_home(
                        settings,
                        board_clamped=self._board_clamped,
                        open_width_m=open_width,
                    )
                    return None
                except HandEyeCancelled:
                    self._cancel_dual_to_center(
                        settings,
                        board_clamped=self._board_clamped,
                        open_width_m=open_width,
                    )
                    return None
                print("HANDEYE_MOTION_HOLD 请发送 CANCEL 后安全退出", flush=True)
        except HandEyeCancelled:
            try:
                self._cancel_dual_to_center(
                    settings,
                    board_clamped=self._board_clamped,
                    open_width_m=open_width,
                )
                return None
            except BaseException:
                failed = True
                raise
        except BaseException:
            failed = True
            raise
        finally:
            self._stop_feedback_stream()
            self._stop_automatic_camera_stream()
            self.hand_camera.stop()
            self.external_camera.stop()
            self._reconfigure_controller_safety(previous_safety)
            if self._owns_controller:
                self.controller.close(emergency=failed)
            elif not failed:
                self.controller.hold_current()
            if board_clamped or self._board_clamped:
                print(
                    "标定中断时夹爪位置保持已释放；取板前请先托住标定板。",
                    flush=True,
                )

        return solve_and_store_dual_phase_session(self.session, role=role)

    def _load_dual_session_data(self) -> None:
        """Load a resumable dual schema without requiring both phases to run."""
        self.session.initialize_dual(config=self.config)
        self.datasets, payload = self.session.read_dual_samples()
        validation_payload = payload.get("validation_datasets", {})
        if isinstance(validation_payload, dict):
            self.validation_datasets = {
                role: [
                    EyeToHandSample.from_dict(value)
                    for value in validation_payload.get(role, [])
                ]
                for role in ("eye_in_hand", "eye_to_hand")
            }
        review_payload = payload.get("operator_reviews", {})
        if isinstance(review_payload, dict):
            self.operator_reviews = {
                role: dict(value)
                for role, value in review_payload.items()
                if role in {"eye_in_hand", "eye_to_hand"}
                and isinstance(value, dict)
            }
        for role, camera in payload.get("cameras", {}).items():
            if role in self.datasets and isinstance(camera, dict):
                self.camera_intrinsics[role] = CameraIntrinsics.from_dict(camera)

    def _run_dual_phase(
        self,
        *,
        role: str,
        settings: dict[str, Any],
        guided_confirmation: bool,
    ) -> None:
        self._select_camera(role)
        self._phase_center = None
        self.intrinsics = self.camera.start()
        self.camera_intrinsics[role] = self.intrinsics
        # A new automatic phase may append training samples to a resumable
        # session; its held-out set must never be reused against that changed
        # fit.
        self.validation_datasets[role] = []
        self.operator_reviews.pop(role, None)
        self._persist_samples()
        self._automatic_preview_context = (
            "D405 桌面固定板 · 检查起始视野"
            if role == "eye_in_hand"
            else "D455 夹爪固定板 · 调整安全中心"
        )
        stage = "table_check" if role == "eye_in_hand" else "center"
        print(f"HANDEYE_STAGE {stage}", flush=True)
        if role == "eye_in_hand":
            # Table board phase is taught by hand. Release arm stiffness only
            # here; stable detection immediately transitions to hold_current.
            self.controller.damping()
        else:
            self.controller.teach_with_gripper_hold()
        while True:
            self._wait_for_guided_center(
                expected_command="CENTER" if guided_confirmation else "READY",
                timeout_s=(
                    None
                    if guided_confirmation
                    else settings.get("initial_target_timeout_s")
                ),
            )

            self._start_automatic_camera_stream()
            self.controller.hold_current()
            if role == "eye_to_hand":
                self.controller.activate_gripper()
            center = self.controller.get_state().joint_position.copy()
            self._phase_center = center.copy()
            try:
                plan, validation_plan = self._phase_pose_plans(
                    center,
                    settings,
                    role=role,
                )
            except AutomaticPosePlanError as exc:
                recovered = self._search_adaptive_center(
                    center,
                    settings,
                    role=role,
                    initial_error=exc,
                )
                if recovered is not None:
                    center, plan, validation_plan = recovered
                    self._phase_center = center.copy()
                    print(
                        "HANDEYE_CENTER_ADJUSTED "
                        f"{role} 自动选定新的安全采样中心",
                        flush=True,
                    )
                    break
                self._stop_automatic_camera_stream()
                if not guided_confirmation:
                    # All automatically searched, camera-visible centers were
                    # exhausted. Only now return the D405 phase to teaching.
                    self._phase_center = None
                    self.controller.damping()
                    print(
                        f"HANDEYE_CENTER_REJECT {exc}；"
                        "已自动搜索安全工作区仍无可用中心，"
                        "眼在手保持自由阻尼，请调整后再次确认",
                        flush=True,
                    )
                    continue
                self._phase_center = None
                self.controller.teach_with_gripper_hold()
                print(
                    f"HANDEYE_CENTER_REJECT {exc}；"
                    "机械臂保持自由阻尼，请调整后再次确认",
                    flush=True,
                )
                continue
            break
        preview = {
            "role": role,
            "total": max(len(plan), int(settings.get("max_pose_attempts", len(plan)))),
            "waypoints": [
                {
                    "index": index,
                    "label": label,
                    "joint_position_rad": target.tolist(),
                }
                for index, (label, target) in enumerate(plan, start=1)
            ],
            "validation_waypoints": [
                {
                    "index": index,
                    "label": label,
                    "joint_position_rad": target.tolist(),
                }
                for index, (label, target) in enumerate(validation_plan, start=1)
            ],
        }
        self._emit_event({"type": "plan", "plan": preview})
        self._notify_stage("plan")
        self._require_stage_input("RUN")
        print(
            f"HANDEYE_STAGE phase_run {role} "
            f"{max(len(plan), int(settings.get('max_pose_attempts', len(plan))))}",
            flush=True,
        )
        try:
            self._collect_automatic_plan(
                role=role,
                center=center,
                plan=plan,
                validation_plan=validation_plan,
                settings=settings,
            )
        except RuntimeError as exc:
            # Automatic attempts are intentionally bounded. Reaching that
            # bound is not a terminal calibration error: switch to the
            # existing shared-controller teaching path so the operator can
            # add informative poses instead of restarting both cameras.
            if "已达到自动采样上限" not in str(exc):
                raise
            self._run_dual_manual_supplement(
                role=role,
                settings=settings,
                reason=str(exc),
            )
        self._review_phase(
            role=role,
            center=center,
            plan=plan,
            validation_plan=validation_plan,
            settings=settings,
        )
        self._stop_automatic_camera_stream()
        self._idle_after_motion()
        self.camera.stop()

    def _collect_automatic_plan(
        self,
        *,
        role: str,
        center: np.ndarray,
        plan: list[tuple[str, np.ndarray]],
        validation_plan: list[tuple[str, np.ndarray]],
        settings: dict[str, Any],
        target_samples: int | None = None,
    ) -> None:
        minimum_samples = int(self.config["quality"].get("min_samples", 10))
        target_samples = max(
            minimum_samples,
            int(
                settings.get("target_valid_samples", minimum_samples + 3)
                if target_samples is None
                else target_samples
            ),
        )
        maximum_attempts = max(
            len(plan), int(settings.get("max_pose_attempts", len(plan)))
        )
        check_interval = max(1, int(settings.get("quality_check_interval", 3)))
        current = center.copy()
        last_quality = "样本数量不足，尚未开始质量估计"
        last_quality_sample_count = -1
        phase_started_at = time.monotonic()
        initial_valid_samples = len(self.samples)
        self._emit_collection_progress(
            role=role,
            stage="collecting",
            attempted=0,
            attempt_limit=maximum_attempts,
            valid=len(self.samples),
            target=target_samples,
            initial_valid=initial_valid_samples,
            started_at=phase_started_at,
        )
        for attempt_index in range(1, maximum_attempts + 1):
            self._raise_if_cancel_requested()
            # Review-stage supplementation continues through unused safe
            # endpoints instead of restarting at the center/first rotations.
            # A persisted session gets the same behaviour after resuming.
            plan_index = (initial_valid_samples + attempt_index - 1) % len(plan)
            label, target = plan[plan_index]
            self._automatic_preview_context = (
                f"{self._camera_label} 姿态 {attempt_index}/{maximum_attempts} · 正在运动"
            )
            if not np.allclose(current, target, atol=1e-10, rtol=0.0):
                _movej_for_duration(
                    self.controller,
                    target,
                    _automatic_motion_duration(
                        self.controller,
                        target - current,
                        float(settings["speed"]),
                    ),
                    allow_large_motion=True,
                )
            current = target.copy()
            self._raise_if_cancel_requested()
            self._automatic_preview_context = (
                f"{self._camera_label} 姿态 {attempt_index}/{maximum_attempts} · 等待稳定"
            )
            self._automatic_preview_context = (
                f"{self._camera_label} 姿态 {attempt_index}/{maximum_attempts} · 正在采样"
            )
            sample = self.capture_sample(keep_active=True)
            if sample is not None:
                self.samples.append(sample)
                self._persist_samples()
                print(
                    f"已采集 {role} {sample.index}/{maximum_attempts}：{label}，"
                    f"reprojection={sample.reprojection_error_px:.3f}px",
                    flush=True,
                )
            print(
                f"HANDEYE_DUAL_POSE {role} {attempt_index}/{maximum_attempts} "
                f"valid={len(self.samples)} target={target_samples} label={label}",
                flush=True,
            )
            self._emit_collection_progress(
                role=role,
                stage="collecting",
                attempted=attempt_index,
                attempt_limit=maximum_attempts,
                valid=len(self.samples),
                target=target_samples,
                initial_valid=initial_valid_samples,
                started_at=phase_started_at,
            )
            self._raise_if_cancel_requested()
            valid_count = len(self.samples)
            should_check_quality = valid_count >= minimum_samples and (
                last_quality_sample_count < minimum_samples
                or (
                    valid_count > last_quality_sample_count
                    and attempt_index % check_interval == 0
                )
            )
            if should_check_quality:
                report = self._phase_quality_report(role, settings)
                last_quality = str(report["detail"])
                last_quality_sample_count = valid_count
                self._emit_event({"type": "quality", "quality": report})
                print(
                    f"HANDEYE_QUALITY {role} warning={int(not report['accepted'])} {last_quality}",
                    flush=True,
                )
            if valid_count >= target_samples:
                print(
                    f"HANDEYE_TARGET_REACHED {role} valid={valid_count} "
                    f"target={target_samples}；转入人工评审",
                    flush=True,
                )
                return
            if not np.allclose(current, center, atol=1e-10, rtol=0.0):
                self._automatic_preview_context = (
                    f"{self._camera_label} 姿态 {attempt_index}/{maximum_attempts} · 返回起点"
                )
                _movej_for_duration(
                    self.controller,
                    center,
                    _automatic_motion_duration(
                        self.controller,
                        center - current,
                        float(settings["speed"]),
                    ),
                    allow_large_motion=True,
                )
                current = center.copy()
                time.sleep(0.25)
        raise RuntimeError(
            f"{self._camera_label} 已达到自动采样上限 {maximum_attempts}，"
            f"但精度未达标：{last_quality}"
        )

    def _adaptive_review_pose_plan(
        self,
        *,
        role: str,
        plan: list[tuple[str, np.ndarray]],
        count: int,
    ) -> list[tuple[str, np.ndarray]]:
        """Choose supplementary poses from the current fit's weak 6-D axes.

        Bootstrap resampling can report a large P95 simply because it omits a
        high-leverage pose.  Do not remove that pose.  Instead, add safe,
        *unvisited* endpoints that both improve the weakest singular direction
        and are far from the already measured EEF poses.  The target detector
        still decides whether each endpoint yields a valid observation.
        """
        requested = max(1, int(count))
        existing = [
            np.asarray(sample.T_base_eef, dtype=np.float64)
            for sample in self.samples
        ]
        existing_joints = [
            np.asarray(sample.joint_position_rad, dtype=np.float64)
            for sample in self.samples
        ]
        if not existing:
            return plan[:requested]
        visited_tolerance_rad = math.radians(1.5)
        candidates: list[tuple[str, np.ndarray, np.ndarray]] = []
        for label, raw_target in plan:
            target = np.asarray(raw_target, dtype=np.float64)
            if target.shape != (6,) or not np.all(np.isfinite(target)):
                continue
            if any(
                np.max(np.abs(target - observed)) <= visited_tolerance_rad
                for observed in existing_joints
            ):
                continue
            try:
                transform = pose6_to_transform(
                    self.controller.forward_kinematics(target)
                )
            except (RuntimeError, OSError, ValueError, SafetyError):
                continue
            candidates.append((label, target.copy(), transform))
        if not candidates:
            raise RuntimeError(
                "没有可用于补采的未访问安全姿态；请重新生成标定计划或手动补采"
            )

        quality = self.config["quality"]
        translation_scale = float(quality.get("min_translation_span_m", 0.04))
        rotation_scale = float(quality.get("min_rotation_span_deg", 20.0))
        selected_transforms = list(existing)
        selected: list[tuple[str, np.ndarray]] = []
        selected_novelty: list[float] = []
        selected_amplitude: list[float] = []
        while candidates and len(selected) < requested:
            def score(candidate: tuple[str, np.ndarray, np.ndarray]) -> tuple[
                int, float, float, float, float, int, float, float
            ]:
                transforms = [*selected_transforms, candidate[2]]
                rank, weakest, spread = _hybrid_motion_information(
                    transforms,
                    translation_scale_m=translation_scale,
                    rotation_scale_deg=rotation_scale,
                )
                diversity = analyze_transform_motion_diversity(transforms)
                distances = [
                    _normalized_transform_distance(
                        candidate[2],
                        transform,
                        translation_scale_m=translation_scale,
                        rotation_scale_deg=rotation_scale,
                    )
                    for transform in selected_transforms
                ]
                novelty = min(distances) if distances else 0.0
                amplitude = _normalized_transform_distance(
                    candidate[2],
                    existing[0],
                    translation_scale_m=translation_scale,
                    rotation_scale_deg=rotation_scale,
                )
                return (
                    min(rank, 6),
                    # The information quantities are intentionally rounded
                    # before the farthest-point terms.  Once two candidates
                    # are numerically equivalent for observability, prefer a
                    # genuinely new combined translation + rotation rather
                    # than another small perturbation near the center.
                    round(weakest, 3),
                    round(spread, 3),
                    novelty,
                    amplitude,
                    diversity.rotation_axis_rank,
                    diversity.rotation_span_deg,
                    diversity.translation_span_m,
                )

            best_index = max(range(len(candidates)), key=lambda index: score(candidates[index]))
            label, target, transform = candidates.pop(best_index)
            selected_novelty.append(
                min(
                    _normalized_transform_distance(
                        transform,
                        observed,
                        translation_scale_m=translation_scale,
                        rotation_scale_deg=rotation_scale,
                    )
                    for observed in selected_transforms
                )
            )
            selected_amplitude.append(
                _normalized_transform_distance(
                    transform,
                    existing[0],
                    translation_scale_m=translation_scale,
                    rotation_scale_deg=rotation_scale,
                )
            )
            selected.append((label, target))
            selected_transforms.append(transform)

        rank, weakest, spread = _hybrid_motion_information(
            selected_transforms,
            translation_scale_m=translation_scale,
            rotation_scale_deg=rotation_scale,
        )
        print(
            "HANDEYE_REVIEW_PLAN "
            f"{role} requested={requested} selected={len(selected)} "
            f"hybrid_rank={rank} hybrid_min_sv={weakest:.3f} "
            f"hybrid_logdet={spread:.3f} "
            f"min_novelty={min(selected_novelty, default=0.0):.3f} "
            f"mean_amplitude={float(np.mean(selected_amplitude)) if selected_amplitude else 0.0:.3f} "
            f"labels="
            + ",".join(label for label, _ in selected),
            flush=True,
        )
        return selected

    def _promote_review_validation_samples(self, role: str) -> int:
        """Move the reviewed holdout into the next fit exactly once.

        A holdout must never be part of the fit it evaluates.  When the
        operator asks for more data, that review is complete: retaining those
        high-quality observations for the next solve is useful, provided the
        following review is collected at fresh, disjoint poses.
        """
        held_out = list(self.validation_datasets.get(role, []))
        if not held_out:
            return 0
        existing = self.datasets[role]
        tolerance_rad = math.radians(1.5)
        promoted = 0
        for sample in held_out:
            joints = np.asarray(sample.joint_position_rad, dtype=np.float64)
            if any(
                np.max(
                    np.abs(
                        joints
                        - np.asarray(previous.joint_position_rad, dtype=np.float64)
                    )
                )
                <= tolerance_rad
                for previous in existing
            ):
                continue
            existing.append(sample)
            promoted += 1
        self.samples = existing
        self.datasets[role] = existing
        self.validation_datasets[role] = []
        self._persist_samples()
        print(
            f"HANDEYE_REVIEW_PROMOTE {role} promoted={promoted} "
            f"fit={len(existing)}; 下一轮将采集全新独立验证集",
            flush=True,
        )
        return promoted

    def _review_phase(
        self,
        *,
        role: str,
        center: np.ndarray,
        plan: list[tuple[str, np.ndarray]],
        validation_plan: list[tuple[str, np.ndarray]],
        settings: dict[str, Any],
    ) -> None:
        """Require an explicit human decision after 20 valid fit poses.

        Numerical targets remain visible as warnings, while acceptance is a
        deliberate operator decision based primarily on an unseen holdout set.
        """
        extra = max(1, int(settings.get("review_extra_samples", 4)))
        active_validation_plan = validation_plan
        validation_round = 0
        while True:
            _, relocation_detail = self._collect_independent_validation(
                role=role,
                center=center,
                plan=active_validation_plan,
                settings=settings,
            )
            report = self._phase_quality_report(role, settings)
            report["review_required"] = True
            report["relocalization_detail"] = relocation_detail
            self._emit_event({"type": "quality", "quality": report})
            self._automatic_preview_context = (
                "已完成有效姿态与独立重定位；请在网页评审采用或补采"
            )
            self.controller.hold_current()
            self._notify_stage("review")
            print(
                "HANDEYE_REVIEW "
                f"{role} fit={len(self.samples)} "
                f"validation={len(self.validation_datasets[role])} "
                f"{relocation_detail}",
                flush=True,
            )
            command = self._require_stage_command()
            if command == "ACCEPT":
                self.operator_reviews[role] = {
                    "accepted": True,
                    "accepted_at": time.time(),
                    "fit_sample_count": len(self.samples),
                    "validation_sample_count": len(self.validation_datasets[role]),
                    "thresholds_passed": bool(report.get("accepted")),
                    "relocalization": report.get("relocalization"),
                }
                self._persist_samples()
                return
            if command != "MORE":
                raise RuntimeError(f"unexpected hand-eye review command: {command}")
            promoted = self._promote_review_validation_samples(role)
            supplement_plan = self._adaptive_review_pose_plan(
                role=role,
                plan=plan,
                count=extra,
            )
            target = len(self.samples) + len(supplement_plan)
            self._automatic_preview_context = (
                f"已纳入 {promoted} 组已评审样本；"
                f"补采 {len(supplement_plan)} 组更大且差异化的姿态"
            )
            self._emit_event(
                {
                    "type": "plan",
                    "plan": {
                        "role": role,
                        "total": len(supplement_plan),
                        "adaptive": True,
                        "waypoints": [
                            {
                                "index": index,
                                "label": label,
                                "joint_position_rad": waypoint.tolist(),
                            }
                            for index, (label, waypoint) in enumerate(
                                supplement_plan, start=1
                            )
                        ],
                        "validation_waypoints": [],
                        "promoted_validation_samples": promoted,
                    },
                }
            )
            self._collect_automatic_plan(
                role=role,
                center=center,
                plan=supplement_plan,
                validation_plan=active_validation_plan,
                settings=settings,
                target_samples=target,
            )
            # The previous independent set now belongs to the fit.  Build a
            # brand-new holdout that excludes every measured fit endpoint (and
            # the just-planned supplement) before reporting another accuracy
            # number.  Reusing the old holdout here would make the next review
            # optimistically biased.
            validation_round += 1
            active_validation_plan = self._independent_validation_pose_plan(
                center,
                settings,
                role=role,
                training_plan=[*plan, *supplement_plan],
                excluded_joint_positions=[
                    np.asarray(sample.joint_position_rad, dtype=np.float64)
                    for sample in self.samples
                ],
                variant=validation_round,
            )

    def _run_dual_manual_supplement(
        self,
        *,
        role: str,
        settings: dict[str, Any],
        reason: str,
    ) -> None:
        """Collect additional high-information samples by guided teaching.

        This is reached only after the bounded automatic phase cannot meet
        its precision target.  It deliberately keeps the same shared
        controller and camera stream alive: no CAN reconnect, no camera
        reopen, and no uncommanded damping transition between samples.
        """
        minimum_samples = int(self.config["quality"].get("min_samples", 10))
        target_samples = max(
            minimum_samples,
            int(settings.get("target_valid_samples", minimum_samples + 3)),
        )
        attempts = 0
        self.controller.teach_with_gripper_hold()
        while True:
            report = self._phase_quality_report(role, settings)
            ready = (
                len(self.samples) >= target_samples
                and self._samples_meet_quality(minimum_samples)
            )
            detail = str(report["detail"])
            self._automatic_preview_context = (
                "手动补采 · 自由阻尼调整到新视角，松手稳定后点击采集"
            )
            self._notify_stage("manual")
            self._emit_event({"type": "quality", "quality": report})
            self._emit_manual_supplement_state(
                role=role,
                attempts=attempts,
                target=target_samples,
                ready=ready,
                detail=detail,
                reason=reason,
            )
            print(
                "HANDEYE_STAGE manual "
                f"role={role} valid={len(self.samples)} target={target_samples} "
                f"attempts={attempts} ready={int(ready)}",
                flush=True,
            )
            command = self._require_stage_command()
            if command == "DONE":
                if not ready:
                    print(
                        "HANDEYE_MANUAL_REJECT "
                        "当前样本尚未达到有效组数或运动分布要求；"
                        "请继续调整到差异更大的安全视角后补采",
                        flush=True,
                    )
                    continue
                self.controller.hold_current()
                if role == "eye_to_hand":
                    self.controller.activate_gripper()
                return
            if command == "HOME":
                self._automatic_preview_context = "手动补采 · 正在回零位"
                self.controller.hold_current()
                if role == "eye_to_hand":
                    self.controller.activate_gripper()
                current = self.controller.get_state().joint_position.copy()
                zero = np.zeros(6, dtype=np.float64)
                if not np.allclose(current, zero, atol=1e-10, rtol=0.0):
                    _movej_for_duration(
                        self.controller,
                        zero,
                        _automatic_motion_duration(
                            self.controller,
                            zero - current,
                            float(settings["speed"]),
                        ),
                        allow_large_motion=True,
                    )
                self.controller.teach_with_gripper_hold()
                continue
            if command != "SAMPLE":
                raise RuntimeError(f"unexpected hand-eye command: {command}")

            attempts += 1
            self._automatic_preview_context = "手动补采 · 正在保持并采样"
            self.controller.hold_current()
            if role == "eye_to_hand":
                self.controller.activate_gripper()
            try:
                sample = self.capture_sample(keep_active=True)
                if sample is not None:
                    self.samples.append(sample)
                    self._persist_samples()
            finally:
                self.controller.teach_with_gripper_hold()
            print(
                "HANDEYE_MANUAL_SAMPLE "
                f"role={role} valid={len(self.samples)} target={target_samples} "
                f"attempts={attempts} accepted={int(sample is not None)}",
                flush=True,
            )

    def _emit_manual_supplement_state(
        self,
        *,
        role: str,
        attempts: int,
        target: int,
        ready: bool,
        detail: str,
        reason: str,
    ) -> None:
        """Synchronize manual-supplement controls and their live counters."""
        valid = len(self.samples)
        self._emit_event(
            {
                "type": "progress",
                "progress": {
                    "phase": role,
                    "stage": "manual",
                    "manual": True,
                    "attempted": max(0, int(attempts)),
                    "attempt_limit": None,
                    "current": max(0, int(attempts)),
                    "total": None,
                    "valid": valid,
                    "target": max(0, int(target)),
                    "elapsed_s": None,
                    "estimated_remaining_s": None,
                    "maximum_remaining_s": None,
                },
            }
        )
        self._emit_event(
            {
                "type": "manual_status",
                "manual": {
                    "ready": bool(ready),
                    "message": (
                        "已进入手动补采。"
                        f"原因：{reason}；当前质量：{detail}"
                    ),
                    "reason": reason,
                },
            }
        )

    def _collect_independent_validation(
        self,
        *,
        role: str,
        center: np.ndarray,
        plan: list[tuple[str, np.ndarray]],
        settings: dict[str, Any],
    ) -> tuple[bool, str]:
        """Collect a disjoint holdout set and evaluate the current solution."""
        required = max(1, int(settings.get("validation_pose_count", 4)))
        maximum_attempts = max(
            required,
            int(settings.get("validation_max_attempts", len(plan))),
        )
        self.validation_datasets[role] = []
        current = self.controller.get_state().joint_position.copy()
        completed = False
        phase_started_at = time.monotonic()
        self._emit_collection_progress(
            role=role,
            stage="validation",
            attempted=0,
            attempt_limit=maximum_attempts,
            valid=0,
            target=required,
            initial_valid=0,
            started_at=phase_started_at,
        )
        try:
            for attempt in range(1, maximum_attempts + 1):
                self._raise_if_cancel_requested()
                label, target = plan[(attempt - 1) % len(plan)]
                self._automatic_preview_context = (
                    f"{self._camera_label} 独立验证 {attempt}/{maximum_attempts} · 正在运动"
                )
                if not np.allclose(current, target, atol=1e-10, rtol=0.0):
                    _movej_for_duration(
                        self.controller,
                        target,
                        _automatic_motion_duration(
                            self.controller,
                            target - current,
                            float(settings["speed"]),
                        ),
                        allow_large_motion=True,
                    )
                current = target.copy()
                self._automatic_preview_context = (
                    f"{self._camera_label} 独立验证 {attempt}/{maximum_attempts} · 正在采样"
                )
                sample = self.capture_sample(
                    keep_active=True,
                    sample_index=(
                        len(self.samples) + len(self.validation_datasets[role]) + 1
                    ),
                )
                if sample is not None:
                    self.validation_datasets[role].append(sample)
                    self._persist_samples()
                    print(
                        f"已采集 {role} 独立验证 "
                        f"{len(self.validation_datasets[role])}/{required}：{label}",
                        flush=True,
                    )
                self._emit_collection_progress(
                    role=role,
                    stage="validation",
                    attempted=attempt,
                    attempt_limit=maximum_attempts,
                    valid=len(self.validation_datasets[role]),
                    target=required,
                    initial_valid=0,
                    started_at=phase_started_at,
                )
                if len(self.validation_datasets[role]) >= required:
                    break
            completed = True
        finally:
            # Return to the taught center even if a frame was rejected, so a
            # failed validation never leaves the arm at an arbitrary holdout.
            # Cancellation and hardware faults are explicitly excluded: their
            # dedicated handlers decide the only safe next motion.
            if completed and not np.allclose(current, center, atol=1e-10, rtol=0.0):
                self._automatic_preview_context = (
                    f"{self._camera_label} 独立验证 · 返回起点"
                )
                _movej_for_duration(
                    self.controller,
                    center,
                    _automatic_motion_duration(
                        self.controller,
                        center - current,
                        float(settings["speed"]),
                    ),
                    allow_large_motion=True,
                )
        return self._independent_validation_meets_target(role, settings)

    def _emit_collection_progress(
        self,
        *,
        role: str,
        stage: str,
        attempted: int,
        attempt_limit: int,
        valid: int,
        target: int,
        initial_valid: int,
        started_at: float,
    ) -> None:
        """Publish one bounded, browser-friendly collection progress update."""
        elapsed_s = max(0.0, time.monotonic() - started_at)
        attempt_count = max(0, int(attempted))
        attempt_cap = max(attempt_count, int(attempt_limit))
        valid_count = max(0, int(valid))
        target_count = max(0, int(target))
        newly_valid = max(0, valid_count - int(initial_valid))
        remaining_attempts = max(0, attempt_cap - attempt_count)
        average_attempt_s = (
            elapsed_s / attempt_count if attempt_count > 0 else None
        )
        maximum_remaining_s = (
            average_attempt_s * remaining_attempts
            if average_attempt_s is not None
            else None
        )
        estimated_remaining_s: float | None = None
        if valid_count >= target_count:
            estimated_remaining_s = 0.0
        elif newly_valid > 0 and average_attempt_s is not None:
            estimated_attempts = (
                (target_count - valid_count) * attempt_count / newly_valid
            )
            # Once observed acceptance would require more than the remaining
            # cap, only the upper-bound time is honest; do not show a falsely
            # optimistic completion estimate.
            if estimated_attempts <= remaining_attempts:
                estimated_remaining_s = average_attempt_s * estimated_attempts
        self._emit_event(
            {
                "type": "progress",
                "progress": {
                    "phase": role,
                    "stage": stage,
                    "attempted": attempt_count,
                    "attempt_limit": attempt_cap,
                    # Compatibility for older clients that only rendered a
                    # current/total position counter.
                    "current": attempt_count,
                    "total": attempt_cap,
                    "valid": valid_count,
                    "target": target_count,
                    "elapsed_s": elapsed_s,
                    "estimated_remaining_s": estimated_remaining_s,
                    "maximum_remaining_s": maximum_remaining_s,
                },
            }
        )

    def _independent_validation_meets_target(
        self,
        role: str,
        settings: dict[str, Any],
    ) -> tuple[bool, str]:
        samples = self.validation_datasets[role]
        required = max(1, int(settings.get("validation_pose_count", 4)))
        if len(samples) < required:
            return False, f"样本 {len(samples)}/{required}"
        try:
            result = (
                solve_eye_in_hand(self.datasets[role], quality=self.config["quality"])
                if role == "eye_in_hand"
                else solve_eye_to_hand(self.datasets[role], quality=self.config["quality"])
            )
            metrics = _evaluate_independent_validation(samples, result)
        except (CalibrationQualityError, RuntimeError, ValueError) as exc:
            return False, str(exc)
        targets = settings.get("validation_targets", {})
        values = {
            "translation_rms_mm": 1000.0 * float(
                metrics["closure_translation_rms_m"]
            ),
            "rotation_rms_deg": float(metrics["closure_rotation_rms_deg"]),
            "translation_p95_mm": 1000.0 * float(
                metrics["closure_translation_p95_m"]
            ),
            "rotation_p95_deg": float(metrics["closure_rotation_p95_deg"]),
        }
        limits = {
            "translation_rms_mm": float(targets.get("translation_rms_mm", 3.0)),
            "rotation_rms_deg": float(targets.get("rotation_rms_deg", 1.0)),
            "translation_p95_mm": float(targets.get("translation_p95_mm", 4.0)),
            "rotation_p95_deg": float(targets.get("rotation_p95_deg", 1.25)),
        }
        passed = all(
            math.isfinite(values[name]) and values[name] <= limits[name]
            for name in limits
        )
        return (
            passed,
            "独立闭环 "
            f"{values['translation_rms_mm']:.2f}mm/"
            f"{values['rotation_rms_deg']:.3f}deg，P95 "
            f"{values['translation_p95_mm']:.2f}mm/"
            f"{values['rotation_p95_deg']:.3f}deg",
        )

    def _select_camera(self, role: str) -> None:
        if role not in self.datasets:
            raise ValueError(f"unknown hand-eye camera role: {role}")
        self._sample_role = role
        self.samples = self.datasets[role]
        if role == "eye_in_hand":
            self.camera = self.hand_camera
            self._sample_image_prefix = "d405_"
            self._camera_label = "眼在手 D405"
        else:
            self.camera = self.external_camera
            self._sample_image_prefix = "d455_"
            self._camera_label = "外部 D455"

    def _phase_solution_meets_target(
        self,
        role: str,
        settings: dict[str, Any],
    ) -> tuple[bool, str]:
        """Solve current phase and judge empirical residual and stability."""
        report = self._phase_quality_report(role, settings)
        return bool(report["accepted"]), str(report["detail"])

    def _phase_quality_report(
        self,
        role: str,
        settings: dict[str, Any],
    ) -> dict[str, Any]:
        """Return browser-safe current error/target comparisons for one phase."""
        samples = self.datasets[role]
        minimum = int(self.config["quality"].get("min_samples", 10))
        if len(samples) < minimum:
            return {
                "phase": role,
                "available": False,
                "accepted": False,
                "sample_count": len(samples),
                "fit_sample_count": len(samples),
                "validation_sample_count": len(
                    self.validation_datasets.get(role, [])
                ),
                "minimum_samples": minimum,
                "metrics": [],
                "detail": f"样本 {len(samples)}/{minimum}",
            }
        try:
            result = (
                solve_eye_in_hand(samples, quality=self.config["quality"])
                if role == "eye_in_hand"
                else solve_eye_to_hand(samples, quality=self.config["quality"])
            )
        except (CalibrationQualityError, RuntimeError, ValueError) as exc:
            return {
                "phase": role,
                "available": False,
                "accepted": False,
                "sample_count": len(samples),
                "fit_sample_count": len(samples),
                "validation_sample_count": len(
                    self.validation_datasets.get(role, [])
                ),
                "minimum_samples": minimum,
                "metrics": [],
                "detail": str(exc),
            }
        residual = result.get("residual", {})
        stability = result.get("leave_one_out_stability", {})
        bootstrap = result.get("bootstrap_stability", {})
        targets = settings.get("quality_targets", {})
        translation_rms_mm = 1000.0 * float(
            residual.get("closure_translation_rms_m", math.inf)
        )
        rotation_rms_deg = float(residual.get("closure_rotation_rms_deg", math.inf))
        stability_translation_mm = 1000.0 * float(
            stability.get("translation_rms_m", math.inf)
        )
        stability_rotation_deg = float(stability.get("rotation_rms_deg", math.inf))
        def numeric_or(value: Any, fallback: float) -> float:
            return float(value) if isinstance(value, (int, float)) else fallback

        bootstrap_success_ratio = numeric_or(bootstrap.get("success_ratio"), 0.0)
        bootstrap_translation_p95_mm = 1000.0 * numeric_or(
            bootstrap.get("translation_p95_m"), math.inf
        )
        bootstrap_rotation_p95_deg = numeric_or(
            bootstrap.get("rotation_p95_deg"), math.inf
        )
        limits = {
            "translation_rms_mm": float(targets.get("translation_rms_mm", 3.0)),
            "rotation_rms_deg": float(targets.get("rotation_rms_deg", 1.0)),
            "stability_translation_rms_mm": float(
                targets.get("stability_translation_rms_mm", 3.0)
            ),
            "stability_rotation_rms_deg": float(
                targets.get("stability_rotation_rms_deg", 1.0)
            ),
            "bootstrap_success_ratio": float(
                self.config["quality"].get("bootstrap_min_success_ratio", 0.90)
            ),
            "bootstrap_translation_p95_mm": float(
                targets.get("bootstrap_translation_p95_mm", 2.0)
            ),
            "bootstrap_rotation_p95_deg": float(
                targets.get("bootstrap_rotation_p95_deg", 0.60)
            ),
        }
        values = {
            "translation_rms_mm": translation_rms_mm,
            "rotation_rms_deg": rotation_rms_deg,
            "stability_translation_rms_mm": stability_translation_mm,
            "stability_rotation_rms_deg": stability_rotation_deg,
            "bootstrap_success_ratio": bootstrap_success_ratio,
            "bootstrap_translation_p95_mm": bootstrap_translation_p95_mm,
            "bootstrap_rotation_p95_deg": bootstrap_rotation_p95_deg,
        }
        relocation: dict[str, float | int] | None = None
        held_out = self.validation_datasets.get(role, [])
        if held_out:
            try:
                relocation = _evaluate_independent_validation(held_out, result)
            except (CalibrationQualityError, RuntimeError, ValueError):
                relocation = None
        if relocation is not None:
            validation_targets = settings.get("validation_targets", {})
            values.update(
                {
                    "relocalization_translation_rms_mm": 1000.0
                    * float(relocation["closure_translation_rms_m"]),
                    "relocalization_rotation_rms_deg": float(
                        relocation["closure_rotation_rms_deg"]
                    ),
                    "relocalization_translation_p95_mm": 1000.0
                    * float(relocation["closure_translation_p95_m"]),
                    "relocalization_rotation_p95_deg": float(
                        relocation["closure_rotation_p95_deg"]
                    ),
                }
            )
            limits.update(
                {
                    "relocalization_translation_rms_mm": float(
                        validation_targets.get("translation_rms_mm", 3.0)
                    ),
                    "relocalization_rotation_rms_deg": float(
                        validation_targets.get("rotation_rms_deg", 1.0)
                    ),
                    "relocalization_translation_p95_mm": float(
                        validation_targets.get("translation_p95_mm", 4.0)
                    ),
                    "relocalization_rotation_p95_deg": float(
                        validation_targets.get("rotation_p95_deg", 1.25)
                    ),
                }
            )
        # Independent re-localization is the primary accuracy estimate: its
        # poses were not used to fit the transform.  Bootstrap intentionally
        # samples the designed, non-IID pose set with replacement, therefore
        # a high P95 often means a high-leverage view was omitted rather than
        # that a measured observation should be deleted.  Keep it visible as
        # a coverage-sensitivity diagnostic, never as an acceptance gate.
        acceptance_names = (
            (
                "relocalization_translation_rms_mm",
                "relocalization_rotation_rms_deg",
                "relocalization_translation_p95_mm",
                "relocalization_rotation_p95_deg",
            )
            if relocation is not None
            else ("translation_rms_mm", "rotation_rms_deg")
        )
        passed = all(
            math.isfinite(values[name])
            and values[name] <= limits[name]
            for name in acceptance_names
        )
        translation_std_mm = 1000.0 * float(
            residual.get("closure_translation_std_m", math.inf)
        )
        rotation_std_deg = float(residual.get("closure_rotation_std_deg", math.inf))
        detail = (
            f"闭环 {translation_rms_mm:.2f}±{translation_std_mm:.2f}mm/"
            f"{rotation_rms_deg:.3f}±{rotation_std_deg:.3f}deg，"
            f"留一法 {stability_translation_mm:.2f}mm/"
            f"{stability_rotation_deg:.3f}deg，"
            f"Bootstrap 覆盖敏感性 P95 {bootstrap_translation_p95_mm:.2f}mm/"
            f"{bootstrap_rotation_p95_deg:.3f}deg"
        )
        if relocation is not None:
            detail += (
                "，独立重定位 "
                f"{values['relocalization_translation_rms_mm']:.2f}mm/"
                f"{values['relocalization_rotation_rms_deg']:.3f}deg，P95 "
                f"{values['relocalization_translation_p95_mm']:.2f}mm/"
                f"{values['relocalization_rotation_p95_deg']:.3f}deg"
            )

        def safe_number(value: float) -> float | None:
            return float(value) if math.isfinite(value) else None

        metric_specs: list[tuple[str, str, str, str, bool]] = []
        if relocation is not None:
            metric_specs.extend(
                [
                    (
                        "relocalization_translation_rms_mm",
                        "独立重定位平移 RMS",
                        "mm",
                        "max",
                        False,
                    ),
                    (
                        "relocalization_rotation_rms_deg",
                        "独立重定位旋转 RMS",
                        "deg",
                        "max",
                        False,
                    ),
                    (
                        "relocalization_translation_p95_mm",
                        "独立重定位平移 P95",
                        "mm",
                        "max",
                        False,
                    ),
                    (
                        "relocalization_rotation_p95_deg",
                        "独立重定位旋转 P95",
                        "deg",
                        "max",
                        False,
                    ),
                ]
            )
        metric_specs.extend(
            [
                ("translation_rms_mm", "闭环平移 RMS", "mm", "max", True),
                ("rotation_rms_deg", "闭环旋转 RMS", "deg", "max", True),
                ("stability_translation_rms_mm", "留一法平移 RMS", "mm", "max", True),
                ("stability_rotation_rms_deg", "留一法旋转 RMS", "deg", "max", True),
                (
                    "bootstrap_translation_p95_mm",
                    "Bootstrap 平移 P95（覆盖敏感性）",
                    "mm",
                    "max",
                    True,
                ),
                (
                    "bootstrap_rotation_p95_deg",
                    "Bootstrap 旋转 P95（覆盖敏感性）",
                    "deg",
                    "max",
                    True,
                ),
                ("bootstrap_success_ratio", "Bootstrap 成功率", "%", "min", True),
            ]
        )
        metrics = []
        for key, label, unit, direction, diagnostic in metric_specs:
            value = values[key]
            target = limits[key]
            display_value = 100.0 * value if unit == "%" else value
            display_target = 100.0 * target if unit == "%" else target
            metric_passed = (
                math.isfinite(value)
                and (value >= target if direction == "min" else value <= target)
            )
            metrics.append(
                {
                    "key": key,
                    "label": label,
                    "value": safe_number(display_value),
                    "target": safe_number(display_target),
                    "unit": unit,
                    "direction": direction,
                    "passed": bool(metric_passed),
                    "diagnostic": diagnostic,
                }
            )
        return {
            "phase": role,
            "available": True,
            "accepted": bool(passed),
            "sample_count": len(samples),
            "fit_sample_count": len(samples),
            "validation_sample_count": len(held_out),
            "minimum_samples": minimum,
            "metrics": metrics,
            "detail": detail,
            "relocalization": relocation,
        }

    def _require_phase_quality(
        self,
        role: str,
        settings: dict[str, Any],
    ) -> None:
        # The operator explicitly accepted the review stage.  Thresholds are
        # diagnostic warnings, not an unattended adoption gate.
        _, detail = self._phase_solution_meets_target(role, settings)
        print(f"HANDEYE_REVIEW_ACCEPTED {role} {detail}", flush=True)

    def _prepare_gripper_target(self, settings: dict[str, Any]) -> float:
        self.controller.hold_current()
        limits = self.controller.get_limits()
        open_width, pregrasp_width, minimum_width = resolve_automatic_gripper_widths(
            settings,
            float(limits.gripper_width_m),
        )
        self._open_gripper(open_width)
        self.controller.teach_with_gripper_hold()
        print("HANDEYE_STAGE board", flush=True)
        self._require_stage_input("BOARD")
        self.controller.hold_current()
        current_width = self.controller.get_state().gripper_position
        pregrasp_duration = max(
            0.5,
            self.controller.minimum_gripper_duration(pregrasp_width - current_width),
        )
        self.controller.move_gripper(pregrasp_width, pregrasp_duration)
        grip = self.controller.force_close_gripper(
            float(settings["closing_torque_nm"]),
            minimum_width_m=minimum_width,
            timeout_s=5.0,
            contact_hold_s=0.75,
            position_hold_after=True,
            torque_hold_after=True,
        )
        if grip.stop_reason != "contact":
            raise RuntimeError("夹爪未确认接触标定板；请重新放置标定板后再试")
        self._board_clamped = True
        self.controller.teach_with_gripper_hold()
        return open_width

    def _open_gripper(self, open_width: float) -> None:
        current_width = self.controller.get_state().gripper_position
        duration = max(
            1.5,
            self.controller.minimum_gripper_duration(open_width - current_width),
        )
        self.controller.move_gripper(open_width, duration)
        self._board_clamped = False

    def _cancel_dual_to_home(
        self,
        settings: dict[str, Any],
        *,
        board_clamped: bool,
        open_width_m: float,
    ) -> bool:
        """Finish a cooperative dual-camera cancellation at joint zero."""
        self.controller.hold_current()
        center = self._phase_center
        if center is not None:
            current = self.controller.get_state().joint_position.copy()
            if not np.allclose(current, center, atol=1e-10, rtol=0.0):
                _movej_for_duration(
                    self.controller,
                    center,
                    _automatic_motion_duration(
                        self.controller,
                        center - current,
                        float(settings["speed"]),
                    ),
                    allow_large_motion=True,
                )
        if board_clamped:
            print("HANDEYE_STAGE cancel_remove", flush=True)
            self._require_stage_input("REMOVE")
            self._open_gripper(open_width_m)
            board_clamped = False
        self._return_to_zero(settings)
        self._idle_after_motion()
        print("HANDEYE_CANCELLED home", flush=True)
        return board_clamped

    def _cancel_dual_to_center(
        self,
        settings: dict[str, Any],
        *,
        board_clamped: bool,
        open_width_m: float,
    ) -> bool:
        print("HANDEYE_STAGE cancel_center", flush=True)
        self.controller.hold_current()
        center = self._phase_center
        if center is not None:
            current = self.controller.get_state().joint_position.copy()
            if not np.allclose(current, center, atol=1e-10, rtol=0.0):
                _movej_for_duration(
                    self.controller,
                    center,
                    _automatic_motion_duration(
                        self.controller,
                        center - current,
                        float(settings["speed"]),
                    ),
                    allow_large_motion=True,
                )
        if board_clamped:
            print("HANDEYE_STAGE cancel_remove", flush=True)
            self._require_stage_input("REMOVE")
            self._open_gripper(open_width_m)
            board_clamped = False
        self._idle_after_motion()
        print("HANDEYE_CANCELLED center", flush=True)
        return board_clamped

    def _run_legacy_automatic(self) -> dict[str, Any] | None:
        settings = {
            "open_width_m": 0.012,
            "pregrasp_width_m": 0.007,
            "closing_torque_nm": 0.30,
            "minimum_width_m": 0.002,
            "settle_s": 0.65,
            "speed": 0.30,
            **self.config.get("automatic", {}),
        }
        failed = False
        board_clamped = False
        try:
            self.intrinsics = self.camera.start()
            self.controller.connect(stabilize=True)
            self._start_feedback_stream()
            self.session.initialize(config=self.config, intrinsics=self.intrinsics)
            self.samples, _ = self.session.read_samples()

            limits = self.controller.get_limits()
            start = self.controller.get_state()
            open_width, pregrasp_width, minimum_width = (
                resolve_automatic_gripper_widths(
                    settings,
                    float(limits.gripper_width_m),
                )
            )
            open_duration = max(
                1.5,
                self.controller.minimum_gripper_duration(
                    open_width - start.gripper_position
                ),
            )
            self.controller.move_gripper(open_width, open_duration)
            print("HANDEYE_STAGE board", flush=True)
            self._require_stage_input("BOARD")

            current_width = self.controller.get_state().gripper_position
            pregrasp_duration = max(
                0.5,
                self.controller.minimum_gripper_duration(
                    pregrasp_width - current_width
                ),
            )
            self.controller.move_gripper(pregrasp_width, pregrasp_duration)
            grip = self.controller.force_close_gripper(
                float(settings["closing_torque_nm"]),
                minimum_width_m=minimum_width,
                timeout_s=5.0,
                contact_hold_s=0.75,
                position_hold_after=True,
                torque_hold_after=True,
            )
            if grip.stop_reason != "contact":
                raise RuntimeError(
                    "夹爪未确认接触标定板；请重新放置标定板后再试"
                )
            board_clamped = True
            print(
                f"标定板夹紧：{grip.final_width_m * 1000.0:.1f} mm，"
                f"停止原因 {grip.stop_reason}，"
                f"持续保持 {grip.commanded_closing_torque_nm:.2f} N·m。",
                flush=True,
            )
            self._raise_if_cancel_requested()
            self.controller.teach_with_gripper_hold()
            print("HANDEYE_STAGE center", flush=True)
            self._wait_for_guided_center()

            self._start_automatic_camera_stream()
            self.controller.hold_current()
            self.controller.activate_gripper()
            center = self.controller.get_state().joint_position.copy()
            plan, _ = self._automatic_pose_plan(
                center,
                settings,
            )
            self._raise_if_cancel_requested()
            print(
                f"HANDEYE_STAGE run {len(plan)} 个相机朝向姿态",
                flush=True,
            )
            self._require_stage_input("RUN")

            minimum_samples = int(self.config["quality"].get("min_samples", 10))
            target_samples = max(
                minimum_samples,
                int(settings.get("target_valid_samples", minimum_samples + 3)),
            )
            current = center.copy()
            for plan_index, (label, target) in enumerate(plan, start=1):
                self._raise_if_cancel_requested()
                self._automatic_preview_context = (
                    f"自动姿态 {plan_index}/{len(plan)} · 正在运动"
                )
                if not np.allclose(current, target, atol=1e-10, rtol=0.0):
                    _movej_for_duration(
                        self.controller,
                        target,
                        _automatic_motion_duration(
                            self.controller,
                            target - current,
                            float(settings["speed"]),
                        ),
                    )
                current = target.copy()
                self._raise_if_cancel_requested()
                self._automatic_preview_context = (
                    f"自动姿态 {plan_index}/{len(plan)} · 等待稳定"
                )
                self._automatic_preview_context = (
                    f"自动姿态 {plan_index}/{len(plan)} · 正在采样"
                )
                sample = self.capture_sample(keep_active=True)
                if sample is not None:
                    self.samples.append(sample)
                    self._persist_samples()
                    print(
                        f"已采集 {sample.index}/{len(plan)}：{label}，"
                        f"reprojection={sample.reprojection_error_px:.3f}px",
                        flush=True,
                    )
                else:
                    print(
                        f"自动姿态 {plan_index}/{len(plan)}：{label} 未获得有效样本。",
                        flush=True,
                    )
                print(
                    f"HANDEYE_POSE {plan_index}/{len(plan)} "
                    f"valid={len(self.samples)} target={target_samples} "
                    f"label={label}",
                    flush=True,
                )
                self._raise_if_cancel_requested()
                enough_samples = len(self.samples) >= target_samples
                if enough_samples:
                    try:
                        diversity = analyze_motion_diversity(self.samples)
                        validate_motion_diversity(
                            diversity,
                            {
                                **self.config["quality"],
                                "min_samples": target_samples,
                            },
                        )
                    except CalibrationQualityError:
                        enough_samples = False
                if not np.allclose(target, center, atol=1e-10, rtol=0.0):
                    self._automatic_preview_context = (
                        f"自动姿态 {plan_index}/{len(plan)} · 返回中心"
                    )
                    _movej_for_duration(
                        self.controller,
                        center,
                        _automatic_motion_duration(
                            self.controller,
                            center - target,
                            float(settings["speed"]),
                        ),
                    )
                    current = center.copy()
                    time.sleep(0.25)
                if enough_samples:
                    print(
                        f"已获得 {len(self.samples)} 组有效且运动分布合格的样本，"
                        "提前结束姿态搜索。",
                        flush=True,
                    )
                    break

            self._raise_if_cancel_requested()
            self._run_manual_supplement(
                minimum_samples=minimum_samples,
                target_samples=target_samples,
                speed=float(settings["speed"]),
            )
            self._raise_if_cancel_requested()
            print("HANDEYE_STAGE remove", flush=True)
            self._require_stage_input("REMOVE")
            current_width = self.controller.get_state().gripper_position
            open_duration = max(
                1.5,
                self.controller.minimum_gripper_duration(
                    open_width - current_width
                ),
            )
            self.controller.move_gripper(open_width, open_duration)
            board_clamped = False
            self._idle_after_motion()
        except HandEyeCancelled:
            try:
                board_clamped = self._cancel_to_home(
                    settings,
                    board_clamped=board_clamped,
                    open_width_m=open_width,
                )
                return None
            except BaseException:
                failed = True
                raise
        except BaseException:
            failed = True
            raise
        finally:
            self._stop_feedback_stream()
            self._stop_automatic_camera_stream()
            self.camera.stop()
            self.controller.close(emergency=failed)
            if board_clamped:
                print(
                    "标定中断时夹爪位置保持已释放；取板前请先托住标定板。",
                    flush=True,
                )

        minimum = int(self.config["quality"].get("min_samples", 10))
        if len(self.samples) < minimum:
            print(
                f"当前只有 {len(self.samples)} 组样本，至少需要 {minimum} 组；"
                f"数据已保存到 {self.session.path}。",
                flush=True,
            )
            return None
        return solve_and_store_session(self.session)

    def _run_manual_supplement(
        self,
        *,
        minimum_samples: int,
        target_samples: int,
        speed: float,
    ) -> None:
        """Allow safe hand-guided samples after the automatic pose search."""
        self.controller.teach_with_gripper_hold()
        while True:
            ready = self._samples_meet_quality(minimum_samples)
            self._automatic_preview_context = (
                "手动补采 · 松手稳定后点击采集当前姿态"
            )
            print(
                "HANDEYE_STAGE manual "
                f"valid={len(self.samples)} minimum={minimum_samples} "
                f"target={target_samples} ready={int(ready)}",
                flush=True,
            )
            command = self._require_stage_command()
            if command == "DONE":
                if not ready:
                    print(
                        "HANDEYE_MANUAL_REJECT "
                        f"当前 {len(self.samples)} 组样本尚未达到数量或运动分布要求",
                        flush=True,
                    )
                    continue
                self.controller.hold_current()
                self.controller.activate_gripper()
                return
            if command == "HOME":
                self._automatic_preview_context = "手动补采 · 正在回零位"
                self.controller.hold_current()
                self.controller.activate_gripper()
                current = self.controller.get_state().joint_position.copy()
                zero = np.zeros(6, dtype=np.float64)
                if not np.allclose(current, zero, atol=1e-10, rtol=0.0):
                    _movej_for_duration(
                        self.controller,
                        zero,
                        _automatic_motion_duration(
                            self.controller,
                            zero - current,
                            speed,
                        ),
                        allow_large_motion=True,
                    )
                print("HANDEYE_MANUAL_HOME", flush=True)
                self.controller.teach_with_gripper_hold()
                continue
            if command != "SAMPLE":
                raise RuntimeError(
                    f"unexpected hand-eye command: {command}"
                )

            self._automatic_preview_context = "手动补采 · 正在保持并采样"
            self.controller.hold_current()
            self.controller.activate_gripper()
            try:
                sample = self.capture_sample(keep_active=True)
                if sample is not None:
                    self.samples.append(sample)
                    self._persist_samples()
            finally:
                self.controller.teach_with_gripper_hold()
            print(
                "HANDEYE_MANUAL_SAMPLE "
                f"valid={len(self.samples)} target={target_samples} "
                f"accepted={int(sample is not None)}",
                flush=True,
            )

    def _samples_meet_quality(self, minimum_samples: int) -> bool:
        if len(self.samples) < minimum_samples:
            return False
        try:
            validate_motion_diversity(
                analyze_motion_diversity(self.samples),
                {
                    **self.config["quality"],
                    "min_samples": minimum_samples,
                },
            )
        except CalibrationQualityError:
            return False
        return True

    def _idle_after_motion(self) -> None:
        """Keep shared web controller holding; standalone CLI may use damping."""
        if self._owns_controller:
            self.controller.damping()
        else:
            self.controller.hold_current()

    def _return_to_reference(self, settings: dict[str, Any]) -> None:
        """Return after successful calibration, never while the board is held."""
        enabled = settings.get("return_to_reference_after_complete", True)
        if not isinstance(enabled, bool):
            raise RuntimeError("自动回参考位设置必须为布尔值")
        if not enabled:
            print("HANDEYE_RETURN_REFERENCE disabled", flush=True)
            return
        raw_target = settings.get("return_reference_joint_position_rad")
        if raw_target is None:
            target = np.zeros(6, dtype=np.float64)
            label = "六关节零位"
        else:
            target = np.asarray(raw_target, dtype=np.float64)
            if target.shape != (6,) or not np.all(np.isfinite(target)):
                raise RuntimeError("自动回参考位的关节目标必须是六个有限数值")
            label = str(settings.get("return_reference_label", "参考位"))
        print(f"HANDEYE_STAGE return_reference {label}", flush=True)
        self._notify_stage("return_reference")
        self.controller.hold_current()
        current = self.controller.get_state().joint_position.copy()
        if not np.allclose(current, target, atol=1e-10, rtol=0.0):
            _movej_for_duration(
                self.controller,
                target,
                _automatic_motion_duration(
                    self.controller,
                    target - current,
                    float(settings["speed"]),
                ),
                allow_large_motion=True,
            )

    def _cancel_to_home(
        self,
        settings: dict[str, Any],
        *,
        board_clamped: bool,
        open_width_m: float,
    ) -> bool:
        """Return to joint zero before a cooperative web cancellation exits."""
        print("HANDEYE_STAGE cancel_home", flush=True)
        self.controller.hold_current()
        current = self.controller.get_state().joint_position.copy()
        zero = np.zeros(6, dtype=np.float64)
        if not np.allclose(current, zero, atol=1e-10, rtol=0.0):
            _movej_for_duration(
                self.controller,
                zero,
                _automatic_motion_duration(
                    self.controller,
                    zero - current,
                    float(settings["speed"]),
                ),
                allow_large_motion=True,
            )

        if board_clamped:
            print("HANDEYE_STAGE cancel_remove", flush=True)
            self._require_stage_input("REMOVE")
            current_width = self.controller.get_state().gripper_position
            open_duration = max(
                1.5,
                self.controller.minimum_gripper_duration(
                    open_width_m - current_width
                ),
            )
            self.controller.move_gripper(open_width_m, open_duration)
            board_clamped = False

        self._idle_after_motion()
        print("HANDEYE_CANCELLED home", flush=True)
        return board_clamped

    def _wait_for_guided_center(
        self,
        *,
        expected_command: str | None = "CENTER",
        timeout_s: float | None = None,
    ) -> None:
        if self.intrinsics is None:
            raise RuntimeError(
                f"{self._camera_label} intrinsics are unavailable for center guidance"
            )
        cv2 = _cv2()
        if expected_command is not None:
            self._notify_stage(expected_command.lower())
        limits = self.controller.get_limits()
        guidance_limits = self._center_guidance_limits()
        latest_guidance: dict[str, Any] | None = None
        last_preview_at = 0.0
        ready_streak = 0
        # The browser receives a separately encoded preview while this loop
        # checks frames. Without a short latch, the UI can display a
        # valid frame but the next local frame can be a one-frame detection
        # miss before the READY command arrives, making a valid click appear
        # to do nothing.  It is only a confirmation grace period, not an
        # automatic motion permission.
        ready_latch_s = 2.5
        ready_latched_at: float | None = None
        ready_latched_guidance: dict[str, Any] | None = None
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while True:
            command = self._poll_stage_input()
            if command is not None:
                if expected_command is None:
                    raise RuntimeError(f"unexpected hand-eye command: {command}")
                if command != expected_command:
                    raise RuntimeError(f"unexpected hand-eye command: {command}")
                latch_is_fresh = (
                    ready_latched_at is not None
                    and time.monotonic() - ready_latched_at <= ready_latch_s
                )
                if latch_is_fresh or (
                    latest_guidance is not None and latest_guidance["ready"]
                ):
                    return
                message = (
                    latest_guidance["message"]
                    if latest_guidance is not None
                    else f"尚未获得 {self._camera_label} 标定板画面"
                )
                print(f"HANDEYE_CENTER_REJECT {message}", flush=True)
                # The web server optimistically switches to "working" as
                # soon as it queues READY/CENTER.  If the fresh local frame
                # rejects it, explicitly restore the actionable stage rather
                # than leaving the page waiting for a command that was not
                # accepted.
                self._notify_stage(expected_command.lower())

            if deadline is not None and time.monotonic() >= deadline:
                message = (
                    latest_guidance["message"]
                    if latest_guidance is not None
                    else f"尚未获得 {self._camera_label} 标定板画面"
                )
                raise RuntimeError(
                    f"{self._camera_label} 起始姿态检查超时：{message}"
                )

            frame = self.camera.read()
            now = time.monotonic()
            if now - last_preview_at < 0.8 / self._web_preview_fps:
                continue
            detection = self.detector.detect(frame, self.intrinsics)
            snapshot = self.controller.get_state()
            latest_guidance = build_center_guidance(
                detection,
                self.intrinsics,
                snapshot.joint_position,
                limits.joint_position_min,
                limits.joint_position_max,
                **guidance_limits,
            )
            latest_guidance["message"] = str(latest_guidance["message"]).replace(
                "D455", self._camera_label
            )
            if latest_guidance["ready"]:
                ready_streak += 1
                if ready_streak < 3:
                    latest_guidance["ready"] = False
                    latest_guidance["message"] = "位置合适，请保持稳定"
                else:
                    ready_latched_at = now
                    ready_latched_guidance = dict(latest_guidance)
                    latest_guidance["message"] = "起始姿态已稳定就绪，点击确认后生成自动采样计划"
                    if expected_command is None:
                        self._write_web_preview(
                            detection.annotated.copy()
                            if detection is not None
                            else frame.copy(),
                            latest_guidance,
                        )
                        return
            else:
                ready_streak = 0
                if (
                    ready_latched_at is not None
                    and now - ready_latched_at <= ready_latch_s
                    and ready_latched_guidance is not None
                ):
                    latest_guidance = dict(ready_latched_guidance)
                    latest_guidance["message"] = (
                        "起始姿态已就绪（短暂画面波动），可点击确认"
                    )
                else:
                    ready_latched_at = None
                    ready_latched_guidance = None
            annotated = (
                frame.copy()
                if detection is None
                else detection.annotated.copy()
            )
            _draw_center_guidance(annotated, latest_guidance)
            self._write_web_preview(annotated, latest_guidance)
            last_preview_at = now

    def _reconfigure_controller_safety(self, safety: Any) -> None:
        """Use the shared controller API so planner and safety stay in sync."""
        reconfigure = getattr(self.controller, "reconfigure_safety", None)
        if callable(reconfigure):
            reconfigure(safety)
        else:
            # Compatibility for small fake controllers used by offline tools.
            self.controller.safety = safety

    def _center_guidance_limits(self) -> dict[str, float]:
        """Return modestly relaxed D405 teaching limits from trusted config.

        These limits only decide whether the operator may generate a plan.
        Every planned pose still goes through IK, hard joint limits, collision
        checks, settling, and the strict image-quality sample gates.
        """
        if self._sample_role not in {"eye_in_hand", "eye_in_hand_validation"}:
            return {}
        automatic = self.config.get("automatic", {})

        def configured(name: str, default: float) -> float:
            value = float(automatic.get(name, default))
            if not math.isfinite(value):
                raise RuntimeError(f"自动标定配置 {name} 必须是有限数")
            return value

        limits = {
            "j4_min_margin_deg": configured("d405_center_j4_margin_deg", 4.0),
            "min_depth_m": configured("d405_center_min_depth_m", 0.20),
            "max_depth_m": configured("d405_center_max_depth_m", 1.70),
            "image_center_bound": configured("d405_center_image_bound", 0.42),
            "max_tilt_deg": configured("d405_center_max_tilt_deg", 62.0),
        }
        if not 0.05 <= limits["j4_min_margin_deg"] <= 15.0:
            raise RuntimeError("D405 中心 J4 余量必须在 0.05..15 度")
        if not 0.10 <= limits["min_depth_m"] < limits["max_depth_m"] <= 2.5:
            raise RuntimeError("D405 中心距离范围无效")
        if not 0.10 <= limits["image_center_bound"] < 0.50:
            raise RuntimeError("D405 中心画面范围必须在 0.10..0.50")
        if not 10.0 <= limits["max_tilt_deg"] <= 80.0:
            raise RuntimeError("D405 中心倾角范围必须在 10..80 度")
        return limits

    def _write_web_preview(
        self,
        frame: np.ndarray,
        details: dict[str, Any],
    ) -> None:
        # The calibration camera is the only D405 source.  This is a separate
        # low-resolution consumer of that high-resolution frame, never a
        # second UVC stream and never part of the sampling hot path.
        if self._on_event is None:
            return
        now = time.monotonic()
        # Permit a little scheduler tolerance so a 30 FPS RealSense source
        # does not accidentally alternate frames and collapse to 15 FPS.
        if now - self._last_preview_emit_at < 0.8 / self._web_preview_fps:
            return
        cv2 = _cv2()
        height, width = frame.shape[:2]
        output = frame
        preview_width = self._web_preview_width
        if width > preview_width:
            scale = preview_width / float(width)
            output = cv2.resize(
                frame,
                (preview_width, max(1, int(round(height * scale)))),
                interpolation=cv2.INTER_AREA,
            )
        encoded_ok, encoded = cv2.imencode(
            ".jpg",
            output,
            [cv2.IMWRITE_JPEG_QUALITY, self._web_preview_jpeg_quality],
        )
        if not encoded_ok:
            return
        self._last_preview_emit_at = now
        payload = {
            **details,
            "jpeg": base64.b64encode(encoded).decode("ascii"),
        }
        self._emit_event({"type": "preview", "preview": payload})

    def _start_automatic_camera_stream(self) -> None:
        thread = self._camera_stream_thread
        if thread is not None and thread.is_alive():
            return
        self._camera_stream_stop.clear()
        self._camera_stream_error = None
        with self._camera_frame_condition:
            self._camera_frame = None
            self._camera_frame_sequence = 0
        self._camera_stream_thread = threading.Thread(
            target=self._automatic_camera_loop,
            name="handeye-camera",
            daemon=True,
        )
        self._camera_stream_thread.start()

    def _stop_automatic_camera_stream(self) -> None:
        self._camera_stream_stop.set()
        with self._camera_frame_condition:
            self._camera_frame_condition.notify_all()
        thread = self._camera_stream_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.5)
        self._camera_stream_thread = None
        with self._camera_frame_condition:
            self._camera_frame = None

    def _automatic_camera_loop(self) -> None:
        last_preview_at = 0.0
        source_sequence = self.camera.latest_sequence()
        try:
            while not self._camera_stream_stop.is_set():
                source = self.camera.wait_for_frame(
                    source_sequence,
                    timeout_s=2.0,
                )
                source_sequence = source.sequence
                frame = source.image
                with self._camera_frame_condition:
                    self._camera_frame = frame
                    self._camera_frame_sequence += 1
                    self._camera_frame_condition.notify_all()
                now = time.monotonic()
                if now - last_preview_at < 0.8 / self._web_preview_fps:
                    continue
                detection = self._preview_detector.detect(frame, self.intrinsics)
                guidance = build_automatic_preview_guidance(
                    detection,
                    self._automatic_preview_context,
                )
                annotated = (
                    frame.copy()
                    if detection is None
                    else detection.annotated.copy()
                )
                _draw_automatic_guidance(annotated, guidance)
                self._write_web_preview(annotated, guidance)
                last_preview_at = now
        except BaseException as exc:
            self._camera_stream_error = exc
        finally:
            with self._camera_frame_condition:
                self._camera_frame_condition.notify_all()

    def _next_automatic_frame(
        self,
        after_sequence: int,
        *,
        timeout_s: float = 5.0,
    ) -> tuple[np.ndarray, int]:
        deadline = time.monotonic() + timeout_s
        with self._camera_frame_condition:
            while self._camera_frame_sequence <= after_sequence:
                if self._camera_stream_error is not None:
                    raise RuntimeError(
                        f"{self._camera_label} 自动标定视频流读取失败"
                    ) from self._camera_stream_error
                if self._camera_stream_stop.is_set():
                    raise RuntimeError(f"{self._camera_label} 自动标定视频流已经停止")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(f"等待 {self._camera_label} 自动标定画面超时")
                self._camera_frame_condition.wait(remaining)
            if self._camera_frame is None:
                raise RuntimeError(f"{self._camera_label} 自动标定视频流没有可用画面")
            return self._camera_frame.copy(), self._camera_frame_sequence

    def _automatic_pose_plan(
        self,
        center: np.ndarray,
        settings: dict[str, Any],
        *,
        role: str | None = None,
    ) -> tuple[list[tuple[str, np.ndarray]], float]:
        limits = self.controller.get_limits()
        plan = self._scene_guided_pose_plan(
            center,
            limits.joint_position_min,
            limits.joint_position_max,
            role=role,
        )
        accepted: list[tuple[str, np.ndarray]] = []
        max_excursion = 0.0
        for label, target in plan:
            delta = np.asarray(target, dtype=np.float64) - center
            # ``table_center`` is deliberately an actual calibration sample.
            # It is already the current measured joint state, so planning a
            # point-to-point no-op would derive a zero velocity scale and be
            # rejected by MotionContext before automatic collection starts.
            # Keep it in the plan for its reference frame, but do not issue a
            # meaningless preview motion.
            if _is_negligible_joint_delta(delta):
                accepted.append((label, target))
                continue
            duration = _automatic_motion_duration(
                self.controller,
                delta,
                float(settings["speed"]),
            )
            preview = self.controller.plan_movej(
                target,
                velocity_scale=_joint_velocity_scale_for_duration(
                    self.controller, delta, duration
                ),
                allow_large_motion=True,
            )
            path = np.asarray(preview.path_xyz_m, dtype=np.float64)
            excursion = float(np.max(np.linalg.norm(path - path[0], axis=1)))
            accepted.append((label, target))
            max_excursion = max(max_excursion, excursion)
        # A sample count is not a waypoint count.  The automatic collector
        # deliberately revisits each safe pose to suppress visual noise, so a
        # constrained tabletop center must not be rejected merely because it
        # cannot expose twelve *unique* IK endpoints at once.
        default_minimum_unique = (
            int(settings.get("d405_minimum_unique_poses", 6))
            if role == "eye_in_hand"
            else int(self.config["quality"].get("min_samples", 10))
        )
        minimum_unique = max(4, default_minimum_unique)
        maximum_poses = max(
            minimum_unique,
            int(settings.get("max_poses", 30)),
        )
        # Joint variation is not a calibration observable: large elbow or
        # wrist compensation can leave the flange orientation almost fixed.
        # Rank waypoints by their *predicted EEF transforms*, not joint deltas,
        # so D405 gets the roll/pitch/yaw coverage required by AX=XB before
        # translations and repeat observations.
        predicted_transforms = []
        for _, target in accepted:
            try:
                endpoint_pose = self.controller.forward_kinematics(target)
            except (RuntimeError, OSError, ValueError, SafetyError) as exc:
                raise AutomaticPosePlanError(
                    f"无法验证自动姿态的末端运动：{exc}"
                ) from exc
            predicted_transforms.append(pose6_to_transform(endpoint_pose))
        accepted = _order_pose_plan_for_eef_diversity(
            accepted,
            predicted_transforms,
            quality=self.config["quality"],
        )
        accepted = accepted[:maximum_poses]
        if len(accepted) < minimum_unique:
            raise AutomaticPosePlanError(
                f"当前中心只能生成 {len(accepted)} 个独立安全姿态，"
                f"至少需要 {minimum_unique} 个；"
                "请重新摆放安全中心"
            )
        if role == "eye_in_hand":
            planned_transforms = [
                pose6_to_transform(self.controller.forward_kinematics(target))
                for _, target in accepted
            ]
            diversity = analyze_transform_motion_diversity(planned_transforms)
            information_rank, information_weakest, information_spread = (
                _hybrid_motion_information(
                    planned_transforms,
                    translation_scale_m=float(
                        self.config["quality"].get("min_translation_span_m", 0.04)
                    ),
                    rotation_scale_deg=float(
                        self.config["quality"].get("min_rotation_span_deg", 20.0)
                    ),
                )
            )
            coupled_count = sum(
                label.startswith("table_coupled_") for label, _ in accepted
            )
            required_rotation = float(
                self.config["quality"].get("min_rotation_span_deg", 20.0)
            )
            required_rank = int(
                self.config["quality"].get("min_rotation_axis_rank", 2)
            )
            required_translation = float(
                self.config["quality"].get("min_translation_span_m", 0.04)
            )
            print(
                "HANDEYE_PLAN_DIVERSITY eye_in_hand "
                f"poses={len(accepted)} translation={1000.0 * diversity.translation_span_m:.1f}mm "
                f"rotation={diversity.rotation_span_deg:.1f}deg "
                f"rank={diversity.rotation_axis_rank} coupled={coupled_count} "
                f"hybrid_rank={information_rank} "
                f"hybrid_min_sv={information_weakest:.3f} "
                f"hybrid_logdet={information_spread:.3f}",
                flush=True,
            )
            if (
                diversity.rotation_span_deg < required_rotation
                or diversity.rotation_axis_rank < required_rank
                or diversity.translation_span_m < required_translation
            ):
                raise AutomaticPosePlanError(
                    "D405 自动姿态的预估运动覆盖不足："
                    f"平移 {1000.0 * diversity.translation_span_m:.1f}mm/"
                    f"{1000.0 * required_translation:.1f}mm，"
                    f"旋转 {diversity.rotation_span_deg:.1f}°/"
                    f"{required_rotation:.1f}°，"
                    f"轴秩 {diversity.rotation_axis_rank}/{required_rank}；"
                    "已在执行前拦截并自动搜索更合适的安全中心"
                )
        return accepted, max_excursion

    def _phase_pose_plans(
        self,
        center: np.ndarray,
        settings: dict[str, Any],
        *,
        role: str,
    ) -> tuple[list[tuple[str, np.ndarray]], list[tuple[str, np.ndarray]]]:
        """Build both fit and held-out plans from one actual arm center."""
        plan, _ = self._automatic_pose_plan(center, settings, role=role)
        validation_plan = self._independent_validation_pose_plan(
            center,
            settings,
            role=role,
            training_plan=plan,
            excluded_joint_positions=[
                np.asarray(sample.joint_position_rad, dtype=np.float64)
                for sample in self.datasets[role]
            ],
        )
        return plan, validation_plan

    def _search_adaptive_center(
        self,
        taught_center: np.ndarray,
        settings: dict[str, Any],
        *,
        role: str,
        initial_error: AutomaticPosePlanError,
    ) -> tuple[
        np.ndarray,
        list[tuple[str, np.ndarray]],
        list[tuple[str, np.ndarray]],
    ] | None:
        """Find a better D405 center without asking the operator to reteach.

        The manually taught center is a seed, not a hard boundary.  If it
        lacks enough independent motions, probe nearby *upward/lateral* D405
        poses.  Every probe is first Cartesian-planned with the same hard
        limits and collision checks as collection, then visually confirmed
        before it can become the new center.  The arm remains in active hold
        for the entire search; free damping is used only after every probe has
        been exhausted.
        """
        if role != "eye_in_hand":
            return None
        candidates = self._adaptive_center_candidates(settings)
        maximum_trials = int(settings.get("center_search_max_trials", 16))
        if maximum_trials < 1:
            return None
        candidates = candidates[:maximum_trials]
        if not candidates:
            return None

        print(
            "HANDEYE_CENTER_SEARCH "
            f"eye_in_hand candidates={len(candidates)} reason={initial_error}",
            flush=True,
        )
        self._automatic_preview_context = "D405 正在自动搜索更优安全采样中心"
        current = taught_center.copy()
        best: tuple[
            tuple[float, float, float],
            str,
            np.ndarray,
            list[tuple[str, np.ndarray]],
            list[tuple[str, np.ndarray]],
        ] | None = None
        for index, (label, target) in enumerate(candidates, start=1):
            self._raise_if_cancel_requested()
            self._automatic_preview_context = (
                f"D405 自动搜索中心 {index}/{len(candidates)} · 正在移动"
            )
            _movej_for_duration(
                self.controller,
                target,
                _automatic_motion_duration(
                    self.controller,
                    target - current,
                    float(settings["speed"]),
                ),
                allow_large_motion=True,
            )
            current = target.copy()
            self._automatic_preview_context = (
                f"D405 自动搜索中心 {index}/{len(candidates)} · 检查标定板"
            )
            visibility = self._probe_adaptive_center(settings)
            if visibility is not None:
                candidate_center = self.controller.get_state().joint_position.copy()
                try:
                    plan, validation_plan = self._phase_pose_plans(
                        candidate_center,
                        settings,
                        role=role,
                    )
                except AutomaticPosePlanError:
                    pass
                else:
                    # Prefer the center that leaves the most independent
                    # primary and held-out options.  Reprojection quality is
                    # a tie-breaker; strict per-sample image gates remain in
                    # force during actual collection.
                    score = (
                        float(len(plan)),
                        float(len(validation_plan)),
                        -visibility,
                    )
                    if best is None or score > best[0]:
                        best = (
                            score,
                            label,
                            candidate_center.copy(),
                            plan,
                            validation_plan,
                        )
                        print(
                            "HANDEYE_CENTER_SEARCH_CANDIDATE "
                            f"{index}/{len(candidates)} label={label} "
                            f"fit={len(plan)} validation={len(validation_plan)}",
                            flush=True,
                        )

            # Search samples a candidate, then returns to the taught seed so
            # every comparison starts from the same bounded path.  Do not put
            # this in a finally block: cancellation/emergency handlers own
            # the only allowed motion after an interruption.
            if not np.allclose(current, taught_center, atol=1e-10, rtol=0.0):
                self._automatic_preview_context = (
                    f"D405 自动搜索中心 {index}/{len(candidates)} · 返回起点"
                )
                _movej_for_duration(
                    self.controller,
                    taught_center,
                    _automatic_motion_duration(
                        self.controller,
                        taught_center - current,
                        float(settings["speed"]),
                    ),
                    allow_large_motion=True,
                )
                current = taught_center.copy()

        if best is None:
            return None
        _, label, expected_center, _, _ = best
        self._automatic_preview_context = "D405 自动中心已选定 · 正在就位"
        _movej_for_duration(
            self.controller,
            expected_center,
            _automatic_motion_duration(
                self.controller,
                expected_center - current,
                float(settings["speed"]),
            ),
            allow_large_motion=True,
        )
        selected_center = self.controller.get_state().joint_position.copy()
        try:
            plan, validation_plan = self._phase_pose_plans(
                selected_center,
                settings,
                role=role,
            )
        except AutomaticPosePlanError:
            # A final plan is deliberately recomputed from feedback rather
            # than trusting a stale candidate target.  If feedback shifted
            # enough to invalidate it, leave the arm held at the selected
            # center and let the normal retry path guide the operator.
            print(
                f"HANDEYE_CENTER_SEARCH_STALE label={label}",
                flush=True,
            )
            return None
        return selected_center, plan, validation_plan

    def _adaptive_center_candidates(
        self,
        settings: dict[str, Any],
    ) -> list[tuple[str, np.ndarray]]:
        """Cartesian-safe D405 center adjustments around the taught pose."""
        snapshot = self.controller.get_state()
        eef_pose = np.asarray(snapshot.eef_pose_6d, dtype=np.float64)
        if eef_pose.shape != (6,) or not np.all(np.isfinite(eef_pose)):
            return []
        target_rotation = pose6_to_transform(eef_pose)[:3, :3]
        limits = self.controller.get_limits()
        margin_deg = float(settings.get("d405_joint_margin_deg", 0.10))
        if not math.isfinite(margin_deg) or not 0.0 <= margin_deg <= 2.0:
            raise RuntimeError("D405 自动标定关节余量必须在 0..2 度")
        margin = math.radians(margin_deg)

        def axis_rotation(axis: np.ndarray, angle_deg: float) -> np.ndarray:
            angle = math.radians(angle_deg)
            axis = np.asarray(axis, dtype=np.float64)
            skew = np.asarray(
                [
                    [0.0, -axis[2], axis[1]],
                    [axis[2], 0.0, -axis[0]],
                    [-axis[1], axis[0], 0.0],
                ]
            )
            return (
                np.eye(3)
                + math.sin(angle) * skew
                + (1.0 - math.cos(angle)) * (skew @ skew)
            )

        # Never probe below the taught tabletop pose.  The candidates only
        # shift laterally/upward or make small camera-relative rotations;
        # plan_movel performs the final IK, hard-limit and collision check.
        candidates: list[tuple[str, np.ndarray, np.ndarray]] = [
            ("search_x_plus", np.array([0.025, 0.0, 0.0]), target_rotation),
            ("search_x_minus", np.array([-0.025, 0.0, 0.0]), target_rotation),
            ("search_y_plus", np.array([0.0, 0.025, 0.0]), target_rotation),
            ("search_y_minus", np.array([0.0, -0.025, 0.0]), target_rotation),
            ("search_z_up", np.array([0.0, 0.0, 0.020]), target_rotation),
            ("search_xy_pp", np.array([0.020, 0.020, 0.012]), target_rotation),
            ("search_xy_mm", np.array([-0.020, -0.020, 0.012]), target_rotation),
        ]
        for axis_name, axis, angle in (
            ("roll", np.array([1.0, 0.0, 0.0]), 6.0),
            ("pitch", np.array([0.0, 1.0, 0.0]), 6.0),
            ("yaw", np.array([0.0, 0.0, 1.0]), 8.0),
        ):
            for sign in (1.0, -1.0):
                candidates.append(
                    (
                        f"search_{axis_name}_{'plus' if sign > 0 else 'minus'}",
                        np.array([0.0, 0.0, 0.012]),
                        target_rotation @ axis_rotation(axis, sign * angle),
                    )
                )
        candidates.extend(
            [
                ("search_x_wide_plus", np.array([0.045, 0.0, 0.020]), target_rotation),
                ("search_x_wide_minus", np.array([-0.045, 0.0, 0.020]), target_rotation),
                ("search_y_wide_plus", np.array([0.0, 0.045, 0.020]), target_rotation),
                ("search_y_wide_minus", np.array([0.0, -0.045, 0.020]), target_rotation),
            ]
        )

        result: list[tuple[str, np.ndarray]] = []
        seen: set[tuple[float, ...]] = set()
        for label, delta_xyz, rotation in candidates:
            try:
                planned = self.controller.plan_movel(
                    Pose(
                        eef_pose[:3] + delta_xyz,
                        quaternion_xyzw_from_matrix(rotation),
                    ),
                    unrestricted=False,
                )
                target = np.asarray(planned.target_joint_position, dtype=np.float64)
            except (RuntimeError, OSError, ValueError):
                continue
            if target.shape != (6,) or not np.all(np.isfinite(target)):
                continue
            if np.any(target < limits.joint_position_min + margin) or np.any(
                target > limits.joint_position_max - margin
            ):
                continue
            key = tuple(np.round(target, 8))
            if key in seen:
                continue
            seen.add(key)
            result.append((label, target.copy()))
        return result

    def _probe_adaptive_center(self, settings: dict[str, Any]) -> float | None:
        """Confirm that a trial center still yields high-quality D405 frames."""
        frames = int(settings.get("center_search_probe_frames", 4))
        minimum_visible = int(settings.get("center_search_min_visible_frames", 2))
        if not 1 <= minimum_visible <= frames <= 12:
            raise RuntimeError("自动中心搜索画面检查配置无效")
        max_reprojection = float(
            settings.get(
                "center_search_max_reprojection_error_px",
                self.config["collection"].get("max_reprojection_error_px", 1.5),
            )
        )
        if not math.isfinite(max_reprojection) or max_reprojection <= 0.0:
            raise RuntimeError("自动中心搜索重投影误差配置无效")
        sequence = self._camera_frame_sequence
        errors: list[float] = []
        for _ in range(frames):
            self._raise_if_cancel_requested()
            frame, sequence = self._next_automatic_frame(sequence, timeout_s=3.0)
            detection = self.detector.detect(frame, self.intrinsics)
            if detection is None:
                continue
            transform = np.asarray(detection.T_camera_target, dtype=np.float64)
            if (
                transform.shape != (4, 4)
                or not np.all(np.isfinite(transform))
                or transform[2, 3] <= 0.05
                or detection.reprojection_error_px > max_reprojection
            ):
                continue
            errors.append(float(detection.reprojection_error_px))
        if len(errors) < minimum_visible:
            return None
        return float(np.mean(errors))

    def _independent_validation_pose_plan(
        self,
        center: np.ndarray,
        settings: dict[str, Any],
        *,
        role: str,
        training_plan: list[tuple[str, np.ndarray]],
        excluded_joint_positions: list[np.ndarray] | None = None,
        variant: int = 0,
    ) -> list[tuple[str, np.ndarray]]:
        """Plan a holdout disjoint from every endpoint in the current fit."""
        if variant < 0:
            raise ValueError("independent validation variant must be non-negative")
        limits = self.controller.get_limits()
        validation_role = (
            f"{role}_validation" if variant == 0 else f"{role}_validation_{variant}"
        )
        candidates = self._scene_guided_pose_plan(
            center,
            limits.joint_position_min,
            limits.joint_position_max,
            role=validation_role,
        )
        requested = max(1, int(settings.get("validation_pose_count", 4)))
        excluded_targets = [
            np.asarray(target, dtype=np.float64)
            for _, target in training_plan
        ]
        excluded_targets.extend(
            np.asarray(target, dtype=np.float64)
            for target in (excluded_joint_positions or [])
        )
        # Same endpoint within this tolerance is effectively the same robot
        # observation.  An exact float comparison let a promoted validation
        # point slip straight back into the next "independent" review.
        exclusion_tolerance_rad = math.radians(1.5)
        accepted: list[tuple[str, np.ndarray]] = []
        for label, target in candidates:
            if any(
                np.max(np.abs(target - used)) <= exclusion_tolerance_rad
                for used in excluded_targets
            ):
                continue
            duration = _automatic_motion_duration(
                self.controller,
                target - center,
                float(settings["speed"]),
            )
            try:
                self.controller.plan_movej(
                    target,
                    velocity_scale=_joint_velocity_scale_for_duration(
                        self.controller,
                        target - center,
                        duration,
                    ),
                    allow_large_motion=True,
                )
            except (RuntimeError, OSError, ValueError, SafetyError):
                continue
            display_label = label if variant == 0 else f"{label}_round_{variant}"
            accepted.append((display_label, target))
            if len(accepted) >= requested:
                return accepted
        raise AutomaticPosePlanError(
            f"当前中心只能生成 {len(accepted)}/{requested} 个独立验证姿态；"
            "请重新摆放安全中心"
        )

    def _plan_handeye_pose_candidate(self, target: Pose):
        """Resolve one bounded calibration pose to the joint endpoint we execute.

        A calibration waypoint is executed with ``movej``. Planning it through
        the same endpoint path is essential: the generic Cartesian geometry
        builder intentionally rejects a zero-translation segment, which used
        to silently remove every pure roll/pitch/yaw sample from D405 plans.
        ``allow_large_motion`` only lifts the arbitrary per-command 30-degree
        delta; hard joint limits, collision checks, velocity and trajectory
        dynamics are still enforced by ``ArmController``.
        """
        plan_movej = getattr(self.controller, "plan_movej", None)
        if callable(plan_movej):
            return plan_movej(
                target,
                velocity_scale=1.0,
                allow_large_motion=True,
            )
        # Compatibility for minimal test doubles; production ArmController
        # always takes the movej endpoint path above.
        return self.controller.plan_movel(
            target,
            velocity_scale=1.0,
            unrestricted=True,
        )

    def _scene_guided_pose_plan(
        self,
        center: np.ndarray,
        joint_minimum: np.ndarray,
        joint_maximum: np.ndarray,
        *,
        role: str | None = None,
    ) -> list[tuple[str, np.ndarray]]:
        """Build a compact, camera-visible waypoint cluster around the center.

        The board in the supplied setup is held approximately vertical at the
        TCP while the fixed D455 looks across the table. We therefore solve one
        camera-facing orientation first, then sample small intrinsic rotations
        and base-frame translations around that orientation. Every candidate is
        IK-checked before it enters the plan; unreachable candidates are simply
        omitted; the automatic workflow never substitutes a pose that can turn
        the marker edge-on to the camera.
        """
        snapshot = self.controller.get_state()
        eef_pose = np.asarray(snapshot.eef_pose_6d, dtype=np.float64)
        if eef_pose.shape != (6,) or not np.all(np.isfinite(eef_pose)):
            return []

        # The operator teaches a center where the real board is visible. Keep
        # that measured orientation as the source of truth; scene geometry is
        # only an approximate visualization and must not overwrite it.
        target_rotation = pose6_to_transform(eef_pose)[:3, :3]

        def axis_rotation(axis: np.ndarray, angle_deg: float) -> np.ndarray:
            angle = math.radians(angle_deg)
            axis = np.asarray(axis, dtype=np.float64)
            skew = np.asarray(
                [
                    [0.0, -axis[2], axis[1]],
                    [axis[2], 0.0, -axis[0]],
                    [-axis[1], axis[0], 0.0],
                ]
            )
            return (
                np.eye(3)
                + math.sin(angle) * skew
                + (1.0 - math.cos(angle)) * (skew @ skew)
            )

        # The local X axis is the measured board normal. Small, interleaved
        # rotations keep the marker broadside longer while still providing the
        # multi-axis span required by robot-world/hand-eye calibration.
        local_axes = (
            ("roll", np.array([1.0, 0.0, 0.0])),
            ("pitch", np.array([0.0, 1.0, 0.0])),
            ("yaw", np.array([0.0, 0.0, 1.0])),
        )
        validation_variant = _validation_variant_from_role(role)
        if validation_variant > 0:
            # Each review round receives a fresh but still camera-visible
            # validation bank.  Alternating a modest local yaw avoids
            # recycling a promoted holdout while staying inside the same
            # tabletop-safe view envelope.
            direction = 1.0 if validation_variant % 2 else -1.0
            magnitude_deg = 3.0 * ((validation_variant + 1) // 2)
            target_rotation = target_rotation @ axis_rotation(
                np.array([0.0, 0.0, 1.0]),
                direction * magnitude_deg,
            )
        candidates: list[tuple[str, np.ndarray, np.ndarray]] = [
            ("camera_facing", np.zeros(3, dtype=np.float64), target_rotation),
        ]
        # Use screw-like motions for the fixed-camera phase as well.  A
        # wrist-only block followed by a translation-only block can look
        # well-conditioned in leave-one-out tests while remaining weak under
        # an unseen board re-localization.  These modest paired offsets are
        # all checked through the normal IK/collision path below; the older
        # single-axis candidates remain as an IK-aware fallback ladder.
        coupled_camera_candidates = (
            ("roll_plus", (0.026, 0.016, 0.014), "roll", 14.0),
            ("roll_minus", (-0.026, -0.016, 0.014), "roll", -14.0),
            ("pitch_plus", (-0.016, 0.026, 0.014), "pitch", 14.0),
            ("pitch_minus", (0.016, -0.026, 0.014), "pitch", -14.0),
            ("yaw_plus", (0.020, -0.016, 0.012), "yaw", 9.0),
            ("yaw_minus", (-0.020, 0.016, 0.012), "yaw", -9.0),
        )
        axis_by_name = dict(local_axes)
        for label, offset_xyz, axis_name, angle_deg in coupled_camera_candidates:
            candidates.append(
                (
                    f"camera_coupled_{label}",
                    np.asarray(offset_xyz, dtype=np.float64),
                    target_rotation @ axis_rotation(axis_by_name[axis_name], angle_deg),
                )
            )
        for label, offset_xyz, first_axis, first_angle, second_axis, second_angle in (
            ("roll_pitch_pp", (0.022, 0.022, 0.016), "roll", 9.0, "pitch", 9.0),
            ("roll_pitch_mm", (-0.022, -0.022, 0.016), "roll", -9.0, "pitch", -9.0),
            ("roll_yaw_pm", (0.024, -0.020, 0.014), "roll", 9.0, "yaw", -8.0),
            ("pitch_yaw_mp", (-0.020, 0.024, 0.014), "pitch", -9.0, "yaw", 8.0),
        ):
            candidates.append(
                (
                    f"camera_coupled_{label}",
                    np.asarray(offset_xyz, dtype=np.float64),
                    target_rotation
                    @ axis_rotation(axis_by_name[first_axis], first_angle)
                    @ axis_rotation(axis_by_name[second_axis], second_angle),
                )
            )
        # Broader candidate set improves transform conditioning. Every pose is
        # still IK, collision, and hard-limit checked below.
        for angle in (8.0, 12.0, 18.0, 24.0):
            for axis_name, axis in local_axes:
                for sign in (1.0, -1.0):
                    candidates.append(
                        (
                            f"camera_{axis_name}_{'plus' if sign > 0 else 'minus'}_{int(angle)}",
                            np.zeros(3, dtype=np.float64),
                            target_rotation @ axis_rotation(axis, sign * angle),
                        )
                    )
        # Smaller alternatives recover centers where 8-12 degree candidates
        # touch an IK, collision, or joint-limit boundary.
        for axis_name, axis in local_axes:
            for sign in (1.0, -1.0):
                candidates.append(
                    (
                        f"camera_{axis_name}_{'plus' if sign > 0 else 'minus'}_4",
                        np.zeros(3, dtype=np.float64),
                        target_rotation @ axis_rotation(axis, sign * 4.0),
                    )
                )
        candidates.extend(
            [
                (
                    "camera_translate_x_plus",
                    np.array([0.050, 0.0, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_x_minus",
                    np.array([-0.050, 0.0, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_y_plus",
                    np.array([0.0, 0.050, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_y_minus",
                    np.array([0.0, -0.050, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_z_plus",
                    np.array([0.0, 0.0, 0.045]),
                    target_rotation,
                ),
                (
                    "camera_translate_z_minus",
                    np.array([0.0, 0.0, -0.045]),
                    target_rotation,
                ),
                (
                    "camera_translate_x_plus_near",
                    np.array([0.015, 0.0, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_x_minus_near",
                    np.array([-0.015, 0.0, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_y_plus_near",
                    np.array([0.0, 0.015, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_y_minus_near",
                    np.array([0.0, -0.015, 0.0]),
                    target_rotation,
                ),
                (
                    "camera_translate_z_plus_near",
                    np.array([0.0, 0.0, 0.015]),
                    target_rotation,
                ),
                (
                    "camera_translate_z_minus_near",
                    np.array([0.0, 0.0, -0.015]),
                    target_rotation,
                ),
            ]
        )
        for axis_name, axis in local_axes:
            for sign in (1.0, -1.0):
                candidates.append(
                    (
                        f"camera_{axis_name}_{'plus' if sign > 0 else 'minus'}_16",
                        np.zeros(3, dtype=np.float64),
                        target_rotation @ axis_rotation(axis, sign * 16.0),
                    )
                )
        candidates.extend(
            [
                ("camera_translate_yz_pp", np.array([0.0, 0.022, 0.022]), target_rotation),
                ("camera_translate_yz_pm", np.array([0.0, 0.022, -0.022]), target_rotation),
                ("camera_translate_yz_mp", np.array([0.0, -0.022, 0.022]), target_rotation),
                ("camera_translate_yz_mm", np.array([0.0, -0.022, -0.022]), target_rotation),
            ]
        )

        if role == "eye_in_hand":
            # The target is fixed on the table below D405. Preserve the taught
            # camera-facing pose; only lateral / upward offsets and modest
            # attitude changes are permitted. This avoids tabletop approach,
            # horizon-facing, and upside-down candidates before the unknown
            # hand-eye transform has been solved.  Bulid_Block's 17-pose
            # easy_handeye pattern establishes the useful +/- local rotation
            # and base-frame translation ranges.  Here the primary poses
            # deliberately *couple* both observables.  With a 20-sample
            # target, the former "all rotations, then translations" ordering
            # could complete before translation diversity entered the fit.
            candidates = [
                ("table_center", np.zeros(3, dtype=np.float64), target_rotation),
            ]
            axis_by_name = dict(local_axes)

            def add_coupled(
                label: str,
                offset_xyz: tuple[float, float, float],
                *rotations: tuple[str, float],
            ) -> None:
                rotation = target_rotation
                for axis_name, angle_deg in rotations:
                    rotation = rotation @ axis_rotation(
                        axis_by_name[axis_name], angle_deg
                    )
                candidates.append(
                    (
                        f"table_coupled_{label}",
                        np.asarray(offset_xyz, dtype=np.float64),
                        rotation,
                    )
                )

            # Full and half local rotations, each paired with a lateral
            # displacement and a small upward table-clearance component.
            # Opposite points span at least 40 mm in translation while their
            # primary roll/pitch orientations span 28 degrees.
            coupled_primary = (
                ("roll_plus_full", (0.030, 0.018, 0.020), (("roll", 14.0),)),
                ("roll_minus_full", (-0.030, -0.018, 0.020), (("roll", -14.0),)),
                ("pitch_plus_full", (-0.018, 0.030, 0.020), (("pitch", 14.0),)),
                ("pitch_minus_full", (0.018, -0.030, 0.020), (("pitch", -14.0),)),
                # A wide optical-axis turn can force this X5 wrist through a
                # near-singular J5/J6 compensation. The paired +/- roll and
                # pitch points already provide the required 28-degree,
                # rank-2 rotation span; yaw is covered by smaller and
                # diagonal coupled motions below.
                ("roll_plus_half", (0.018, -0.024, 0.014), (("roll", 8.0),)),
                ("roll_minus_half", (-0.018, 0.024, 0.014), (("roll", -8.0),)),
                ("pitch_plus_half", (0.024, 0.018, 0.014), (("pitch", 8.0),)),
                ("pitch_minus_half", (-0.024, -0.018, 0.014), (("pitch", -8.0),)),
                ("yaw_plus_small", (0.016, 0.020, 0.012), (("yaw", 5.0),)),
                ("yaw_minus_small", (-0.016, -0.020, 0.012), (("yaw", -5.0),)),
                ("roll_pitch_pp", (0.022, 0.022, 0.018), (("roll", 9.0), ("pitch", 9.0))),
                ("roll_pitch_mm", (-0.022, -0.022, 0.018), (("roll", -9.0), ("pitch", -9.0))),
                ("roll_yaw_pm", (0.024, -0.020, 0.016), (("roll", 9.0), ("yaw", -9.0))),
                ("pitch_yaw_mp", (-0.020, 0.024, 0.016), (("pitch", -9.0), ("yaw", 9.0))),
            )
            for label, offset_xyz, rotations in coupled_primary:
                add_coupled(label, offset_xyz, *rotations)

            # IK-aware fallback ladder. These are still mixed translation +
            # rotation poses; no center-only rotation can displace the fit
            # back to a low-translation subset when a primary point is
            # unreachable.
            for axis_name, _ in local_axes:
                for sign in (1.0, -1.0):
                    side = 1.0 if axis_name in {"roll", "yaw"} else -1.0
                    add_coupled(
                        f"fallback_{axis_name}_{'plus' if sign > 0 else 'minus'}",
                        (
                            sign * 0.014,
                            side * sign * 0.012,
                            0.010,
                        ),
                        (axis_name, sign * 5.0),
                    )
        elif isinstance(role, str) and role.startswith("eye_in_hand_validation"):
            # These combined motions are disjoint from the single-axis fit
            # poses above.  They remain close to the taught camera-down view
            # so an independent validation cannot send D405 toward the table.
            candidates = [
                (
                    "table_validate_roll_pitch_pp",
                    np.array([0.014, 0.0, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 7.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 7.0),
                ),
                (
                    "table_validate_roll_pitch_mm",
                    np.array([-0.014, 0.0, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -7.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -7.0),
                ),
                (
                    "table_validate_roll_pitch_pm",
                    np.array([0.0, 0.014, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 7.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -7.0),
                ),
                (
                    "table_validate_roll_pitch_mp",
                    np.array([0.0, -0.014, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -7.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 7.0),
                ),
                (
                    "table_validate_yaw_x_plus",
                    np.array([0.018, 0.0, 0.012]),
                    target_rotation @ axis_rotation(np.array([0.0, 0.0, 1.0]), 9.0),
                ),
                (
                    "table_validate_yaw_x_minus",
                    np.array([-0.018, 0.0, 0.012]),
                    target_rotation @ axis_rotation(np.array([0.0, 0.0, 1.0]), -9.0),
                ),
                (
                    "table_validate_small_roll_pitch_pp",
                    np.array([0.008, 0.0, 0.008]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 4.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 4.0),
                ),
                (
                    "table_validate_small_roll_pitch_mm",
                    np.array([-0.008, 0.0, 0.008]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -4.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -4.0),
                ),
                (
                    "table_validate_small_yaw_y_plus",
                    np.array([0.0, 0.008, 0.008]),
                    target_rotation @ axis_rotation(np.array([0.0, 0.0, 1.0]), 4.0),
                ),
                (
                    "table_validate_small_yaw_y_minus",
                    np.array([0.0, -0.008, 0.008]),
                    target_rotation @ axis_rotation(np.array([0.0, 0.0, 1.0]), -4.0),
                ),
                # Keep a second, wider held-out bank.  A review can promote
                # the first six validation samples into the fit; the next
                # review must then have enough genuinely new endpoints rather
                # than silently recycling those poses.
                (
                    "table_validate_wide_roll_pitch_pp",
                    np.array([0.022, 0.012, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 11.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 11.0),
                ),
                (
                    "table_validate_wide_roll_pitch_mm",
                    np.array([-0.022, -0.012, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -11.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -11.0),
                ),
                (
                    "table_validate_wide_roll_pitch_pm",
                    np.array([0.012, -0.022, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 11.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -11.0),
                ),
                (
                    "table_validate_wide_roll_pitch_mp",
                    np.array([-0.012, 0.022, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -11.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 11.0),
                ),
                (
                    "table_validate_wide_roll_yaw_pm",
                    np.array([0.022, -0.014, 0.015]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 11.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), -8.0),
                ),
                (
                    "table_validate_wide_roll_yaw_mp",
                    np.array([-0.022, 0.014, 0.015]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -11.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), 8.0),
                ),
                (
                    "table_validate_wide_pitch_yaw_pm",
                    np.array([0.014, 0.022, 0.015]),
                    target_rotation
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 11.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), -8.0),
                ),
                (
                    "table_validate_wide_pitch_yaw_mp",
                    np.array([-0.014, -0.022, 0.015]),
                    target_rotation
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -11.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), 8.0),
                ),
            ]
        elif isinstance(role, str) and role.startswith("eye_to_hand_validation"):
            # Validation poses use combined, modest motions and are distinct
            # from the single-axis production candidates used by the fit.
            candidates = [
                (
                    "external_validate_roll_pitch_pp",
                    np.array([0.018, 0.0, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 8.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 8.0),
                ),
                (
                    "external_validate_roll_pitch_mm",
                    np.array([-0.018, 0.0, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -8.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -8.0),
                ),
                (
                    "external_validate_roll_yaw_pm",
                    np.array([0.0, 0.018, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 8.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), -8.0),
                ),
                (
                    "external_validate_pitch_yaw_mp",
                    np.array([0.0, -0.018, 0.012]),
                    target_rotation
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -8.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), 8.0),
                ),
                (
                    "external_validate_yaw_xy",
                    np.array([0.015, 0.015, 0.012]),
                    target_rotation @ axis_rotation(np.array([0.0, 0.0, 1.0]), 10.0),
                ),
                (
                    "external_validate_yaw_xy_inverse",
                    np.array([-0.015, -0.015, 0.012]),
                    target_rotation @ axis_rotation(np.array([0.0, 0.0, 1.0]), -10.0),
                ),
                # A review-stage promotion consumes the first held-out set.
                # Reserve a broader second bank so re-evaluation remains an
                # honest independent test after each 4-pose supplement.
                (
                    "external_validate_wide_roll_pitch_pp",
                    np.array([0.028, 0.014, 0.018]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 13.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 13.0),
                ),
                (
                    "external_validate_wide_roll_pitch_mm",
                    np.array([-0.028, -0.014, 0.018]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -13.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -13.0),
                ),
                (
                    "external_validate_wide_roll_pitch_pm",
                    np.array([0.014, -0.028, 0.018]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 13.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -13.0),
                ),
                (
                    "external_validate_wide_roll_pitch_mp",
                    np.array([-0.014, 0.028, 0.018]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -13.0)
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 13.0),
                ),
                (
                    "external_validate_wide_roll_yaw_pm",
                    np.array([0.026, -0.020, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), 12.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), -10.0),
                ),
                (
                    "external_validate_wide_roll_yaw_mp",
                    np.array([-0.026, 0.020, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([1.0, 0.0, 0.0]), -12.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), 10.0),
                ),
                (
                    "external_validate_wide_pitch_yaw_pm",
                    np.array([0.020, 0.026, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), 12.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), -10.0),
                ),
                (
                    "external_validate_wide_pitch_yaw_mp",
                    np.array([-0.020, -0.026, 0.016]),
                    target_rotation
                    @ axis_rotation(np.array([0.0, 1.0, 0.0]), -12.0)
                    @ axis_rotation(np.array([0.0, 0.0, 1.0]), 10.0),
                ),
            ]

        result: list[tuple[str, np.ndarray]] = []
        seen: set[tuple[float, ...]] = set()
        maximum_center_joint_delta_rad: float | None = None
        if isinstance(role, str) and role.startswith("eye_in_hand"):
            # D405 is sampled from this taught center and returns here between
            # captures. A large IK branch switch (typically the wrist folding
            # through a J5/J6 singularity) adds no new camera observability
            # but can leave the loaded wrist short of its target. This filters
            # only those discontinuities; normal wide calibration motion
            # remains allowed.
            maximum_delta_deg = float(
                self.config.get("automatic", {}).get(
                    "d405_max_center_joint_delta_deg", 30.0
                )
            )
            if (
                not math.isfinite(maximum_delta_deg)
                or not 10.0 <= maximum_delta_deg <= 90.0
            ):
                raise RuntimeError(
                    "D405 自动标定单次关节变化必须在 10..90 度"
                )
            maximum_center_joint_delta_rad = math.radians(maximum_delta_deg)
        for label, delta_xyz, rotation in candidates:
            try:
                # Keep the taught center as an actual sample. Pure rotations
                # cannot be represented as a non-zero Cartesian translation,
                # so the remaining candidates are resolved through the same
                # joint endpoint path that automatic collection executes.
                if label in {"table_center", "camera_facing"}:
                    target = np.asarray(center, dtype=np.float64).copy()
                else:
                    state = self.controller.get_state()
                    planned = self._plan_handeye_pose_candidate(
                        Pose(
                            state.eef_pose_6d[:3] + delta_xyz,
                            quaternion_xyzw_from_matrix(rotation),
                        )
                    )
                    target = np.asarray(
                        planned.target_joint_position,
                        dtype=np.float64,
                    )
            except (RuntimeError, OSError, ValueError, SafetyError):
                continue
            # Hard limits, collisions and dynamics are checked by plan_movej.
            # A 1-degree extra rejection was unnecessarily discarding valid
            # D405 tabletop poses near a taught center. Keep the same small
            # configurable buffer as the scoped hand-eye planner there.
            margin_deg = (
                float(
                    self.config.get("automatic", {}).get(
                        "d405_joint_margin_deg", 0.10
                    )
                )
                if isinstance(role, str) and role.startswith("eye_in_hand")
                else 1.0
            )
            if not math.isfinite(margin_deg) or not 0.0 <= margin_deg <= 2.0:
                raise RuntimeError("D405 自动标定关节余量必须在 0..2 度")
            margin = math.radians(margin_deg)
            if np.any(target < joint_minimum + margin) or np.any(
                target > joint_maximum - margin
            ):
                continue
            if (
                maximum_center_joint_delta_rad is not None
                and np.max(np.abs(target - center))
                > maximum_center_joint_delta_rad
            ):
                continue
            key = tuple(np.round(target, 8))
            if key in seen:
                continue
            seen.add(key)
            result.append((label, target.copy()))
        return result

    def _start_feedback_stream(self) -> None:
        self._feedback_stop.clear()
        self._feedback_thread = threading.Thread(
            target=self._feedback_loop,
            name="handeye-feedback",
            daemon=True,
        )
        self._feedback_thread.start()

    def _stop_feedback_stream(self) -> None:
        self._feedback_stop.set()
        thread = self._feedback_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._feedback_thread = None

    def _feedback_loop(self) -> None:
        while not self._feedback_stop.wait(0.25):
            try:
                snapshot = self.controller.get_state()
                payload = {
                    "connected": True,
                    "mode": self.controller.mode.value,
                    "busy": True,
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
                    "gripper_position_mm": 1000.0 * snapshot.gripper_position,
                    "gripper_velocity_m_s": float(snapshot.gripper_velocity),
                    "gripper_torque_nm": float(snapshot.gripper_torque),
                    "eef_pose_6d": [float(value) for value in snapshot.eef_pose_6d],
                }
                self._emit_event({"type": "feedback", "feedback": payload})
            except BaseException:
                return

    def capture_sample(
        self,
        *,
        keep_active: bool = False,
        sample_index: int | None = None,
    ) -> EyeToHandSample | None:
        if self.intrinsics is None:
            raise RuntimeError("collector is not initialized")
        cv2 = _cv2()
        collection = self.config["collection"]
        hold_during_sample = bool(collection.get("hold_during_sample", True))
        held = False
        try:
            if hold_during_sample:
                self.controller.hold_current()
                held = True
                try:
                    self._wait_for_sampling_stability()
                except RuntimeError as exc:
                    print(f"采样拒绝：{exc}。")
                    return None
            before = self.controller.get_state()
            frames_per_burst = int(
                collection.get(
                    "frames_per_burst",
                    collection.get("frames_per_sample", 24),
                )
            )
            burst_count = int(collection.get("bursts_per_pose", 1))
            burst_pause_s = float(collection.get("burst_interval_s", 0.0))
            if frames_per_burst < 3 or burst_count < 1 or burst_pause_s < 0.0:
                raise RuntimeError("标定 burst 采样配置无效")
            burst_transforms: list[np.ndarray] = []
            burst_stats: list[dict[str, float]] = []
            valid_detection_count = 0
            last_frame = None
            last_annotated = None
            stream_thread = self._camera_stream_thread
            if stream_thread is not None and not stream_thread.is_alive():
                raise RuntimeError(
                    f"{self._camera_label} 自动标定视频流已经意外停止"
                ) from self._camera_stream_error
            use_shared_stream = stream_thread is not None
            frame_sequence = self._camera_frame_sequence
            max_reprojection = float(
                collection.get("max_reprojection_error_px", 1.5)
            )
            minimum_valid_per_burst = max(
                int(collection.get("min_valid_frames", 12)),
                math.ceil(
                    frames_per_burst
                    * float(collection.get("min_valid_ratio", 0.50))
                ),
            )
            for burst_index in range(burst_count):
                detections: list[TargetDetection] = []
                for _ in range(frames_per_burst):
                    if use_shared_stream:
                        frame, frame_sequence = self._next_automatic_frame(
                            frame_sequence
                        )
                    else:
                        frame = self.camera.read()
                    last_frame = frame
                    detection = self.detector.detect(frame, self.intrinsics)
                    if detection is None:
                        continue
                    if detection.reprojection_error_px <= max_reprojection:
                        detections.append(detection)
                        last_annotated = detection.annotated
                if len(detections) < minimum_valid_per_burst:
                    print(
                        "采样拒绝：第 "
                        f"{burst_index + 1}/{burst_count} 个 burst 有效标定板帧 "
                        f"{len(detections)}/{frames_per_burst}，至少需要 "
                        f"{minimum_valid_per_burst}。"
                    )
                    return None
                transform, stats = _average_detections(detections)
                if stats["translation_std_m"] > float(
                    collection.get("max_translation_std_m", 0.0015)
                ):
                    print("采样拒绝：标定板平移检测抖动过大。")
                    return None
                if stats["rotation_std_deg"] > float(
                    collection.get("max_rotation_std_deg", 0.35)
                ):
                    print("采样拒绝：标定板姿态检测抖动过大。")
                    return None
                burst_transforms.append(transform)
                burst_stats.append(stats)
                valid_detection_count += len(detections)
                if burst_index + 1 < burst_count and burst_pause_s > 0.0:
                    time.sleep(burst_pause_s)
            after = self.controller.get_state()
            if not self._robot_was_stable(before, after):
                return None
            T_camera_target = average_transforms(burst_transforms)
            burst_translation_errors = np.asarray(
                [
                    np.linalg.norm(transform[:3, 3] - T_camera_target[:3, 3])
                    for transform in burst_transforms
                ],
                dtype=np.float64,
            )
            burst_rotation_errors = np.asarray(
                [
                    rotation_distance_rad(
                        transform[:3, :3], T_camera_target[:3, :3]
                    )
                    for transform in burst_transforms
                ],
                dtype=np.float64,
            )
            stats = {
                "reprojection_error_px": float(
                    np.mean([value["reprojection_error_px"] for value in burst_stats])
                ),
                "translation_std_m": float(
                    np.sqrt(
                        np.mean(
                            [value["translation_std_m"] ** 2 for value in burst_stats]
                        )
                    )
                ),
                "rotation_std_deg": float(
                    np.sqrt(
                        np.mean(
                            [value["rotation_std_deg"] ** 2 for value in burst_stats]
                        )
                    )
                ),
                "burst_translation_std_m": float(
                    np.sqrt(np.mean(burst_translation_errors**2))
                ),
                "burst_rotation_std_deg": math.degrees(
                    float(np.sqrt(np.mean(burst_rotation_errors**2)))
                ),
            }

            index = len(self.samples) + 1 if sample_index is None else int(sample_index)
            if index < 1:
                raise ValueError("sample index must be positive")
            image_name = f"{self._sample_image_prefix}sample_{index:03d}.png"
            annotated_name = (
                f"{self._sample_image_prefix}sample_{index:03d}_annotated.png"
            )
            if last_frame is not None:
                cv2.imwrite(str(self.session.images / image_name), last_frame)
            if last_annotated is not None:
                cv2.imwrite(
                    str(self.session.images / annotated_name), last_annotated
                )
            T_base_eef = average_transforms(
                [
                    pose6_to_transform(before.eef_pose_6d),
                    pose6_to_transform(after.eef_pose_6d),
                ]
            )
            return EyeToHandSample(
                index=index,
                timestamp=time.time(),
                T_base_eef=T_base_eef,
                T_camera_target=T_camera_target,
                joint_position_rad=(
                    np.asarray(before.joint_position)
                    + np.asarray(after.joint_position)
                )
                / 2.0,
                reprojection_error_px=stats["reprojection_error_px"],
                translation_std_m=stats["translation_std_m"],
                rotation_std_deg=stats["rotation_std_deg"],
                valid_frames=valid_detection_count,
                total_frames=frames_per_burst * burst_count,
                image=str(Path("images") / image_name),
                annotated_image=str(Path("images") / annotated_name),
                burst_count=burst_count,
                burst_translation_std_m=stats["burst_translation_std_m"],
                burst_rotation_std_deg=stats["burst_rotation_std_deg"],
            )
        finally:
            if held and not keep_active and self.controller.mode == ControllerMode.ACTIVE:
                self._idle_after_motion()

    def _wait_for_sampling_stability(self) -> None:
        """Wait for a quiet dwell using velocity and measured pose stability.

        X5 CAN feedback reports velocity in coarse steps (about 0.011 rad/s).
        An actively held, stationary arm can therefore intermittently report
        0.033--0.055 rad/s even when its joint positions do not move.  Treat
        those isolated readings as telemetry jitter only after a full dwell
        proves the joint positions are stable.  The subsequent sample-window
        position / EEF checks remain the authority for calibration quality.
        """
        collection = self.config["collection"]
        velocity_limit = float(collection.get("max_joint_velocity_rad_s", 0.025))
        dwell_s = float(collection.get("hold_settle_s", 0.5))
        timeout_s = float(collection.get("settle_timeout_s", 8.0))
        poll_s = float(collection.get("settle_poll_s", 0.10))
        transient_velocity_limit = float(
            collection.get(
                "max_transient_joint_velocity_rad_s",
                max(0.060, 2.0 * velocity_limit),
            )
        )
        settle_drift_deg = float(
            collection.get(
                "max_settle_joint_drift_deg",
                collection.get("max_joint_drift_deg", 0.15),
            )
        )
        if not all(math.isfinite(value) and value > 0.0 for value in (
            velocity_limit,
            dwell_s,
            timeout_s,
            poll_s,
            transient_velocity_limit,
            settle_drift_deg,
        )):
            raise RuntimeError("采样稳定等待配置必须为正的有限数值")
        if transient_velocity_limit < velocity_limit:
            raise RuntimeError("采样瞬时速度阈值不能小于稳定速度阈值")
        deadline = time.monotonic() + timeout_s
        peak_velocity = 0.0
        observations: deque[tuple[float, np.ndarray, float]] = deque()
        last_velocity = math.inf
        last_median_velocity = math.inf
        last_joint_drift = math.inf
        while True:
            snapshot = self.controller.get_state()
            velocity = float(np.max(np.abs(snapshot.joint_velocity)))
            peak_velocity = max(peak_velocity, velocity)
            now = time.monotonic()
            observations.append(
                (now, np.asarray(snapshot.joint_position, dtype=np.float64), velocity)
            )
            # Retain one sample just before the dwell boundary.  Dropping all
            # samples older than exactly ``dwell_s`` would make the remaining
            # window perpetually shorter than the required observation time.
            while (
                len(observations) > 1
                and now - observations[1][0] >= dwell_s
            ):
                observations.popleft()
            if observations and now - observations[0][0] >= dwell_s:
                joint_window = np.asarray(
                    [position for _, position, _ in observations], dtype=np.float64
                )
                last_joint_drift = float(
                    np.max(np.ptp(joint_window, axis=0))
                )
                reported = np.asarray(
                    [reported_velocity for _, _, reported_velocity in observations],
                    dtype=np.float64,
                )
                last_median_velocity = float(np.median(reported))
                if (
                    last_median_velocity <= velocity_limit
                    and float(np.max(reported)) <= transient_velocity_limit
                    and last_joint_drift <= math.radians(settle_drift_deg)
                ):
                    return
            last_velocity = velocity
            if now >= deadline:
                raise RuntimeError(
                    "机械臂未在采样前稳定："
                    f"当前 {last_velocity:.3f} rad/s，"
                    f"中位 {last_median_velocity:.3f} rad/s（阈值 "
                    f"{velocity_limit:.3f} rad/s，观察峰值 "
                    f"{peak_velocity:.3f} rad/s），"
                    f"关节观察漂移 {math.degrees(last_joint_drift):.3f}°"
                )
            time.sleep(min(poll_s, max(0.0, deadline - now)))

    def _robot_was_stable(self, before, after) -> bool:
        collection = self.config["collection"]
        velocity_limit = float(collection.get("max_joint_velocity_rad_s", 0.025))
        transient_velocity_limit = float(
            collection.get(
                "max_transient_joint_velocity_rad_s",
                max(0.060, 2.0 * velocity_limit),
            )
        )
        velocity = max(
            float(np.max(np.abs(before.joint_velocity))),
            float(np.max(np.abs(after.joint_velocity))),
        )
        joint_drift = float(
            np.max(np.abs(after.joint_position - before.joint_position))
        )
        joint_limit = math.radians(
            float(collection.get("max_joint_drift_deg", 0.15))
        )
        first = pose6_to_transform(before.eef_pose_6d)
        second = pose6_to_transform(after.eef_pose_6d)
        translation, rotation = transform_error(first, second)
        if joint_drift > joint_limit:
            print("采样拒绝：采样窗口内关节角发生变化。")
            return False
        if translation > float(collection.get("max_eef_drift_m", 0.001)):
            print("采样拒绝：采样窗口内 EEF 平移超过 1 mm。")
            return False
        if math.degrees(rotation) > float(
            collection.get("max_eef_rotation_drift_deg", 0.20)
        ):
            print("采样拒绝：采样窗口内 EEF 姿态发生变化。")
            return False
        # Velocity feedback is quantized and can pulse above the nominal
        # threshold while the encoder positions and EEF stay inside the much
        # tighter calibration tolerances above.  Reject sustained / large
        # readings, but do not discard an otherwise static high-quality frame
        # solely for a known CAN telemetry spike.
        if velocity > transient_velocity_limit:
            print(
                f"采样拒绝：关节报告速度异常 ({velocity:.3f} rad/s > "
                f"{transient_velocity_limit:.3f} rad/s)。"
            )
            return False
        return True

    def _persist_samples(self) -> None:
        if self.intrinsics is None:
            raise RuntimeError("collector is not initialized")
        if self._dual_mode:
            self.datasets[self._sample_role] = self.samples
            self.session.write_dual_samples(
                self.datasets,
                config=self.config,
                cameras=self.camera_intrinsics,
                validation_datasets=self.validation_datasets,
                operator_reviews=self.operator_reviews,
            )
            return
        self.session.write_samples(
            self.samples,
            config=self.config,
            intrinsics=self.intrinsics,
        )

    def _draw_status(
        self,
        image: np.ndarray,
        detection: TargetDetection | None,
    ) -> None:
        cv2 = _cv2()
        status = (
            "TARGET OK"
            if detection is not None
            else "MOVE ARUCO TARGET INTO VIEW"
        )
        color = (95, 230, 135) if detection is not None else (80, 175, 255)
        cv2.rectangle(image, (12, 12), (620, 86), (18, 22, 24), -1)
        cv2.putText(
            image,
            f"{status} | samples {len(self.samples)}",
            (26, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            color,
            2,
            cv2.LINE_AA,
        )
        detail = "SPACE/S sample  D delete  Q finish"
        if detection is not None:
            detail += f"  reproj {detection.reprojection_error_px:.2f}px"
        cv2.putText(
            image,
            detail,
            (26, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (220, 224, 226),
            1,
            cv2.LINE_AA,
        )

    def _emit_event(self, event: dict[str, Any]) -> None:
        if self._on_event is not None:
            self._on_event(event)
            return
        # CLI mode has no web event consumer; preserve its line protocol.
        if event.get("type") == "preview":
            print("HANDEYE_PREVIEW " + json.dumps(event["preview"], ensure_ascii=False, separators=(",", ":")), flush=True)
        elif event.get("type") == "feedback":
            print("HANDEYE_FEEDBACK " + json.dumps(event["feedback"], ensure_ascii=False, separators=(",", ":")), flush=True)

    def _notify_stage(self, stage: str) -> None:
        if self._on_stage is not None:
            self._on_stage(stage)

    def _require_stage_input(self, expected: str) -> None:
        self._notify_stage(expected.lower())
        command = self._require_stage_command()
        if command != expected:
            raise RuntimeError(f"unexpected hand-eye command: {command}")

    def _require_stage_command(self) -> str:
        if self._commands is None:
            value = sys.stdin.readline()
        else:
            value = self._commands.get()
        command = value.strip().upper()
        if not value or command == "CANCEL":
            raise HandEyeCancelled
        if command == "HOME":
            raise HandEyeReturnHome
        if command == "EMERGENCY":
            raise HandEyeEmergency
        return command

    def _poll_stage_input(self) -> str | None:
        if self._commands is None:
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            except (OSError, ValueError):
                return None
            if not ready:
                return None
            value = sys.stdin.readline()
        else:
            try:
                value = self._commands.get_nowait()
            except queue.Empty:
                return None
        command = value.strip().upper()
        if not value or command == "CANCEL":
            raise HandEyeCancelled
        if command == "HOME":
            raise HandEyeReturnHome
        if command == "EMERGENCY":
            raise HandEyeEmergency
        return command

    def _raise_if_cancel_requested(self) -> None:
        command = self._poll_stage_input()
        if command is not None:
            raise RuntimeError(f"unexpected hand-eye command: {command}")


def solve_and_store_session(session: CalibrationSession) -> dict[str, Any]:
    try:
        with session.samples_path.open("r", encoding="utf-8") as stream:
            calibration_type = json.load(stream).get("calibration_type")
    except (OSError, json.JSONDecodeError, AttributeError):
        calibration_type = None
    if calibration_type == "dual_camera_hand_eye":
        return solve_and_store_dual_session(session)
    samples, payload = session.read_samples()
    config = payload["config"]
    result = solve_eye_to_hand(samples, quality=config.get("quality"))
    result["session"] = str(session.path)
    result["camera"] = payload["camera"]
    result["target"] = config["target"]
    result["sample_metrics"] = [
        {
            "index": sample.index,
            "reprojection_error_px": sample.reprojection_error_px,
            "translation_std_m": sample.translation_std_m,
            "rotation_std_deg": sample.rotation_std_deg,
            "burst_count": sample.burst_count,
            "burst_translation_std_m": sample.burst_translation_std_m,
            "burst_rotation_std_deg": sample.burst_rotation_std_deg,
        }
        for sample in samples
    ]
    output_root = Path(config["collection"]["output_root"])
    session.write_result(result, cache_root=output_root)
    residual = result["residual"]
    stability = result["leave_one_out_stability"]
    print(f"标定完成：{session.result_path}")
    print(
        f"方法 {result['best_method']}，闭环 RMS "
        f"{residual['closure_translation_rms_m'] * 1000.0:.2f} mm / "
        f"{residual['closure_rotation_rms_deg']:.3f} deg"
    )
    print(
        f"留一法外参稳定度 RMS "
        f"{stability['translation_rms_m'] * 1000.0:.2f} mm / "
        f"{stability['rotation_rms_deg']:.3f} deg"
    )
    return result


def _sample_metrics(samples: list[EyeToHandSample]) -> list[dict[str, Any]]:
    return [
        {
            "index": sample.index,
            "reprojection_error_px": sample.reprojection_error_px,
            "translation_std_m": sample.translation_std_m,
            "rotation_std_deg": sample.rotation_std_deg,
            "burst_count": sample.burst_count,
            "burst_translation_std_m": sample.burst_translation_std_m,
            "burst_rotation_std_deg": sample.burst_rotation_std_deg,
        }
        for sample in samples
    ]


def _solve_dual_phase(
    *,
    role: str,
    samples: list[EyeToHandSample],
    validation_samples: list[EyeToHandSample],
    payload: dict[str, Any],
) -> dict[str, Any]:
    """Solve and annotate a single camera dataset from a dual session."""
    if role == "eye_in_hand":
        result = solve_eye_in_hand(samples, quality=payload["config"].get("quality"))
    elif role == "eye_to_hand":
        result = solve_eye_to_hand(samples, quality=payload["config"].get("quality"))
    else:
        raise ValueError(f"unknown hand-eye camera role: {role}")
    cameras = payload.get("cameras", {})
    reviews = payload.get("operator_reviews", {})
    result["calibration_role"] = role
    result["camera"] = cameras.get(role) if isinstance(cameras, dict) else None
    result["target"] = payload["config"]["target"]
    result["sample_metrics"] = _sample_metrics(samples)
    result["independent_validation"] = (
        _evaluate_independent_validation(validation_samples, result)
        if validation_samples
        else None
    )
    result["independent_validation_sample_metrics"] = _sample_metrics(
        validation_samples
    )
    result["operator_review"] = (
        dict(reviews[role])
        if isinstance(reviews, dict) and isinstance(reviews.get(role), dict)
        else None
    )
    return result


def _cached_phase_result(
    cache_root: Path,
    role: str,
) -> dict[str, Any] | None:
    """Read a phase cache, with one-time compatibility for old current.json."""
    cached = read_cached_result(cache_root, phase_cache_filename(role))
    expected_type = role
    if isinstance(cached, dict) and cached.get("calibration_type") == expected_type:
        return cached
    current = read_cached_result(cache_root, "current.json")
    if not isinstance(current, dict):
        return None
    if current.get("calibration_type") == "dual_camera_hand_eye":
        phase = current.get(role)
        if isinstance(phase, dict) and phase.get("calibration_type") == expected_type:
            phase = dict(phase)
            cameras = current.get("cameras", {})
            if phase.get("camera") is None and isinstance(cameras, dict):
                phase["camera"] = cameras.get(role)
            phase.setdefault("target", current.get("target"))
            phase.setdefault("session", current.get("session"))
            phase.setdefault("calibration_role", role)
            # Materialize the compatibility source so subsequent independent
            # calibrations no longer rely on a legacy combined cache.
            write_cached_result(cache_root, phase_cache_filename(role), phase)
            return phase
    if current.get("calibration_type") == expected_type:
        write_cached_result(cache_root, phase_cache_filename(role), current)
        return current
    return None


def _combine_phase_results(
    eye_in_hand: dict[str, Any],
    eye_to_hand: dict[str, Any],
    *,
    updated_role: str,
) -> dict[str, Any]:
    """Create the existing dual-profile schema from independently saved fits."""
    hand_camera = eye_in_hand.get("camera")
    external_camera = eye_to_hand.get("camera")
    return {
        "schema_version": 3,
        # The combined cache changes whenever either independently saved fit
        # changes, so validation/enabling correctly becomes stale.
        "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "calibration_type": "dual_camera_hand_eye",
        "sample_count": min(
            int(eye_in_hand["sample_count"]),
            int(eye_to_hand["sample_count"]),
        ),
        "sample_counts": {
            "eye_in_hand": eye_in_hand["sample_count"],
            "eye_to_hand": eye_to_hand["sample_count"],
        },
        "best_method": {
            "eye_in_hand": eye_in_hand["best_method"],
            "eye_to_hand": eye_to_hand["best_method"],
        },
        "T_eef_camera": eye_in_hand["T_eef_camera"],
        "T_camera_eef": eye_in_hand["T_camera_eef"],
        "T_base_camera": eye_to_hand["T_base_camera"],
        "T_camera_base": eye_to_hand["T_camera_base"],
        # Backward-compatible external-camera quality fields.
        "residual": eye_to_hand["residual"],
        "leave_one_out_stability": eye_to_hand["leave_one_out_stability"],
        "bootstrap_stability": eye_to_hand.get("bootstrap_stability", {}),
        "eye_in_hand": eye_in_hand,
        "eye_to_hand": eye_to_hand,
        "independent_validation": {
            "eye_in_hand": eye_in_hand.get("independent_validation"),
            "eye_to_hand": eye_to_hand.get("independent_validation"),
        },
        "operator_reviews": {
            role: phase["operator_review"]
            for role, phase in (
                ("eye_in_hand", eye_in_hand),
                ("eye_to_hand", eye_to_hand),
            )
            if isinstance(phase.get("operator_review"), dict)
        },
        "validation_sample_counts": {
            "eye_in_hand": len(
                eye_in_hand.get("independent_validation_sample_metrics", [])
            ),
            "eye_to_hand": len(
                eye_to_hand.get("independent_validation_sample_metrics", [])
            ),
        },
        "cameras": {
            role: camera
            for role, camera in (
                ("eye_in_hand", hand_camera),
                ("eye_to_hand", external_camera),
            )
            if isinstance(camera, dict)
        },
        "camera": external_camera,
        "target": eye_to_hand.get("target") or eye_in_hand.get("target"),
        "session": (
            eye_in_hand if updated_role == "eye_in_hand" else eye_to_hand
        ).get("session"),
        "phase_sessions": {
            "eye_in_hand": eye_in_hand.get("session"),
            "eye_to_hand": eye_to_hand.get("session"),
        },
    }


def solve_and_store_dual_phase_session(
    session: CalibrationSession,
    *,
    role: str,
) -> dict[str, Any]:
    """Persist a standalone camera solve and refresh the combined cache if able."""
    if role not in {"eye_in_hand", "eye_to_hand"}:
        raise ValueError(f"unknown hand-eye camera role: {role}")
    datasets, payload = session.read_dual_samples()
    raw_validation = payload.get("validation_datasets", {})
    validation_samples = (
        [
            EyeToHandSample.from_dict(value)
            for value in raw_validation.get(role, [])
        ]
        if isinstance(raw_validation, dict)
        else []
    )
    result = _solve_dual_phase(
        role=role,
        samples=datasets[role],
        validation_samples=validation_samples,
        payload=payload,
    )
    result["session"] = str(session.path)
    output_root = Path(payload["config"]["collection"]["output_root"])
    session.write_phase_result(result, cache_root=output_root, role=role)

    other_role = "eye_to_hand" if role == "eye_in_hand" else "eye_in_hand"
    other_result = _cached_phase_result(output_root, other_role)
    if other_result is not None:
        hand = result if role == "eye_in_hand" else other_result
        external = result if role == "eye_to_hand" else other_result
        combined = _combine_phase_results(
            hand,
            external,
            updated_role=role,
        )
        write_cached_result(output_root, "current.json", combined)
        result["combined_profile_updated"] = True
        result["combined_profile_created_at"] = combined["created_at"]
    elif role == "eye_to_hand":
        # D455 remains useful by itself for the existing cube/world workflow.
        # Keep its legacy single-camera activation path until D405 is saved.
        write_cached_result(output_root, "current.json", result)
        result["combined_profile_updated"] = False
        result["standalone_profile_updated"] = True
    else:
        result["combined_profile_updated"] = False
        result["standalone_profile_updated"] = False

    label = "眼在手 D405" if role == "eye_in_hand" else "外部 D455"
    residual = result["residual"]
    print(f"{label} 独立标定完成：{session.result_path}", flush=True)
    print(
        f"{label}：方法 {result['best_method']}，闭环 RMS "
        f"{residual['closure_translation_rms_m'] * 1000.0:.2f} mm / "
        f"{residual['closure_rotation_rms_deg']:.3f} deg",
        flush=True,
    )
    return result


def solve_and_store_dual_session(session: CalibrationSession) -> dict[str, Any]:
    datasets, payload = session.read_dual_samples()
    config = payload["config"]
    quality = config.get("quality")
    eye_in_hand = solve_eye_in_hand(datasets["eye_in_hand"], quality=quality)
    eye_to_hand = solve_eye_to_hand(datasets["eye_to_hand"], quality=quality)
    raw_validation = payload.get("validation_datasets", {})
    validation_datasets = {
        role: [
            EyeToHandSample.from_dict(value)
            for value in raw_validation.get(role, [])
        ]
        if isinstance(raw_validation, dict)
        else []
        for role in ("eye_in_hand", "eye_to_hand")
    }
    raw_reviews = payload.get("operator_reviews", {})
    operator_reviews = {
        role: dict(raw_reviews[role])
        for role in ("eye_in_hand", "eye_to_hand")
        if isinstance(raw_reviews, dict) and isinstance(raw_reviews.get(role), dict)
    }
    independent_validation: dict[str, dict[str, float | int] | None] = {}
    for role, phase in (
        ("eye_in_hand", eye_in_hand),
        ("eye_to_hand", eye_to_hand),
    ):
        phase["sample_metrics"] = [
            {
                "index": sample.index,
                "reprojection_error_px": sample.reprojection_error_px,
                "translation_std_m": sample.translation_std_m,
                "rotation_std_deg": sample.rotation_std_deg,
                "burst_count": sample.burst_count,
                "burst_translation_std_m": sample.burst_translation_std_m,
                "burst_rotation_std_deg": sample.burst_rotation_std_deg,
            }
            for sample in datasets[role]
        ]
        held_out = validation_datasets[role]
        metrics = (
            _evaluate_independent_validation(held_out, phase)
            if held_out
            else None
        )
        independent_validation[role] = metrics
        phase["independent_validation"] = metrics
        phase["operator_review"] = operator_reviews.get(role)
        phase["independent_validation_sample_metrics"] = [
            {
                "index": sample.index,
                "reprojection_error_px": sample.reprojection_error_px,
                "translation_std_m": sample.translation_std_m,
                "rotation_std_deg": sample.rotation_std_deg,
                "burst_count": sample.burst_count,
                "burst_translation_std_m": sample.burst_translation_std_m,
                "burst_rotation_std_deg": sample.burst_rotation_std_deg,
            }
            for sample in held_out
        ]
        # Preserve enough context for a later independent recalibration of
        # the other camera to rebuild the combined profile from this phase.
        phase["calibration_role"] = role
        phase["camera"] = (
            payload.get("cameras", {}).get(role)
            if isinstance(payload.get("cameras"), dict)
            else None
        )
        phase["target"] = config["target"]
        phase["session"] = str(session.path)
    result = {
        "schema_version": 3,
        "created_at": eye_to_hand["created_at"],
        "calibration_type": "dual_camera_hand_eye",
        "sample_count": min(
            int(eye_in_hand["sample_count"]),
            int(eye_to_hand["sample_count"]),
        ),
        "sample_counts": {
            "eye_in_hand": eye_in_hand["sample_count"],
            "eye_to_hand": eye_to_hand["sample_count"],
        },
        "best_method": {
            "eye_in_hand": eye_in_hand["best_method"],
            "eye_to_hand": eye_to_hand["best_method"],
        },
        "T_eef_camera": eye_in_hand["T_eef_camera"],
        "T_camera_eef": eye_in_hand["T_camera_eef"],
        "T_base_camera": eye_to_hand["T_base_camera"],
        "T_camera_base": eye_to_hand["T_camera_base"],
        # Backward-compatible external-camera quality fields.
        "residual": eye_to_hand["residual"],
        "leave_one_out_stability": eye_to_hand["leave_one_out_stability"],
        "bootstrap_stability": eye_to_hand["bootstrap_stability"],
        "eye_in_hand": eye_in_hand,
        "eye_to_hand": eye_to_hand,
        "independent_validation": independent_validation,
        "operator_reviews": operator_reviews,
        "validation_sample_counts": {
            role: len(values) for role, values in validation_datasets.items()
        },
        "cameras": payload.get("cameras", {}),
        "camera": payload.get("cameras", {}).get("eye_to_hand"),
        "target": config["target"],
        "session": str(session.path),
    }
    output_root = Path(config["collection"]["output_root"])
    session.write_result(result, cache_root=output_root)
    # Keep per-camera artifacts alongside the combined profile.  A later
    # D405-only or D455-only recalibration can now replace just its own fit.
    write_cached_result(
        output_root,
        phase_cache_filename("eye_in_hand"),
        eye_in_hand,
    )
    write_cached_result(
        output_root,
        phase_cache_filename("eye_to_hand"),
        eye_to_hand,
    )
    print(f"双相机标定完成：{session.result_path}")
    for role, label in (
        ("eye_in_hand", "眼在手 D405"),
        ("eye_to_hand", "外部 D455"),
    ):
        phase = result[role]
        residual = phase["residual"]
        print(
            f"{label}：方法 {phase['best_method']}，闭环 RMS "
            f"{residual['closure_translation_rms_m'] * 1000.0:.2f} mm / "
            f"{residual['closure_rotation_rms_deg']:.3f} deg",
            flush=True,
        )
        held_out = phase["independent_validation"]
        if held_out is not None:
            print(
                f"{label} 独立验证：闭环 RMS "
                f"{held_out['closure_translation_rms_m'] * 1000.0:.2f} mm / "
                f"{held_out['closure_rotation_rms_deg']:.3f} deg，P95 "
                f"{held_out['closure_translation_p95_m'] * 1000.0:.2f} mm / "
                f"{held_out['closure_rotation_p95_deg']:.3f} deg",
                flush=True,
            )
    return result


def build_automatic_pose_plan(
    center: np.ndarray,
    joint_minimum: np.ndarray,
    joint_maximum: np.ndarray,
) -> list[tuple[str, np.ndarray]]:
    center = np.asarray(center, dtype=np.float64)
    joint_minimum = np.asarray(joint_minimum, dtype=np.float64)
    joint_maximum = np.asarray(joint_maximum, dtype=np.float64)
    if any(value.shape != (6,) for value in (center, joint_minimum, joint_maximum)):
        raise RuntimeError("自动标定姿态必须包含六个关节角")
    if not all(np.all(np.isfinite(value)) for value in (center, joint_minimum, joint_maximum)):
        raise RuntimeError("自动标定姿态包含无效数值")
    margin = math.radians(3.0)
    if np.any(center < joint_minimum + margin) or np.any(
        center > joint_maximum - margin
    ):
        raise RuntimeError("当前安全中心离关节限位不足 3°")

    def delta(**degrees: float) -> np.ndarray:
        value = np.zeros(6, dtype=np.float64)
        for name, amount in degrees.items():
            value[int(name.removeprefix("j")) - 1] = math.radians(amount)
        return value

    variants = [
        ("j4_plus", delta(j4=14.0)),
        ("j4_minus", delta(j4=-14.0)),
        ("j5_plus", delta(j5=14.0)),
        ("j5_minus", delta(j5=-14.0)),
        ("j6_plus", delta(j6=18.0)),
        ("j6_minus", delta(j6=-18.0)),
        ("diag_j2_j4_plus", delta(j2=5.0, j4=10.0)),
        ("diag_j2_j4_minus", delta(j2=-5.0, j4=-10.0)),
        ("diag_j3_j5_plus", delta(j3=5.0, j5=-10.0)),
        ("diag_j3_j5_minus", delta(j3=-5.0, j5=10.0)),
        ("center", delta()),
        ("j2_plus", delta(j2=8.0)),
        ("j2_minus", delta(j2=-8.0)),
        ("j3_plus", delta(j3=8.0)),
        ("j3_minus", delta(j3=-8.0)),
        ("j1_plus", delta(j1=6.0)),
        ("j1_minus", delta(j1=-6.0)),
    ]
    result: list[tuple[str, np.ndarray]] = []
    for label, offset in variants:
        target = center + offset
        if np.any(target < joint_minimum + margin) or np.any(
            target > joint_maximum - margin
        ):
            continue
        result.append((label, target))
    return result


def resolve_automatic_gripper_widths(
    settings: dict[str, Any],
    maximum_width_m: float,
) -> tuple[float, float, float]:
    try:
        open_width = float(settings["open_width_m"])
        pregrasp_width = float(settings["pregrasp_width_m"])
        minimum_width = float(settings["minimum_width_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("自动标定夹爪开度配置无效") from exc
    values = (minimum_width, pregrasp_width, open_width, maximum_width_m)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("自动标定夹爪开度必须是有限数值")
    if not 0.0 <= minimum_width < pregrasp_width < open_width <= maximum_width_m:
        raise RuntimeError(
            "自动标定夹爪开度需要满足：最小开度 < 预夹开度 < 装板开度 < 机械限位"
        )
    return open_width, pregrasp_width, minimum_width


def build_automatic_preview_guidance(
    detection: TargetDetection | None,
    context: str,
) -> dict[str, Any]:
    visible = detection is not None
    result: dict[str, Any] = {
        "stage": "collecting",
        "visible": visible,
        "ready": visible,
        "message": f"{context} · {'标定板可见' if visible else '未识别到标定板'}",
        "distance_m": None,
        "tilt_deg": None,
        "reprojection_error_px": None,
        "j4_margin_deg": None,
        "target_pose_camera": None,
        "target_position_camera_m": None,
    }
    if detection is None:
        return result
    transform = np.asarray(detection.T_camera_target, dtype=np.float64)
    if transform.shape == (4, 4) and np.all(np.isfinite(transform)):
        result["distance_m"] = float(transform[2, 3])
        result["target_pose_camera"] = transform.tolist()
        result["target_position_camera_m"] = [
            float(value) for value in transform[:3, 3]
        ]
        result["tilt_deg"] = math.degrees(
            math.acos(float(np.clip(abs(transform[2, 2]), 0.0, 1.0)))
        )
    result["reprojection_error_px"] = float(
        detection.reprojection_error_px
    )
    return result


def build_center_guidance(
    detection: TargetDetection | None,
    intrinsics: CameraIntrinsics,
    joint_position: np.ndarray,
    joint_minimum: np.ndarray,
    joint_maximum: np.ndarray,
    *,
    j4_min_margin_deg: float = 8.0,
    min_depth_m: float = 0.25,
    max_depth_m: float = 1.50,
    image_center_bound: float = 0.36,
    max_tilt_deg: float = 55.0,
) -> dict[str, Any]:
    limits = {
        "j4_min_margin_deg": float(j4_min_margin_deg),
        "min_depth_m": float(min_depth_m),
        "max_depth_m": float(max_depth_m),
        "image_center_bound": float(image_center_bound),
        "max_tilt_deg": float(max_tilt_deg),
    }
    if not all(math.isfinite(value) for value in limits.values()):
        raise RuntimeError("center guidance limits must be finite")
    if not 0.0 <= limits["j4_min_margin_deg"] <= 30.0:
        raise RuntimeError("center guidance J4 margin is invalid")
    if not 0.05 <= limits["min_depth_m"] < limits["max_depth_m"]:
        raise RuntimeError("center guidance depth limits are invalid")
    if not 0.05 <= limits["image_center_bound"] < 0.50:
        raise RuntimeError("center guidance image bound is invalid")
    if not 1.0 <= limits["max_tilt_deg"] <= 89.0:
        raise RuntimeError("center guidance tilt limit is invalid")
    joints = np.asarray(joint_position, dtype=np.float64)
    minimum = np.asarray(joint_minimum, dtype=np.float64)
    maximum = np.asarray(joint_maximum, dtype=np.float64)
    if any(value.shape != (6,) for value in (joints, minimum, maximum)):
        raise RuntimeError("center guidance requires six joint values")
    j4_margin_deg = math.degrees(
        min(joints[3] - minimum[3], maximum[3] - joints[3])
    )
    result: dict[str, Any] = {
        "visible": detection is not None,
        "ready": False,
        "message": "D455 未识别到 ID 0 标定板，请转动或移动标定板",
        "marker_center_norm": None,
        "distance_m": None,
        "tilt_deg": None,
        "reprojection_error_px": None,
        "j4_margin_deg": j4_margin_deg,
        "target_pose_camera": None,
        "target_position_camera_m": None,
    }
    if detection is None:
        return result

    transform = np.asarray(detection.T_camera_target, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        result["message"] = "标定板位姿无效，请调整板面角度"
        return result
    translation = transform[:3, 3]
    depth = float(translation[2])
    if depth <= 1e-6:
        result["message"] = "标定板不在相机前方，请调整板面方向"
        return result

    camera_matrix = np.asarray(intrinsics.camera_matrix, dtype=np.float64)
    pixel_x = float(camera_matrix[0, 0] * translation[0] / depth + camera_matrix[0, 2])
    pixel_y = float(camera_matrix[1, 1] * translation[1] / depth + camera_matrix[1, 2])
    center_x = 2.0 * (pixel_x / float(intrinsics.width) - 0.5)
    center_y = 2.0 * (pixel_y / float(intrinsics.height) - 0.5)
    tilt_deg = math.degrees(
        math.acos(float(np.clip(abs(transform[2, 2]), 0.0, 1.0)))
    )
    result.update(
        {
            "marker_center_norm": [center_x, center_y],
            "distance_m": depth,
            "tilt_deg": tilt_deg,
            "reprojection_error_px": float(detection.reprojection_error_px),
            "target_pose_camera": transform.tolist(),
            "target_position_camera_m": [float(value) for value in translation],
        }
    )

    if j4_margin_deg < limits["j4_min_margin_deg"]:
        direction = "正" if joints[3] > 0.0 else "负"
        result["message"] = f"J4 靠近{direction}限位，请将 J4 向中间角度调整"
    elif depth < limits["min_depth_m"]:
        result["message"] = "标定板距离 D455 太近，请稍微远离相机"
    elif depth > limits["max_depth_m"]:
        result["message"] = "标定板距离 D455 太远，请稍微靠近相机"
    elif center_x < -limits["image_center_bound"]:
        result["message"] = "标定板偏左，请向画面右侧移动"
    elif center_x > limits["image_center_bound"]:
        result["message"] = "标定板偏右，请向画面左侧移动"
    elif center_y < -limits["image_center_bound"]:
        result["message"] = "标定板偏上，请向画面下方移动"
    elif center_y > limits["image_center_bound"]:
        result["message"] = "标定板偏下，请向画面上方移动"
    elif tilt_deg > limits["max_tilt_deg"]:
        result["message"] = "标定板倾斜过大，请让板面更正对 D455"
    else:
        result["ready"] = True
        result["message"] = "标定板可见且 J4 余量合适，可以确认安全中心"
    return result


def _draw_center_guidance(frame: np.ndarray, guidance: dict[str, Any]) -> None:
    cv2 = _cv2()
    height, width = frame.shape[:2]
    color = (88, 220, 150) if guidance["ready"] else (70, 190, 245)
    cv2.rectangle(
        frame,
        (int(width * 0.32), int(height * 0.32)),
        (int(width * 0.68), int(height * 0.68)),
        color,
        2,
    )
    cv2.drawMarker(
        frame,
        (width // 2, height // 2),
        color,
        markerType=cv2.MARKER_CROSS,
        markerSize=max(18, min(width, height) // 18),
        thickness=2,
    )
    center = guidance.get("marker_center_norm")
    if isinstance(center, list) and len(center) == 2:
        pixel = (
            int(round((float(center[0]) * 0.5 + 0.5) * width)),
            int(round((float(center[1]) * 0.5 + 0.5) * height)),
        )
        cv2.circle(frame, pixel, max(7, min(width, height) // 60), color, 2)
        cv2.line(frame, pixel, (width // 2, height // 2), color, 1)
    cv2.putText(
        frame,
        "CENTER READY" if guidance["ready"] else "ADJUST CENTER",
        (14, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        color,
        2,
        cv2.LINE_AA,
    )


def _draw_automatic_guidance(
    frame: np.ndarray,
    guidance: dict[str, Any],
) -> None:
    cv2 = _cv2()
    visible = bool(guidance.get("visible"))
    color = (88, 220, 150) if visible else (70, 190, 245)
    label = "TARGET VISIBLE" if visible else "TARGET NOT DETECTED"
    cv2.rectangle(frame, (10, 10), (330, 48), (18, 22, 24), -1)
    cv2.putText(
        frame,
        label,
        (22, 37),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.66,
        color,
        2,
        cv2.LINE_AA,
    )


def _automatic_motion_duration(
    controller: ArmController,
    delta: np.ndarray,
    speed: float,
) -> float:
    if not math.isfinite(speed) or not 0.2 <= speed <= 1.0:
        raise RuntimeError("自动标定速度必须在 0.2..1.0")
    minimum = max(controller.minimum_duration(float(value)) for value in delta)
    controller_floor = float(controller.safety.minimum_motion_duration_s)
    return max(controller_floor, minimum / speed)


def _hybrid_motion_information(
    transforms: list[np.ndarray],
    *,
    translation_scale_m: float,
    rotation_scale_deg: float,
) -> tuple[int, float, float]:
    """Return rank, weakest constrained direction and spread for a pose set.

    Hand-eye calibration observes a screw motion, not an independent list of
    joint angles.  Express every endpoint relative to the taught center as a
    normalized 6-D vector ``[translation, rotation-vector]``.  The smallest
    singular value rewards poses that add a previously missing *combined*
    translation/rotation direction, instead of spending the first twenty
    samples on wrist-only rotations.
    """
    if not transforms:
        return 0, 0.0, 0.0
    reference = np.asarray(transforms[0], dtype=np.float64)
    rotation_scale_rad = math.radians(rotation_scale_deg)
    if translation_scale_m <= 0.0 or rotation_scale_rad <= 0.0:
        raise ValueError("hand-eye motion information scales must be positive")
    features = np.asarray(
        [
            np.concatenate(
                (
                    (transform[:3, 3] - reference[:3, 3]) / translation_scale_m,
                    rotation_vector_from_matrix(
                        reference[:3, :3].T @ transform[:3, :3]
                    ) / rotation_scale_rad,
                )
            )
            for transform in transforms[1:]
        ],
        dtype=np.float64,
    )
    if features.size == 0:
        return 0, 0.0, 0.0
    singular_values = np.linalg.svd(features, compute_uv=False)
    rank = int(np.count_nonzero(singular_values > 0.12))
    weakest = float(singular_values[-1]) if singular_values.size else 0.0
    spread = float(np.sum(np.log1p(singular_values)))
    return rank, weakest, spread


def _normalized_transform_distance(
    first: np.ndarray,
    second: np.ndarray,
    *,
    translation_scale_m: float,
    rotation_scale_deg: float,
) -> float:
    """Measure one EEF motion in the same normalized 6-D units as the fit."""
    if translation_scale_m <= 0.0 or rotation_scale_deg <= 0.0:
        raise ValueError("hand-eye motion distance scales must be positive")
    first_transform = np.asarray(first, dtype=np.float64)
    second_transform = np.asarray(second, dtype=np.float64)
    if first_transform.shape != (4, 4) or second_transform.shape != (4, 4):
        raise ValueError("hand-eye motion distance needs 4x4 transforms")
    translation = float(
        np.linalg.norm(first_transform[:3, 3] - second_transform[:3, 3])
    ) / translation_scale_m
    rotation = float(
        np.linalg.norm(
            rotation_vector_from_matrix(
                first_transform[:3, :3].T @ second_transform[:3, :3]
            )
        )
    ) / math.radians(rotation_scale_deg)
    return math.hypot(translation, rotation)


def _validation_variant_from_role(role: str | None) -> int:
    """Decode the optional fresh-holdout round encoded in a planner role."""
    if not isinstance(role, str) or "_validation" not in role:
        return 0
    _, suffix = role.rsplit("_validation", maxsplit=1)
    if not suffix:
        return 0
    if not suffix.startswith("_"):
        return 0
    try:
        return max(0, int(suffix[1:]))
    except ValueError:
        return 0


def _order_pose_plan_for_eef_diversity(
    plan: list[tuple[str, np.ndarray]],
    transforms: list[np.ndarray],
    *,
    quality: dict[str, Any],
) -> list[tuple[str, np.ndarray]]:
    """Order safe endpoints by predicted calibration observability.

    The robot may use large but compensating joint motions for a nearly fixed
    flange attitude.  Hand-eye observability depends on the resulting EEF
    transforms, therefore this deliberately scores EEF rotation span and axis
    rank before joint travel or repeated image observations.
    """
    if len(plan) != len(transforms):
        raise ValueError("automatic pose plan and transform count must match")
    entries = [
        (
            label,
            np.asarray(target, dtype=np.float64).copy(),
            np.asarray(transform, dtype=np.float64).copy(),
        )
        for (label, target), transform in zip(plan, transforms)
    ]
    for _, target, transform in entries:
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            raise ValueError("automatic pose plan contains invalid joint target")
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("automatic pose plan contains invalid EEF transform")
    if not entries:
        return []

    required_rotation = float(quality.get("min_rotation_span_deg", 20.0))
    required_translation = float(quality.get("min_translation_span_m", 0.04))
    required_rank = int(quality.get("min_rotation_axis_rank", 2))
    if (
        not math.isfinite(required_rotation)
        or not math.isfinite(required_translation)
        or required_rotation <= 0.0
        or required_translation <= 0.0
        or required_rank < 1
    ):
        raise ValueError("hand-eye motion-diversity configuration is invalid")

    remaining = entries.copy()
    ordered: list[tuple[str, np.ndarray]] = []
    selected: list[np.ndarray] = []
    # A measured center frame establishes a stable reference for the intrinsic
    # +/- rotations. It has no diversity by itself, but avoids comparing every
    # later point against an arbitrary first rotation.
    for index, (label, target, transform) in enumerate(remaining):
        if label in {"table_center", "camera_facing"}:
            ordered.append((label, target))
            selected.append(transform)
            remaining.pop(index)
            break

    def score(
        candidate: np.ndarray,
    ) -> tuple[int, int, float, float, float, float, float, float]:
        transforms = [*selected, candidate]
        diversity = analyze_transform_motion_diversity(transforms)
        information_rank, weakest_direction, information_spread = (
            _hybrid_motion_information(
                transforms,
                translation_scale_m=required_translation,
                rotation_scale_deg=required_rotation,
            )
        )
        return (
            min(diversity.rotation_axis_rank, required_rank),
            min(information_rank, 6),
            weakest_direction,
            information_spread,
            min(diversity.rotation_span_deg, required_rotation),
            min(diversity.translation_span_m, required_translation),
            diversity.rotation_span_deg,
            diversity.translation_span_m,
        )

    while remaining:
        best_index = max(
            range(len(remaining)),
            key=lambda index: score(remaining[index][2]),
        )
        label, target, transform = remaining.pop(best_index)
        ordered.append((label, target))
        selected.append(transform)
    return ordered


def _minimum_joint_duration(controller: ArmController, delta: np.ndarray) -> float:
    values = np.asarray(delta, dtype=np.float64)
    return max(
        controller.minimum_duration(float(value))
        for value in values
    )


def _is_negligible_joint_delta(delta: np.ndarray) -> bool:
    """Whether a joint target is already at the measured endpoint.

    This threshold is far below the controller's terminal position tolerance;
    it only prevents generating an invalid zero-speed no-op command from
    floating-point bookkeeping differences.
    """
    values = np.asarray(delta, dtype=np.float64)
    return values.shape == (6,) and bool(np.all(np.abs(values) <= 1e-7))


def _joint_velocity_scale_for_duration(
    controller: ArmController,
    delta: np.ndarray,
    duration_s: float,
) -> float:
    """Return one valid command scale for a non-empty timed joint motion."""
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise RuntimeError("joint motion duration must be positive")
    minimum = _minimum_joint_duration(controller, delta)
    if not math.isfinite(minimum) or minimum <= 0.0:
        raise RuntimeError("joint motion minimum duration must be positive")
    scale = min(1.0, minimum / duration_s)
    if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
        raise RuntimeError("joint motion velocity scale is invalid")
    return scale


def _movej_for_duration(
    controller: ArmController,
    target: np.ndarray,
    duration_s: float,
    *,
    allow_large_motion: bool = False,
):
    target_position = np.asarray(target, dtype=np.float64)
    delta = target_position - controller.get_state().joint_position
    if _is_negligible_joint_delta(delta):
        # A stale local plan cursor can request the pose the feedback already
        # reports. Never send a zero-speed trajectory just to satisfy it.
        return None
    velocity_scale = _joint_velocity_scale_for_duration(
        controller, delta, duration_s
    )
    try:
        return controller.movej(
            target_position,
            velocity_scale=velocity_scale,
            allow_large_motion=allow_large_motion,
        )
    except SafetyError as initial_error:
        if not _is_terminal_settle_error(initial_error):
            raise
        # A terminal settle miss is not a hard safety fault: ArmController
        # has already replaced the completed trajectory with an active hold
        # at measured feedback. Replan twice from that real pose at reduced
        # speed. This keeps primary motions fast while letting a loaded wrist
        # close the final few degrees.
        last_error = initial_error
        for maximum_retry_scale in (0.35, 0.18):
            retry_scale = min(velocity_scale, maximum_retry_scale)
            if retry_scale >= velocity_scale - 1e-9:
                continue
            if not _controller_kept_active_hold(controller):
                raise last_error
            retry_delta = (
                target_position - controller.get_state().joint_position
            )
            if _is_negligible_joint_delta(retry_delta):
                return None
            print(
                "HANDEYE_SETTLE_RETRY "
                f"speed={retry_scale:.2f} reason={last_error}",
                flush=True,
            )
            try:
                return controller.movej(
                    target_position,
                    velocity_scale=retry_scale,
                    allow_large_motion=allow_large_motion,
                )
            except SafetyError as retry_error:
                if not _is_terminal_settle_error(retry_error):
                    raise
                last_error = retry_error

        if not _controller_kept_active_hold(controller):
            raise last_error
        # The calibration sample records this measured joint state, never the
        # nominal target. A stable held endpoint is therefore still a valid
        # observation. Let the quality gate decide whether it adds enough
        # diversity; do not abandon the entire calibration over this one
        # wrist-limited waypoint.
        feedback = controller.get_state().joint_position
        error_deg = np.degrees(np.abs(target_position - feedback))
        joint = int(np.argmax(error_deg))
        print(
            "HANDEYE_SETTLE_ACCEPT_FEEDBACK "
            f"joint={joint + 1} target_error={error_deg[joint]:.2f}deg "
            "retries_exhausted=1",
            flush=True,
        )
        return None


def _is_terminal_settle_error(exc: SafetyError) -> bool:
    if isinstance(exc, MotionSettleError):
        return True
    # Compatibility for lightweight controller fakes and previously persisted
    # diagnostics that still use the base SafetyError type.
    message = str(exc)
    return (
        "did not settle at the trajectory target" in message
        or "did not stabilize at the trajectory target" in message
    )


def _controller_kept_active_hold(controller: ArmController) -> bool:
    """Return false only when the terminal miss already released control."""
    mode = getattr(controller, "mode", None)
    return mode is None or mode is ControllerMode.ACTIVE


def _require_stage_input(expected: str) -> None:
    command = _require_stage_command()
    if command != expected:
        raise RuntimeError(f"unexpected hand-eye command: {command}")


def _require_stage_command() -> str:
    value = sys.stdin.readline()
    command = value.strip().upper()
    if not value or command == "CANCEL":
        raise HandEyeCancelled
    return command


def _poll_stage_input() -> str | None:
    try:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    except (OSError, ValueError):
        return None
    if not ready:
        return None
    value = sys.stdin.readline()
    command = value.strip().upper()
    if not value or command == "CANCEL":
        raise HandEyeCancelled
    return command


def _raise_if_cancel_requested() -> None:
    command = _poll_stage_input()
    if command is None:
        return
    raise RuntimeError(f"unexpected hand-eye command: {command}")


def _average_detections(
    detections: list[TargetDetection],
) -> tuple[np.ndarray, dict[str, float]]:
    translations = np.asarray(
        [value.T_camera_target[:3, 3] for value in detections]
    )
    median = np.median(translations, axis=0)
    distances = np.linalg.norm(translations - median, axis=1)
    distance_median = float(np.median(distances))
    mad = float(np.median(np.abs(distances - distance_median)))
    threshold = distance_median + max(0.001, 3.5 * 1.4826 * mad)
    filtered = [
        detection
        for detection, distance in zip(detections, distances)
        if float(distance) <= threshold
    ]
    if len(filtered) < 3:
        raise CalibrationQualityError("too few stable target detections")
    transform = average_transforms(
        [detection.T_camera_target for detection in filtered]
    )
    translation_errors = np.asarray(
        [
            np.linalg.norm(detection.T_camera_target[:3, 3] - transform[:3, 3])
            for detection in filtered
        ]
    )
    rotation_errors = np.asarray(
        [
            rotation_distance_rad(
                detection.T_camera_target[:3, :3], transform[:3, :3]
            )
            for detection in filtered
        ]
    )
    return transform, {
        "reprojection_error_px": float(
            np.mean([value.reprojection_error_px for value in filtered])
        ),
        "translation_std_m": float(
            np.sqrt(np.mean(translation_errors**2))
        ),
        "rotation_std_deg": math.degrees(
            float(np.sqrt(np.mean(rotation_errors**2)))
        ),
    }


def _cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV contrib is required; run uv sync --only-group handeye"
        ) from exc
    return cv2
