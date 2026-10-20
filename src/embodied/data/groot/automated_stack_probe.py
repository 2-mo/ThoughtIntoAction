"""Collect one scripted blue-on-white stack episode without recording home return."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import time
from typing import Any

from embodied.core.project import port

import numpy as np
from websockets.sync.client import connect as websocket_connect

from legacy.arx5_control import ArmController, ControllerMode, SafetyError
from legacy.arx5_control.gripper_calibration import (
    load_gripper_calibration,
    load_gripper_open_readout,
)
from embodied.core.arx5_paths import PROJECT_ROOT
from embodied.console.features.cube.stack import (
    CubeStackPlan,
    execute_cube_stack,
    plan_cube_stack,
)
from embodied.console.kernel.control import WEB_MOTION_SAFETY
from embodied.tasks.cube.config import load_config as load_cube_config
from embodied.tasks.cube.perception import (
    CubeObservation,
    capture_cube_scene_observation,
)
from embodied.tasks.cube.planner import (
    build_pick_actions,
    grasp_orientation_candidates,
    validate_initial_pose,
)

from embodied.data.groot.config import load_collection_config
from embodied.data.groot.home import load_collection_home, load_training_start
from embodied.data.groot.raw_episode import RawEpisodeStore
from embodied.data.groot.recording import ArmStateRecorder
from embodied.data.groot.validator import validate_raw_episode


CAMERA_URL = f"ws://127.0.0.1:{port('collection_camera')}/api/v1/cameras/collection"
HOME_VELOCITY_SCALE = 0.75
TRANSIT_VELOCITY_SCALE = 0.55
PRECISION_VELOCITY_SCALE = 0.18
PREFERRED_GRASP = "side_0_tilt_45_roll_0"


def _camera_request(connection: Any, message_type: str, session_id: str) -> dict[str, Any]:
    request_id = f"probe-{message_type}-{time.monotonic_ns()}"
    connection.send(
        json.dumps(
            {"type": message_type, "id": request_id, "session_id": session_id},
            separators=(",", ":"),
        )
    )
    response = json.loads(connection.recv(timeout=30.0))
    if not isinstance(response, dict) or response.get("id") != request_id:
        raise RuntimeError("双相机服务返回了无效响应")
    if response.get("ok") is False:
        raise RuntimeError(str(response.get("error") or "双相机请求失败"))
    return response


def _wait_for_port(process: subprocess.Popen[bytes], timeout_s: float = 15.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("双相机服务启动失败")
        try:
            with socket.create_connection(("127.0.0.1", port("collection_camera")), timeout=0.25):
                return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("等待双相机服务启动超时")


def _start_camera_service(log_path: Path) -> tuple[subprocess.Popen[bytes], Any]:
    web_python = PROJECT_ROOT / "environments" / "arx_motion" / ".venv-web" / "bin" / "python"
    if not web_python.is_file():
        raise RuntimeError(f"视觉环境不存在：{web_python}")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open("wb")
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(PROJECT_ROOT / "src")
    process = subprocess.Popen(
        [
            str(web_python),
            "-B",
            "-m",
            "embodied.console.features.camera.service",
        ],
        cwd=PROJECT_ROOT,
        env=environment,
        stdout=stream,
        stderr=subprocess.STDOUT,
    )
    try:
        _wait_for_port(process)
    except BaseException:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        stream.close()
        raise
    return process, stream


def _stop_camera_service(process: subprocess.Popen[bytes] | None, stream: Any | None) -> None:
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)
    if stream is not None:
        stream.close()


def _move_home(controller: ArmController, home: dict[str, Any]) -> float:
    target = np.asarray(home["joint_position_rad"], dtype=np.float64)
    controller.activate()
    for attempt in range(3):
        snapshot = controller.get_state()
        plan = controller.plan_movej(
            target,
            velocity_scale=HOME_VELOCITY_SCALE,
            allow_large_motion=True,
            start_snapshot=snapshot,
        )
        try:
            controller.execute_motion_plan(plan)
        except SafetyError as exc:
            if str(exc) == "joint position changed after planning; plan again" and attempt < 2:
                continue
            raise
        break
    target_gripper = float(home["gripper_position_m"])
    state = controller.get_state()
    delta = target_gripper - state.gripper_position
    if abs(delta) > controller.safety.gripper_position_tolerance_m:
        controller.move_gripper(
            target_gripper,
            max(
                controller.safety.minimum_motion_duration_s,
                controller.minimum_gripper_duration(delta) + 0.05,
            ),
        )
    controller.hold_current()
    final = controller.get_state()
    return math.degrees(float(np.max(np.abs(final.joint_position - target))))


def _target_observation(x_m: float, y_m: float, cube_edge_m: float) -> CubeObservation:
    return CubeObservation(
        base_xyz_m=(x_m, y_m, cube_edge_m / 2.0),
        yaw_rad=0.0,
        size_m=cube_edge_m,
        confidence=1.0,
        position_source="calibrated_pixel_projection",
        accepted_samples=1,
        depth_samples=1,
        xy_rms_m=0.0,
        yaw_rms_rad=0.0,
        timestamp=time.time(),
        candidate_id="manual-white-target",
        color="white",
    )


def _plan_stack(
    controller: ArmController,
    source: CubeObservation,
    target: CubeObservation,
    reference_joint_position: tuple[float, ...],
) -> tuple[CubeStackPlan, str]:
    config = load_cube_config()
    initial = controller.get_state()
    validate_initial_pose(initial.eef_pose_6d, config)
    candidates = tuple(
        candidate
        for candidate in grasp_orientation_candidates(source, initial.eef_pose_6d)
        if candidate.name == PREFERRED_GRASP
    )
    failures: list[str] = []
    plans: list[tuple[float, CubeStackPlan, str]] = []
    for candidate in candidates:
        try:
            pick_actions = build_pick_actions(
                source,
                config,
                candidate.orientation_xyzw,
                grasp_mode=candidate.grasp_mode,
                approach_xy=candidate.approach_xy,
            )
            preview = plan_cube_stack(
                controller,
                initial=initial,
                pick_actions=pick_actions,
                target=target,
                config=config,
                reference_joint_position=reference_joint_position,
                transit_velocity_scale=TRANSIT_VELOCITY_SCALE,
                precision_velocity_scale=PRECISION_VELOCITY_SCALE,
            )
        except (RuntimeError, ValueError) as exc:
            failures.append(f"{candidate.name}: {exc}")
            continue
        score = max(
            (phase.plan.max_joint_delta_rad for phase in preview.phases),
            default=math.inf,
        )
        plans.append((score, preview, candidate.name))
    if plans:
        _, preview, name = min(plans, key=lambda item: item[0])
        return preview, name
    raise RuntimeError("完整叠放轨迹无可用逆解：" + "；".join(failures[-4:]))


def _record_metadata(
    store: RawEpisodeStore,
    session_id: str,
    controller: ArmController,
    training_start: dict[str, Any],
    safe_home: dict[str, Any],
) -> None:
    snapshot = controller.get_state()
    limits = controller.get_limits()
    calibration = load_gripper_calibration(interface=controller.interface)
    store.update(
        session_id,
        stage="recording",
        started_host_monotonic_ns=time.monotonic_ns(),
        teaching_mode="autonomous_cartesian_stack_probe",
        collection_home={
            "fixed": True,
            "calibrated_at": training_start.get("calibrated_at"),
            "method": training_start.get("method"),
            "joint_position_rad": list(training_start["joint_position_rad"]),
            "gripper_position_m": training_start["gripper_position_m"],
        },
        safe_disconnect_home={
            "calibrated_at": safe_home.get("calibrated_at"),
            "method": safe_home.get("method"),
            "joint_position_rad": list(safe_home["joint_position_rad"]),
            "gripper_position_m": safe_home["gripper_position_m"],
        },
        gripper_calibration={
            "schema_version": None if calibration is None else calibration.get("schema_version"),
            "calibrated_at": None if calibration is None else calibration.get("calibrated_at"),
            "gripper_open_readout": controller.gripper_open_readout,
            "zero_calibrated": bool(calibration and calibration.get("zero_calibrated", True)),
            "status": "calibrated" if calibration is not None else "fallback_readout",
        },
        initial_state=ArmStateRecorder.snapshot_payload(snapshot, time.monotonic_ns()),
        limits={
            "joint_position_min_rad": np.asarray(limits.joint_position_min).tolist(),
            "joint_position_max_rad": np.asarray(limits.joint_position_max).tolist(),
            "gripper_width_m": float(limits.gripper_width_m),
            "gripper_position_tolerance_m": float(
                controller.safety.gripper_position_tolerance_m
            ),
        },
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="自动采集一条蓝块叠到白块的过拟合样本")
    parser.add_argument("--white-x-mm", type=float, default=322.0)
    parser.add_argument("--white-y-mm", type=float, default=-53.4)
    parser.add_argument("--plan-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    collection_config = load_collection_config()
    cube_config = load_cube_config()
    safe_home = load_collection_home(interface=cube_config.interface)
    if safe_home is None:
        raise RuntimeError("断开安全零位不存在")
    training_start = load_training_start(interface=cube_config.interface)
    if training_start is None:
        raise RuntimeError("模型训练起点不存在")

    print("正在扫描蓝色源方块；不会连接机械臂...")
    scene = capture_cube_scene_observation(
        cube_config,
        debug_image_path=PROJECT_ROOT / "outputs" / "cube-pick" / "blue-on-white-probe.jpg",
    )
    blue = next((item for item in scene.observations if item.color == "blue"), None)
    if blue is None:
        raise RuntimeError("没有找到稳定蓝色方块")
    white = _target_observation(
        args.white_x_mm / 1000.0,
        args.white_y_mm / 1000.0,
        collection_config.cube_edge_m,
    )
    print(
        "目标坐标："
        f"蓝 ({1000 * blue.base_xyz_m[0]:.1f}, {1000 * blue.base_xyz_m[1]:.1f}) mm；"
        f"白 ({args.white_x_mm:.1f}, {args.white_y_mm:.1f}) mm"
    )

    controller = ArmController(
        interface=cube_config.interface,
        gain_scale=1.0,
        gripper_gain_scale=1.0,
        gripper_open_readout=load_gripper_open_readout(interface=cube_config.interface),
        safety=WEB_MOTION_SAFETY,
    )
    camera_process: subprocess.Popen[bytes] | None = None
    camera_stream: Any | None = None
    recorder: ArmStateRecorder | None = None
    connection: Any | None = None
    location = None
    recording_active = False
    accepted = False
    try:
        controller.connect(stabilize=True)
        print("正在进入固定模型训练起点...")
        home_error_deg = _move_home(controller, training_start)
        print(f"模型训练起点就绪：最大关节误差 {home_error_deg:.2f} deg")
        reference = tuple(float(value) for value in training_start["joint_position_rad"])
        preview, grasp_name = _plan_stack(controller, blue, white, reference)
        print(f"完整轨迹预检通过：{grasp_name}，{len(preview.actions)} 个动作")
        if args.plan_only:
            print("仅预检完成；未启动相机，未执行抓取。")
            return 0

        camera_process, camera_stream = _start_camera_service(
            PROJECT_ROOT / "outputs" / "logs" / "blue-on-white-camera.log"
        )
        connection = websocket_connect(CAMERA_URL, open_timeout=10.0)
        store = RawEpisodeStore(config=collection_config)
        location = store.begin()
        _record_metadata(
            store,
            location.session_id,
            controller,
            training_start,
            safe_home,
        )
        recorder = ArmStateRecorder(
            location.path,
            controller,
            rate_hz=collection_config.sampling.arm_rate_hz,
        )
        recorder.start()
        _camera_request(connection, "collection_camera_start", location.session_id)
        recording_active = True
        time.sleep(1.0)

        def finish_recording() -> None:
            nonlocal recorder, recording_active, accepted
            time.sleep(0.8)
            assert connection is not None and location is not None and recorder is not None
            _camera_request(connection, "collection_camera_stop", location.session_id)
            recording_active = False
            state_summary = recorder.stop()
            recorder = None
            store.finish_recording(location.session_id, state_summary)
            report = validate_raw_episode(location.path, collection_config)
            if not report["ok"]:
                store.reject(location.session_id, "；".join(report["errors"]))
                raise RuntimeError("采集质量检查未通过：" + "；".join(report["errors"]))
            accepted_location = store.accept(location.session_id, report)
            accepted = True
            print(f"样本已接受：{accepted_location.path}")

        print("开始自动夹取并叠放；回零不会录入样本。")
        result = execute_cube_stack(controller, preview, before_return=finish_recording)
        print(
            "叠放和回零完成："
            f"回零误差 {result['final_joint_error_deg']:.2f} deg"
        )
        return 0
    finally:
        if connection is not None and location is not None and recording_active:
            try:
                _camera_request(connection, "collection_camera_abort", location.session_id)
            except BaseException:
                pass
        if recorder is not None:
            try:
                recorder.stop()
            except BaseException:
                pass
        if connection is not None:
            try:
                connection.close()
            except BaseException:
                pass
        _stop_camera_service(camera_process, camera_stream)
        if controller.mode not in {
            ControllerMode.DISCONNECTED,
            ControllerMode.CLOSED,
            ControllerMode.FAULT,
        }:
            try:
                print("断开前自动回原机械零位...")
                home_error_deg = _move_home(controller, safe_home)
                print(f"原机械零位完成：最大关节误差 {home_error_deg:.2f} deg")
            finally:
                if controller.mode == ControllerMode.ACTIVE:
                    controller.damping()
        if controller.mode not in {ControllerMode.DISCONNECTED, ControllerMode.CLOSED}:
            controller.close(emergency=controller.mode == ControllerMode.FAULT)
        if location is not None and not accepted:
            try:
                store.reject(location.session_id, "automated stack probe did not complete")
            except (FileNotFoundError, RuntimeError, ValueError):
                pass


if __name__ == "__main__":
    raise SystemExit(main())
