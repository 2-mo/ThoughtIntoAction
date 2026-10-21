#!/usr/bin/env python3
from __future__ import annotations

import argparse
from contextlib import AbstractContextManager
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty
from typing import TextIO

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[4]
SOURCE_ROOT = PROJECT_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from embodied.core.arx5_paths import OUTPUTS_ROOT, PROJECT_ROOT as PATHS_PROJECT_ROOT  # noqa: E402
from embodied.policy.groot.cameras import CAMERA_IDS, DualRgbCamera, SharedRgbCameras, default_camera_specs  # noqa: E402
from embodied.policy.groot.checkpoint import (  # noqa: E402
    ACTION_HORIZON,
    CheckpointError,
    FineTunedCheckpoint,
    select_checkpoint,
)
from embodied.policy.groot.policy import ModelServer, PolicyClient, PolicyError  # noqa: E402
from embodied.policy.groot.runtime import (  # noqa: E402
    ActionPreview,
    FineTunedRuntime,
    RuntimeFault,
    synthetic_observation,
)


TRAINING_ROOT = OUTPUTS_ROOT / "groot-training"
GROOT_ROOT = PATHS_PROJECT_ROOT / "third_party" / "Isaac-GR00T"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="使用最新 ARX5 微调 GR00T 权重进行终端交互控制",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        help="指定完整 checkpoint-N；默认自动选择 outputs/groot-training 下最大的 N",
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--check",
        action="store_true",
        help="只检查权重和运行环境，不加载模型、相机或 CAN",
    )
    modes.add_argument(
        "--smoke",
        action="store_true",
        help="加载模型并执行一次合成观测推理，不打开相机或 CAN",
    )
    parser.add_argument("--run", type=Path, help="经 arx 内核执行（control/run/<RUN>）；不直接打开 CAN")
    parser.add_argument("--task", help="与 --run 一起使用的任务描述")
    parser.add_argument("--seconds", type=float, default=60.0, help="与 --run 一起使用的运行时长，秒")
    options = parser.parse_args(argv)
    if options.run and not (options.task or options.check or options.smoke):
        parser.error("--run 需要 --task")
    if not 0 < options.seconds <= 300:
        parser.error("--seconds 范围 0..300")
    return options


def parse_live_key(value: str) -> str | None:
    key = value.upper()
    return key if key in {"S", "H", "N", "Q"} else None


class LiveKeys(AbstractContextManager["LiveKeys"]):
    def __init__(self, stream: TextIO = sys.stdin) -> None:
        self.stream = stream
        self._attributes: list | None = None

    def __enter__(self) -> LiveKeys:
        if not self.stream.isatty():
            raise RuntimeFault("运行控制需要交互式终端")
        self._attributes = termios.tcgetattr(self.stream.fileno())
        tty.setcbreak(self.stream.fileno())
        return self

    def poll(self) -> str | None:
        readable, _, _ = select.select([self.stream], [], [], 0.0)
        if not readable:
            return None
        value = self.stream.read(1)
        return "Q" if value == "" else parse_live_key(value)

    def __exit__(self, *_: object) -> None:
        if self._attributes is not None:
            termios.tcsetattr(
                self.stream.fileno(),
                termios.TCSADRAIN,
                self._attributes,
            )
            self._attributes = None


def _validate_smoke_actions(
    actions: dict[str, object],
    action_keys: tuple[str, ...],
) -> None:
    shapes = {
        "eef_9d": (1, ACTION_HORIZON, 9),
        "gripper_position": (1, ACTION_HORIZON, 1),
        "joint_position": (1, ACTION_HORIZON, 6),
    }
    expected = {key: shapes[key] for key in action_keys}
    if set(actions) != set(expected):
        raise PolicyError(f"GR00T smoke action keys are invalid: {sorted(actions)}")
    for key, shape in expected.items():
        value = actions[key]
        if (
            not isinstance(value, np.ndarray)
            or value.shape != shape
            or not np.all(np.isfinite(value))
        ):
            raise PolicyError(f"GR00T smoke action {key} is invalid")


def _validate_rtc_smoke_actions(
    previous: dict[str, object],
    current: dict[str, object],
    action_keys: tuple[str, ...],
) -> None:
    _validate_smoke_actions(current, action_keys)
    for key in action_keys:
        previous_value = previous[key]
        current_value = current[key]
        assert isinstance(previous_value, np.ndarray)
        assert isinstance(current_value, np.ndarray)
        if not np.allclose(
            current_value[:, :4],
            previous_value[:, 8:12],
            atol=2e-4,
            rtol=2e-4,
        ):
            maximum_difference = float(
                np.max(np.abs(current_value[:, :4] - previous_value[:, 8:12]))
            )
            raise PolicyError(
                f"GR00T RTC frozen overlap is invalid for {key} "
                f"(max difference {maximum_difference:.6f})"
            )


def _session_directory(checkpoint: FineTunedCheckpoint) -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    root = OUTPUTS_ROOT / "gr-zeroshot" / f"{stamp}-{os.getpid()}"
    root.mkdir(parents=True, exist_ok=False)
    (root / "session.json").write_text(
        json.dumps(
            {
                "checkpoint": str(checkpoint.path),
                "checkpoint_step": checkpoint.step,
                "embodiment": checkpoint.embodiment,
                "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            },
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return root


def _print_preview(preview: ActionPreview) -> None:
    delta = preview.first_delta_mm
    print("\n模型预览（机械臂未激活）：")
    print(
        f"  可用动作：{preview.usable_actions}/16，推理 {preview.latency_ms:.0f} ms"
    )
    print(f"  首目标位移：[X {delta[0]:+.1f}, Y {delta[1]:+.1f}, Z {delta[2]:+.1f}] mm")
    print(f"  首目标姿态变化：{preview.first_rotation_delta_deg:.1f} deg")
    print(f"  夹爪：{preview.gripper_state.value}")


def _prompt_task() -> str | None:
    while True:
        try:
            value = input("\n任务描述（输入 Q 退出） > ").strip()
        except EOFError:
            return None
        if value.upper() == "Q":
            return None
        if value:
            return value
        print("任务描述不能为空。")


def _interactive(runtime: FineTunedRuntime) -> int:
    while True:
        task = _prompt_task()
        if task is None:
            return 0
        try:
            confirmation = input("输入 Y 自动进入模型训练起点并开始执行 > ").strip()
        except EOFError:
            return 0
        if confirmation.upper() != "Y":
            print("未执行。")
            continue
        plan, gripper = runtime.preview_training_start()
        print(
            "正在回到训练起始零位："
            f"预计 {plan.duration_s:.1f} s，"
            f"最大关节变化 {np.degrees(plan.max_joint_delta_rad):.1f} deg"
        )
        runtime.execute_training_start(plan, gripper)
        print("已回到训练起始零位，开始执行模型任务。")
        print("\n运行中：[S/H] 停止并回零  [N] 回零后下一任务  [Q] 回零后退出")
        with LiveKeys() as keys:
            command = runtime.run_task(task, keys.poll)
        print(f"\n本次动作已结束并回零（{command}）。")
        if command == "Q":
            return 0


def run(argv: list[str] | None = None) -> int:
    options = parse_args(argv)
    checkpoint = select_checkpoint(TRAINING_ROOT, options.checkpoint)
    specs = default_camera_specs()
    print(f"检查点：{checkpoint.path}")
    print(f"模式：{checkpoint.embodiment}，16 actions @ 30 Hz")
    active_specs = tuple(
        spec for spec in specs if f"{spec.name}_rgb" in checkpoint.video_keys
    )
    camera_names = " / ".join(f"{CAMERA_IDS[spec.name]} {spec.serial}" for spec in active_specs)
    print(f"模型视觉输入：{', '.join(checkpoint.video_keys)}")
    print(f"模型状态/动作：{', '.join(checkpoint.state_keys)}")
    print(f"相机：{camera_names}")
    if not (GROOT_ROOT / ".venv" / "bin" / "python").is_file():
        raise PolicyError(f"GR00T 环境不存在：{GROOT_ROOT / '.venv'}")
    if options.check:
        print("检查通过；未加载模型、相机或 CAN。")
        return 0
    session = _session_directory(checkpoint)
    print("正在加载并预热 GR00T 模型...")
    with ModelServer(
        checkpoint,
        groot_root=GROOT_ROOT,
        log_path=session / "model-server.log",
    ) as server:
        with PolicyClient("127.0.0.1", server.port, timeout_s=30.0) as client:
            observation = synthetic_observation(
                video_keys=checkpoint.video_keys,
                state_keys=checkpoint.state_keys,
            )
            actions, _ = client.get_action(observation)
            _validate_smoke_actions(actions, checkpoint.action_keys)
            rtc_actions, _ = client.get_action(
                observation,
                {
                    "previous_action": actions,
                    "action_horizon": ACTION_HORIZON,
                    "rtc_overlap_steps": 8,
                    "rtc_frozen_steps": 4,
                    "rtc_ramp_rate": 4.0,
                },
            )
            _validate_rtc_smoke_actions(actions, rtc_actions, checkpoint.action_keys)
        print("模型推理与 RTC 重叠通过。")
        if options.smoke:
            print("GPU smoke 通过；未打开相机或 CAN。")
            return 0
        print(f"正在启动 {camera_names}...")
        # On the kernel, frames come from the shared camera service; the legacy path opens the devices itself.
        names = tuple(spec.name for spec in active_specs)
        with (SharedRgbCameras(names) if options.run else DualRgbCamera(active_specs)) as cameras:
            print("相机数据通过。")
            if options.run:
                return _run_on_kernel(options, checkpoint, cameras, server.port)
            runtime = FineTunedRuntime(
                cameras,
                policy_host="127.0.0.1",
                policy_port=server.port,
                video_keys=checkpoint.video_keys,
                state_keys=checkpoint.state_keys,
            )
            try:
                return _interactive(runtime)
            finally:
                runtime.close()


def _run_on_kernel(options, checkpoint, cameras, port: int) -> int:
    """One task through the arx kernel; S or Q in the terminal stops it and the kernel holds."""
    from embodied.contracts.runtime.control import ControlSession
    from embodied.policy.groot.kernel_runtime import run as run_on_kernel
    from embodied.policy.groot.policy import PolicyWorker
    worker = PolicyWorker("127.0.0.1", port, timeout_s=1.0)
    try:
        with ControlSession(options.run, source="policy", purpose="operation",
                            metadata={"policy": "groot", "checkpoint": str(checkpoint.path)}) as session, LiveKeys() as keys:
            print(f"经内核执行：{options.task}；S / Q 停止，机械臂保持。")
            summary = run_on_kernel(session, worker, cameras, options.task, seconds=options.seconds,
                                    poll_command=keys.poll, video_keys=checkpoint.video_keys,
                                    state_keys=checkpoint.state_keys)
    finally:
        worker.close()
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["status"] in ("time_limit", "stopped", "interrupted") else 1


def main() -> int:
    try:
        return run()
    except KeyboardInterrupt:
        print("\n已中断并清理资源。")
        return 130
    except (CheckpointError, PolicyError, RuntimeFault, OSError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
