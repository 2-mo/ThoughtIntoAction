"""Command line entry point for ARX X5 dual-camera hand-eye calibration."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import sys
from typing import Any

from embodied.core.arx5_paths import PROJECT_ROOT

from embodied.calibration.hand_eye.collector import ManualEyeToHandCollector, solve_and_store_session
from embodied.calibration.hand_eye.core.storage import CalibrationSession
from embodied.calibration.hand_eye.core.targets import build_target_detector
from embodied.calibration.hand_eye.core.validation import disable_result, enable_validated_result, validate_cached_result


ROOT = PROJECT_ROOT
DEFAULT_CONFIG = Path(__file__).resolve().parent / "configs" / "x5_d455_eye_to_hand.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ARX X5 + eye-in-hand D405 + fixed D455 calibration"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    subparsers = parser.add_subparsers(dest="command", required=True)

    target = subparsers.add_parser("generate-target", help="生成可打印 ChArUco 标定板")
    target.add_argument("--output", type=Path)
    target.add_argument("--dpi", type=int, default=300)

    capture = subparsers.add_parser("capture", help="手动摆位并采集眼在手外样本")
    capture.add_argument("--session", help="会话目录名；默认使用当前时间")
    capture.add_argument(
        "--confirm-safe",
        action="store_true",
        help="确认标定板安装正确、工作区安全且实体急停可用",
    )
    capture.add_argument(
        "--auto",
        action="store_true",
        help="网页双阶段自动采集：先桌面板 D405，再夹爪板 D455",
    )

    solve = subparsers.add_parser("solve", help="从已有 samples.json 离线求解")
    solve.add_argument("--session", required=True, help="会话目录名或绝对路径")

    subparsers.add_parser("validate", help="验证当前缓存结果的质量门")
    subparsers.add_parser("enable", help="启用已经通过验证的当前结果")
    subparsers.add_parser("disable", help="停用当前启用结果但保留缓存")
    return parser.parse_args()


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    required = ("robot", "camera", "target", "collection", "quality")
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError(f"config is missing sections: {', '.join(missing)}")
    output_root = Path(config["collection"]["output_root"])
    if not output_root.is_absolute():
        config["collection"]["output_root"] = str((ROOT / output_root).resolve())
    scene_hint = os.environ.get("ARX5_HANDEYE_SCENE_HINT")
    if scene_hint:
        try:
            parsed_hint = json.loads(scene_hint)
        except json.JSONDecodeError as exc:
            raise ValueError("网页场景提示不是有效 JSON") from exc
        if isinstance(parsed_hint, dict):
            config["scene_hint"] = parsed_hint
    return config


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config.resolve())
        output_root = Path(config["collection"]["output_root"])
        if args.command == "generate-target":
            detector = build_target_detector(config["target"])
            if not hasattr(detector, "generate_image"):
                raise ValueError("configured target cannot generate a printable image")
            default_name = (
                f"aruco_id{config['target'].get('marker_id', 0)}.png"
                if config["target"].get("type") == "aruco"
                else "charuco.png"
            )
            output = args.output or output_root / "targets" / default_name
            if not output.is_absolute():
                output = ROOT / output
            details = detector.generate_image(output.resolve(), dpi=args.dpi)
            metadata = output.with_suffix(".json")
            with metadata.open("w", encoding="utf-8") as stream:
                json.dump(
                    {"target": config["target"], **details},
                    stream,
                    ensure_ascii=False,
                    indent=2,
                )
                stream.write("\n")
            print(f"标定板：{output.resolve()}")
            print(
                f"打印尺寸必须为 {details['printed_width_mm']:.1f} x "
                f"{details['printed_height_mm']:.1f} mm，缩放 100%。"
            )
            return 0
        if args.command == "capture":
            if not args.confirm_safe:
                raise RuntimeError(
                    "capture 会连接机械臂并切换保持/阻尼；"
                    "确认安全后加 --confirm-safe"
                )
            name = args.session or dt.datetime.now().strftime("%Y%m%d-%H%M%S")
            session = CalibrationSession((output_root / "sessions" / name).resolve())
            collector = ManualEyeToHandCollector(config, session)
            if args.auto and not config.get("hand_camera", {}).get("serial"):
                raise ValueError("双相机自动标定缺少 hand_camera.serial")
            result = collector.run_automatic() if args.auto else collector.run()
            return 0 if result is not None else 3
        if args.command == "solve":
            requested = Path(args.session)
            session_path = (
                requested
                if requested.is_absolute()
                else output_root / "sessions" / requested
            )
            solve_and_store_session(CalibrationSession(session_path.resolve()))
            return 0
        if args.command == "validate":
            report = validate_cached_result(output_root)
            print(f"验证结果：{'通过' if report['passed'] else '未通过'}")
            for name, check in report["checks"].items():
                print(f"{name}: {check['value']} {check['operator']} {check['limit']}")
            return 0 if report["passed"] else 3
        if args.command == "enable":
            enabled = enable_validated_result(output_root)
            print(f"已启用双相机手眼标定结果：{output_root / 'enabled.json'}")
            return 0 if enabled.get("enabled") else 3
        if args.command == "disable":
            disable_result(output_root)
            print(f"已停用手眼标定结果：{output_root / 'enabled.json'}")
            return 0
    except KeyboardInterrupt:
        print("手眼标定已取消；机械臂已进入阻尼。", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"手眼标定失败：{exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
