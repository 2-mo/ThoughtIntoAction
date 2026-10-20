"""Command-line entry point for ARX5 Isaac-GR00T data collection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from arx5_paths import DATASETS_ROOT

from isaac_groot_data_collection.config import load_collection_config
from isaac_groot_data_collection.exporter import export_dataset, find_groot_root, validate_with_groot
from isaac_groot_data_collection.raw_episode import BATCHED_RAW_ROOT, RawEpisodeStore
from isaac_groot_data_collection.validator import validate_exported_dataset, validate_raw_episode


def _print(payload: object) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="ARX5 data collection utilities for Isaac-GR00T"
    )
    parser.add_argument("--config", type=Path, help="collection config JSON")
    subparsers = parser.add_subparsers(dest="command", required=True)

    status = subparsers.add_parser("status", help="show raw collection counts")
    status.add_argument("--raw-root", type=Path)

    raw_validate = subparsers.add_parser(
        "validate-raw", help="validate raw stopped/accepted episodes"
    )
    raw_validate.add_argument("--raw-root", type=Path)
    raw_validate.add_argument("session_id", nargs="?")

    export = subparsers.add_parser(
        "export", help="export accepted episodes to LeRobot v2.1"
    )
    export.add_argument("--raw-root", type=Path)
    export.add_argument("--output", type=Path)
    export.add_argument("--groot-root", type=Path)
    export.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="allow a deliberate test export before the configured target count",
    )
    export.add_argument(
        "--initial-arm-motion-lead-s",
        type=float,
        help="trim leading idle frames while retaining this many seconds before arm motion",
    )
    export.add_argument(
        "--maximum-internal-static-seconds",
        type=float,
        help="collapse long internal pose holds while retaining this many seconds",
    )

    dataset_validate = subparsers.add_parser(
        "validate-dataset", help="validate an exported dataset with GR00T"
    )
    dataset_validate.add_argument("dataset", type=Path)
    dataset_validate.add_argument("--groot-root", type=Path)

    args = parser.parse_args(argv)
    config = load_collection_config(args.config) if args.config else load_collection_config()
    configured_raw_root = BATCHED_RAW_ROOT / config.batch_id

    if args.command == "status":
        _print(RawEpisodeStore(args.raw_root or configured_raw_root, config).counts())
        return 0
    if args.command == "validate-raw":
        root = args.raw_root or configured_raw_root
        store = RawEpisodeStore(root, config)
        if args.session_id:
            paths = [store.locate(args.session_id).path]
        else:
            paths = store.paths_for_stage("inprogress") + store.paths_for_stage(
                "complete"
            )
        reports = [validate_raw_episode(path, config) for path in paths]
        _print(reports)
        return 0 if all(report["ok"] for report in reports) else 1
    if args.command == "export":
        output = args.output or (
            DATASETS_ROOT / "isaac-groot" / "processed" / config.dataset_id
        )
        _print(
            export_dataset(
                args.raw_root or configured_raw_root,
                output,
                config=config,
                groot_root=args.groot_root,
                allow_incomplete=args.allow_incomplete,
                initial_arm_motion_lead_s=args.initial_arm_motion_lead_s,
                maximum_internal_static_seconds=args.maximum_internal_static_seconds,
            )
        )
        return 0
    if args.command == "validate-dataset":
        report = validate_exported_dataset(args.dataset)
        if report["ok"]:
            root = args.groot_root or find_groot_root(Path(__file__))
            if root is None:
                raise RuntimeError("Isaac-GR00T checkout not found; pass --groot-root")
            validate_with_groot(args.dataset, groot_root=root)
        _print(report)
        return 0 if report["ok"] else 1
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
