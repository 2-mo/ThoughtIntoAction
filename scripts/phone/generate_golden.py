#!/usr/bin/env python3
"""Regenerate the checked-in cross-language protocol vectors."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from pathlib import Path

from robot_teleop.protocol.golden import GOLDEN_PACKETS, golden_manifest
from robot_teleop.protocol.packet import encode_packet

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = REPOSITORY_ROOT / "tests" / "golden"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="destination directory (default: tests/golden)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    for filename, packet in GOLDEN_PACKETS.items():
        (output_dir / filename).write_bytes(encode_packet(packet))
    manifest = json.dumps(golden_manifest(), indent=2, sort_keys=True) + "\n"
    (output_dir / "manifest.json").write_text(manifest, encoding="utf-8")
    print(f"wrote {len(GOLDEN_PACKETS)} vectors to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
