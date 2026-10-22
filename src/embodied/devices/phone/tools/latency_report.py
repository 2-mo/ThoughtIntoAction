#!/usr/bin/env python3
"""Summarize finite millisecond samples using runtime percentile conventions."""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Sequence
from pathlib import Path

from embodied.devices.phone.tools._common import distribution_with_count, print_report


def _samples(value: object, field: str) -> list[float]:
    if isinstance(value, dict):
        if "samples_ms" in value:
            value = value["samples_ms"]
        else:
            value = [item[field] for item in value.get("samples", [])]
    if not isinstance(value, list):
        raise ValueError("input JSON must contain a sample list")
    samples: list[float] = []
    for item in value:
        if isinstance(item, dict):
            item = item[field]
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError("every latency sample must be numeric")
        sample = float(item)
        if not math.isfinite(sample) or sample < 0.0:
            raise ValueError("latency samples must be non-negative and finite")
        samples.append(sample)
    return samples


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--field", default="latency_ms")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    loaded: object = json.loads(args.input.read_text(encoding="utf-8"))
    samples = _samples(loaded, args.field)
    print_report(distribution_with_count(samples))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
