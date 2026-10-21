#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
exec env PYTHONPATH="$ROOT/src" "${QUEST_TELEOP_PYTHON:-$ROOT/.venv/bin/python}" -m embodied.policy.pi05_zeroshot.recover --pregrasp "$@"
