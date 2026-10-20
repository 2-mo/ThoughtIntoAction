#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
exec "${QUEST_TELEOP_PYTHON:-$ROOT/.venv/bin/python}" "$ROOT/src/embodied/policy/pi05_zeroshot/recover.py" --pregrasp "$@"
