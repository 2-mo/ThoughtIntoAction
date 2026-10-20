#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PYTHON="${QUEST_TELEOP_PYTHON:-$ROOT/.venv/bin/python}"
exec "$PYTHON" "$ROOT/src/embodied/policy/pi05_zeroshot/real.py" "$@"
