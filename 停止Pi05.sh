#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${QUEST_TELEOP_PYTHON:-$ROOT/.venv/bin/python}" "$ROOT/src/zero-shot/recover.py" "$@"
