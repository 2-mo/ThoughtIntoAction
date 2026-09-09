#!/usr/bin/env bash
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${QUEST_BOOTSTRAP_PYTHON:-/usr/bin/python3}" -B "$ROOT_DIR/scripts/launch.py" "$@"
