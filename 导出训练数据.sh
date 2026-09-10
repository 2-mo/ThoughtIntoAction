#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$ROOT/outputs/envs/lerobot-export/bin/python"
if [[ ! -x "$PYTHON" ]]; then "$ROOT/scripts/datasets/setup_export.sh"; fi
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON" -m quest_teleop.export_lerobot "$@"
