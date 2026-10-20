#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PYTHON="${PI05_PYTHON:-$ROOT/outputs/envs/pi05-zero-shot/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
    echo '请先运行 bash src/embodied/policy/pi05_zeroshot/setup.sh' >&2
    exit 1
fi
exec "$PYTHON" "$ROOT/src/embodied/policy/pi05_zeroshot/infer.py" "$@"
