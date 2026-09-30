#!/usr/bin/env bash
set -euo pipefail
ARX5_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$ARX5_ROOT"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
if [[ ! -x "$ARX5_ROOT/.venv/bin/python" ]]; then
    printf '%s\n' '缺少本地 .venv，请先运行 ./setup.sh。' >&2
    exit 1
fi
exec "$ARX5_ROOT/.venv/bin/python" "$ARX5_ROOT/scripts/self_test.py" "$@"
