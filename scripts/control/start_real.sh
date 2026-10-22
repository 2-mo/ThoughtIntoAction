#!/usr/bin/env bash
set -euo pipefail
CONTROL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd -- "$CONTROL_ROOT"
unset PYTHONPATH PYTHONHOME VIRTUAL_ENV LD_LIBRARY_PATH UV_PROJECT UV_PROJECT_ENVIRONMENT
export PYTHONPATH="$CONTROL_ROOT/outputs/build/control/arx-gripper/source/python:$CONTROL_ROOT/src"
export QUEST_ARX_GRIPPER_TURNS=1
export LD_LIBRARY_PATH="$CONTROL_ROOT/outputs/envs/control/lib/python3.10/site-packages/arx5_interface.libs"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 PYTHONUNBUFFERED=1
if [[ ! -x "$CONTROL_ROOT/outputs/envs/control/bin/python" ]]; then
    printf '%s\n' '缺少控制环境，请先运行 ./scripts/control/setup.sh。' >&2
    exit 1
fi
exec "$CONTROL_ROOT/outputs/envs/control/bin/python" -m embodied.control run "$@" --backend real --input vr --record-tuning
