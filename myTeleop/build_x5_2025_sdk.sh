#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SDK_ROOT="${SCRIPT_DIR}/../X5/py/arx_x5_python"
readonly CONDA_PREFIX_XROBOT="/home/r2/miniconda3/envs/xrobot312"

if [[ ! -d "${SDK_ROOT}/bimanual" ]]; then
    printf 'Official X5 SDK is missing: %s\n' "${SDK_ROOT}" >&2
    printf 'Clone https://github.com/ARXroboticsX/X5 into the workspace X5 directory.\n' >&2
    exit 1
fi

cmake \
    -S "${SDK_ROOT}/bimanual" \
    -B "${SDK_ROOT}/bimanual/build" \
    -DCMAKE_PREFIX_PATH="${CONDA_PREFIX_XROBOT}" \
    -DPYTHON_EXECUTABLE="${CONDA_PREFIX_XROBOT}/bin/python" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "${SDK_ROOT}/bimanual/build" -j2
cmake --install "${SDK_ROOT}/bimanual/build"

printf 'Official ARX X5-2025 Python SDK built successfully.\n'
