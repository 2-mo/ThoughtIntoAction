#!/usr/bin/env bash
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
SDK_ROOT="$(PYTHONPATH="$ROOT_DIR/src" python3 -c 'from embodied.devices.robots.arx5.paths import official_sdk_python_root; print(official_sdk_python_root())')"
BUILD_DIR="$ROOT_DIR/outputs/build/x5-2025"
PYTHON_BIN="${QUEST_TELEOP_PYTHON:?Run this script through ./run.sh build-x5-2025-sdk}"
PYTHON_PREFIX="${QUEST_TELEOP_PREFIX:?Python prefix is required}"
CMAKE_BIN="$PYTHON_PREFIX/bin/cmake"
if [[ ! -x "$CMAKE_BIN" ]]; then
    printf 'Install optional build tools first: uv sync --locked --group sdk-build\n' >&2
    exit 1
fi
PYBIND_DIR="$("$PYTHON_BIN" -m pybind11 --cmakedir)"
if [[ ! -d "$SDK_ROOT/bimanual" ]]; then
    printf 'Official X5 SDK missing. Run ./run.sh fetch-dependencies --name x5-official first.\n' >&2
    exit 1
fi
"$CMAKE_BIN" -S "$SDK_ROOT/bimanual" -B "$BUILD_DIR" \
    -DCMAKE_PREFIX_PATH="$PYTHON_PREFIX" \
    -Dpybind11_DIR="$PYBIND_DIR" \
    -DPYTHON_EXECUTABLE="$PYTHON_BIN" -DCMAKE_BUILD_TYPE=Release
"$CMAKE_BIN" --build "$BUILD_DIR" -j2
"$CMAKE_BIN" --install "$BUILD_DIR"
