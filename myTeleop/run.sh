#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PYTHON_BIN="/home/r2/miniconda3/envs/xrobot312/bin/python"
readonly CONDA_LIB="/home/r2/miniconda3/envs/xrobot312/lib"

export LD_LIBRARY_PATH="${CONDA_LIB}:${LD_LIBRARY_PATH:-}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf 'Python environment not found: %s\n' "${PYTHON_BIN}" >&2
    exit 1
fi

command_name="${1:-}"
if [[ -n "${command_name}" ]]; then
    shift
fi

case "${command_name}" in
    check-xr)
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/check_xr.py" "$@"
        ;;
    visualize-xr)
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/visualize_xr.py" "$@"
        ;;
    check-arm)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/check_arm.py" "$@"
        ;;
    teleop)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/teleop_single_r5.py" "$@"
        ;;
    *)
        printf 'Usage: %s {check-xr|visualize-xr|check-arm|teleop} [arguments...]\n' "$0" >&2
        exit 2
        ;;
esac
