#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PYTHON_BIN="/home/r2/miniconda3/envs/xrobot312/bin/python"
readonly CONDA_LIB="/home/r2/miniconda3/envs/xrobot312/lib"
readonly ARX_X5_PYTHON_ROOT="${SCRIPT_DIR}/../X5/py/arx_x5_python"
readonly ARX_X5_API_ROOT="${ARX_X5_PYTHON_ROOT}/bimanual/api"

export LD_LIBRARY_PATH="${CONDA_LIB}:${ARX_X5_API_ROOT}/arx_x5_src:${ARX_X5_API_ROOT}:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${ARX_X5_PYTHON_ROOT}:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-${CONDA_PREFIX:-/home/r2/miniconda3/envs/xrobot312}}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    printf 'Python environment not found: %s\n' "${PYTHON_BIN}" >&2
    exit 1
fi

command_name="${1:-}"
if [[ -n "${command_name}" ]]; then
    shift
fi

case "${command_name}" in
    setup-quest-usb)
        exec env -u LD_LIBRARY_PATH "${SCRIPT_DIR}/setup_quest_usb.sh" "$@"
        ;;
    build-x5-2025-sdk)
        exec "${SCRIPT_DIR}/build_x5_2025_sdk.sh" "$@"
        ;;
    check-xr)
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/check_xr.py" "$@"
        ;;
    visualize-xr)
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/visualize_xr.py" "$@"
        ;;
    simulate-x5|simulate-r5)
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/simulate_x5.py" "$@"
        ;;
    simulate-official-x5)
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/simulate_official_x5.py" "$@"
        ;;
    check-arm)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/check_arm.py" "$@"
        ;;
    test-joint)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/test_x5_joint_tracking.py" "$@"
        ;;
    test-joint-2025)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/test_x5_joint_tracking.py" --backend arx2025 "$@"
        ;;
    teleop-dry-run)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/teleop_single_x5.py" --dry-run "$@"
        ;;
    teleop-official-x5)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/teleop_official_x5.py" "$@"
        ;;
    teleop-presets-x5)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/teleop_official_x5_presets.py" "$@"
        ;;
    teleop-x5-2025)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/teleop_arx_x5_2025.py" "$@"
        ;;
    teleop)
        if ! ip link show can0 >/dev/null 2>&1; then
            printf 'SocketCAN interface can0 does not exist or is unavailable.\n' >&2
            exit 1
        fi
        exec "${PYTHON_BIN}" "${SCRIPT_DIR}/teleop_single_x5.py" "$@"
        ;;
    *)
        printf 'Usage: %s {setup-quest-usb|build-x5-2025-sdk|check-xr|visualize-xr|simulate-x5|simulate-official-x5|check-arm|test-joint|test-joint-2025|teleop-official-x5|teleop-presets-x5|teleop-x5-2025|teleop-dry-run|teleop} [arguments...]\n' "$0" >&2
        exit 2
        ;;
esac
