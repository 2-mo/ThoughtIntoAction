#!/usr/bin/env bash
set -euo pipefail

# Ubuntu-side foreground launcher for the ARX5 teleop backend.
# Run on Ubuntu:
#   cd /home/r2/work/projects/embodied/robot-teleop
#   ./run_arx5_backend_foreground.sh
#
# This does not pull code, reinstall packages, or use nohup. Ctrl-C stops it.
# If an old backend is still running, this script stops it first and restarts
# in this foreground terminal.

ROOT_DIR="${ROBOT_TELEOP_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
UV_BIN="${UV_BIN:-/home/r2/.local/bin/uv}"
CONFIG_PATH="${CONFIG_PATH:-configs/profiles/arkit_arx5.yaml}"
ROBOT_INTERFACE="${ROBOT_INTERFACE:-can0}"
DURATION_SECONDS="${DURATION_SECONDS:-86400}"
DEBUG_WEB_HOST="${DEBUG_WEB_HOST:-0.0.0.0}"
DEBUG_WEB_PORT="${DEBUG_WEB_PORT:-8080}"
GRIPPER_OPEN_READOUT="${GRIPPER_OPEN_READOUT:--3.24846}"

cd "${ROOT_DIR}"

if pgrep -f 'robot-teleop run.*arkit_arx5.yaml' >/dev/null; then
  echo "Existing ARX5 backend process found; stopping it before foreground recovery:"
  pgrep -af 'robot-teleop run.*arkit_arx5.yaml'
  pkill -f 'robot-teleop run.*arkit_arx5.yaml'
  sleep 1
  if pgrep -f 'robot-teleop run.*arkit_arx5.yaml' >/dev/null; then
    echo "Backend did not stop cleanly. Remaining process(es):"
    pgrep -af 'robot-teleop run.*arkit_arx5.yaml'
    exit 1
  fi
  echo "Old backend stopped. Starting a fresh foreground backend."
fi

echo "Starting ARX5 backend in the foreground."
echo "Working dir: ${ROOT_DIR}"
echo "Phone endpoint: 100.112.78.10:5005"
echo "Dashboard: http://100.112.78.10:${DEBUG_WEB_PORT}/"
echo "Stop: Ctrl-C"

exec "${UV_BIN}" run robot-teleop run \
  --config "${CONFIG_PATH}" \
  --enable-hardware \
  --allow-active-command \
  --allow-gripper-command \
  --gripper-open-readout "${GRIPPER_OPEN_READOUT}" \
  --allow-unverified-safety-envelope \
  --robot-interface "${ROBOT_INTERFACE}" \
  --duration "${DURATION_SECONDS}" \
  --debug-web \
  --debug-web-host "${DEBUG_WEB_HOST}" \
  --debug-web-port "${DEBUG_WEB_PORT}"
