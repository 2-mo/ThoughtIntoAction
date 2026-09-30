#!/usr/bin/env bash
set -euo pipefail

# Start the Ubuntu-side ARX5 backend from a Mac terminal over SSH.
# Override any setting with environment variables before running this script.

REMOTE_HOST="${REMOTE_HOST:-100.112.78.10}"
REMOTE_USER="${REMOTE_USER:-r2}"
REMOTE_DIR="${REMOTE_DIR:-/home/r2/work/projects/embodied/robot-teleop}"
REMOTE_BRANCH="${REMOTE_BRANCH:-main}"
ROBOT_INTERFACE="${ROBOT_INTERFACE:-can0}"
DURATION_SECONDS="${DURATION_SECONDS:-86400}"
DEBUG_WEB_PORT="${DEBUG_WEB_PORT:-8080}"
LOG_FILE="${LOG_FILE:-logs/arkit-arx5.log}"
PID_FILE="${PID_FILE:-logs/arkit-arx5.pid}"
UV_BIN="${UV_BIN:-/home/r2/.local/bin/uv}"
GRIPPER_OPEN_READOUT="${GRIPPER_OPEN_READOUT:--3.24846}"

echo "Starting ARX5 backend on ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}"
echo "Dashboard will be available at http://${REMOTE_HOST}:${DEBUG_WEB_PORT}/"

ssh "${REMOTE_USER}@${REMOTE_HOST}" \
  REMOTE_DIR="${REMOTE_DIR}" \
  REMOTE_BRANCH="${REMOTE_BRANCH}" \
  ROBOT_INTERFACE="${ROBOT_INTERFACE}" \
  DURATION_SECONDS="${DURATION_SECONDS}" \
  DEBUG_WEB_PORT="${DEBUG_WEB_PORT}" \
  LOG_FILE="${LOG_FILE}" \
  PID_FILE="${PID_FILE}" \
  UV_BIN="${UV_BIN}" \
  GRIPPER_OPEN_READOUT="${GRIPPER_OPEN_READOUT}" \
  'bash -s' <<'REMOTE'
set -euo pipefail

cd "${REMOTE_DIR}"
git pull --ff-only origin "${REMOTE_BRANCH}"
mkdir -p "$(dirname "${LOG_FILE}")" "$(dirname "${PID_FILE}")"

if pgrep -f 'robot-teleop run.*arkit_arx5.yaml' >/dev/null; then
  echo "ARX5 backend is already running:"
  pgrep -af 'robot-teleop run.*arkit_arx5.yaml'
  echo "Log: ${REMOTE_DIR}/${LOG_FILE}"
  exit 0
fi

nohup "${UV_BIN}" run robot-teleop run \
  --config configs/profiles/arkit_arx5.yaml \
  --enable-hardware \
  --allow-active-command \
  --allow-gripper-command \
  --gripper-open-readout "${GRIPPER_OPEN_READOUT}" \
  --allow-unverified-safety-envelope \
  --robot-interface "${ROBOT_INTERFACE}" \
  --duration "${DURATION_SECONDS}" \
  --debug-web \
  --debug-web-host 0.0.0.0 \
  --debug-web-port "${DEBUG_WEB_PORT}" \
  > "${LOG_FILE}" 2>&1 < /dev/null &

pid=$!
echo "${pid}" > "${PID_FILE}"
echo "Started ARX5 backend: pid=${pid}"
echo "Log: ${REMOTE_DIR}/${LOG_FILE}"
echo "Dashboard: http://${HOSTNAME}:${DEBUG_WEB_PORT}/ or http://100.112.78.10:${DEBUG_WEB_PORT}/"
REMOTE

