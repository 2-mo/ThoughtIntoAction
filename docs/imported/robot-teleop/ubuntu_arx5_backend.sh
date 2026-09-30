#!/usr/bin/env bash
set -euo pipefail

# Mac-side control helper for the Ubuntu ARX5 backend.
# Usage:
#   ./ubuntu_arx5_backend.sh run
#   ./ubuntu_arx5_backend.sh start
#   ./ubuntu_arx5_backend.sh stop
#   ./ubuntu_arx5_backend.sh restart
#   ./ubuntu_arx5_backend.sh status
#   ./ubuntu_arx5_backend.sh logs

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

REMOTE_HOST="${REMOTE_HOST:-100.112.78.10}"
REMOTE_USER="${REMOTE_USER:-r2}"
REMOTE_DIR="${REMOTE_DIR:-/home/r2/work/projects/embodied/robot-teleop}"
REMOTE_BRANCH="${REMOTE_BRANCH:-main}"
ROBOT_INTERFACE="${ROBOT_INTERFACE:-can0}"
DURATION_SECONDS="${DURATION_SECONDS:-86400}"
LOG_FILE="${LOG_FILE:-logs/arkit-arx5.log}"
PID_FILE="${PID_FILE:-logs/arkit-arx5.pid}"
DEBUG_WEB_PORT="${DEBUG_WEB_PORT:-8080}"
UV_BIN="${UV_BIN:-/home/r2/.local/bin/uv}"
GRIPPER_OPEN_READOUT="${GRIPPER_OPEN_READOUT:--3.24846}"
TAIL_LINES="${TAIL_LINES:-80}"
ACTION="${1:-status}"

usage() {
  cat <<USAGE
Usage: $0 {run|start|stop|restart|status|logs}

  run      Start in the Mac terminal foreground; Ctrl-C stops the backend.
  start    Start with nohup in the Ubuntu background.
  stop     Stop the Ubuntu backend.
  restart  Stop, then start in the Ubuntu background.
  status   Show remote process status.
  logs     Tail the remote log.

Environment overrides:
  REMOTE_HOST=${REMOTE_HOST}
  REMOTE_USER=${REMOTE_USER}
  REMOTE_DIR=${REMOTE_DIR}
  REMOTE_BRANCH=${REMOTE_BRANCH}
  ROBOT_INTERFACE=${ROBOT_INTERFACE}
  DEBUG_WEB_PORT=${DEBUG_WEB_PORT}
USAGE
}

remote_stop() {
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    REMOTE_DIR="${REMOTE_DIR}" \
    PID_FILE="${PID_FILE}" \
    'bash -s' <<'REMOTE'
set -euo pipefail
cd "${REMOTE_DIR}"
stopped=0
if [[ -f "${PID_FILE}" ]]; then
  pid="$(cat "${PID_FILE}")"
  if [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null; then
    echo "Stopped ARX5 backend pid=${pid}"
    stopped=1
  fi
fi
if pkill -f 'robot-teleop run.*arkit_arx5.yaml' 2>/dev/null; then
  echo "Stopped remaining ARX5 backend process(es)"
  stopped=1
fi
if [[ "${stopped}" == "0" ]]; then
  echo "ARX5 backend was not running"
fi
REMOTE
}

remote_status() {
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    REMOTE_DIR="${REMOTE_DIR}" \
    LOG_FILE="${LOG_FILE}" \
    'bash -s' <<'REMOTE'
set -euo pipefail
cd "${REMOTE_DIR}"
if pgrep -f 'robot-teleop run.*arkit_arx5.yaml' >/dev/null; then
  echo "ARX5 backend running:"
  pgrep -af 'robot-teleop run.*arkit_arx5.yaml'
else
  echo "ARX5 backend not running"
fi
echo "Log: ${REMOTE_DIR}/${LOG_FILE}"
REMOTE
}

remote_logs() {
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    REMOTE_DIR="${REMOTE_DIR}" \
    LOG_FILE="${LOG_FILE}" \
    TAIL_LINES="${TAIL_LINES}" \
    'bash -s' <<'REMOTE'
set -euo pipefail
cd "${REMOTE_DIR}"
tail -n "${TAIL_LINES}" "${LOG_FILE}"
REMOTE
}

remote_run_foreground() {
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
  echo "ARX5 backend is already running; stop it first or use restart:"
  pgrep -af 'robot-teleop run.*arkit_arx5.yaml'
  exit 1
fi

echo "Running ARX5 backend in foreground."
echo "Mac Ctrl-C will stop this remote process."
echo "Dashboard: http://100.112.78.10:${DEBUG_WEB_PORT}/"

set +e
"${UV_BIN}" run robot-teleop run \
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
  2>&1 | tee "${LOG_FILE}"
status=${PIPESTATUS[0]}
set -e
exit "${status}"
REMOTE
}

case "${ACTION}" in
  run|foreground|fg)
    remote_stop
    remote_run_foreground
    ;;
  start)
    "${ROOT_DIR}/start_ubuntu_arx5_backend.sh"
    ;;
  stop)
    remote_stop
    ;;
  restart)
    remote_stop
    "${ROOT_DIR}/start_ubuntu_arx5_backend.sh"
    ;;
  status)
    remote_status
    echo "Dashboard: http://${REMOTE_HOST}:${DEBUG_WEB_PORT}/"
    ;;
  logs)
    remote_logs
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
