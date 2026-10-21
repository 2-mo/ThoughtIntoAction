#!/usr/bin/env bash
set -u

readonly CAMERA_RESTART_EXIT_CODE=75
CAMERA_CHILD_PID=""

stop_camera_child() {
  if [[ "$CAMERA_CHILD_PID" =~ ^[0-9]+$ ]] \
      && kill -0 "$CAMERA_CHILD_PID" >/dev/null 2>&1; then
    kill "$CAMERA_CHILD_PID" >/dev/null 2>&1 || true
    wait "$CAMERA_CHILD_PID" 2>/dev/null || true
  fi
  CAMERA_CHILD_PID=""
}

trap 'stop_camera_child; exit 130' INT
trap 'stop_camera_child; exit 143' TERM

while true; do
  python3 -B -m legacy.arx_tools.web_exec_guard -- "$@" &
  CAMERA_CHILD_PID=$!
  if wait "$CAMERA_CHILD_PID"; then
    CAMERA_STATUS=0
  else
    CAMERA_STATUS=$?
  fi
  CAMERA_CHILD_PID=""

  if [[ "$CAMERA_STATUS" -ne "$CAMERA_RESTART_EXIT_CODE" ]]; then
    exit "$CAMERA_STATUS"
  fi
  sleep 0.2
done
