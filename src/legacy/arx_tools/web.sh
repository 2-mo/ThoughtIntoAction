#!/usr/bin/env bash
set -Eeuo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/project_env.sh"
ROOT_DIR="$ARX5_PROJECT_ROOT"
CONTROL_PYTHON="$ROOT_DIR/environments/arx_motion/.venv/bin/python"
WEB_PYTHON="$ROOT_DIR/environments/arx_motion/.venv-web/bin/python"

source "$ARX5_SCRIPTS_DIR/web_service_lifecycle.sh"
source "$ARX5_SCRIPTS_DIR/frontend_runtime.sh"

if [[ ! -x "$CONTROL_PYTHON" || ! -x "$WEB_PYTHON" ]]; then
  echo "缺少 .venv-handeye 或 .venv-web。请查看 docs/guides/consolidated-components.md。" >&2
  exit 1
fi
if ! "$CONTROL_PYTHON" -B -c 'import arx5_interface, fastapi, ruckig, uvicorn, websockets' >/dev/null 2>&1; then
  echo "控制环境缺少 ARX SDK、FastAPI、Uvicorn、Ruckig 或 websockets。" >&2
  echo "执行：./run.sh components setup analysis" >&2
  exit 1
fi
FRONTEND_DIST="$(frontend_dist_dir "$ROOT_DIR")"
if ! "$WEB_PYTHON" -B -c 'import aiortc, av, cv2, numpy, pyrealsense2, websockets' >/dev/null 2>&1; then
  echo "视觉环境缺少 WebRTC 或 RealSense 依赖。" >&2
  exit 1
fi

cd "$ROOT_DIR"
CONTROL_PID=""
VISION_PID=""
CAMERA_PID=""
CLEANUP_STARTED=0

cleanup() {
  ((CLEANUP_STARTED == 0)) || return
  CLEANUP_STARTED=1
  trap '' INT TERM
  web_stop_process_groups 12 \
    "控制:$CONTROL_PID" \
    "视觉:$VISION_PID" \
    "相机:$CAMERA_PID"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid python3 -B -m legacy.arx_tools.web_service_supervisor --name 控制 -- \
  "$CONTROL_PYTHON" -B -m embodied.console \
  --port 8000 --frontend-directory "$FRONTEND_DIST" --auto-connect-arm \
  --camera-url "ws://127.0.0.1:8768/api/v1/cameras/collection" &
CONTROL_PID=$!
setsid python3 -B -m legacy.arx_tools.web_service_supervisor --name 视觉 -- \
  "$WEB_PYTHON" -B -m embodied.console.features.vision.service &
VISION_PID=$!
setsid python3 -B -m legacy.arx_tools.web_service_supervisor --name 相机 -- \
  "$ARX5_SCRIPTS_DIR/camera_webrtc_supervisor.sh" \
  "$WEB_PYTHON" -B -m embodied.console.features.camera.service &
CAMERA_PID=$!
sleep 0.3
if ! kill -0 "$CONTROL_PID" >/dev/null 2>&1; then
  echo "网页控制服务启动失败。" >&2
  wait "$CONTROL_PID"
fi
if ! kill -0 "$VISION_PID" >/dev/null 2>&1; then
  echo "视觉桥启动失败。" >&2
  wait "$VISION_PID"
fi
if ! kill -0 "$CAMERA_PID" >/dev/null 2>&1; then
  echo "D455 WebRTC 服务启动失败。" >&2
  wait "$CAMERA_PID"
fi

echo "网页：http://127.0.0.1:8000"
echo "GR00T 采集：http://127.0.0.1:8000/collection"
echo "控制：ws://127.0.0.1:8000/api/v1/control"
echo "视觉：ws://127.0.0.1:8765/api/v1/vision/landmarks"
echo "D455 WebRTC：ws://127.0.0.1:8767/api/v1/cameras/d455/webrtc"
echo "按 Ctrl+C 停止全部网页服务并安全释放机械臂。"
wait -n "$CONTROL_PID" "$VISION_PID" "$CAMERA_PID"
