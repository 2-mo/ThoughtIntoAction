#!/usr/bin/env bash
set -Eeuo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/project_env.sh"
ROOT_DIR="$ARX5_PROJECT_ROOT"
WEB_PYTHON="$ROOT_DIR/environments/arx_motion/.venv-web/bin/python"
source "$ARX5_SCRIPTS_DIR/frontend_runtime.sh"

if [[ ! -x "$WEB_PYTHON" ]]; then
  echo "缺少 .venv-web。请执行：" >&2
  echo "  ./run.sh components setup web" >&2
  exit 1
fi
FRONTEND_DIST="$(frontend_dist_dir "$ROOT_DIR")"
if ! "$WEB_PYTHON" -B -c 'import aiortc, av, cv2, numpy, pyrealsense2, websockets' >/dev/null 2>&1; then
  echo ".venv-web 缺少 WebRTC 或 RealSense 依赖；请执行 uv sync --only-group web。" >&2
  exit 1
fi

cd "$ROOT_DIR"
"$WEB_PYTHON" -B -m embodied.console.features.vision.service &
BRIDGE_PID=$!
"$ARX5_SCRIPTS_DIR/camera_webrtc_supervisor.sh" \
  "$WEB_PYTHON" -B -m embodied.console.features.camera.service &
CAMERA_PID=$!

cleanup() {
  for pid in "$BRIDGE_PID" "$CAMERA_PID"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait "$BRIDGE_PID" 2>/dev/null || true
  wait "$CAMERA_PID" 2>/dev/null || true
}
trap cleanup EXIT TERM

sleep 0.2
for pid in "$BRIDGE_PID" "$CAMERA_PID"; do
  if ! kill -0 "$pid" >/dev/null 2>&1; then
    echo "视觉服务启动失败。" >&2
    exit 1
  fi
done

echo "网页：http://127.0.0.1:8000"
echo "WebSocket：ws://127.0.0.1:8765/api/v1/vision/landmarks"
echo "D455 WebRTC：ws://127.0.0.1:8767/api/v1/cameras/d455/webrtc"
echo "按 Ctrl+C 同时停止网页和视觉桥。"
python3 -B -m legacy.arx_tools.https_static_server \
  --host 127.0.0.1 --directory "$FRONTEND_DIST" --spa
