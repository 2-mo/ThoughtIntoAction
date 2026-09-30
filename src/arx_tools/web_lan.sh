#!/usr/bin/env bash
set -Eeuo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/project_env.sh"
ROOT_DIR="$ARX5_PROJECT_ROOT"
CONTROL_PYTHON="$ROOT_DIR/environments/arx_motion/.venv/bin/python"
WEB_PYTHON="$ROOT_DIR/environments/arx_motion/.venv-web/bin/python"
CERT_DIR="$ROOT_DIR/runtime/https"
TOKEN_FILE="$ROOT_DIR/runtime/web-pairing-token"
PID_FILE="$ROOT_DIR/runtime/web-lan.pid"
CHILD_PID_FILE="$ROOT_DIR/runtime/web-lan.children"
BIND_IP="${ARX5_WEB_BIND:-0.0.0.0}"

source "$ARX5_SCRIPTS_DIR/web_service_lifecycle.sh"
source "$ARX5_SCRIPTS_DIR/frontend_runtime.sh"

LAN_IP="${ARX5_WEB_IP:-}"
if [[ -z "$LAN_IP" ]]; then
  DEFAULT_DEVICE="$(ip -4 route show default 2>/dev/null | awk '{print $5; exit}')"
  if [[ -n "$DEFAULT_DEVICE" ]]; then
    LAN_IP="$(ip -o -4 address show dev "$DEFAULT_DEVICE" scope global \
      | awk '{split($4, value, "/"); print value[1]; exit}')"
  fi
fi
python3 -c 'import ipaddress, sys; ipaddress.ip_address(sys.argv[1])' "$BIND_IP"
if [[ -z "$LAN_IP" ]]; then
  echo "无法自动确定局域网 IP。可运行：ARX5_WEB_IP=192.168.x.x ./arx5 web-lan" >&2
  exit 1
fi

if [[ ! -x "$CONTROL_PYTHON" || ! -x "$WEB_PYTHON" ]]; then
  echo "缺少 .venv-handeye 或 .venv-web。请查看 docs/consolidated-components.md。" >&2
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

umask 077
mkdir -p "$(dirname "$TOKEN_FILE")"
if [[ ! -s "$TOKEN_FILE" ]]; then
  openssl rand -hex 16 >"$TOKEN_FILE"
fi
PAIRING_TOKEN="$(tr -d '\r\n' <"$TOKEN_FILE")"
ORIGIN="https://$LAN_IP:8000"
CERT="$CERT_DIR/arx5-web.crt"
KEY="$CERT_DIR/arx5-web.key"
CA_CERT="$CERT_DIR/arx5-local-ca.crt"

if [[ -s "$PID_FILE" ]]; then
  RUNNING_PID="$(tr -d '[:space:]' <"$PID_FILE")"
  if [[ "$RUNNING_PID" =~ ^[0-9]+$ ]] && kill -0 "$RUNNING_PID" >/dev/null 2>&1; then
    RUNNING_COMMAND="$(ps -p "$RUNNING_PID" -o args= 2>/dev/null || true)"
    if [[ "$RUNNING_COMMAND" == *"src/scripts/web_lan.sh"* ]]; then
      echo "局域网网页服务已经运行（PID $RUNNING_PID）。"
      echo "直接访问：$ORIGIN/?token=$PAIRING_TOKEN"
      echo "GR00T 采集：$ORIGIN/collection?token=$PAIRING_TOKEN"
      echo "如需重启，请先在原终端按 Ctrl+C。"
      exit 0
    fi
  fi
  rm -f "$PID_FILE"
fi

port_available() {
  "$WEB_PYTHON" -B -c '
import errno
import socket
import sys

host, port = sys.argv[1], int(sys.argv[2])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.bind((host, port))
    sock.listen(1)
except OSError as exc:
    raise SystemExit(2 if exc.errno == errno.EADDRINUSE else 3)
finally:
    sock.close()
' "$1" "$2"
}

OCCUPIED_PORTS=()
for SERVICE_PORT in 8000 8765 8767; do
  if port_available "$BIND_IP" "$SERVICE_PORT"; then
    continue
  else
    PORT_STATUS=$?
  fi
  if [[ "$PORT_STATUS" -eq 2 ]]; then
    OCCUPIED_PORTS+=("$SERVICE_PORT")
  else
    echo "无法检查监听地址 $BIND_IP:$SERVICE_PORT；请确认该地址属于本机。" >&2
    exit 1
  fi
done
if port_available 127.0.0.1 8768; then
  :
else
  PORT_STATUS=$?
  if [[ "$PORT_STATUS" -eq 2 ]]; then
    OCCUPIED_PORTS+=("127.0.0.1:8768")
  else
    echo "无法检查主机内部相机端口 127.0.0.1:8768。" >&2
    exit 1
  fi
fi
if ((${#OCCUPIED_PORTS[@]})); then
  echo "无法启动：$BIND_IP 上的端口已被占用：${OCCUPIED_PORTS[*]}" >&2
  if ((${#OCCUPIED_PORTS[@]} == 4)); then
    echo "很可能已有一组 ./arx5 web-lan 或 ./arx5 web-tailnet 正在运行。" >&2
  fi
  if [[ -s "$CHILD_PID_FILE" ]]; then
    echo "检测到上次网页服务的子进程记录：" >&2
    while IFS='=' read -r service_name service_pid; do
      if [[ "$service_name" != "launcher" ]] \
          && web_process_running "$service_pid"; then
        service_command="$(ps -p "$service_pid" -o args= 2>/dev/null || true)"
        echo "  $service_name PID $service_pid：$service_command" >&2
      fi
    done <"$CHILD_PID_FILE"
  fi
  echo "请直接使用已有服务，或在原终端按 Ctrl+C 后重试。" >&2
  exit 1
fi

"$ARX5_SCRIPTS_DIR/create_web_cert.sh" "$LAN_IP" "$CERT_DIR"
printf '%s\n' "$$" >"$PID_FILE"

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
  if [[ -s "$PID_FILE" ]] && [[ "$(tr -d '[:space:]' <"$PID_FILE")" == "$$" ]]; then
    rm -f "$PID_FILE" "$CHILD_PID_FILE"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid python3 -B -m arx_tools.web_service_supervisor --name 控制 -- \
  "$CONTROL_PYTHON" -B -m arx5_backend \
  --host "$BIND_IP" --port 8000 --cert "$CERT" --key "$KEY" \
  --origin "$ORIGIN" --token "$PAIRING_TOKEN" \
  --frontend-directory "$FRONTEND_DIST" --ca "$CA_CERT" --auto-connect-arm \
  --camera-url "ws://127.0.0.1:8768/api/v1/cameras/collection" &
CONTROL_PID=$!
setsid python3 -B -m arx_tools.web_service_supervisor --name 视觉 -- \
  "$WEB_PYTHON" -B -m arx5_backend.features.vision.service \
  --host "$BIND_IP" --cert "$CERT" --key "$KEY" \
  --origin "$ORIGIN" --token "$PAIRING_TOKEN" &
VISION_PID=$!
setsid python3 -B -m arx_tools.web_service_supervisor --name 相机 -- \
  "$ARX5_SCRIPTS_DIR/camera_webrtc_supervisor.sh" \
  "$WEB_PYTHON" -B -m arx5_backend.features.camera.service \
  --host "$BIND_IP" --cert "$CERT" --key "$KEY" \
  --origin "$ORIGIN" --token "$PAIRING_TOKEN" &
CAMERA_PID=$!

{
  printf 'launcher=%s\n' "$$"
  printf 'control=%s\n' "$CONTROL_PID"
  printf 'vision=%s\n' "$VISION_PID"
  printf 'camera=%s\n' "$CAMERA_PID"
} >"$CHILD_PID_FILE"

sleep 0.4
for pid in "$CONTROL_PID" "$VISION_PID" "$CAMERA_PID"; do
  if ! kill -0 "$pid" >/dev/null 2>&1; then
    echo "局域网网页服务启动失败。" >&2
    exit 1
  fi
done

echo
echo "直接访问：$ORIGIN/?token=$PAIRING_TOKEN"
echo "GR00T 采集：$ORIGIN/collection?token=$PAIRING_TOKEN"
echo "监听地址：$BIND_IP（其他本机网络地址不接受连接）"
echo "首次使用的 Mac CA 下载地址：$ORIGIN/arx5-local-ca.crt"
echo "控制台只接受上述网页来源和配对令牌。"
echo "D455 WebRTC 信令：wss://$LAN_IP:8767/api/v1/cameras/d455/webrtc"
echo "视觉：wss://$LAN_IP:8765/api/v1/vision/landmarks"
echo "控制 API：$ORIGIN/api/v1/control（与网页同源）"
echo "按 Ctrl+C 停止全部服务并安全释放机械臂。"
wait -n "$CONTROL_PID" "$VISION_PID" "$CAMERA_PID"
