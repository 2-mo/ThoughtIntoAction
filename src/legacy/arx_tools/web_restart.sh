#!/usr/bin/env bash
set -Eeuo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/project_env.sh"
ROOT_DIR="$ARX5_PROJECT_ROOT"
CHILD_PID_FILE="$ROOT_DIR/runtime/web-lan.children"
PID_FILE="$ROOT_DIR/runtime/web-lan.pid"

if [[ "${1:-}" != "--confirm-arm-supported" || $# -ne 1 ]]; then
  echo "用法：./arx5 web-restart --confirm-arm-supported" >&2
  echo "仅在机械臂已有支撑、实体急停可用时执行。" >&2
  exit 2
fi
if [[ ! -s "$CHILD_PID_FILE" ]]; then
  echo "找不到网页服务 PID 记录；为避免误停其他程序，未执行重启。" >&2
  exit 1
fi

source "$ARX5_SCRIPTS_DIR/web_service_lifecycle.sh"

expected_process() {
  local service_name="$1"
  local command="$2"
  [[ "$command" == *"$ROOT_DIR"* ]] || return 1
  case "$service_name" in
    control)
      [[ "$command" == *"embodied.console"* \
        || "$command" == *"legacy.arx_tools.web_service_supervisor --name 控制"* ]]
      ;;
    vision)
      [[ "$command" == *"embodied.console.features.vision.service"* \
        || "$command" == *"legacy.arx_tools.web_service_supervisor --name 视觉"* ]]
      ;;
    camera)
      [[ "$command" == *"arx_tools.camera_webrtc_supervisor"* \
        || "$command" == *"embodied.console.features.camera.service"* \
        || "$command" == *"legacy.arx_tools.web_service_supervisor --name 相机"* ]]
      ;;
    https)
      [[ "$command" == *"legacy.arx_tools.https_static_server"* \
        || "$command" == *"legacy.arx_tools.web_service_supervisor --name 网页"* ]]
      ;;
    *) return 1 ;;
  esac
}

declare -a services=()
while IFS='=' read -r service_name service_pid; do
  case "$service_name" in
    control|vision|camera|https) ;;
    *) continue ;;
  esac
  if ! web_process_running "$service_pid"; then
    continue
  fi
  service_command="$(ps -p "$service_pid" -o args= 2>/dev/null || true)"
  if ! expected_process "$service_name" "$service_command"; then
    echo "PID 记录与当前进程不匹配；拒绝停止 $service_name PID $service_pid。" >&2
    exit 1
  fi
  services+=("$service_name:$service_pid")
done <"$CHILD_PID_FILE"

if ((${#services[@]})); then
  echo "正在停止记录的网页服务：${services[*]}" >&2
  web_stop_process_groups 12 "${services[@]}"
fi

for service in "${services[@]}"; do
  service_pid="${service#*:}"
  if web_process_running "$service_pid"; then
    echo "服务 PID $service_pid 仍在运行；为避免端口冲突，未启动新服务。" >&2
    exit 1
  fi
done

rm -f "$PID_FILE" "$CHILD_PID_FILE"
exec "$ROOT_DIR/arx5" web-tailnet
