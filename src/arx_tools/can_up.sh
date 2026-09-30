#!/usr/bin/env bash
set -Eeuo pipefail

INTERFACE="can0"
DEVICE=""
SPEED_CODE="8"
FORCE_RESTART=0
STATUS_ONLY=0
DRY_RUN=0

usage() {
  cat <<'EOF'
用法：
  ./arx5 can-up [--device PATH] [--interface can0] [--force]
  ./arx5 can-status

选项：
  --device PATH       USB-CAN 串口；默认自动查找 ARX USB2CAN by-id
  --interface NAME    SocketCAN 接口名（默认：can0）
  --speed-code 0..8   SLCAN 速率代码（默认：8，即 1 Mbit/s）
  --force             即使接口存在，也重启对应的 slcand
  --status            只显示状态，不修改系统
  --dry-run           检查设备并显示恢复计划，不请求 sudo
EOF
}

while (($#)); do
  case "$1" in
    --device)
      DEVICE="${2:?--device 需要路径}"
      shift 2
      ;;
    --interface)
      INTERFACE="${2:?--interface 需要名称}"
      shift 2
      ;;
    --speed-code)
      SPEED_CODE="${2:?--speed-code 需要 0..8}"
      shift 2
      ;;
    --force)
      FORCE_RESTART=1
      shift
      ;;
    --status)
      STATUS_ONLY=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$INTERFACE" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo "接口名只能包含字母、数字、下划线和连字符。" >&2
  exit 2
fi
if [[ ! "$SPEED_CODE" =~ ^[0-8]$ ]]; then
  echo "SLCAN speed code 必须是 0..8。" >&2
  exit 2
fi

show_status() {
  if ! ip link show dev "$INTERFACE" >/dev/null 2>&1; then
    echo "$INTERFACE 不存在。" >&2
    return 1
  fi
  ip -details -statistics link show dev "$INTERFACE"
}

if ((STATUS_ONLY)); then
  show_status
  exit $?
fi

for command_name in ip slcand flock; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "缺少命令：$command_name" >&2
    exit 1
  fi
done

if ip link show dev "$INTERFACE" >/dev/null 2>&1 && ((FORCE_RESTART == 0)); then
  echo "$INTERFACE 已存在；无需重建。使用 can-restart 可强制重启。"
  show_status
  exit 0
fi

lock_file="/tmp/arx5-control-${INTERFACE}.lock"
if ! flock -n "$lock_file" -c true; then
  echo "机械臂控制器正在占用 $INTERFACE；拒绝重启 CAN。" >&2
  exit 1
fi

if [[ -z "$DEVICE" ]]; then
  shopt -s nullglob
  candidates=(/dev/serial/by-id/usb-ARX_USB2CAN_*-if00)
  shopt -u nullglob
  if ((${#candidates[@]} == 0)); then
    echo "未找到 /dev/serial/by-id/usb-ARX_USB2CAN_*-if00。" >&2
    echo "请检查 USB-CAN，或用 --device /dev/ttyACM0 指定。" >&2
    exit 1
  fi
  if ((${#candidates[@]} > 1)); then
    echo "检测到多个 ARX USB-CAN，请用 --device 明确指定：" >&2
    printf '  %s\n' "${candidates[@]}" >&2
    exit 1
  fi
  DEVICE="${candidates[0]}"
fi

if [[ ! -c "$DEVICE" ]]; then
  echo "USB-CAN 设备不可用：$DEVICE" >&2
  exit 1
fi

mapfile -t stale_pids < <(
  ps -eo pid=,comm=,args= |
    awk -v interface="$INTERFACE" '$2 == "slcand" && $NF == interface {print $1}'
)
if ((DRY_RUN)); then
  echo "dry-run：不会修改系统。"
  echo "  USB-CAN：$DEVICE"
  echo "  接口：$INTERFACE"
  echo "  SLCAN speed code：s$SPEED_CODE"
  if ((${#stale_pids[@]})); then
    echo "  将停止旧进程：${stale_pids[*]}"
  else
    echo "  没有找到对应的旧 slcand"
  fi
  echo "  将创建并拉起 $INTERFACE"
  exit 0
fi

if ((EUID == 0)); then
  ROOT=()
else
  if ! command -v sudo >/dev/null 2>&1; then
    echo "需要 root 权限，但系统没有 sudo。" >&2
    exit 1
  fi
  sudo -v
  ROOT=(sudo)
fi

if ((${#stale_pids[@]})); then
  echo "停止旧的 $INTERFACE SLCAN 进程：${stale_pids[*]}"
  "${ROOT[@]}" kill "${stale_pids[@]}"
fi

if ip link show dev "$INTERFACE" >/dev/null 2>&1; then
  "${ROOT[@]}" ip link set dev "$INTERFACE" down 2>/dev/null || true
fi
for _ in {1..20}; do
  if ! ip link show dev "$INTERFACE" >/dev/null 2>&1; then
    break
  fi
  sleep 0.05
done
if ip link show dev "$INTERFACE" >/dev/null 2>&1; then
  "${ROOT[@]}" ip link delete dev "$INTERFACE"
fi

echo "创建 $INTERFACE：$DEVICE，SLCAN speed code s$SPEED_CODE"
"${ROOT[@]}" slcand -o -f "-s${SPEED_CODE}" "$DEVICE" "$INTERFACE"

for _ in {1..40}; do
  if ip link show dev "$INTERFACE" >/dev/null 2>&1; then
    break
  fi
  sleep 0.05
done
if ! ip link show dev "$INTERFACE" >/dev/null 2>&1; then
  echo "slcand 已启动，但没有创建 $INTERFACE。" >&2
  exit 1
fi

"${ROOT[@]}" ip link set dev "$INTERFACE" up
echo "$INTERFACE 已恢复。ERROR-ACTIVE 是正常工作状态。"
show_status
