#!/usr/bin/env bash
# RealMan through the standard control kernel; no CAN or ARX lifecycle.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PROFILE="real-quest" YES=0 ARGS=()
while (($#)); do
  case "$1" in
    --profile) PROFILE="$2"; shift 2 ;;
    --yes) YES=1; shift ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
echo "◇ robot · RealMan · $PROFILE"
echo '启动只连接并读取状态，不自动使能、回零或张开；Ctrl+C 请求停止并保持。'
if ((YES == 0)); then
  read -r -p "准备好后输入 y 启动 robot：[y/N] " answer
  [[ "$answer" == [yY] ]] || { echo "未启动。"; exit 0; }
fi
if [[ "$PROFILE" == *quest* ]]; then
  "$ROOT/run.sh" quest-transport prepare
fi
exec "$ROOT/run.sh" kernel profiles start --profile "$PROFILE" "${ARGS[@]}"
