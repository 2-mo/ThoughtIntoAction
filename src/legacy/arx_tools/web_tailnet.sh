#!/usr/bin/env bash
set -Eeuo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/project_env.sh"
ROOT_DIR="$ARX5_PROJECT_ROOT"

if ! command -v tailscale >/dev/null 2>&1; then
  echo "未安装 tailscale。" >&2
  exit 1
fi
TAILSCALE_IP="$(tailscale ip -4 2>/dev/null | head -n 1)"
if [[ -z "$TAILSCALE_IP" ]]; then
  echo "Tailscale 尚未连接；请先检查 tailscale status。" >&2
  exit 1
fi
python3 -c 'import ipaddress, sys; ipaddress.ip_address(sys.argv[1])' "$TAILSCALE_IP"

export ARX5_WEB_IP="$TAILSCALE_IP"
export ARX5_WEB_BIND="$TAILSCALE_IP"
exec "$ARX5_SCRIPTS_DIR/web_lan.sh" "$@"
