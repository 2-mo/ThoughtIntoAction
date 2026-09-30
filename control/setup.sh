#!/usr/bin/env bash
# Rebuild dependencies locally; never connect a robot or configure its network.
set -euo pipefail
if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
    printf '%s\n' '用法：./setup.sh [arx|sim]' \
        '使用 uv.lock 创建本目录的 Python 3.10 环境；默认 arx。' \
        'sim 不安装机械臂 SDK。可设置 ARX5_PYTHON 指定 Python 解释器。' \
        '需要 Linux x86_64 和 uv；不会启动 VR 或连接机械臂。'
    exit 0
fi
if [[ $# -gt 1 ]]; then
    printf '%s\n' '参数过多；运行 ./setup.sh --help。' >&2
    exit 2
fi
profile="${1:-arx}"
extras=()
case "$profile" in
    sim) ;;
    arx) extras=(--extra arx) ;;
    *) printf '%s\n' '安装模式必须是 arx 或 sim。' >&2; exit 2 ;;
esac
ARX5_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$ARX5_ROOT"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
    printf '%s\n' '本交付包支持 Linux x86_64；其他平台需重建原生程序并重新验证。' >&2
    exit 1
fi
if ! command -v uv >/dev/null 2>&1; then
    printf '%s\n' '未找到 uv。安装说明：https://docs.astral.sh/uv/getting-started/installation/' >&2
    exit 1
fi
uv sync --locked --python "${ARX5_PYTHON:-3.10}" "${extras[@]}"
exec "$ARX5_ROOT/doctor.sh"
