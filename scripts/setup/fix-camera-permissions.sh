#!/usr/bin/env bash
# Run as the normal project user; only installing/applying rules needs sudo.
set -euo pipefail
CAMERA_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$CAMERA_PROJECT_DIR"
if [[ "$EUID" -eq 0 ]]; then
    echo '请以普通用户运行 bash scripts/setup/fix-camera-permissions.sh；脚本会自行请求 sudo。' >&2
    exit 1
fi
if [[ " $(id -nG) " != *" plugdev "* ]]; then
    echo '当前会话不属于 plugdev 组，请先配置用户组后重新登录。' >&2
    exit 1
fi
udevadm verify scripts/setup/udev/99-questarm-realsense.rules
sudo install -m 0644 scripts/setup/udev/99-questarm-realsense.rules /etc/udev/rules.d/99-questarm-realsense.rules
sudo udevadm control --reload-rules
for camera_device in /sys/bus/usb/devices/*; do
    [[ -f "$camera_device/idVendor" && -f "$camera_device/idProduct" ]] || continue
    [[ "$(<"$camera_device/idVendor")" == 8086 ]] || continue
    case "$(<"$camera_device/idProduct")" in
        0b07|0b3a)
            sudo udevadm trigger --action=change --parent-match="$(readlink -f -- "$camera_device")"
            ;;
    esac
done
sudo udevadm settle --timeout=10
echo '相机权限规则已安装；等待现有相机服务自动重连。'
PYTHONPATH="$CAMERA_PROJECT_DIR/src" "$CAMERA_PROJECT_DIR/.venv/bin/python" -u -B - <<'PY'
import time
from robot_core.camera_client import ensure_service, socket_path
from robot_core.session_protocol import request

ensure_service()
deadline = time.monotonic() + 20
while True:
    feeds = request(socket_path(), 'monitor', timeout=2)['cameras']
    if len(feeds) == 2 and all(
        not f['camera']['error'] and f['camera']['metadata']
        and f['preview']['jpeg_base64'] and f['preview']['age_s'] < .5
        for f in feeds.values()
    ):
        break
    if time.monotonic() >= deadline:
        raise SystemExit('权限已更新，相机尚未出图：' + str(
            {key: f['camera']['error'] for key, f in feeds.items()}))
    time.sleep(.5)
for key, feed in feeds.items():
    meta = feed['camera']['metadata']
    rgb = meta.get('rgb_controls', {})
    print(key, meta['model'], 'USB', meta['usb_type'], str(meta['fps']) + ' FPS',
          '白平衡', rgb.get('white_balance'), 'K',
          '曝光', rgb.get('exposure', 0) * meta.get('rgb_exposure_unit_us', 100) / 1000, 'ms',
          '增益', rgb.get('gain'))
print('双相机画面已恢复：http://127.0.0.1:18081/ 。未启动机械臂服务。')
PY
