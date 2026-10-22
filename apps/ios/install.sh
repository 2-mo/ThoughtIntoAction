#!/usr/bin/env bash
# Rebuild and renew the existing personal development signature on a Mac.
set -euo pipefail

IOS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IOS_DEVICE="${1:-Surface Phone}"
IOS_BUILD="${IOS_ROOT}/../../outputs/build/ios-device"
if [[ "$(uname -s)" != Darwin ]]; then
  echo '请在已登录 Apple 开发者账号的 Mac 上执行此脚本。' >&2
  exit 1
fi
if [[ -z "${DEVELOPER_DIR:-}" && -d /Applications/Xcode.app/Contents/Developer ]]; then
  export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer
fi
xcodebuild -version
xcrun devicectl list devices
echo "重新签名并安装到：${IOS_DEVICE}。请保持手机解锁；如需登录，在 Xcode → Settings → Accounts 中操作。"
xcodebuild build -quiet \
  -project "${IOS_ROOT}/RobotTeleop/RobotTeleop.xcodeproj" \
  -scheme RobotTeleop -configuration Debug -destination 'generic/platform=iOS' \
  -derivedDataPath "${IOS_BUILD}" -allowProvisioningUpdates -allowProvisioningDeviceRegistration
IOS_APP="${IOS_BUILD}/Build/Products/Debug-iphoneos/RobotTeleop.app"
test -d "${IOS_APP}"
xcrun devicectl device install app --device "${IOS_DEVICE}" "${IOS_APP}"
if ! xcrun devicectl device process launch --device "${IOS_DEVICE}" com.mo.robotteleop; then
  echo 'App 已安装，但启动失败；请保留上方错误。若提示 profile 未信任：' >&2
  echo '在 iPhone 设置 → 通用 → VPN 与设备管理 → 开发者 App 中信任开发者，然后手动打开 RobotTeleop。' >&2
  exit 1
fi
echo 'RobotTeleop 已重新签名、安装并请求打开。服务器填写电脑在手机网络中的地址，端口 5005。'
