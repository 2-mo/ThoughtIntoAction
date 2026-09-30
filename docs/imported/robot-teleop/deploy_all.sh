#!/usr/bin/env bash
set -euo pipefail

# Fast three-endpoint sync/deploy from the Mac:
# 1. local git commit/push
# 2. Ubuntu ARX5 backend restart from the pushed branch
# 3. connected iPhone build/install
#
# Override any setting with environment variables before running.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

REMOTE_HOST="${REMOTE_HOST:-100.112.78.10}"
REMOTE_USER="${REMOTE_USER:-r2}"
REMOTE_DIR="${REMOTE_DIR:-/home/r2/work/projects/embodied/robot-teleop}"
REMOTE_BRANCH="${REMOTE_BRANCH:-$(git branch --show-current)}"
ROBOT_INTERFACE="${ROBOT_INTERFACE:-can0}"
DEBUG_WEB_PORT="${DEBUG_WEB_PORT:-8080}"
LOG_FILE="${LOG_FILE:-logs/arkit-arx5.log}"
PID_FILE="${PID_FILE:-logs/arkit-arx5.pid}"
UV_BIN="${UV_BIN:-/home/r2/.local/bin/uv}"
GRIPPER_OPEN_READOUT="${GRIPPER_OPEN_READOUT:--3.24846}"

XCODE_DEVELOPER_DIR="${XCODE_DEVELOPER_DIR:-/Applications/Xcode.app/Contents/Developer}"
IOS_PROJECT="${IOS_PROJECT:-apps/ios/RobotTeleop/RobotTeleop.xcodeproj}"
IOS_SCHEME="${IOS_SCHEME:-RobotTeleop}"
IOS_CONFIGURATION="${IOS_CONFIGURATION:-Debug}"
IOS_DERIVED_DATA="${IOS_DERIVED_DATA:-build/ios-device-derived}"
IOS_DEVICE="${IOS_DEVICE:-Surface Phone}"
IOS_BUNDLE_ID="${IOS_BUNDLE_ID:-com.mo.robotteleop}"

AUTO_COMMIT="${AUTO_COMMIT:-1}"
PUSH="${PUSH:-1}"
RUN_QUICK_TESTS="${RUN_QUICK_TESTS:-0}"
DEPLOY_UBUNTU="${DEPLOY_UBUNTU:-1}"
DEPLOY_IOS="${DEPLOY_IOS:-1}"
IOS_LAUNCH="${IOS_LAUNCH:-1}"
COMMIT_MESSAGE="${COMMIT_MESSAGE:-chore: sync deploy updates}"

echo "==> Local branch: ${REMOTE_BRANCH}"
git diff --check

if [[ "${RUN_QUICK_TESTS}" == "1" ]]; then
  echo "==> Running quick checks"
  make ios-format
  uv run pytest tests/unit/test_config.py tests/unit/test_registry.py tests/integration/test_cli.py -q
fi

if [[ "${AUTO_COMMIT}" == "1" ]]; then
  if [[ -n "$(git status --porcelain)" ]]; then
    echo "==> Committing local changes"
    git status --short
    git add -A
    git commit -m "${COMMIT_MESSAGE}"
  else
    echo "==> No local changes to commit"
  fi
fi

if [[ "${PUSH}" == "1" ]]; then
  echo "==> Pushing ${REMOTE_BRANCH}"
  git push origin "${REMOTE_BRANCH}"
fi

if [[ "${DEPLOY_UBUNTU}" == "1" ]]; then
  echo "==> Restarting Ubuntu ARX5 backend on ${REMOTE_USER}@${REMOTE_HOST}"
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    REMOTE_DIR="${REMOTE_DIR}" \
    PID_FILE="${PID_FILE}" \
    'bash -s' <<'REMOTE'
set -euo pipefail
cd "${REMOTE_DIR}"
if [[ -f "${PID_FILE}" ]]; then
  kill "$(cat "${PID_FILE}")" 2>/dev/null || true
fi
pkill -f 'robot-teleop run.*arkit_arx5.yaml' 2>/dev/null || true
REMOTE

  REMOTE_HOST="${REMOTE_HOST}" \
  REMOTE_USER="${REMOTE_USER}" \
  REMOTE_DIR="${REMOTE_DIR}" \
  REMOTE_BRANCH="${REMOTE_BRANCH}" \
  ROBOT_INTERFACE="${ROBOT_INTERFACE}" \
  DEBUG_WEB_PORT="${DEBUG_WEB_PORT}" \
  LOG_FILE="${LOG_FILE}" \
  PID_FILE="${PID_FILE}" \
  UV_BIN="${UV_BIN}" \
  GRIPPER_OPEN_READOUT="${GRIPPER_OPEN_READOUT}" \
    "${ROOT_DIR}/start_ubuntu_arx5_backend.sh"
fi

if [[ "${DEPLOY_IOS}" == "1" ]]; then
  echo "==> Building iPhone app for ${IOS_DEVICE}"
  DEVELOPER_DIR="${XCODE_DEVELOPER_DIR}" xcodebuild build -quiet \
    -configuration "${IOS_CONFIGURATION}" \
    -project "${IOS_PROJECT}" \
    -scheme "${IOS_SCHEME}" \
    -destination 'generic/platform=iOS' \
    -derivedDataPath "${IOS_DERIVED_DATA}"

  APP_PATH="${IOS_DERIVED_DATA}/Build/Products/${IOS_CONFIGURATION}-iphoneos/${IOS_SCHEME}.app"
  if [[ ! -d "${APP_PATH}" ]]; then
    echo "Missing built app: ${APP_PATH}" >&2
    exit 1
  fi

  echo "==> Installing ${APP_PATH} on ${IOS_DEVICE}"
  DEVELOPER_DIR="${XCODE_DEVELOPER_DIR}" xcrun devicectl device install app \
    --device "${IOS_DEVICE}" \
    "${APP_PATH}"

  if [[ "${IOS_LAUNCH}" == "1" ]]; then
    echo "==> Launching ${IOS_BUNDLE_ID}"
    DEVELOPER_DIR="${XCODE_DEVELOPER_DIR}" xcrun devicectl device process launch \
      --device "${IOS_DEVICE}" \
      "${IOS_BUNDLE_ID}" || true
  fi
fi

echo "==> Deploy complete"
echo "Dashboard: http://${REMOTE_HOST}:${DEBUG_WEB_PORT}/"

