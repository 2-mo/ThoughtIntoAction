#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly RULE_SOURCE="${SCRIPT_DIR}/udev/99-arx-r5.rules"
readonly RULE_DESTINATION="/etc/udev/rules.d/99-arx-r5.rules"
readonly ADAPTER_SERIAL="0047002A4348571520323137"
readonly STABLE_BY_ID="/dev/serial/by-id/usb-ARX_USB2CAN_${ADAPTER_SERIAL}-if00"
readonly DEVICE_ALIASES=("/dev/arxcan0" "/dev/arx_r5")

if (( EUID == 0 )); then
    PRIVILEGED=()
else
    PRIVILEGED=(sudo)
fi

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        printf 'Error: required command not found: %s\n' "$1" >&2
        exit 1
    fi
}

require_command install
require_command readlink
require_command udevadm
if (( EUID != 0 )); then
    require_command sudo
fi

if [[ ! -f "${RULE_SOURCE}" ]]; then
    printf 'Error: udev rule not found: %s\n' "${RULE_SOURCE}" >&2
    exit 1
fi

printf 'Validating udev rule...\n'
udevadm verify "${RULE_SOURCE}"

if (( EUID != 0 )); then
    printf 'Administrator access is required to install the udev rule.\n'
    sudo -v
fi

printf 'Installing %s...\n' "${RULE_DESTINATION}"
"${PRIVILEGED[@]}" install -m 0644 "${RULE_SOURCE}" "${RULE_DESTINATION}"

printf 'Reloading udev rules...\n'
"${PRIVILEGED[@]}" udevadm control --reload-rules
"${PRIVILEGED[@]}" udevadm trigger --subsystem-match=tty
"${PRIVILEGED[@]}" udevadm settle --timeout=10

if [[ ! -e "${STABLE_BY_ID}" ]]; then
    printf 'Rule installed successfully. The ARX USB2CAN adapter is not currently connected.\n'
    printf 'Connect it later and verify with: ls -l /dev/arxcan0 /dev/arx_r5\n'
    exit 0
fi

readonly EXPECTED_TARGET="$(readlink -f -- "${STABLE_BY_ID}")"
for device_alias in "${DEVICE_ALIASES[@]}"; do
    if [[ ! -L "${device_alias}" ]]; then
        printf 'Error: expected alias was not created: %s\n' "${device_alias}" >&2
        printf 'Unplug and reconnect the USB2CAN adapter, then run this script again.\n' >&2
        exit 1
    fi

    alias_target="$(readlink -f -- "${device_alias}")"
    if [[ "${alias_target}" != "${EXPECTED_TARGET}" ]]; then
        printf 'Error: %s points to %s, expected %s\n' \
            "${device_alias}" "${alias_target}" "${EXPECTED_TARGET}" >&2
        exit 1
    fi
done

printf '\nARX R5 device naming is configured successfully:\n'
printf '  Stable hardware path: %s -> %s\n' "${STABLE_BY_ID}" "${EXPECTED_TARGET}"
for device_alias in "${DEVICE_ALIASES[@]}"; do
    printf '  Alias: %s -> %s\n' "${device_alias}" "$(readlink -f -- "${device_alias}")"
done
printf '  Permissions: %s\n' "$(stat -Lc '%A %U:%G' "${EXPECTED_TARGET}")"
printf '\nUse this device in future CAN commands: /dev/arx_r5\n'
