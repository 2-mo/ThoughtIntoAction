#!/usr/bin/env bash
set -Eeuo pipefail
case "${1:-start}" in
    start)
        adb -d shell input keyevent KEYCODE_WAKEUP
        adb -d shell am broadcast -a com.oculus.vrpowermanager.prox_close
        adb -d shell am start -n com.xrobotoolkit.client.quest/com.unity3d.player.UnityPlayerActivity
        ;;
    unwear)
        adb -d shell am broadcast -a com.oculus.vrpowermanager.prox_far
        ;;
    stop)
        adb -d shell am broadcast -a com.oculus.vrpowermanager.prox_far
        adb -d shell input keyevent KEYCODE_SLEEP
        ;;
    *)
        printf 'Usage: quest-wake [start|unwear|stop]\n' >&2
        exit 2
        ;;
esac
