adb shell input keyevent KEYCODE_WAKEUP

adb shell am broadcast -a com.oculus.vrpowermanager.prox_close

adb shell am start -n com.xrobotoolkit.client.quest/com.unity3d.player.UnityPlayerActivity


# 停止模拟
# adb shell am broadcast -a com.oculus.vrpowermanager.prox_far