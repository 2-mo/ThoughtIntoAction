# QuestArmTeleop




```bash
sudo apt update
sudo apt install adb
adb version
```


```bash
sudo chgrp plugdev /dev/bus/usb/004/024
sudo chmod 660 /dev/bus/usb/004/024

sudo kill 2869626
sudo slcand -o -f -s8 /dev/serial/by-id/usb-ARX_USB2CAN_0047002A4348571520323137-if00 can0
sudo ip link set can0 up

```


```bash

adb kill-server
adb start-server
adb devices -l

```

```bash
adb devices
adb install "/home/r2/work/code/QuestArmTeleop/XRoboToolkit-Quest-1.0.1.apk"

# 如果出现签名不一致, 需要先卸载旧版本再安装. 报错会显示 INSTALL_FAILED_UPDATE_INCOMPATIBLE
adb install -r "/home/r2/work/code/QuestArmTeleop/XRoboToolkit-Quest-1.0.1.apk"
```


cd ~/work/code/QuestArmTeleop/myTeleop

HTTPS_PROXY=http://127.0.0.1:7890 \
curl -fL -C - \
  -o XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb \
  https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb

sudo apt install ./XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb


dpkg -L roboticsservice | grep -E 'RoboticsServiceProcess|runService.sh'


./run.sh check-xr
./run.sh check-arm
./run.sh teleop