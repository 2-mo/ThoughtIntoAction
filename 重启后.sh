adb shell cmd wifi connect-network "ZTE-YhhPKP" wpa2 cqupt@1805

cd ~/work/code/QuestArmTeleop/myTeleop

# 1. 确认 Quest USB 已连接且授权
adb devices

# 2. 建立 XRoboToolkit USB 反向端口
./run.sh setup-quest-usb

sudo slcand -o -f -s8 /dev/arx_r5 can0
sudo ip link set can0 up

./myTeleop/run.sh check-xr
./myTeleop/run.sh check-arm


cd ~/work/code/QuestArmTeleop

./myTeleop/run.sh teleop-official-x5 \
  --control-mode position \
  --scale-factor 0.5 \
  --max-joint-speed-rad-s 0.5 \
  --preview-time-s 0.05 \
  --actuate