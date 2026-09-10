# 夹爪上电角度跨圈修复

现场确认夹爪仅张开约 2 mm。被动 CAN 捕获 `08bf597ff800231d` 和
`18bf597ff801231d`，使能前后电机角度均为 6.18657969 rad。
直接按 -3.4 rad / 82 mm 换算得到 -149.2057 mm；减去 2π 后为 2.3299 mm。
此前 -74 mm 报错也有唯一的一圈补偿候选，对应 77.5356 mm。

`src/robot_control/gripper_turns.h` 在启动第一份角度读数上选择唯一有效圈数，
之后固定该偏移。读数减去偏移，位置命令加回偏移；速度和力矩方向不变。
没有写入电机零点，没有自动闭合动作。仍使用原 SDK 的 5 mm 反馈容差；
无法确定圈数时拒绝初始化，运行中的跳变不重新选圈，继续由 SDK 越界急停。
这只适用于已确认的本机夹爪上电跨圈，不是任意电机的自动标定。

项目使用 SDK PyPI 分支提交 `8c797d5` 构建补丁模块，保留已安装 0.1.3 wheel
的硬件和动力学共享库。原 wheel 没有覆盖。`configs/project.toml` 的
`environment.arm_sdk_python` 让项目启动器明确加载补丁模块；缺失时停止并提示。
硬件适配器验证补丁版本，再启用 `gripper_startup_turns`。

## 构建

需要 g++、dpkg-deb、tar、git，以及 Python 环境中的 pybind11。
开发头文件可以通过下面的命令下载并提取到项目中，无需系统安装：

```bash
mkdir -p outputs/build/arx-gripper/debs
cd outputs/build/arx-gripper/debs
apt-get download libeigen3-dev libspdlog-dev libfmt-dev liborocos-kdl-dev
for package in ./*.deb; do dpkg-deb -x "$package" ../deps; done
cd ../../../..
.venv/bin/python scripts/setup/build_gripper_sdk.py
```

构建模块与当前虚拟环境 Python ABI 和 wheel 共享库路径绑定，移动虚拟环境后需重建。
仅重启遥操不会更新已经运行的 SDK；需要退出并重启机械臂服务，再启动遥操。
不要求给机械臂断电或预先闭合夹爪。首次实机启动仍需观察开度读数与实物一致。

只读抓包工具：`python scripts/diagnostics/gripper_startup.py --seconds 60`。
