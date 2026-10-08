# 头显发现广播开关

Wi-Fi 地址提示来自独立的 `/opt/apps/roboticsservice/RoboticsServiceProcess`，不是机械臂服务。原版 `TcpServerWorker::run()` 每 5 秒无条件调用 `onBraodCastTCPServer()`，通过 UDP 29888 广播本机 IP。USB 追踪使用 ADB reverse 转发的 TCP 63901，是另一条连接。

遥操启动或按 W 恢复时，会先检查主机 TCP 63901 接收端口，未运行则自动启动 XR PC Service，已运行则复用；随后配置 ADB reverse。启动日志在 `outputs/logs/xr-service.log`。头显通过 USB 连接时，服务器地址填写 `127.0.0.1`；头显自己的 Wi-Fi 地址不是服务器地址。只有 ADB 转发而没有主机接收服务时，Connect 仍会失败。

本项目给原生广播函数增加配置检查：缺省不发送，只有显式设置开启时才发送。没有防火墙规则，没有修改 USB 转发、TCP 监听、连接心跳或机械臂控制。

机械臂 TUI 使用 **B 开广播 / N 关广播**。也可以在普通终端执行：

```bash
./run.sh quest-discovery off
./run.sh quest-discovery on
./run.sh quest-discovery status
```

配置保存在 `~/.config/QuestArmTeleop/xr-discovery.ini`（遵循 `XDG_CONFIG_HOME`），初始默认关闭，显式选择会保存。可以通过 `QUEST_XR_DISCOVERY_CONFIG` 指定绝对路径，但 XR 服务和终端必须使用相同路径。

```ini
[TCP]
BroadcastEnabled=false
```

XR 服务每 5 秒读取配置；开关不重启服务，不断开 USB。CLI 最多等待 7 秒，从同目录 `.status` 文件确认当前服务进程已应用配置。旧模块不会写确认文件，因此不会把“配置已保存”误报成“服务已生效”。

## 构建与首次加载

```bash
python3 scripts/setup/build_xr_discovery.py
python3 scripts/setup/build_xr_discovery.py --install
```

构建脚本固定上游提交 `85bac4dbc1fd5cef42c74a160d9c30aa3491f122`，只重编译 `DeviceConnectionManager`。Ubuntu Qt 开发头文件和 moc 提取到 `outputs/build/xr-discovery/qt`，链接本机安装的 Qt 运行库和 CommonUtils，不安装系统软件包。当前验证环境是 x86_64、Qt 开发工具 6.4.2、服务运行库 6.6.3；其他版本需重新验证。

安装前会运行原生测试：默认关闭、显式开启后发送、再次关闭后不发送，并验证 TCP 连接持续传输数据。测试将 IP 枚举替换为 loopback，不向 Wi-Fi 发送测试广播。

`--install` 原子替换 `/opt/apps/roboticsservice/libDeviceConnectionManager.so`，同时创建带原文件 SHA256 前缀的 `.backup-*` 副本。目录不可写时需由管理员安装。首次需要重启一次 **XR 服务**才能加载新模块，会有一次 USB 追踪重连；日常 B/N 切换不需要重启。构建/安装脚本不会自行中断正在运行的 XR 服务。

重启入口仍为 `./run.sh start-xr-service`；不要在旧 XR 服务仍运行时重复启动。无需重启机械臂服务来加载原生模块，但已经打开的机械臂 TUI 要重启才能显示新增 B/N 快捷键。

回滚时在 XR 服务停止后，用对应 `.backup-*` 副本恢复原库，再启动 XR 服务。配置文件可以保留，原版不会读取它。
