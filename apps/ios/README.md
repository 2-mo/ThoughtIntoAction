# RobotTeleop · iPhone 输入

源码工程：`apps/ios/RobotTeleop/RobotTeleop.xcodeproj`。真机使用 ARKit，模拟器使用合成姿态，复用同一 UDP 协议、相对映射、Clutch 和设置握手。工程最低目标 iOS 26.0，使用 Swift 6；bundle ID 为 `com.mo.robotteleop`。

## 安装或更新过期 App

在已配置 Xcode 和签名账号的 **Mac 项目根目录**执行，iPhone 解锁并连接 Mac：

```bash
bash apps/ios/install.sh
# 也可指定 devicectl 列出的设备名称或标识：
bash apps/ios/install.sh 'Surface Phone'
```

脚本重新构建、签名、安装并请求启动。构建产物在 `outputs/build/ios-device/`。首次设置时，打开 Xcode 工程的 Signing & Capabilities，选择自己的 Team，并保留自动签名；账号在 Xcode → Settings → Accounts 中登录。iPhone 按系统提示信任电脑、开启开发者模式，首次启动允许相机和本地网络。

如果输出 **App installed**，随后提示 `profile has not been explicitly trusted`，说明安装已成功、启动被签名信任检查拒绝：在 iPhone **设置 → 通用 → VPN 与设备管理 → 开发者 App** 中信任开发者，然后手动打开 RobotTeleop。若信任后仍失败，保留完整错误检查签名和描述文件，不能仅凭安装成功认为启动成功。

Personal Team 描述文件有有效期，到期需要重新构建安装。参考 [Apple 账号能力说明](https://developer.apple.com/help/account/basics/about-your-developer-account)、[真机运行](https://help.apple.com/xcode/mac/current/en.lproj/dev5a825a1ca.html) 和 [开发者信任](https://help.apple.com/xcode/mac/current/en.lproj/dev96a12fb84.html)。

## 连接工作台

1. Ubuntu 项目根目录运行 `./run.sh`，进入 **8 手机**；接收服务自动启动，手机晚打开也可连接。
2. App 当前默认自动连接 `192.168.5.8:5005`。换电脑或网络后，填写手机窗口显示的可达电脑地址和端口，再 Connect。
3. 窗口和网页显示输入帧率、ARKit 追踪、数据年龄及 XYZ。数据年龄是主机最新收包距今，不是端到端延迟。
4. robot 已运行、追踪正常且 Clutch 松开后，窗口按 `T` 或网页点击“启用手机遥操”。先松开再按住 Clutch 移动，松开保持；夹爪滑块在接管后可独立控制。
5. 窗口 `S` / 空格或网页“停止遥操”释放控制权，输入监看继续。启动接收、打开或刷新网页不会自动控制机械臂。

ARKit 只发送新的 `ARFrame`，不通过重发旧姿态伪造帧率。追踪受限时让后摄像头观察有纹理、光线充足的环境。Home 请求走内核预览轨迹。ESTOP 锁存停止请求；当前真机组合不接受手机 RESET 解除，断开再连接也不会清除锁存，需按真机停止/恢复流程处理。手机 joint-zero 不能代替电机零位标定。

独立协议调试和源插件配置见 [插件工作台](../../docs/guides/plugin-workbench.md#手机与切换来源)。工作台接收器已占用 UDP 5005 时，独立调试接收器使用另一端口并同步修改 App 设置。
