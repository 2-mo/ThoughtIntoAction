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
4. robot 已运行、追踪正常后，在手机点击“开始遥操”。按住底部控制区跟随，拇指上滑打开夹爪、下滑闭合，松手保持；夹爪初始值同步机器人反馈，按下时不会跳到默认闭合。
5. “停止遥操”释放控制权并保持，“停止锁定”还会锁定手机输入；“恢复”重新连接或请求清除手机锁定，成功后仍需点击“开始遥操”。手机姿态断流超过 0.5 秒，接收器也会释放控制权。网页和窗口 8 的启用 / 停止仍可使用。
6. 默认位移与旋转均为 **1:1 相对映射**，相对按下时的位置和姿态计算，松开后可重新摆放手机。倍率在手机调节，松开后生效；配置在 `configs/inputs/phone.json`。超出跟随范围的目标增量被丢弃，不积累成之后的大动作；不修改机械臂速度、加速度配置。
7. 松开 Clutch 后可点“回零”或“预抓取”。回零先张开夹爪，反馈达到最大开度附近（误差不超过 2 mm）才移动机械臂；未张开则报错取消。预抓取使用与 robot 窗口 P 键相同的 X5 位姿，保留夹爪开度。动作期间显示进度，“停止遥操”随时取消；追踪丢失或断流也取消，不自动续跑。速度仍使用内核运动配置。

竖屏 Clutch 区随可用高度扩大，优先保留按住和滑动的操作空间；追踪详情和倍率设置可滚动。网页手机卡片显示独立 3D 手机朝向，仅监看也可使用，不需开启遥操。此姿态相对于 App 首个有效相机帧；下方“姿态对齐”仍显示基座坐标下的映射、目标与实际夹爪，不能直接拿两种坐标的角度比较。

ARKit 只发送新的 `ARFrame`，不通过重发旧姿态伪造帧率。追踪受限时整张 Tracking 卡片黄色高亮，丢失时红色高亮，并显示 ARKit 的具体原因；正常后恢复常态。Home 请求走内核预览轨迹。手机停止只撤销手机自身控制并保持，不退出 robot 或自动进入阻尼；它不代替物理急停。停止锁定保存于本机输出目录，重启接收器也保留。恢复需要当前会话、正常追踪和松开的 Clutch，恢复 ACK 不获取机械臂控制权。若 robot 已退出，仍需在电脑启动，手机不会自动上电或重启它。手机 joint-zero 不能代替电机零位标定。

独立协议调试和源插件配置见 [插件工作台](../../docs/guides/plugin-workbench.md#手机与切换来源)。工作台接收器已占用 UDP 5005 时，独立调试接收器使用另一端口并同步修改 App 设置。

手机控制通道与重试规则见 [手机遥操协议](../../docs/reference/phone-control.md)。
