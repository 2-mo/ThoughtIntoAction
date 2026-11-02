# RealMan RM65-BI 平台接入与 Quest 遥操

本项目提供独立 RealMan JSON/TCP 后端，遵循平台 `RobotState`、`ControlCommand` 数据接口，并注册为 `arm:realman` 插件。使用 `./run.sh realman` 入口；不使用 ARX 的 CAN 初始化、使能、回零或固定 250 Hz 原生路由。不需要新增厂商 Python 包。

当前适配范围是六轴 RM65 的状态查询、Quest 右手柄相对 TCP 位姿预览和显式开启的低速笛卡尔透传。未接入通用工作台按钮、ARX 原生控制内核、模型监看或录制流程；不支持关节目标、拖动示教和夹爪。换机械臂不能只改 IP：型号、协议、坐标系与末端配置均需重新核对。

## 配置与已确认状态

部署配置：`configs/robots/realman/rm65-19.json`。2026-11-02 对 `192.168.1.19:8080` 的实际只读查询结果：

- 型号 RM65-BI，控制/规划软件 V1.7.0，算法 1.5.2-0cabec5b。
- 系统错误列表 `[0]`。
- 工具 `Lq`，工具坐标 `[0,0,140000,0,0,0]`（Z=140 mm），配置负载 500（0.5 kg），质心配置为零；这不是实测负载或已验证的 TCP 标定。
- 工作坐标系 `Base`，位姿为零。

后端启动时核对型号、工具名/位姿及工作坐标系名/位姿，不写入配置、不清错误、不自动使能。Quest 会话每秒复查当前工具与工作坐标系，变化即退出。`RobotState` 的关节角为 rad，TCP 为 m + xyzw 四元数；未提供的速度和夹爪反馈为 NaN。`mode` 表示本软件控制状态，不代表电机实际上电状态。

`eno1` 需要能直达机械臂网段。当前附加地址 `192.168.1.100/24` 是临时地址，重启失效；没有该地址时，在本机执行 `sudo ip address add 192.168.1.100/24 dev eno1`，不要替换 DHCP 地址或默认路由。

## 只读操作

工作目录 `/home/guest/下载/ThoughtIntoAction-local`：

```bash
./run.sh realman status
./run.sh quest usb prepare --launch-client
./run.sh realman quest --duration 30
```

Quest 中打开 XRoboToolkit，选择头显与 Controller 数据，IP 填 `127.0.0.1`，Connect，保持佩戴与应用前台运行。第三条命令默认只读：先松开右 Grip，再握紧并移动右手柄，终端显示 `target_m_xyzw`。松开后再次握持会重新建立手柄与机械臂当前 TCP 的参考。`xr_fresh:false` 说明没有新鲜追踪帧，TCP 已连通不等于有追踪数据。

配置中的 XR→机器人基变换是明确的初始约定：XR 右→机器人 -Y、XR 上→机器人 +Z、XR 前（-Z）→机器人 +X。首次运动前必须通过预览确认现场朝向；需要改变朝向时修改 `xr_to_robot_basis`，其必须为正交、行列式 +1 的旋转矩阵。

## 真机控制入口（尚未实机运动验证）

完成预览方向核对、确认工具/负载、清空运动区域、现场急停就绪后，才使用下列独立入口。按官方要求关闭示教器的同时控制，不要让其他程序并发发指令。本机文件锁只能约束采用同一锁的程序，不能阻止示教器或其他电脑。

```bash
./run.sh realman quest --execute --duration 30
```

该命令允许运动，但不会自动使能机械臂。若控制器未处于可执行状态，应先在现场按厂商流程确认，不能用 ARX 脚本代替。

- 首次新鲜 XR 数据到达后必须松开右 Grip，再握到 >0.8 才开始跟随；松开到 <0.2 重新设参考。中间区域停止发送目标并尝试停止已有运动。
- 右 B 退出；Ctrl+C、异常退出、数据超时也尝试发 `set_arm_stop`。退出不回零、不失能、不操作夹爪。
- 默认周期 25 Hz，低跟随 `movep_canfd`；位置缩放 0.3，指令线速度上限 0.02 m/s、角速度上限 0.1 rad/s。
- 目标限制为会话起点周围 5 cm、旋转 0.25 rad；指令相对当前反馈超前超过 1 cm 或 0.1 rad 时拒绝。每次重新握持不扩大该会话总边界。
- XR 时间戳缺失/停滞/回退后终止执行会话，需重新启动；独立线程监测命令与反馈 0.3 s 超时。停止发送受 TCP 超时和线程调度影响，并非硬实时保证。

这些是软件目标限制，不是实测末端/关节速度保证，也不提供碰撞检测、奇异点规避或安全认证。网络断开、进程被强杀、主机掉电时无法保证停止命令送达；现场急停与控制器自身保护不可替代。低跟随透传在该固件上的实际运动效果仍需现场验证。

## 验证边界与夹爪

针对性测试覆盖协议分帧、单位换算、只读限制、身份核对、速度/边界、Grip 相对参考、XR 时间戳、超时停止及 CLI 预览/执行分支；所有运动测试使用假设备。

真实 `.19` 状态查询通过。真实 Quest 曾验证双手柄与 Grip/Trigger 0–1 数据；本次新入口的最后一轮 5 秒只读预览有 125 次机器人查询，但新鲜 XR 样本为 0，尚未验证真实手柄目标预览与真机跟随。未发任何真机运动/使能/停止/夹爪指令。

夹爪贴纸 `24V RS485` 不足以识别品牌或协议；仍需交付手册、型号或原控制程序。后端拒绝夹爪目标，不修改工具电压或 RS485 参数。

## 官方协议依据

- [JSON 快速开始](https://develop.realman-robotics.com/robot/json/getStartedJson/)：TCP JSON，以 CRLF 结束，不与示教器并发控制。
- [状态查询](https://develop.realman-robotics.com/robot/json/armState/)：`get_current_arm_state`，关节 0.001°，位置 0.001 mm，Euler 角 0.001 rad。
- [系统配置](https://develop.realman-robotics.com/robot/json/systemConfig/)：`get_arm_software_info`。
- [坐标系](https://develop.realman-robotics.com/robot/json/coordinate/)：工具及工作坐标查询。
- [运动指令](https://develop.realman-robotics.com/robot/json/motionConfig/)：`movep_canfd`、低/高跟随及 `set_arm_stop`（停止后保持动力）。

在线文档当前标注 V1.7.20，设备固件 V1.7.0；本次只读格式已真机核实，运动分支只做假设备验证，未把新固件附加字段用于旧固件。
