# 源码分层

机械臂的通用能力与 Quest 应用并列。跨包使用绝对导入，包内使用相对导入；各包的 `__init__.py` 不初始化设备。

```text
src/
├── robot_core/             共享基础
│   ├── project.py          标准库实现的根路径、配置读取与路径解析
│   ├── plugins.py          显式插件注册表、延迟加载和设备类型检查
│   ├── interfaces.py       后端、位姿输入、状态输出、显示与相机契约
│   ├── telemetry.py        单向状态发布/订阅，不依赖显示进程
│   ├── model.py            当前 URDF、FK/IK、关节限位、模型元数据
│   ├── transforms.py       坐标变换与姿态计算
│   ├── camera.py           单 D435i RGB 流、内参与时间戳
│   ├── device_lease.py     CAN/本机会话的共享所有权锁
│   └── storage.py          原子 JSON 写入
├── robot_control/          与 Quest 无关的机械臂控制
│   ├── service.py          独立设备所有权、指令仲裁、待机反馈与状态发布
│   ├── client.py           本机控制/反馈客户端，不导入硬件 SDK
│   ├── trajectory.py       五次插值、速度/加速度约束、关节目标限幅
│   ├── motion.py           轨迹执行、跟踪误差/超时/到位静止检查
│   ├── hardware.py         Stanford X5 适配器、指令平滑与夹爪控制
│   ├── cli.py             虚拟机械臂控制 CLI：终端或位姿输入
│   └── backends/           插件工厂、独立虚拟 X5 和可选官方 SDK 后端
├── handeye_calibration/    眼在手上标定
│   ├── cli.py              marker/manual/plan/auto/solve 入口
│   ├── arm.py              标定控制权申请与服务反馈适配
│   ├── motion.py           标定采样计划、范围验证与采样调度
│   ├── session.py          静止多帧采样与标定数据保存
│   ├── target.py           ArUco 检测、PnP 与歧义检查
│   └── solver.py           外参求解与独立验证
├── robot_simulation/      独立运动学仿真
│   ├── scene.py            共享 Three.js 场景服务、URDF 变换与路径
│   ├── web/                只读网页、固定版本 Three.js 与纹理（离线可用）
│   └── cli.py              独立三维监看：订阅状态，不创建控制器
└── quest_teleop/           X5 遥操与采集
    ├── cli.py              主遥操和 A/B 预设入口的参数解析
    ├── base_controller.py  X5 映射、组件生命周期与控制线程，复用通用 IK 算法
    ├── controller.py       X5TeleopController，服务接入、会话与控制权释放
    ├── presets.py          A/B/Grip 预设动作状态机
    ├── inputs.py           Quest 与离线 idle 位姿输入插件
    ├── legacy/             历史自定义控制器及 CSV 记录
    ├── experimental/      官方 SDK 实验遥操入口
    ├── xr_visualizer/      Quest 只读可视化服务及 HTML
    ├── runtime/            会话、IPC、合成演练和只读监看
    ├── recording.py        原始 episode 异步写盘
    ├── datasets.py         数据校验、标注与离线回放
    └── diagnostics.py      当前采集环境与设备诊断
```

依赖方向：`robot_core` 不依赖其他业务包；`robot_control` 使用共享基础；标定与独立仿真复用模型和控制能力；`quest_teleop` 使用共享组件，数据回放独立发布状态。基础、控制、标定、仿真场景和默认终端输入不导入 `quest_teleop` 或 XRoboToolkit；虚拟控制 CLI 仅在选择位姿输入时延迟导入 `quest_teleop` 的 presets 控制器。

平滑控制分三部分：`trajectory.py` 生成离线点到点轨迹；`motion.py` 通过调用者提供的机械臂执行轨迹并等待静止；`hardware.py` 保留遥操原有指令限速、超前限制、SDK preview 和夹爪保持逻辑。运动执行函数不取得设备所有权、不自动启用或回零；调用者负责启用和异常后的阻尼清理，标定的 `execute_plan` 保留这层管理。

`robot_control.hardware` 导入时加载 `arx5_interface`，实例化适配器才打开 CAN。轨迹计算、独立仿真和离线标定不需要机械臂 SDK；相机 SDK 只在打开相机或需要原生畸变处理时加载。独立仿真仍是运动学可视化，没有新增动力学或完整碰撞检测。

常用入口：

```bash
./run.sh arm-service --backend simulated
./scripts/viewer/启动仿真.sh
./run.sh control-arm --input quest
./run.sh plugins
./run.sh handeye --help
./run.sh --profile presets teleop-presets-x5 --help
./run.sh session demo
```

主 Quest 控制器与预设状态机位于 `quest_teleop`，按插件注册表创建 `robot_control.client.ArmClient`。虚拟 Quest 控制组合主 presets 控制器与虚拟服务，终端控制连接同一个服务；监看不依赖控制输入。配置读取统一使用 `robot_core.project`，所有 `src/` 代码均不依赖 `scripts/` 的 Python 模块。X5 入口已解耦 R5；IK 算法与通用运行方法仍复用固定版本 XRoboToolkit，组件初始化和 IK/控制线程由项目维护，主控制链路通过服务客户端读状态、发目标，三维渲染在独立监看进程。迁入代码保留 MIT 许可，见 [X5 控制器说明](../docs/x5-controller.md) 和 [插件接口](../docs/plugins.md)。

模型资源统一使用 `assets/robots/x5/`，配置位于 `configs/`，生成结果位于 `outputs/`。采集源码指纹覆盖 `src/`、`scripts/`、插件注册表和源码/Python 依赖锁；会话记录选中的后端、位姿输入和场景。
