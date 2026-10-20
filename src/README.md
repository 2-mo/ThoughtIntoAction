# 源码结构

平台代码都在 `embodied` 包中；机械臂控制内核是仓库根目录的 `control/`（arx-televr，Python 3.10）。`legacy/` 存放已被内核取代、待真机验收后删除的旧实现。

```text
src/
├── embodied/
│   ├── contracts/      L0 契约：注册表、RPC、内核客户端、设备锁、运动 profile、机器人配置读取。
│   │                   控制内核只允许导入这个子包。
│   ├── core/           项目配置与路径、X5 模型（Pinocchio）、坐标变换、遥测、预设位姿
│   ├── sensors/        相机服务（唯一设备所有者）、客户端、传输、合成相机
│   ├── inputs/         内核控制来源：手机、idle、手柄手势
│   ├── phone/          手机 UDP 协议、相对位姿映射与诊断工具
│   ├── operations/     MoveJ / MoveL、点动、夹爪、示教点（内核客户端）
│   ├── motion/         离线规划、时间参数化、碰撞检查，不依赖 SDK
│   ├── data/
│   │   ├── episode/    MCAP / 视频 / 元数据读写与质量检查
│   │   ├── capture/    从内核 tick 采集
│   │   ├── alignment/  图像与状态的因果对齐
│   │   ├── export/     LeRobot 导出（schema 1 / 2 适配器、视频、解码）
│   │   ├── groot/      GR00T 双相机采集与导出
│   │   └── datasets.py、replay.py、task_package.py
│   ├── policy/         pi05_zeroshot、pi05_sft（OpenPI）、groot；模型代码在各自环境运行
│   ├── calibration/    hand_eye（D455 / D405）、wrist（腕部 D435i，经内核）
│   ├── monitor/        18081 监看页与 MuJoCo 工作台
│   ├── console/        apps/console 的后端
│   ├── workstation/    工作台装配、内核启动、组件注册与命令
│   ├── tasks/cube/     方块抓取 / 旋转 / 堆叠任务
│   └── robots/rebot/   B601（实验，尚未接入内核）
└── legacy/             robot_control（旧机械臂服务）、quest_teleop（旧遥操）、
                        arx5_control（直连 SDK 控制器）、arx_tools
```

依赖方向：`control/` → `embodied.contracts`；`embodied.*` 之间只通过 `contracts` 交换数据，不导入彼此的业务状态机；新代码不再导入 `legacy`。`tests/test_package_boundaries.py` 与 `tests/test_convergence.py` 检查这些规则。

入口与命令见 [命令说明](../docs/reference/commands.md)，迁移计划见 [todo/](../todo/README.md)。
