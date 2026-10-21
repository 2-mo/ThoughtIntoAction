# 文档

| 先读 | |
| --- | --- |
| [入门](getting-started.md) | 安装、第一次仿真、第一次真机、数据到策略的完整流程 |
| [架构概览](architecture/overview.md) | 分层、进程、数据流与安全边界 |
| [命令](reference/commands.md) | `./run.sh <功能> <命令>` 与全部命令 |

## 指南

| 文档 | 内容 |
| --- | --- |
| [安装](guides/setup.md) | 环境、依赖、设备驱动与权限 |
| [日常操作](guides/operation.md) | tmux 工作台、总控按键、真机与仿真切换 |
| [真机验收（arx 内核）](guides/real-arm-acceptance.md) | 切换到内核前的现场验收清单 |
| [内核工作台](guides/plugin-workbench.md) | 内核 profile、多来源控制权、MoveJ / MoveL、手机与标定适配 |
| [采集与监看](guides/recording.md) | 采集按键、episode、回放与校验 |
| [机械臂服务与监看](guides/monitoring.md) | 旧机械臂服务、三维监看、反馈与断流 |
| [网页与相机控制](guides/viewer-camera-controls.md) | 18081 监看页的相机与视图 |
| [训练数据导出](guides/openpi-dataset.md) | LeRobot 导出与 π0.5 训练 |
| [仿真与手眼标定](guides/simulation-calibration.md) | 独立仿真、D435i 眼在手上标定 |
| [组件与环境](guides/consolidated-components.md) | 合并进来的算法、设备、标定、GR00T 组件及其环境 |
| [Quest 发现广播](guides/quest-discovery.md) | 头显发现广播开关 |
| [夹爪上电修正](guides/gripper-startup.md) | 夹爪跨圈读数修复与 SDK 编译 |
| [电机诊断](guides/motor-feedback.md) | 电流、温度与故障码采集 |
| [旧 X5 遥操控制器](guides/x5-controller.md) | 当前真机链路的控制器（待退役） |

## 参考

| 文档 | 内容 |
| --- | --- |
| [命令](reference/commands.md) | 全部命令与参数优先级 |
| [可替换组件](reference/plugins.md) | 注册表与插件契约 |

## 架构

| 文档 | 内容 |
| --- | --- |
| [概览](architecture/overview.md) | 当前架构 |
| [一屏架构图](architecture/index.html)、[详细导览](architecture/explorer.html) | 浏览器打开，离线可用 |
| [部件边界](architecture/component-boundaries.md) | 状态所有权、契约与独立验证方式 |
| [arx 组件复核](architecture/arx-component-review.md) | 内核内部的组件边界 |
| [多输入与运动](architecture/control-inputs-and-motion.md) | 手机、点动与动作接入方案 |

## 历史

迁移方案、实施记录与原项目文档保存在 [history/](history/)，不再更新；以代码和上面的文档为准。
