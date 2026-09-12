# 第一阶段重构记录

基准：`teleop-presets-x5`、当前 X5-2025 hybrid 模型、单腕部 D435i、本机推理、H800 训练。

## 本轮边界

- 外部依赖与参考源码统一放在 `third_party/`；官方 `R5` 与嵌套 XR R5 fork 分别锁定，现均为可选参考。项目 X5 控制器已解耦 R5，默认构建和启动不使用它们。
- URDF 与七个必要网格按字节保全，附来源和 SHA-256。
- 主入口、官方基线与官方仿真采用项目模型；历史自定义控制器保留原 SDK 模型和行为。
- 中文快捷入口只调用一个模式；真机保留原 presets 参数，仿真清除原来夹杂的真机命令。
- 修复新增夹爪参数导致三个入口构造失败的问题，默认值与主入口一致。
- 日志、安装包、权重与代码分开；历史 TXT 记录保留。
- 原有未提交修改和旧启动片段备份在 `.cache/refactor-before/`，不提交。

## 备份项目

桌面项目仅作设计参考，未移动、未修改，未复制其环境、模型或标定结果。本项目没有指向桌面的 import、软链接、运行路径或构建依赖。

新增迁移已实现独立运动学仿真、单 D435i 标定采样/求解、可预览的自动采样，以及共享 X5 后端提取。见 [迁移审查](migration-review.md) 和 [操作说明](simulation-calibration.md)。训练 episode 的异步写盘、生命周期、导出与策略进程仍按下述边界继续推进。

## 第二阶段：采集

共享基础、机械臂控制、标定与独立仿真现已从 `quest_teleop` 拆为 `src/` 下的并列包，见 [源码分层](../src/README.md)。五次轨迹、通用运动执行与 X5 适配器集中到 `robot_control`；采集源码指纹覆盖全部 `src/`。控制参数与原始数据格式不变，Python 内部导入使用新包路径。

主链路通过 `configs/plugins.toml` 选择机械臂、输入、状态输出、显示和相机。设备服务、控制与监看现为独立进程：`scripts/viewer/启动仿真.sh` / `view-arm` 只订阅状态，`control-arm` 负责虚拟控制，真机继续使用 presets 入口；控制通过本机 RPC 连接 `robot_control.service`，服务通过 `robot_core.telemetry` 向监看持续发布反馈。旧 `simulate` 命令及 simulation profile 已移除，不保留转发别名。详见 [机械臂监看](monitoring.md)。
