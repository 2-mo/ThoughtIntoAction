# 外部源码

源码目录不提交，精确版本在根 `dependencies.lock.toml` 管理。

- XRoboToolkit 及其 PC-Service-Pybind：当前遥操的通用 Quest/IK 与服务绑定依赖。
- arx5-sdk：模型与 SDK 源码参考；运行使用已安装的 `arx5-interface`。
- X5：可选官方 SDK 实验后端。
- openpi：后续策略候选实现，使用独立环境。
- R5：ARX 官方参考仓库；XRoboToolkit 内的 `dependencies/R5/` 是另一个历史 fork。两者仅保留作可选参考，默认不获取、不检查、不构建，也不加入启动库路径；`--all` 才纳入源码获取/检查。

迁移保留各仓库 `.git` 和未提交文件。模型改动应在主仓库 `assets/robots/x5/` 内进行；依赖源码改动需以明确补丁或维护的 fork 管理。

项目自己的 X5 控制器位于 `src/quest_teleop/`，已脱离上游 R5 控制器和 SDK；迁入代码的来源与许可见 [X5 控制器说明](../docs/x5-controller.md)。

历史设计参考包括 `arx5_test` 的采集/异步推理设计和 `reBotArm_control_py` 的运动学/轨迹分层。本项目无需访问这些备份，构建和运行不依赖它们；不迁移旧 URDF、设备参数或标定结果。
