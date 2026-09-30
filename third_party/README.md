# 外部源码

源码目录不提交，精确版本在根 `dependencies.lock.toml` 管理。

- XRoboToolkit 及其 PC-Service-Pybind：当前遥操的通用 Quest/IK 与服务绑定依赖。
- arx5-sdk：模型与 SDK 源码参考；运行使用已安装的 `arx5-interface`。
- X5：可选官方 SDK 实验后端。
- openpi：后续策略候选实现，使用独立环境。
- R5：ARX 官方参考仓库；XRoboToolkit 内的 `dependencies/R5/` 是另一个历史 fork。两者仅保留作可选参考，默认不获取、不检查、不构建，也不加入启动库路径；`--all` 才纳入源码获取/检查。

迁移保留各仓库 `.git` 和未提交文件。模型改动应在主仓库 `assets/robots/x5/` 内进行；依赖源码改动需以明确补丁或维护的 fork 管理。

项目自己的 X5 控制器位于 `src/quest_teleop/`，已脱离上游 R5 控制器和 SDK；迁入代码的来源与许可见 [X5 控制器说明](../docs/x5-controller.md)。

`arx5_test`、`reBotArm_control_py`、`robot-teleop` 的源码、配置、模型和工具已集中迁入本仓库，三个原目录已退役。完整原工作树仅作为恢复副本存放于 `archives/migration/originals/`，不参与运行。实际入口、控制边界和验证结果见 [合并后的使用与维护](../docs/consolidated-components.md)，逐文件来源见 `migration/manifest.json`。原生 Quest 链路继续复用 arx；部分迁入操作保留独立控制器，通过统一设备锁协调使用。机器人专属 URDF、网格和标定参数随各组件维护，不作为 X5 默认值。
