# 只读参考项目

本目录只用于阅读、源码对照和来源追踪。参考 checkout 的 URL 与固定提交记录在根目录 [dependencies.lock.toml](../dependencies.lock.toml) 的 `repositories` 中；它们不加入运行时 `PYTHONPATH`、动态库搜索路径或模型路径，不作为环境安装位置，也不在其中打运行补丁。

| 内容 | 用途 |
| --- | --- |
| XRoboToolkit、PC-Service-Pybind | 输入协议、原生绑定及坐标处理的上游对照 |
| arx5-sdk、X5、R5 | 机器人 SDK、型号参数与控制语义参考；R5 两个历史来源分别记录 |
| openpi、Isaac-GR00T | 策略与训练实现参考；运行使用各自正式环境和锁版本依赖 |
| `migration/`、`phone_input/` | 迁移索引、来源和补丁记录；不是可导入的实现目录 |

正式依赖分两种处理：

- 可安装的软件包由 `pyproject.toml`、`uv.lock` 或独立环境的锁文件固定版本，安装到项目环境。
- 需要本地构建或上游完整工作区的依赖，在 `dependencies.lock.toml` 的 `managed` 中声明精确提交与用途，工作区放 `outputs/dependencies/`，环境放 `outputs/envs/`。

项目确实需要维护的代码放入对应的 `src/` 模块；保留 URL、提交、原许可证和本地补丁说明。当前 ARX SDK 归 `src/embodied/devices/robots/arx5/vendor/`，Quest 原生资源和辅助代码归 `src/embodied/devices/inputs/quest/`，纯运动数学归 `src/embodied/contracts/vendor/arx_motion/`。它们是正式维护的组件，不再依赖本目录中的参考工作树。

模型维护在 `assets/robots/`，设备参数在 `configs/robots/`。arx-televr 的上游身份和许可证见 [docs/provenance/arx-televr](../docs/provenance/arx-televr/)。迁入的手机和 rebot 实现见 [合并后的使用与维护](../docs/guides/consolidated-components.md)，逐文件来源见 [迁移索引](migration/manifest.json)。原始工作树归档在 `outputs/archives/migration/originals/`，不参与运行。

保留已有参考目录的 `.git` 和未提交文件；不为运行环境准备而改写它们。新增参考可以仅记录固定链接，无需复制或安装整个项目，例如 [PhyAgentOS 参考取舍](../docs/architecture/phyagent-reference.md)。
