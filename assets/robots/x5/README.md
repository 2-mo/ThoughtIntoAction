# X5-2025 hybrid 模型

从本项目迁移前的 `arx5-sdk/models/X5_2025_hybrid.urdf` 与相邻 meshes 提取，按字节保全。本轮未改变惯量、运动学、碰撞几何、关节限制或末端坐标系。

运动学及七个 STL 网格来自 real-stanford/arx5-sdk；惯性参数来自 ARXroboticsX/X5 的 X5-2025 模型。来源 commit 与文件 SHA-256 在 `provenance.json`，许可证保留为 `LICENSE.stanford` 和 `LICENSE.arx`。

`./run.sh test` 校验资源内容和网格引用。模型修改需同步更新来源与哈希，以便采集数据追溯。
