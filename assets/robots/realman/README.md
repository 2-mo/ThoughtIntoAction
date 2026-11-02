# RM65 官方名义模型

来源版本和逐文件校验值见 [UPSTREAM.json](UPSTREAM.json)，许可证见 [LICENSE](LICENSE)。`upstream/rm_65.urdf` 与 `meshes/` 是固定版本的官方原始文件；`runtime/` 由 `scripts/realman/build_model.py` 生成。

派生模型加入 Lq 的 Z +140 mm 工具、地面、可视化 TCP、力矩执行器和工程仿真参数。夹爪使用自建的简化平行滑块，不是 EG2-4C2 的标定模型。模型中机器人接触关闭；不用于碰撞或真实抓取判断。显示 URDF 保留六轴机械臂和固定工具掌部，动态夹指只在 MuJoCo 中显示。

使用、重新生成、离线对照与当前验证范围见 [仿真指南](../../../docs/realman/simulation.md)。
