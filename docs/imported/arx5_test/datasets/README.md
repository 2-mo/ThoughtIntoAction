# 采集数据

本机数据统一保存到 `~/work/data/arx5/`，不进入项目 Git。

- `data-coll/`、`data-coll-batches/`、`data-coll-single-camera-v1-20260731/`：已有原始采集。
- `data4train-gr*`：已有训练导出和验证数据。
- `isaac-groot/processed/`：采集 CLI 默认的新 LeRobot/GR00T 导出目录。

默认采集路径由 `src/arx5_paths.py` 与 `raw_episode.py` 定义。
