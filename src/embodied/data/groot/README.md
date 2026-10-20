# ARX5 → Isaac-GR00T 数据采集

本模块用于 D455 外部视角 + D405 腕部视角下的 ARX5 阻尼示教。当前数据集任务固定为：

> Stack the black cube on top of the white cube.

目标是采集 50 条人工确认成功的 episode，导出为 Isaac-GR00T N1.7 使用的
GR00T-flavored LeRobot v2.1。新原始数据按配置中的时间批次写入
`/home/r2/work/data/arx5/data-coll-batches/<batch_id>/`，不会进入 Git；旧批次保持原位。

## 安全与设备边界

- 机械臂只能由 `backend.arx5_backend.kernel.control` 持有 CAN 控制器。
- D455/D405 只能由 `backend.arx5_backend.features.camera.service` 持有 `librealsense` pipeline。
- 录制必须在 `ControllerMode.DAMPING` 中开始；录制期间除停止和急停外，
  控制服务拒绝所有运动命令。
- 浏览器断开、服务关闭或急停会停止录制，未完成数据保留在 `in-progress/`，
  不会被导出。
- 数据页不会判定任务成功。自动检查只判断数据完整性；最终接受必须由操作者确认。

真机开始前请确认机械臂可安全手动拖动、夹爪可手动推拉，工作区无人、无松动物体，
急停路径可用。

## 启动和采集

首次更新依赖：

```bash
UV_PROJECT_ENVIRONMENT=.venv-handeye uv sync --only-group handeye
UV_PROJECT_ENVIRONMENT=.venv-web uv sync --only-group web
```

恢复 CAN 后启动本机、局域网或 Tailscale 页面：

```bash
./arx5 can-up
./arx5 web
# 或 ./arx5 web-lan
# 或 ./arx5 web-tailnet
```

浏览器打开启动日志中的 `/collection` 地址。局域网模式必须保留 URL 中的
`token`。

采集页在单个视口内显示有效/丢弃/剩余数量、当前状态采样数与实际频率、双 RGB 帧数
与实际 FPS、两路写入队列和最坏丢帧率。J1–J6 各自使用一个最近 60 秒实时曲线块，
状态频率、RGB 帧率、队列占用和丢帧组成健康热力带。停止后会显示有效时长、
状态/图像数量、同步 P99 以及首尾静止时长。快捷键直接标在对应按钮上。

采集页直接通过加密 WebSocket 发送主机端相机录制命令，不建立 WebRTC 连接，也
不向浏览器传输 D455/D405 视频；双路 RGB 仍在机械臂主机上直接写入 episode。这样
通过 Tailscale 使用手机采集时不会让 SDP/ICE 协商、视频编码、下行带宽或浏览器
解码影响控制交互。

首次采集前把机械臂放到希望每条样本结束后返回的安全参考位，然后只点击一次
“准备机械臂”。页面会连接控制器、记录该参考位并进入阻尼示教。夹爪闭合零点是
持久化的一次性校准；已有校准时采集直接复用，不要求逐条处理。

每条 episode：

1. 摆好黑白方块，尽量让机械臂和夹爪静止约 0.5 秒，按 `Space` 开始。
2. 手动完成叠放，结束后再次静止约 0.5 秒。
3. 任务成功则再按一次 `Space`；页面自动停止、检查、保存并回参考位。
4. 示教中发现明显失误时按 `X`；数据自动归档并回参考位。

结构、同步或视频完整性检查失败时，页面会自动把 episode 归档到 `discarded/`，
不要求再次确认。检查通过仍以操作者按下“停止并保存”代表任务实际成功。

快捷键仅在页面没有输入焦点且对应操作可用时生效：

| 按键 | 操作 |
| --- | --- |
| `Space` | 开始采集；录制中则停止、检查、保存并自动回参考位 |
| `S` | 录制中停止、校验并保存；检查页直接保存 |
| `X` | 丢弃录制中或待审核 episode |

“丢弃”不会删除原始数据，而是移动到 `discarded/` 供追溯。自动检查通过只代表
数据结构和时序合格；按 `S` 或点击保存仍表示操作者已确认真实任务成功。

单条有效时长为 3–60 秒。页面会在接近 60 秒时自动停止。

## 原始数据

```text
/home/r2/work/data/arx5/data-coll-batches/
  <YYYYMMDD-HHMMSS_批次名>/
    batch.json
    in-progress/
      <本机时间>_id-<8位ID>/  # 正在录制、待审核或异常中断
    success/
      <本机时间>_id-<8位ID>/  # 自动检查和人工验收均通过
    discarded/
      <本机时间>_id-<8位ID>/  # 人工重做，仍保留用于追溯
    episode.json
    validation.json
    state/
      states.jsonl
      summary.json
    cameras/
      exterior_rgb/
        camera.json
        frames.jsonl
        video.mkv
      wrist_rgb/
        camera.json
        frames.jsonl
        video.mkv
```

机械臂原始状态目标频率为 100 Hz；D455 与 D405 都以 RGB8 1280×720 30 FPS
采集。RGB 训练不使用虚构的 12-bit 数据：RealSense 当前输出是 8-bit RGB，保留
12-bit 容器不会增加真实图像信息。两台相机默认关闭自动曝光和自动白平衡，固定为
8000 µs 与 4500 K；请求值、实际应用值和驱动警告都会写入 `camera.json`，方便后处理
筛选。原始图像不抽帧，保存为高质量 H.264 Matroska；`frames.jsonl` 独立保存每帧
主机时间戳、设备时间戳、设备帧号和视频帧索引。所有流都记录
`time.monotonic_ns()`，因此可在同一主机时钟上对齐。

检查采集数量或重新检查原始数据：

```bash
./arx5 groot-status
.venv/bin/python -m embodied.data.groot.cli validate-raw
```

## 状态与动作定义

每个视频帧通过相邻 100 Hz 状态线性插值得到 16 维状态：

| 索引 | GR00T group | 单位 | 内容 |
| --- | --- | --- | --- |
| 0:3 | `eef_9d` | m | EEF XYZ |
| 3:9 | `eef_9d` | — | 旋转矩阵前两行展平的 Rot6D |
| 9:10 | `gripper_position` | m | 夹爪开度 |
| 10:16 | `joint_position` | rad | J1–J6 |

`action[t] = state[t+1]`，最后一帧重复自身。训练配置使用 16 步 action horizon：

- `eef_9d`：relative、EEF、`XYZ_ROT6D`
- `gripper_position`：absolute
- `joint_position`：relative

相机键为 `observation.images.exterior_rgb` 和 `observation.images.wrist_rgb`，语言键为
`annotation.human.task_description`，由 `task_index` 映射到固定英文指令。

## 导出与 GR00T 验收

50 条数据全部接受后执行：

```bash
./arx5 groot-export
```

默认输出：

```text
/home/r2/work/data/arx5/isaac-groot/processed/arx5_black_on_white_dual_rgb_v2/
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.exterior_rgb/episode_000000.mp4
  videos/chunk-000/observation.images.wrist_rgb/episode_000000.mp4
  meta/
    info.json
    episodes.jsonl
    tasks.jsonl
    modality.json
    stats.json
    relative_stats.json
    export.json
```

导出器先在同级临时目录构建，逐条复验原始数据，编码 H.264 视频，调用本地
`Isaac-GR00T/gr00t/data/stats.py` 生成普通和 relative action 统计，再用
`LeRobotEpisodeLoader` 实际加载首条数据。所有步骤成功后才原子发布最终目录。
已存在的输出目录不会被覆盖；默认也会拒绝不足或超过 50 条的数据。仅做管线测试时
可显式传入 `--allow-incomplete`。

指定其他输出或 GR00T checkout：

```bash
./arx5 groot-export \
  --output /home/r2/work/data/arx5/isaac-groot/processed/run-002 \
  --groot-root /path/to/Isaac-GR00T

./arx5 groot-validate \
  /home/r2/work/data/arx5/isaac-groot/processed/run-002 \
  --groot-root /path/to/Isaac-GR00T
```

## 自动完整性门槛

- 状态和相机时间戳严格递增。
- 所有状态字段维度正确且为有限数值。
- 两路相机元数据、MKV、逐帧时间戳和写入摘要一致。
- 状态最近邻对齐最大误差 ≤ 25 ms，p99 ≤ 15 ms。
- 相邻视频帧最大间隔 ≤ 67 ms。
- D455/D405 最近帧同步 p99 ≤ 20 ms。
- 相机写队列丢帧比例 ≤ 1%。
- 状态/相机重叠时长为 3–60 秒。
- 夹爪反馈使用录制时控制器的机械公差检查，不因允许范围内的编码器偏差误拒绝。

开始和结束连续静止时间仍会记录，并在短于 0.5 秒时显示操作提示，但不会把结构和
时序完整的 episode 判为失败。

这些门槛位于 `config.json`；修改数据定义后应创建新 dataset ID，不要把不同 schema
混入同一个数据集。
