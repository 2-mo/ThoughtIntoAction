# X5 原始采集与 π0.5 训练导出

新采集位置是项目根目录下 `datasets/<任务名>_<本地时间>-<随机编号>/`。任务名来自 `启动遥操.sh` 的 `COLLECTION_TASK`；例如 `把杯子放入盒子_20260911-143000-12ab34cd`。非法路径字符转成下划线。每次 A 开始/保存产生一条 episode；同一轮任务连续采集的所有有效 episode 合成一个训练数据集，长按右摇杆 1 秒结束任务时生成。旧数据保留原编号，不修改原始元数据。

## 原始数据

- `episode.json`：任务、机械臂模型、相机内参、配置、完整性与计数。
- `wrist_rgb.mp4`：640×480，30 FPS，H.264/libx264，yuv420p，CRF 20。可在 `[recording].video_crf` 改为 18–23。这是配置的采集分辨率，非 D435i 最大分辨率。
- `frames.jsonl`：逐帧 `frame_number`、`device_timestamp_ms`、`device_timestamp_domain`、`host_received_monotonic_ns`、`video_frame_index`、`video_pts_s`。
- `states.jsonl`：实际状态、请求动作、最终发送命令、力矩、控制模式及原有时间戳；含 `host_monotonic_ns`、`requested_monotonic_ns`、SDK `feedback_timestamp` / `command_timestamp` 等。

VR 遥操处理配置为 90 Hz，机械臂服务控制配置为 240 Hz。机器人 state/action 按服务每个实际控制周期逐条保存，不因为图像 30 FPS 降频。原始整数纳秒字段不转换成浮点秒。相机设备时间是毫秒浮点数，保留 SDK 返回值及其时钟域。相机与机器人设备时钟未硬件同步；关联使用主机 monotonic 时钟，接收时刻不是曝光时刻。MP4 固定帧率的 PTS 只是媒体索引，不能替代采集时间戳。视频写盘/封装失败会将 episode 标为失败，而不是宣称完整。

已有 PNG episode 的读取、质量检查和网页/真机回放保持兼容。配置更新后需重启遥操，已运行的采集会话不会热切换目录或编码格式。相机继续使用现有 640×480、30 FPS，无需更改模式。

## 自动保存 LeRobot v2.1

每天启动脚本使用 90 Hz 遥操、240 Hz 机械臂控制，相机保持 30 FPS。实际达到的频率取决于运行负载，应使用记录时间戳核对；这些配置不保证头显硬件输出频率。

例如 pick place 采集 50 条：设置 COLLECTION_TASK，A 开始/保存一条，重复 50 次；长按右摇杆 1 秒结束任务。完成后直接上传 `datasets/pick_place_时间-编号/` 整个目录，其中直接包含 `meta/`、`data/`、`videos/`，没有额外的 lerobot 子目录。任务不要求固定 50 条，保存时按实际有效条数导出。

A 保存原始记录到 `datasets/.raw/<episode>/`，供即时回放和备份，不逐条转换。原始数据不混入任务上传目录；无需上传 .raw。暂停/恢复继续同一任务，结束后再开始则新建任务目录。右摇杆上拨丢弃的 episode 不参与任务导出。旧数据保留原位置。

任务结束时转换在采集操作工作线程执行，完成前下一次采集操作会等待。成功时控制台输出可上传目录；失败保留本轮任务及原始数据，可再次结束任务重试。转换状态及日志位于 `datasets/.raw/tasks/<任务目录名>/lerobot-status.json` 和 `lerobot-save.log`。程序异常退出时原始记录仍保留，可手动导出，不会自动恢复上一轮任务。

自动保存保留每条机器人样本，不做状态降采样。LeRobot 标准时间为行号 / 240，精确实际时刻保留在 source 时间戳列中。相机首帧前的样本使用黑图、`observation.wrist_valid=false` 和 RGB 时间戳 0；之后取此前最新图像。原始图像帧及设备时间戳完整保留在 .raw 中。

首次应用频率配置需要重启机械臂服务和遥操。

## 手动合并导出 LeRobot v2.1

OpenPI 上游固定 LeRobot revision `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`，其数据格式为 v2.1。导出使用该版本官方 `LeRobotDataset.create/add_frame/save_episode`，并用同版本真实加载器验证首尾样本和动作 chunk。不会上传 Hugging Face。

首次环境安装（与遥操 Python 完全隔离）：

```bash
./scripts/datasets/setup_export.sh
```

按任务导出：

```bash
./导出训练数据.sh --source datasets/.raw --task '把杯子放入盒子' --fps 240 --preserve-samples --output datasets/lerobot/cup_task
```

默认拒绝不完整、已丢弃、失败、丢帧、时序间断、混合模型或包含预设/回放的 episode；明确使用 `--skip-invalid` 可以跳过，并在 `meta/quest_export.json` 查看每条拒绝原因。`unknown` 表示未经成功标注，不等于成功演示；训练前应筛选任务完成质量。工具不覆盖已有导出目录。

自动保存和上述合并命令使用 **240 Hz** 标称训练时间轴，逐条保留机器人样本。手动工具未指定参数时仍为 50 Hz 重采样模式；采集新数据应使用上述参数。最大允许间隔为 100 ms，导出报告包含状态/图像最大年龄及图像复用次数。

LeRobot v2.1 标准加载器要求图像和动作共享样本时间轴，因此训练版 MP4 是 **240 FPS 的对齐容器**：复用 30 FPS 原图，不插值生成新图像，不代表相机采到了 240 FPS。原始视频始终 30 FPS。训练视频用 H.264、CRF 20。

| 字段 | 定义 |
|---|---|
| `observation.state` | 6 个实际关节角 rad + 实际夹爪宽度 m |
| `action` | 6 个请求关节目标 rad + 请求夹爪宽度 m，都是绝对值 |
| `observation.images.wrist` | 腕部 RGB 视频；RGB 解码，原采集分辨率 |
| `source.state_monotonic_ns` | 所选状态的原始主机时间戳 |
| `source.action_monotonic_ns` | 所选请求动作的原始主机时间戳 |
| `source.rgb_monotonic_ns` | 所选图像的原始主机接收时间戳 |
| `timestamp` | LeRobot episode 内的规则训练时间，不取代上述原始时间 |

夹持力矩模式仍由部署控制器处理：训练动作中的夹爪是位置请求，不是牛顿或电机力矩；部署时必须沿用现有接触后力矩保持逻辑。

## OpenPI π0.5

已提供 `integrations/openpi/quest_x5.py` 和 `run.py`。采用单腕部相机，缺失的两路图像显式 mask；7 维 state/action 按模型标准补齐；6 关节动作转为相对当前状态的 delta，夹爪保留绝对宽度，输出逆变换回绝对关节目标。action horizon 根据数据集 FPS 设置为约 0.5 秒，在 240 Hz 下为 120。训练读取任务文本作为 prompt。旧版缺少 wrist_valid 列的导出数据需要重新导出后使用本适配器。

在已经按 OpenPI 官方说明配置好的 OpenPI 环境中运行（替换 OPENPI 路径）：

```bash
python /home/r2/work/code/QuestArmTeleop/integrations/openpi/run.py \
  /path/to/openpi /home/r2/work/code/QuestArmTeleop/datasets/lerobot/cup_task norm
python /home/r2/work/code/QuestArmTeleop/integrations/openpi/run.py \
  /path/to/openpi /home/r2/work/code/QuestArmTeleop/datasets/lerobot/cup_task train --exp-name=cup_x5
```

包装入口在运行时注册 `pi05_quest_x5`，无需手改 OpenPI 配置文件。首次会在项目 `outputs/openpi-datasets/local/quest_x5` 建立导出数据集链接；切换数据集时明确调整链接，入口不会静默覆盖。训练以 `pi05_base` 权重初始化，需要下载模型和合适 GPU。本项目尚未执行完整 π0.5 训练或真机策略推理；数据能加载不代表训练效果已经验证。

上游参考：
- https://github.com/Physical-Intelligence/openpi/blob/main/pyproject.toml
- https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/training/config.py
- https://github.com/huggingface/lerobot/blob/0cf864870cf29f4738d3ade893e6fd13fbd7cdb5/lerobot/common/datasets/lerobot_dataset.py
