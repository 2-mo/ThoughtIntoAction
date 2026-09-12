# X5 原始采集与 π0.5 训练导出

本地只采集、回放和打包，不再在任务结束时转换 LeRobot 或重编码视频。任务名来自 `启动遥操.sh` 的 COLLECTION_TASK。

## 本地压缩记录

- `datasets/.raw/<episode>/episode.json`：任务、结果、配置及完整性。
- `states.jsonl.gz`：机器人逐周期状态、请求动作和原始精确时间戳，gzip 无损压缩。
- `frames.jsonl.gz`：每帧设备/主机时间戳及视频帧索引，gzip 无损压缩。
- `wrist_rgb.mp4`：640×480、30 FPS、H.264/libx264 CRF 20，不重新编码。

每条结束后压缩 JSONL；回读并核对 SHA256 后才替换未压缩文件。旧的未压缩 JSONL、PNG episode 仍可读取。机器人采样保持 240 Hz，遥操处理 90 Hz。时间戳精度不变；相机主机接收时间并不等于曝光时间，设备未硬件同步。

## 任务打包与服务器转换

例如采集 110 条：A 开始/保存每条，长按右摇杆 1 秒结束任务，得到：

```text
datasets/任务名_时间-编号.zip
  任务名_时间-编号/
    manifest.json       # 格式版本、任务条目、文件大小和 SHA256
    README.md           # 字段、单位、时间戳、筛选、服务器转换和训练说明
    转换为LeRobot.sh      # 解压即可运行，自动定位数据并准备导出环境
    tools/              # 随包携带的转换器及 OpenPI 适配器快照
    episodes/
      <episode>/
        episode.json
        states.jsonl.gz
        frames.jsonl.gz
        wrist_rgb.mp4
```

直接上传 ZIP。其内部 JSONL 已 gzip 压缩、MP4 已 H.264 压缩，ZIP 只封装不重复压缩。包内都是实际文件，不依赖本机符号链接。包含本轮所有已停止录制条目，丢弃、失败和中断条目连同原始标记一并上传，由服务器筛选；生成后进行 ZIP CRC 校验。状态记录在 `datasets/.raw/tasks/<任务名>/package-status.json`。失败保留原始记录，临时包不会当成成功产物。

服务器无需安装 QuestArmTeleop 项目。Linux 上准备好 `uv` 和 FFmpeg 共享库后：

```bash
unzip 任务名_时间-编号.zip
cd 任务名_时间-编号
bash 转换为LeRobot.sh
# 明确决定跳过无效条目时：
bash 转换为LeRobot.sh --skip-invalid --output /path/to/lerobot_dataset
```

首次运行自动下载 CPU 导出依赖，需要网络；环境位于包内 `tools/outputs/envs/lerobot-export`，与遥操环境隔离。默认读取包内 `episodes/`，输出包内 `lerobot_dataset/`，路径不依赖终端当前目录。`tools/convert.sh` 是兼容入口。可用 `LEROBOT_EXPORT_PYTHON=/path/to/python` 复用已安装的固定版本环境。`--help` 无需安装环境即可查看。

导出器直接读取 .jsonl.gz，无需单独解压 JSONL。最终生成一个含 meta/data/videos 的 LeRobot 数据集。暂停恢复保持同一任务；程序退出不会自动恢复任务，原始记录仍可手动打包。现有包和输出目录不会自动覆盖。

### 转换进度与速度

终端显示数据检查进度、总帧数、当前条目、帧/秒、预计剩余时间，以及表格/统计写入、加载验证阶段。完成后打印耗时与平均吞吐量，并将参数和速度记录在 `meta/quest_export.json`。`--no-progress` 关闭本工具进度条，最终报告仍保留。

默认使用与 PyTorch 2.7.1 匹配的 **TorchCodec 0.5.0** 批量 CPU 解码，每批 32 帧，`--decode-workers 0` 在主进程内解码，避免小任务的进程开销。`--decode-workers 2` 或 `4` 启用多进程；进程池跨 episode 复用，按提交顺序消费结果，最多保留进程数加 1 个批次，避免把全部 RGB 图像放入内存。`--decoder opencv` 可选择顺序 OpenCV 解码。TorchCodec 需要可加载的 FFmpeg 共享库，Debian/Ubuntu 可安装系统 `ffmpeg` 包。

H.264 使用流式编码，默认 CRF 20、GOP 2、`veryfast`，编码线程数最多 4。仅为固定版 LeRobot 官方图像统计保存必要的无损 PNG 样本，其余帧直接编码；完成后移除临时图片。检查与转换每次只读取一条原始 episode。分辨率、30 FPS 观测频率、动作含义和原始 int64 时间戳保持不变。

```bash
# 多进程批量解码，适合测试服务器吞吐量
bash 转换为LeRobot.sh --skip-invalid --decode-workers 4 --decode-batch-size 32 --video-threads 4
# 小任务减少进程启动/传输开销
bash 转换为LeRobot.sh --skip-invalid --decode-workers 0
```

更多进程不保证更快：解码、跨进程传输、编码、磁盘、统计都影响总耗时。调小 `--decode-batch-size` 可减少 RGB 缓冲内存；每个 TorchCodec 子进程还会加载自己的 PyTorch。`--video-preset fast`/`medium` 用更多编码时间换取压缩效率；各档位均保持 CRF 20，但文件大小和解码像素可能因重编码发生变化。原始 MP4 始终保留在采集包中。

本机 5 条真实 episode、共 1,325 帧的完整转换测试（包含 Python 启动、检查、状态保存、编码、统计及加载验证，单次测量）：OpenCV 单进程 14.49 秒、TorchCodec 单进程 13.78 秒、TorchCodec 双进程 14.97 秒。该小样本中进程启动与传输成本高于收益，因此默认主进程内批量解码；大任务应按服务器实测选择进程数。此处是端到端耗时，不是单独解码时间。

脚本与转换代码是打包时的快照；已经生成的 ZIP 需要重新打包才能包含更新。

升级后需要重启遥操并刷新网页，让运行进程加载压缩读取支持。上传完成不会自动删除本地备份。

## 手动合并导出 LeRobot v2.1

OpenPI 上游固定 LeRobot revision `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`，其数据格式为 v2.1。导出使用该版本官方 `LeRobotDataset.create/add_frame/save_episode`，并用同版本真实加载器验证首尾样本和动作 chunk。不会上传 Hugging Face。

首次环境安装（与遥操 Python 完全隔离）：

```bash
./scripts/datasets/setup_export.sh
```

按任务导出：

```bash
./scripts/datasets/导出训练数据.sh --source datasets/.raw --task '把杯子放入盒子' --fps 30 --skip-invalid --output datasets/lerobot/cup_task
```

默认拒绝不完整、已丢弃、失败、丢帧、时序间断、混合模型或包含预设/回放的 episode；明确使用 `--skip-invalid` 可以跳过，并在 `meta/quest_export.json` 查看每条拒绝原因。`unknown` 表示未经成功标注，不等于成功演示；训练前应筛选任务完成质量。工具不覆盖已有导出目录。

服务器转换输出 **30 Hz 观测表和 30 FPS 视频**，每个真实图像只出现一次，不重复扩帧。机器人按图像主机接收时刻取最近的此前状态；没有对应此前状态的边界图像不作为训练锚点，数量写入导出报告，原始包不删除这些图像。

独立的 `robot/episode_NNNNNN.npz` 完整保留该条机器人 state/action 和 int64 原始时间戳（约 240 Hz），包括观测锚点前后的机器人样本。无需 pickle，机器人数值以 float64 保存。其余诊断和元数据仍完整保留在原始包中。

OpenPI 使用标准 LeRobot 动作加载器：训练 state/action 与图像统一为 30 Hz。每张真实图像按主机接收时间戳选择此前最新机器人状态和目标动作，连续 15 行 action 构成约 0.5 秒的动作块。独立的 240 Hz NPZ 只用于原始数据保留、核查和将来重新处理，不参与默认训练动作分块。相机抖动保留在 source 时间戳中，标准训练时间轴仍为行号/30。

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

已提供 `integrations/openpi/quest_x5.py` 和 `run.py`。采用单腕部相机，缺失的两路图像显式 mask；7 维 state/action 按模型标准补齐；6 关节动作转为相对当前状态的 delta，夹爪保留绝对宽度，输出逆变换回绝对关节目标。action horizon 根据 action_fps 设置为约 0.5 秒，30 Hz 动作下为 15；观测同为 30 Hz。训练读取任务文本作为 prompt。旧导出需要重新转换为 aligned30_raw240_v1 才能使用新适配器。

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
