# GR00T 微调权重控制

共享预训练权重位于 `~/work/pre_weights/GR00T-N1.7-3B/`。

`./arx5 zeroshot` 只使用 ARX5 微调权重，不包含 DROID 跨机器人适配。
启动时递归扫描 `outputs/groot-training`，选择数字最大的完整
`checkpoint-N`，并在本次进程内固定该路径。也可用 `--checkpoint` 指定一个
完整检查点。

## 准备

同步控制环境：

```bash
UV_PROJECT_ENVIRONMENT=.venv-handeye uv sync --only-group handeye
```

模型环境必须存在于：

```text
reference_projects/Isaac-GR00T/.venv
```

先执行无硬件检查和 GPU 冒烟：

```bash
./arx5 zeroshot --check
./arx5 zeroshot --smoke
```

`--check` 不加载模型、相机或 CAN。`--smoke` 只启动本地 GR00T 服务并发送
一组合成观测，不打开相机或 CAN。

## 运行

确保网页控制、采集和其他控制程序已经退出，CAN 与两台相机没有被占用。机械臂
必须有可靠支撑，实体急停可触及，工作区内不得有人。

```bash
./arx5 zeroshot
```

程序加载模型并检查 D455/D405 后提示输入任务描述。模型预览阶段只读取相机与
机械臂状态，不激活机械臂。检查首目标位移、姿态变化和夹爪状态后，必须精确输入
`EXECUTE` 才会开始控制。

运行时按键不需要回车：

| 按键 | 行为 |
|---|---|
| `S` | 平滑停止、进入阻尼并返回任务输入 |
| `H` | 停止后预览保存的采集零位；精确输入 `HOME` 才执行 |
| `N` | 停止并输入下一条任务描述 |
| `Q` | 停止、进入阻尼并退出 |

故障只会停止并进入阻尼，不会自动回零或自动重启模型。

## 执行边界

- 观测固定为当前 D455 `exterior_rgb`、D405 `wrist_rgb`、ARX5 TCP、夹爪和
  六关节状态。
- 模型使用 `new_embodiment`，每次输出 16 步，执行频率 30 Hz，每 8 步滚动重规划。
- 推理在独立线程滚动运行；上一安全 chunk 的尾部通过 RTC 回馈模型，新旧动作按
  观测时间映射到统一 30 Hz 时间槽，并在最多 8 个重叠槽内渐进融合。
- 坏 chunk 不覆盖现有安全轨迹；系统保持上一安全目标并后台重试。首次 3 秒或运行中
  连续 1 秒没有安全动作时才平滑停机。
- 模型预览最多自动重采样 4 次，避免单次随机扩散样本阻塞操作。
- 每个动作块必须通过形状、有限值、旋转正交、工作区和跳变检查。
- 单步位置跳变上限为 20 mm，姿态跳变上限为 15 度。
- 实际速度、加速度、jerk、IK、关节限位和反馈看门狗由现有
  `TeleoperationServo` 与 `ArmController` 继续执行。
- 返回零位使用 `calibrations/collection/home.json`，先生成完整轨迹预览，再
  二次确认。

模型子进程只监听 `127.0.0.1`。每次启动的检查点和模型日志记录在
`outputs/gr-zeroshot/<timestamp-pid>/`，不记录图像。
