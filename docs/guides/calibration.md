# 手眼标定

每个配置的相机（`configs/local.toml` 的 `[cameras.<id>]`）各有一个标定结果，标定经运行中的内核进行，不直接打开 CAN。

| 相机 | 安装 | 标定板 | 结果 |
| --- | --- | --- | --- |
| `wrist_rgb`（腕部 D435i） | `wrist` | 固定在桌面 | `T_eef_camera`：TCP → RGB 光学系 |
| `exterior_rgb`（固定 D435） | `fixed` | 夹在夹爪中 | `T_base_camera`：基座 → RGB 光学系 |

默认 `wrist_rgb` 为腕部安装，其他相机为固定安装；需要时在 `[cameras.<id>]` 中写 `mount = "wrist"` 或 `"fixed"`。TCP 是内核的 `tcp_link`（link6 沿 +x 0.145 m），与遥操、采集、策略使用同一末端坐标系。相机坐标系是 RGB 光学系（+X 右、+Y 下、+Z 前）。

## 1. 标定板

```bash
./run.sh calib handeye marker --size-m 0.08     # 生成 outputs/calibration/marker.svg
```

按 100% 打印，量出**黑色方块**的实际边长，写入 `configs/local.toml`，之后的命令不再需要 `--size-m`：

```toml
[hand_eye]
marker_size_m = 0.0795   # 实测值
marker_id = 0
```

## 2. 采集

先启动内核（`./run.sh kernel sim` 或 `./run.sh kernel real`）；共享相机服务未运行时会自动启动。

```bash
./run.sh calib handeye capture wrist_rgb --auto        # 腕部相机，标定板固定在桌面
./run.sh calib handeye capture exterior_rgb --auto     # 固定相机，标定板夹在夹爪中
```

1. 将机械臂摆到相机能清楚看到标定板的位置，夹爪保持当前宽度。是否支持手动拖动取决于后端；RealMan 当前将该示教命令映射为保持，需通过受支持的示教器或控制入口调整，不能直接硬拉机械臂。
2. 窗口显示标定板检测结果，按 `c` 确认中心姿态。
3. 内核在中心姿态周围依次访问 26 个关节目标（20 个拟合、6 个留出验证）。每一步先由内核预览（限位、桌面平面、速度和加加速度界限），被拒绝的目标跳过；到位后保持，采集 20 帧静止图像；连续 4 个姿态看不到标定板则停下并保持。
4. 完成后回到中心姿态，自动求解；留出验证通过时保存为该相机的当前结果。

不加 `--auto` 时为手动采集：示教状态下摆好姿态，按 `s` 采拟合样本、`v` 采验证样本，`q` 结束并求解（拟合至少 10 个，验证至少 4 个）。

自动移动、静止采样和图像写盘期间，窗口持续处理刷新和按键，显示当前样本数。采集中按 `q`、Esc 或关闭窗口会中止；Ctrl+C、实体急停或内核故障也会让机械臂停在原地保持，不会自动回中心。准备阶段按 `q` 则正常结束并求解已采样本。`--speed`（默认 0.3 rad/s）、`--step-rad`（默认 0.25）可调整采样范围与速度。

腕部相机经常报 `Marker not visible` 时，先在新一轮采集前改善中心姿态：可适当抬高相机、增加相机与板的距离，让黑色方块和外侧白边完整可见、接近画面中央，四周留足采样时转动的余量；同时保证方块仍足够大且清晰。采样期间标定板必须固定，不能跟着机械臂挪动。不要靠不断缩小 `--step-rad` 消除丢板，否则可能无法满足姿态多样性要求。

## 3. 结果

```bash
./run.sh calib handeye status                        # 每个相机的当前结果
./run.sh calib handeye solve outputs/calibration/hand_eye/<相机>/<会话> --save   # 离线重新求解
```

- 会话（原始图像、每帧机器人状态、`session.json`、`result.json`）在 `outputs/calibration/hand_eye/<相机>/<时间>/`。
- 当前结果在 `outputs/calibrations/hand_eye/<相机>.json`，上一个保留为 `<相机>.previous.json`。
- 求解：`relative_svd`、Shah、Li 三种方法在拟合样本上做鲁棒拟合（最多剔除 20%），选内点最多、残差最小者；留一法检查安装变换稳定性；留出验证样本从不参与拟合。`validated` 需要留一稳定且留出验证通过：至少 4 个验证姿态、满足运动多样性，且每个验证姿态均不超过 8 mm / 2°，不是仅看整体 RMS。终端会同时列出验证姿态不足、多样性不足或单姿态误差超限等未通过原因。
- 未通过验证的结果不会自动保存；检查后可用 `solve <会话> --save --force` 采用（`status` 会标注）。
- 结果绑定相机序列号：换相机后旧结果不再被使用。
- `embodied.applications.calibration.load(<相机>)` 返回可用结果；网页控制台据此在三维场景中显示相机。

## 4. 验证

`./run.sh check all` 的 `sim` 组在 MuJoCo 内核上运行完整的自动采样（合成检测，已知外参），`platform` 组的 `tests/test_calibration.py` 覆盖求解器两种安装方式、标定板、计划、静止采样和结果存储。
