# 手眼标定

每个配置的相机（`configs/local.toml` 的 `[cameras.<id>]`）各有一个标定结果，标定经运行中的内核进行，不直接打开 CAN。

| 相机 | 安装 | 标定板 | 结果 |
| --- | --- | --- | --- |
| `wrist_rgb`（腕部 D435i） | `wrist` | 固定在桌面 | `T_eef_camera`：TCP → RGB 光学系 |
| `exterior_rgb`（固定 D435） | `fixed` | 夹在夹爪中 | `T_base_camera`：基座 → RGB 光学系 |

默认 `wrist_rgb` 为腕部安装，其他相机为固定安装；需要时在 `[cameras.<id>]` 中写 `mount = "wrist"` 或 `"fixed"`。TCP 是当前机器人配置的 `tcp_link`，与遥操、采集、策略使用同一末端坐标系；偏移以对应机器人配置及控制器读回为准，不套用其他机械臂的工具偏移。相机坐标系是 RGB 光学系（+X 右、+Y 下、+Z 前）。

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
3. 内核在中心姿态周围依次访问 26 个关节目标（20 个拟合、6 个留出验证）。ARX 保留原随机偏移；RealMan 使用单轴／双轴预设偏移，按 RM65 TCP 几何分别选取多样的拟合与验证点。每一步先由内核预览（限位、桌面平面、速度和加加速度界限），被拒绝的目标跳过；到位后保持，采集 20 帧静止图像；连续 4 个姿态看不到标定板则停下并保持。
4. 完成后回到中心姿态，自动求解；留出验证通过时保存为该相机的当前结果。

不加 `--auto` 时为手动采集：示教状态下摆好姿态，按 `s` 采拟合样本、`v` 采验证样本，`q` 结束并求解（拟合至少 10 个，验证至少 4 个）。

自动移动、静止采样和图像写盘期间，窗口持续处理刷新和按键，显示当前样本数。采集中按 `q`、Esc 或关闭窗口会中止；Ctrl+C、实体急停或内核故障也会让机械臂停在原地保持，不会自动回中心。准备阶段按 `q` 则正常结束并求解已采样本。`--speed`（默认 0.3 rad/s）、`--step-rad`（默认 0.25）可调整采样范围与速度。

腕部相机经常报 `Marker not visible` 时，先在新一轮采集前改善中心姿态：可适当抬高相机、增加相机与板的距离，让黑色方块和外侧白边完整可见、接近画面中央，四周留足采样时转动的余量；同时保证方块仍足够大且清晰。采样期间标定板必须固定，不能跟着机械臂挪动。不要靠不断缩小 `--step-rad` 消除丢板，否则可能无法满足姿态多样性要求。

### RealMan：先预设、仿真，再采集

沿用同一计划 JSON、`CalibrationArm` 和内核 MoveJ，不使用旧 ARX 直连控制器。没有相机安装估计时，`capture --auto` 使用上述单轴／双轴点位；这只能控制点位变化方式，不能保证标记始终在画面内。已有能求解的腕部会话时，推荐先用它生成带视野筛选的计划：

```bash
# 工作目录：仓库根目录。<SESSION> 是已有 wrist 会话目录。
./run.sh calib handeye plan <SESSION> \
  --robot-config configs/robots/realman/sim.json --step-rad 0.25 \
  --output outputs/calibration/plans/realman-wrist.json
```

只读取原会话的拟合内点和 `result.json` 中的安装估计，不读取留出验证观测、不修改原结果，也不将 `candidate` 启用为有效标定。板的位置根据这些拟合观测和 RM65 标称模型重新估计，来源及用途记在计划的 `view` 中。预设只扰动一或两个关节；先留出 6 个多样验证点，再从剩余候选中选 20 个拟合点，两组不重复。全套点位先检查关节限位、两组 TCP 多样性，以及标记连同白边的预测视野，距图像边界保留 24 px。各段含返回中心的关节插值另取 11 点检查视野；这不是遮挡检测或连续碰撞证明。

再用独立的 RM65 仿真内核检查运动。第一个终端启动后，从日志取得它的 `<SIM_RUN>` 目录：

```bash
./run.sh kernel start --backend sim --input idle --managed --headless \
  --config configs/robots/realman/sim.json
```

另一个终端执行：

```bash
./run.sh calib handeye preview outputs/calibration/plans/realman-wrist.json \
  --run <SIM_RUN> --speed 0.17
```

需要观察时，在第三个终端运行 `./run.sh kernel viewer --run <SIM_RUN>`，沿用现有只读查看器。

`preview` 先检查运行确实是仿真和模型一致，再经原内核移动到计划中心、逐点执行并返回中心。它不打开相机、不生成标定样本、不运行求解；实际到位关节、TCP、跳过原因与两组实际运动跨度保存在计划旁的 `.preview.json`。传入真机 RUN 会在申请控制权前拒绝。全部点位到达才报告完成。

仿真确认后，在真机通过支持的示教器或控制入口摆回计划的 `center`（各关节相差不超过 0.03 rad），保持相机安装不变、标记完整可见，再执行：

```bash
./run.sh calib handeye capture wrist_rgb --auto --run <REAL_RUN> \
  --plan outputs/calibration/plans/realman-wrist.json --speed 0.17 --size-m 0.10
```

按 `c` 时会核对相机 ID、序列号和标记规格，并用当前中心观测重新锚定板的位置、检查所有点的预测视野；不满足时在开始自动运动前退出。`--size-m` 必须是黑方块实测边长。标称模型与近似安装关系只用于规划；实际遮挡、检测稳定性、真实控制器 DH 偏差与标定误差仍需现场验证。仿真通过不会改变 8 mm / 2° 质量门，也不会采用旧候选结果。

## 3. 结果

```bash
./run.sh calib handeye status                        # 每个相机的当前结果
./run.sh calib handeye solve outputs/calibration/hand_eye/<相机>/<会话> --save   # 离线重新求解
```

- 会话（原始图像、每帧机器人状态、`session.json`、`result.json`）在 `outputs/calibration/hand_eye/<相机>/<时间>/`。
- 当前结果在 `outputs/calibrations/hand_eye/<相机>.json`，上一个保留为 `<相机>.previous.json`。
- 求解：使用 `relative_svd` 初始化；当前 OpenCV 提供 `calibrateRobotWorldHandEye` 时，也比较 Shah、Li 初始化。每个候选、内点重拟合及留一检查均联合优化安装变换与固定目标变换的旋转、平移，使 `A_i X C_i = Z` 的几何残差最小；按 8 mm / 2° 归一化两种残差。之后按原门槛筛选（最多剔除 20%），选内点最多、残差最小者。输出的 `method` 记录初始化方法，`refinement` 记录联合几何优化；留出验证样本从不参与拟合、剔除或模型选择。
- `validated` 需要留一稳定且留出验证通过：至少 4 个验证姿态、满足运动多样性，且每个验证姿态均不超过 8 mm / 2°，不是仅看整体 RMS。40 mm 平移跨度、20° 旋转跨度、旋转轴秩及误差阈值均为本项目的工程质量门，不是论文规定的通用精度保证。终端会同时列出验证姿态不足、多样性不足或单姿态误差超限等未通过原因。
- 未通过验证的结果不会自动保存；检查后可用 `solve <会话> --save --force` 采用（`status` 会标注）。
- 结果绑定相机序列号：换相机后旧结果不再被使用。
- `embodied.applications.calibration.load(<相机>)` 返回可用结果；网页控制台据此在三维场景中显示相机。

边长必须是检测到的那个黑色方块的实测长度（米），不是纸张或外框尺寸。标记 PnP 的平移尺度随输入边长同比变化：`--size-m 0.10` 和 `--size-m 1.0` 相差十倍。采集启动时会打印实际使用的尺寸；已有会话不能只修改 `target` 字段，必须由原始角点、相机内参重新计算每帧位姿并重新检查静止性。不能把尺寸当作调参项来降低拟合误差。

联合优化参考 [Horaud 与 Dornaika，Hand-Eye Calibration，1995，§5.2](https://doi.org/10.1177/027836499501400301) 的同时估计旋转和平移思路。本实现使用两个 SE(3) 变换、旋转向量及按项目门槛归一化的绝对姿态残差，属于工程实现，不是对论文四元数目标函数及权重的逐式复现。坐标方向与最少非平行旋转条件见 [OpenCV 手眼标定文档](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)；尺寸与位移同单位的约定见 [ArUco 位姿估计文档](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)。

## 4. 验证

`./run.sh check all` 的 `sim` 组在 MuJoCo 内核上运行完整的自动采样（合成检测，已知外参），`platform` 组的 `tests/test_calibration.py` 覆盖求解器两种安装方式、标定板、计划、静止采样和结果存储。
