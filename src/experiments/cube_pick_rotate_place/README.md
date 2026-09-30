# D455 方块夹取、旋转与放置实验

这个实验复用项目已有的黑色方块检测、固定 D455 手眼标定和
`arx5_control.ArmController`。目标物为平放在已配置桌面上的黑色
`30 x 30 x 30 mm` 方块。默认在原位置放下，并绕基座 `+Z` 方向旋转 `+45 deg`。

## 运行前

- 关闭网页/视觉服务，避免 D455 或 `can0` 被其他进程占用。
- 确认 `calibrations/hand-eye/eye-to-hand/enabled.json` 是当前 D455 的已验证标定。
- 让方块完整出现在桌面区域内，桌面高度与 `config.json` 一致。
- 真机执行前，手动把 TCP 放在桌面上方至少 80 mm 的无障碍位置。
- 清空整个运动包络，准备物理急停。异常阻尼会释放持续夹持力，需考虑物体掉落。

## 先做感知预览

在项目根目录执行：

```bash
PYTHONPATH=src .venv-handeye/bin/python -m experiments.cube_pick_rotate_place
```

该命令只启动 D455、输出基座坐标和动作计划，不连接机械臂。最后一帧检测结果默认
保存在 `outputs/cube-pick/last-observation.jpg`。

强制关闭深度流、验证 RGB-only 路径：

```bash
PYTHONPATH=src .venv-handeye/bin/python -m experiments.cube_pick_rotate_place --rgb-only
```

RGB-D 模式始终使用 RGB 射线与已知方块顶面平面求交得到抓取 `x/y`。深度仅验证该
几何结果是否自洽，不会让小方块表面的单次深度噪声改写抓取坐标。

## 扫描全部方块

在机械臂尚未动作、所有待抓取方块静止时，执行：

```bash
PYTHONPATH=src .venv-handeye/bin/python -m experiments.cube_pick_rotate_place --scan-all
```

该命令约采集 90 帧，给每个稳定候选方块分配 ID，并将基座坐标、方向、颜色、重复测量
RMS 和深度校验数保存到 `outputs/cube-pick/last-scene.json`。它不会连接 CAN 或机械臂。
先检查该快照及 `last-observation.jpg`，再把选定 ID 接入后续堆叠规划；每次放置后仍应复核
堆叠顶面，不能把初始快照当作放置误差的替代品。

## 真机执行

先恢复 CAN，再执行：

```bash
./arx5 can-up
PYTHONPATH=src .venv-handeye/bin/python -m experiments.cube_pick_rotate_place --execute
```

程序连接后会对整条笛卡尔动作链做连续 IK 预检，并打印每段最大关节变化。只有输入
`EXECUTE_PICK_ROTATE_PLACE` 后才发送动作。夹爪未检测到接触时不会抬升。

常用覆盖参数：

```bash
PYTHONPATH=src .venv-handeye/bin/python -m experiments.cube_pick_rotate_place \
  --rotation-deg 45 \
  --place-offset-x-mm 0 \
  --place-offset-y-mm 0 \
  --table-z-mm 0 \
  --tcp-z-offset-mm 0 \
  --speed-percent 5
```

`tcp-z-offset-mm` 用于实测指尖夹持中心相对官方 `eef_link` 的小量修正。正值会让抓取
TCP 更高。不要用这个参数掩盖错误的桌面高度或手眼外参。

## 坐标与尺寸验证

1. 在桌面上标出一个相对机械臂基座已知的点，把方块中心放到该点。
2. 运行默认预览，检查 `base xyz` 的 `x/y` 与标记位置一致，`z` 应为桌面高度加 15 mm。
3. 检查 `measured edge` 接近 30 mm；默认接受范围是 20 到 48 mm。顶面轮廓可能包含少量
   可见侧面，尺寸门限用于排除明显错误的候选，不代表物理尺寸的高精度测量。
4. 再运行 `--rgb-only`，其 `x/y` 应与 RGB-D 结果在手眼标定和桌面模型误差范围内一致。
5. 查看调试图，黄色轮廓必须落在真实方块上，绿色四边形是参与检测的桌面区域。

## 当前限制

- 没有碰撞模型；程序约束 TCP 路径、关节限位和初始高度，但无法识别临时障碍物。
- 官方 URDF 的 `eef_link` 位于第六关节沿工具 `+X` 方向 145 mm，实验按该 TCP 抓取。
  实物指尖中心仍可能需要毫米级 `tcp-z-offset-mm` 修正。
- 正方形外观存在 90 度方向等价，绝对 yaw 不是唯一的；相对旋转 45 度不受影响。
- RGB 黑色轮廓法要求方块与桌面有足够亮度差，并依赖正确的桌面位置和手眼标定。
- 放置默认回到原位置；可通过基座坐标系的 X/Y 偏移做小范围搬运，但仍需人工确认包络。
