# 接入配置（总览页）

总览页现在也是「接入界面」：选机械臂厂商与型号、单臂/双臂、每条臂的 CAN 口，再选相机并把每台相机绑定到某一台臂的腕部或作为固定外部相机。以前这一步靠手改配置和试错，现在在网页上完成。

## 数据放在哪

| 文件 | 作用 | 是否入库 |
|---|---|---|
| `configs/setups/<slug>.json` | 命名保存的配置库，一条一套机械臂 + 相机 | 否（gitignore） |
| `configs/devices.json` | 当前生效的配置（点「载入」后写入） | 否（gitignore） |
| `outputs/runtime/setup/*.json` | 载入时生成的内核 profile（机器人 profile + 两个 workbench profile） | 否 |
| `configs/local.toml` | 其余本机覆盖（socket、host 等），相机仍可在这里留颜色/标定等键 | 否 |

`load_config()` 会把 `devices.json` 合并进 `[cameras]`，所以相机服务、采集、标定、策略适配器读到的仍是原来的 `[cameras.<id>]` 结构，无需改动。**相机集合以 `devices.json` 为准**：在页面上删掉的相机不会再从 `local.toml` 里冒出来。

## 页面结构

1. **配置库**：已保存的配置显示为一行可点的卡片，当前生效的高亮。点一下即整套载入；名称相同的保存会覆盖原条目，换个名字则新增一条。
2. **机械臂**：厂商、型号、单臂/双臂，以及每台臂的 CAN 接口（从已探测的适配器和现有接口里选）。
3. **相机**：先选数量，再逐台从「已探测设备」里挑序列号；每格顶部是该相机的实时画面，下方选角色（左腕/右腕/外部）。相机 ID 由角色自动生成并沿用既有命名（`wrist_rgb`、`wrist_second_rgb`、`exterior_rgb`），所以数据格式不需要迁移。
4. **保存/载入**：服务端校验通过后才写入；载入会生成内核 profile，并列出需要重启的服务和你确认，不会自己重启。

画面来源：共享相机服务在跑时直接用它已编码的预览；未启动时临时打开该相机取一帧（3 秒缓存），相机被别的进程占用时格内显示原因。

### 校验规则（服务端）

- 同一套设备只能有一个厂商；厂商未接入时返回 409「待接入」。
- 双臂必须 left / right 各一台，CAN 接口互不重复。
- 相机 ID 必须匹配 `[a-z][a-z0-9_]*_rgb`，序列号不能重复（留空表示尚未分配）。
- 每台臂最多一台腕部相机；`fixed` 相机不绑定臂。

## 生效方式

内核启动只有两个选择——**采集**（Quest 遥操接管）和**推理**（策略接管），另有一个「仿真内核」勾选框。单臂还是双臂、用哪套 CAN，都由当前生效的配置决定，页面不再让你选 profile 名。

保存只写配置与生成的 profile，不会打断正在进行的遥操或采集：

| 改动内容 | 需要重启 |
|---|---|
| 相机清单、序列号、分辨率、帧率、绑定 | 共享相机服务（`sensor`） |
| 机械臂、厂商、CAN | 内核（`kernel`） |
| 仅名称/标签 | 无需重启 |

采集中时后端会拒绝重启相机服务（409「采集中，先停止采集再重启相机服务」）。内核重启会结束当前采集，请在停止后再操作。

## 接入新厂商（例如 RealMan）

厂商表在 `src/embodied/core/device_setup.py` 的 `VENDORS` 里。接入一个新厂商需要：

1. 驱动与 profile：`src/embodied/devices/robots/<vendor>/`（profile、plugin、运动学、real/sim 后端）。
2. 注册表 `configs/plugins.json`：`robot_profile`、`robot`、`kinematics` 各加条目。
3. 工作台 profile：`configs/workbench/` 下按「臂数 × 接管方式」各一个（采集 `real-quest` / `real-dual-quest`，推理 `real-manual` / `real-dual-manual`）。
4. 在 `VENDORS` 里填 `models`、`robot_profile{1,2}`、`workbench_profile{1,2} = {"collect": ..., "policy": ...}`，把 `implemented` 置为 `true`。
5. 如有独立的发现方式（以太网等），在 `core/devices.py` 的 `discover()` 里加一类设备；CAN 类厂商沿用现有的 USB2CAN 枚举。

保存时 `compile_setup()` 用 `VENDORS[vendor]` 里的名字生成 `<vendor profile>.json` 和对应 workbench profile，内核启动时通过 `operations._setup_profile()` 使用生成的 workbench 路径——厂商名字与路径都来自注册表，不接受浏览器传入的任意路径。

## 排障

| 现象 | 处理 |
|---|---|
| 页面「已探测设备」为空 | 相机或 USB-CAN 没插好；点「重新探测」；后端枚举失败时会在同一区域显示原因 |
| 保存报「RealMan 尚未接入」 | 该厂商还没注册，属预期 |
| 保存后相机没变化 | 没重启相机服务；按页面提示点「重启相机服务」 |
| 保存后内核还是旧臂数 | 没重启内核；重启时控制台会用生成 profile，若臂数与所选 profile 不符会直接拒绝并提示 |

## 验证（离线）

```bash
PYTHONPATH=src configs/environments/arx_motion/.venv/bin/python -m pytest tests/motion/test_console_setup.py -q
PYTHONPATH=src .venv/bin/python -m pytest tests/test_package_boundaries.py -q
cd apps/console && npm run typecheck && npm run build
```

真机验证：插拔相机后点「重新探测」→ 保存 → 重启相机服务 → 三路画面；重启内核确认 12 关节与两条 CAN；录一条 episode 确认相机与锚点正确。
