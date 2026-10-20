# ARX X5 单臂控制

这个项目控制一台 ARX X5（`arx5-interface 0.1.3`）。它不会自动回零或校准；所有日常操作统一通过根目录的 `./arx5`。

## 快速开始

首次创建或更新控制环境：

```bash
UV_PROJECT_ENVIRONMENT=.venv uv sync --only-group control
```

这会安装 ARX SDK、Ruckig、Pinocchio 和 Coal（hpp-fcl 后继项目）。

网页控制台使用完全独立的前后端工程。首次安装或接口依赖更新后执行：

```bash
UV_PROJECT_ENVIRONMENT=.venv-handeye uv sync --only-group handeye
UV_PROJECT_ENVIRONMENT=.venv-web uv sync --only-group web
cd src/frontend
npm install
npm run build
cd ../..
```

```bash
cd /home/r2/work/projects/embodied/arx5_test
./arx5 can-up
./arx5 web-tailnet
```

命令会显示带配对令牌的 `https://Tailscale-IP:8000` 地址，并且服务只监听
Tailscale 地址。Mac 首次使用需要信任项目生成的本地 CA，之后不要求位于
同一 Wi-Fi，也不需要 SSH 隧道。

USB-CAN 重新插拔后只需再次运行 `./arx5 can-up`。`can0` 起不来时见
[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)。

若网页服务因原终端异常退出而残留、再次启动提示端口被占用，在机械臂已有可靠
支撑且实体急停可用时，可一键停止记录中的网页服务并重启：

```bash
./arx5 web-restart --confirm-arm-supported
```

## 手势遥操作

M3 Max 用 HTTPS 地址打开控制台并允许 Mac 摄像头权限；机械臂主机继续独占
CAN 和 D455。连接机械臂后选择 `Hand`，先保持“机械臂跟随”关闭，用服务端验证过
的半透明 Ghost 检查 XYZ 与旋转方向。确认无误后才启用跟随：非控制手张开并稳定
5 帧接通离合，放下或收拢该手即停车；控制手捏合关闭夹爪，松开捏合恢复默认
`40 mm` 张开宽度。

首次实机必须用“精细”档、空载和清空的工作区，验证丢手、松离合、网页急停和
实体急停后才可使用“快速”档。默认安全盒及完整步骤见
[docs/WEB_CONSOLE.md](docs/WEB_CONSOLE.md)；运行时边界见
[docs/SAFETY.md](docs/SAFETY.md)。

## GR00T 微调控制

使用 `outputs/groot-training` 下数字最大的完整 ARX5 微调检查点：

```bash
./arx5 zeroshot --check
./arx5 zeroshot --smoke
./arx5 zeroshot
```

正常启动后输入任务描述，检查只读模型预览，再精确输入 `EXECUTE`。运行中直接按
`S/H/N/Q` 可停止、确认回零、输入下一任务或退出。完整流程与安全边界见
[docs/GR00T_CONTROL.md](docs/GR00T_CONTROL.md)。

## 常用命令

| 命令 | 用途 |
|---|---|
| `./arx5 can-up` | `can0` 缺失时自动恢复；接口正常时只显示状态 |
| `./arx5 can-restart` | 强制重启 `can0` 的 SLCAN 桥 |
| `./arx5 can-status` | 查看链路、bus-off、丢包和错误计数 |
| `./arx5 state` | 读取状态，不主动运动 |
| `./arx5 control` | 长驻交互控制，动作间不重复初始化 |
| `./arx5 test-gripper` | 位置控制 `0 → 40 mm → 0` |
| `./arx5 test-force` | 位置张开 20 mm，再以 0.15 N·m 限扭矩闭合 |
| `./arx5 calibrate-payload` | 示教并采集末端工具/新增负载数据 |
| `./arx5 calibrate-hand-eye` | D405/D455 双相机手眼标定 |
| `./arx5 web-tailnet` | 只监听 Tailscale IP 的 HTTPS 控制台（推荐） |
| `./arx5 web-restart --confirm-arm-supported` | 停止记录的网页服务后重新启动 Tailscale 控制台；仅在机械臂有支撑、急停可用时使用 |
| `./arx5 web-lan` | 局域网 HTTPS 初学者控制台，不需要 SSH 隧道 |
| `./arx5 web` | 只监听 localhost 的控制台，适合 SSH 隧道 |
| `./arx5 vision` | 启动 Mac 网页和只读关键点接收桥 |
| `./arx5 zeroshot` | 使用最新 ARX5 微调 GR00T 权重进行任务描述交互控制 |
| `./arx5 test` | 运行不连接硬件的单元测试 |

查看所有参数：

```bash
./arx5 help
./arx5 can-up --help
```

## 文档

| 文档 | 内容 |
|---|---|
| [docs/SAFETY.md](docs/SAFETY.md) | 关节、XYZ、夹爪和运行时保护边界 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Vue/FastAPI 边界、模块和 API v1 |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | USB-CAN、接口占用、网页缓存 |
| [docs/CONTROL_TERMINAL.md](docs/CONTROL_TERMINAL.md) | `./arx5 control` 命令与夹爪策略 |
| [docs/TRAJECTORY_CONTROL.md](docs/TRAJECTORY_CONTROL.md) | 轨迹、插值、Ruckig 与动态约束边界 |
| [docs/WEB_CONSOLE.md](docs/WEB_CONSOLE.md) | 末端位姿编辑、Tailscale 访问、快捷键 |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | 末端负载重力标定与双相机手眼标定 |
| [docs/CUBE_VISION.md](docs/CUBE_VISION.md) | D455 RGB 方块检测与自动抓取 |
| [docs/MEDIAPIPE.md](docs/MEDIAPIPE.md) | 人体遥操作准备清单与软件边界 |
| [docs/GR00T_CONTROL.md](docs/GR00T_CONTROL.md) | 微调权重选择、终端流程与连续控制边界 |
| [src/frontend/README.md](src/frontend/README.md) | 前端构建、证书信任、第一次手动操作 |
| [src/backend/README.md](src/backend/README.md) | FastAPI 服务、版本化协议和功能模块 |
| [src/hand_eye_calibration/README.md](src/hand_eye_calibration/README.md) | 标定板打印、采样覆盖度、复算 |
| [src/experiments/cube_pick_rotate_place/README.md](src/experiments/cube_pick_rotate_place/README.md) | 终端方块夹取实验 |

## 目录

```text
arx5                 统一命令入口
src/                 全部运行源码
src/arx5_control/    长驻控制器、安全检查和 motion/ 轨迹算法子包
src/experiments/     成套的端到端实验，抓取几何的唯一定义处
src/hand_eye_calibration/  眼在手外标定求解与坐标变换
src/scripts/         当前可运行工具和服务编排
src/frontend/        Vue 3 + TypeScript 浏览器控制台
src/backend/         FastAPI API、控制内核及相机/视觉功能服务
assets/              纹理、图标、WASM 和前端运行库
weights/browser/     版本化 X5/D405 网格和 MediaPipe 模型
calibrations/        手眼、夹爪和负载标定结果，不进入 Git
datasets/            采集数据位置说明
outputs/             可人工查看的诊断图像和实验输出，不进入 Git
configs/             项目级运行配置
docs/                设计与准备说明
pyproject.toml       uv 项目元数据与分环境依赖组
uv.lock              uv 锁定依赖版本
tests/               不连接硬件的单元测试
runtime/             PID、令牌、证书和可再生成缓存，不进入 Git
```

GR00T、CLIP、YOLO 预训练权重位于共享目录 `~/work/pre_weights/`；原始采集与训练导出数据位于 `~/work/data/arx5/`。

## 安全

网页的所有自动动作都是预览、执行两段式，软件层做 IK、关节限位、轨迹速度、
扭矩和反馈检查。完整边界见 [docs/SAFETY.md](docs/SAFETY.md)。这些软件保护
不能替代物理急停、现场隔离和机械限位；网页急停也不能替代实体急停。直接切断
机械臂电源会失去保持力并可能坠落，只能在人员和设备已避开坠落区时作为最后手段。
