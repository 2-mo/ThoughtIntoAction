# 共享机械臂场景

`view-arm` / `scripts/viewer/monitor.sh` 单独创建 `embodied.applications.monitor.scene.Scene`，订阅通用机械臂状态并服务本机只读网页。机械臂服务与独立数据回放使用 `telemetry.local` 发布数据，不创建或关闭场景；显示无需区分输入方式。详见 [独立监看](../../../../../docs/guides/monitoring.md)。

显示处理参考桌面备份 `arx5_test/src/frontend/src/features/scene/runtime.ts` 的 `createLinkMaterial`、RoomEnvironment、灯光和木纹工作台。只迁移显示处理及有来源记录的纹理；没有复制其硬编码 `JOINT_SPECS`、旧 STL、D405 安装偏移、夹爪网格形变、伺服操作或网页控制逻辑。工作台仅为示意，不参与碰撞判断。此实现不依赖、也不改动 Desktop 工程。

`gripper.js` 对本仓库 X5 `link6.STL` 校验 SHA-256 后，按原始连通部件拆出左右夹指、滑块、齿条和随动紧固件；壳体、导轨和端部止挡保留固定。所有三角面和材质保持原样。原网格指尖内侧间距约 60.286 mm，左右组件各沿网格局部 Y 轴移动开度差的一半，随后继承 URDF visual origin、scale 与手腕世界变换。该动画仅用于显示，不修改 URDF、六轴 FK 或控制数据。实际与目标开度分别驱动实体和目标虚影，夹爪单独运动也可显示虚影；回放读取录制开度。缺少开度时显示原始参考几何，非匹配 STL 保留静态显示。

后端由实际传入的 `RobotModel` 构建 Pinocchio visual geometry，计算每个网格的世界矩阵（含 URDF visual origin）与末端矩阵。统一内核监看通过 `RuntimeModel` 读取运行时快照的 URDF、TCP frame 和 `data_contract`，验证关节数量与顺序；支持 STL 及 box/sphere/cylinder 几何，不绑定六轴或夹爪。浏览器按 column-major 直接应用，另行应用 URDF mesh scale；不重新实现 FK、关节变换或反馈平滑。显示具有独立 FK 数据缓冲，不修改控制器数据。目标与实际重合时隐藏透明目标，避免覆盖实际材质。内核更换型号时清除旧场景，加载新的模型描述后才显示反馈。

资源随本项目保存，运行无需 Node、npm、CDN 或桌面服务器：

- `vendor/three/`：Three.js **0.180.0** 的必需 ES 模块，来自官方 npm tarball，MIT 许可证原文随附。
- `textures/oak_veneer_01/`：Poly Haven，作者 Jenelle van Heerden，CC0，原来源说明和许可随附。
- `provenance.json`：下载来源、版本与各资产 SHA-256。离线测试校验哈希。

HTTP 提供本模块静态资源、指定模型的 STL，以及模型、状态、传感器和采集复核 API（GET 也支持 HEAD）。监听 `127.0.0.1` 上的独立端口，退出监看进程才关闭。网页不发送运动指令。页面轮询状态，连接断开会明确标记最后画面；每次启动应打开终端新打印的地址。

Python 检查：`./run.sh test`。可选浏览器验证（依赖仅用于测试，不进入运行环境）：

```bash
npm install --prefix .cache/viewer-validation --no-audit --no-fund playwright@1.55.1
# 首次使用且没有 Chromium 时：
.cache/viewer-validation/node_modules/.bin/playwright install chromium
node tests/browser/browser_scene.mjs
```

可通过 `SCENE_TEST_BROWSER=/path/to/chromium` 指定现有浏览器。脚本使用软件 WebGL，从 `/tmp` 分别启动监看、虚拟服务与控制；验证控制来源更换不刷新页面、监看重启不影响控制、实际渲染、运动、过期状态与显示开关。截图及日志写入 `outputs/logs/`，不连接真机。

实时与网页回放共用 `robot-canvas`，浏览器本地模式由 `display-state.js` 管理，切换不改变真机控制权。回放的模型、速度、力矩与 RGB 使用同一播放时间；实时连接与 Quest 状态独立保留。速度反馈沿 telemetry 传输，末端速度由 Scene 的 URDF 雅可比计算。隔离回放验证：`node tests/browser/browser_replay.mjs`（不启动相机或设备服务）。

## 统一内核质量与任务复核

工作台默认入口是 `./run.sh web monitor --kernel --no-camera --no-connections`（18081），对应本目录，区别于 `apps/console` 的另一套 ARX 控制前端。顶部“实测速度 · 质量”显示反馈 `dq`、各关节速度、反馈帧龄、命令位置与反馈的偏差和两个独立时间戳。该偏差未经时间对齐，不是严格同一时刻的跟踪误差。周期均值、标准差、最大值仅来自最近最多 256 个连续序号的内核 tick；监看跳样、遥测发布丢样和 observer 缺口分别显示，缺失或过期数据为 `—`。

相机质量复用现有 `CameraFeed`，比较最新主机接收时间，不宣称曝光同步。源帧号缺口在采集线程累计，每次重连重置；无法可靠计数时显示未提供。页面预览降采样不计作源丢帧，也不等同于录制写入丢帧。

采集面板读取每条 Episode 的 `capture-status.json`，检查 runtime/boot、PID 与进程启动标识。在线计数、队列占用、写盘丢弃和错误来自心跳；超过 2 秒标为过期，进程退出或 PID 复用标为中断。只有 `episode.json` 完成边界能确认完整保存，心跳计数不承诺已经持久写盘。同一内核存在多个采集时显示最近启动的在线采集，并提示数量；相机帧数按实际相机 ID 展示。无夹爪时隐藏夹爪行，上限未声明时保留实测开度，但不绘制比例动画。

“任务 / 技能复核”通过 `GET /api/review` 查看当前内核 `tasks/*/result.json` 的阶段与结果，以及配置采集目录里已完整保存 Episode 的技能区间、证据和置信度。弹窗打开时每秒通过 `?tasks_only=1` 刷新阶段，保持当前 Episode、标注版本和未提交复核备注；不重复读取大体积录制文件。`POST /api/annotations/review` 只调用数据层的版本化复核接口，携带 `expected_revision`，不发送运动命令、不修改原始录制；旧版本或源记录变化会拒绝复核。任务完成不自动变成正确技能标签。

无服务、无硬件的针对性验证：`PYTHONPATH=src <项目 Python> -m pytest -q tests/test_monitor_quality.py tests/test_kernel_monitor.py tests/test_monitor_live_capture.py`；前述 Python 环境需已含 Pinocchio。`node tests/browser/monitor_quality.mjs` 与 `node tests/browser/monitor_joint_contract.mjs` 使用 DOM stub 检查心跳过期、阶段刷新、复核版本、七轴无夹爪行数，不替代真实浏览器视觉验证。
