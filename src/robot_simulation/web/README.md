# 共享机械臂场景

`view-arm` / `启动仿真.sh` 单独创建 `robot_simulation.scene.Scene`，订阅通用机械臂状态并服务本机只读网页。机械臂服务与独立数据回放使用 `telemetry.local` 发布数据，不创建或关闭场景；显示无需区分输入方式。详见 [独立监看](../../../docs/monitoring.md)。

显示处理参考桌面备份 `arx5_test/src/frontend/src/features/scene/runtime.ts` 的 `createLinkMaterial`、RoomEnvironment、灯光和木纹工作台。只迁移显示处理及有来源记录的纹理；没有复制其硬编码 `JOINT_SPECS`、旧 STL、D405 安装偏移、夹爪网格形变、伺服操作或网页控制逻辑。夹爪保留当前 URDF 静态几何；工作台仅为示意，不参与碰撞判断。此实现不依赖、也不改动 Desktop 工程。

后端由实际传入的 `RobotModel` 构建 Pinocchio visual geometry，计算每个网格的世界矩阵（含 URDF visual origin）与末端矩阵。浏览器按 column-major 直接应用，另行应用 URDF mesh scale；不重新实现 FK、关节变换或反馈平滑。显示具有独立 FK 数据缓冲，不修改控制器数据。目标与实际重合时隐藏透明目标，避免覆盖实际材质。

资源随本项目保存，运行无需 Node、npm、CDN 或桌面服务器：

- `vendor/three/`：Three.js **0.180.0** 的必需 ES 模块，来自官方 npm tarball，MIT 许可证原文随附。
- `textures/oak_veneer_01/`：Poly Haven，作者 Jenelle van Heerden，CC0，原来源说明和许可随附。
- `provenance.json`：下载来源、版本与各资产 SHA-256。离线测试校验哈希。

HTTP 只暴露本模块静态资源、指定模型的 STL、`GET /api/model` 和 `GET /api/state`（也支持 HEAD）。监听 `127.0.0.1` 上的独立端口，退出监看进程才关闭。网页不发送运动指令。页面轮询状态，连接断开会明确标记最后画面；每次启动应打开终端新打印的地址。

Python 检查：`./run.sh test`。可选浏览器验证（依赖仅用于测试，不进入运行环境）：

```bash
npm install --prefix .cache/viewer-validation --no-audit --no-fund playwright@1.55.1
# 首次使用且没有 Chromium 时：
.cache/viewer-validation/node_modules/.bin/playwright install chromium
node tests/browser_scene.mjs
```

可通过 `SCENE_TEST_BROWSER=/path/to/chromium` 指定现有浏览器。脚本使用软件 WebGL，从 `/tmp` 分别启动监看、虚拟服务与控制；验证控制来源更换不刷新页面、监看重启不影响控制、实际渲染、运动、过期状态与显示开关。截图及日志写入 `outputs/logs/`，不连接真机。
