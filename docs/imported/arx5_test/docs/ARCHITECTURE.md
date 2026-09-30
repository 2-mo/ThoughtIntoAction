# 前后端架构

## 边界

`src/frontend/` 是 Vue 3 + TypeScript 单页应用，只负责界面、Three.js/MediaPipe
可视化和发送指令。它不连接 CAN、不写标定结果、不持有相机，也不执行轨迹。

`src/backend/` 是硬件和任务状态的唯一所有者：

```text
src/backend/arx5_backend/
  app.py                 FastAPI 生命周期
  protocol/              v1 消息模型与功能目录
  transport/             HTTP/WebSocket 传输
  kernel/control.py      控制器所有权和安全执行内核
  features/arm/          机械臂命令
  features/gripper/      夹爪命令
  features/teleoperation/低延迟遥操作目标流
  features/cube/         方块抓取流程
  features/calibration/  负载、夹爪、手眼标定
  features/collection/   GR00T episode 采集
  features/camera/       RealSense/WebRTC 服务
  features/vision/       只读关键点桥
```

相机与控制服务继续使用独立进程和 Python 环境，避免 RealSense/视频依赖影响 CAN
控制循环。控制 FastAPI 同时在 `8000` 提供 `src/frontend/dist`，保证网页和控制
WebSocket 同源；`src/scripts/web*.sh` 只负责编排这些后端进程。纹理和运行库位于 `assets/`；版本化浏览器模型位于
`weights/browser/`，CLIP、YOLO、GR00T 预训练权重放在 `~/work/pre_weights/`。
持久标定和采集数据分别位于 `calibrations/` 与 `~/work/data/arx5/`；诊断产物位于 `outputs/`；`runtime/` 仅存放可丢弃
的服务状态和缓存。

## API v1

控制连接：`ws(s)://host:8000/api/v1/control`（与网页同源）

普通命令：

```json
{
  "type": "command",
  "request_id": "request-1",
  "feature": "arm",
  "command": "state",
  "payload": {}
}
```

高频伺服流：

```json
{
  "type": "stream",
  "feature": "arm",
  "stream": "cartesian",
  "command": "update",
  "sequence": 12,
  "payload": {}
}
```

服务端事件使用点分命名，如 `session.ready`、`state.snapshot`、
`command.started`、`command.succeeded`、`command.failed`、`motion.progress`。
所有功能和命令可通过 `GET /api/v1/system/capabilities` 发现。

相机和视觉服务使用独立路径：

- `ws(s)://host:8767/api/v1/cameras/d455/webrtc`
- `ws(s)://host:8765/api/v1/vision/landmarks`
- `ws://127.0.0.1:8768/api/v1/cameras/collection`（仅主机采集后端）

## 安全不变量

- 控制内核始终只允许一个网页所有者。
- 预览与执行仍为两个独立命令；执行会校验预览时效与起始姿态。
- 忙碌、实时伺服、标定和数据采集状态继续互斥。
- 浏览器断开时，普通操作进入后台阻尼待机；活动采集由后台继续保持并受时长上限约束。
- 服务退出和急停继续走后端阻尼/释放流程。
- 前端参数不能覆盖后端安全包络。
