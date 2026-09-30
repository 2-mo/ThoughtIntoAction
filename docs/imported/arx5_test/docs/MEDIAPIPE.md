# MediaPipe 人体遥操作准备清单

## 当前环境结论

- 控制机是 x86_64、Python 3.12；MediaPipe 0.10.35 有对应的 Linux wheel。
- 手势遥操作采用 M3 Max 的内置摄像头和浏览器 Hand Landmarker，本机直接完成
  双手估计，避免把视频送到 5090 主机再返回所增加的延迟。Mac 摄像头只在
  `Hand` 模式的手势工具中开启，页面隐藏、离开模式或摄像头停止都会释放视频轨。
- 连接在机械臂主机上的 Intel RealSense D455 RGB 和 Z16 深度节点继续负责场景
  观察、人体姿态预览、方块检测和手眼标定。主机使用一个 librealsense pipeline
  同步采集双流，以 WebRTC 传输 RGB 与经准确性优先处理的深度伪彩色画面。
  深度链不填补无效像素，预览以黑色明确标出无效/越界区域，并使用非循环
  Turbo 色图、连续 3 帧有效门控和邻域一致性检查抑制闪烁与飞点。
- 机械臂和视觉建议使用两个独立 Python 进程/虚拟环境，避免视觉卡顿或依赖升级影响 CAN 控制。

## 需要准备

1. Mac 摄像头需要稳定照明，并在 M3 Max 上通过 `./arx5 web-tailnet` 输出的 HTTPS
   地址授权浏览器摄像头权限。D455 应直接连接机械臂主机的 USB 3.x 端口
   （`lsusb -t` 显示 `5000M` 或更高）；D455 画面仍通过 WebRTC 传输。MJPEG
   端点只用于 WebRTC 无法建立时的兼容回退。

   D455 默认采用 `Default` 深度 preset；需要更密集的有效深度时可用
   `ARX5_DEPTH_PRESET=high-density` 启动。`high-accuracy` 也受支持，但通常会
   牺牲较多覆盖率。

2. 浏览器使用 `@mediapipe/tasks-vision` 0.10.35，无需在 Mac 安装 Python
   MediaPipe。Linux 使用独立的视觉/WebRTC 环境：

   ```bash
   UV_PROJECT_ENVIRONMENT=.venv-web uv sync --only-group web
   ```

3. 两个 Google 模型文件：

   - Pose Landmarker：33 个身体关键点，用肩、肘、腕估计人的手臂姿态。
   - Hand Landmarker：21 个手部关键点，用捏合距离或手势控制夹爪和“离合”。

4. 手势控制采用接通瞬间为原点的相对位姿和固定轴映射，不依赖 D455 手眼标定。
   D455 方块定位等绝对坐标功能仍需完成相机到机械臂基座的外参标定。

5. 强烈建议物理急停、独立观察员和空载测试区。单目摄像头存在深度歧义；若要可靠跟随人在三维空间中的平移，优先选 RGB-D 相机或双目相机。MediaPipe 的 world landmarks 不能直接当作已标定的机器人基座坐标。

## 推荐软件边界

```text
摄像头 → MediaPipe 视觉进程 → 有时间戳的目标/手势
                                ↓
                限速、滤波、工作空间、IK、watchdog
                                ↓
                    长驻 ARX 控制进程 → can0
```

视觉进程不直接创建 `Arx5JointController`。它只输出目标、置信度、帧时间和启停状态；长驻控制进程独占 CAN，并在目标过期时冻结新动作。

## 当前手势映射

- 控制手的掌心位移和朝向映射为 TCP 相对 6DoF 目标，不复制人的关节角；接通离合
  时锁定人手与机械臂原点，因此手可以在舒适位置重新接通。
- 非控制手张开并连续稳定 5 帧才接通离合；放下或收拢该手会发送停车。左右手可在
  页面互换，两个手的识别置信度均需达标。
- 控制手拇指与食指捏合连续 3 帧后关闭夹爪，松开捏合后恢复页面设置的张开宽度，
  默认 `40 mm`；闭合扭矩默认 `0.15 N·m`，允许范围 `0.05–0.25 N·m`。
- 浏览器先做 One Euro 滤波、每帧平移/旋转限幅和固定轴变换；服务端再次执行速度、
  加速度、跃度、安全盒和 IK 检查。3D Ghost 只使用服务端接受的关节目标。
- 目标超过 `200 ms` 未更新时，看门狗同时平滑制动机械臂与夹爪；反馈停滞、超速、
  超扭矩或急停会进入故障阻尼。

## 分阶段验证

1. 摄像头画面叠加关键点，只显示数据，不连接机械臂。
2. 记录目标轨迹，离线检查抖动、延迟、丢帧和左右手识别。
3. 运行 IK/工作空间检查，但仍不发送电机命令。
4. 低增益、小范围、低速度空载实机测试。
5. 通过全部停车路径后再启用快速档；扩大工作空间必须修改受版本控制的安全配置并
   重新完成 Ghost 验证。

## 官方资料

- [Pose Landmarker Web](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/web_js)
- [Hand Landmarker Web](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/web_js)
- [Pose Landmarker Python](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/python)
- [Hand Landmarker Python](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python)
- [Gesture Recognizer Python](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer/python)
- [ARX5 SDK](https://github.com/yihuai-gao/arx5-sdk)
