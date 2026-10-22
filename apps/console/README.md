# ARX5 初学者网页控制台

前端是独立的 `Vue 3 + TypeScript + Vite` 工程，只负责显示状态、可视化预览和
向版本化 API 发送指令。机械臂、安全检查、任务状态机、相机所有权和数据写入均在
`src/embodied/applications/console/`，前端不会导入 Python 控制代码。

功能按目录拆分：

```text
src/api/                    API 协议与服务地址
src/features/control/       机械臂和夹爪控制界面
src/features/scene/         Three.js 场景与目标姿态编辑
src/features/camera/        D455/D405 WebRTC 显示
src/features/cube/          方块姿态显示算法
src/features/teleoperation/ 手势遥操作界面
src/features/collection/    GR00T 采集页面
../../assets/arx_motion/   MediaPipe 运行库、WASM、图标和纹理
../../assets/arx_motion/weights/browser/  MediaPipe 模型与 X5/D405 网格
```

页面包含三块功能：官方 X5 模型驱动的 3D XYZ/单关节控制、夹爪/高级关节调试，
以及连接在机械臂主机上的 Intel RealSense D455 RGB-D 画面与 MediaPipe 预览。
主机使用一个 `librealsense` pipeline 同步采集 RGB 与 Depth，经 WebRTC 双视频轨
送到浏览器，MediaPipe 仍只处理 RGB；关键点目前不会转换成机械臂动作。
深度处理顺序为 threshold、disparity、spatial、temporal，再转回 Z16；不做
空洞填补。预览使用固定 `0.2–4.0 m` 的非循环 Turbo 色图，无效或越界像素保持
黑色，并要求像素连续 3 帧有效且有足够一致的邻域支持后才显示，以减少飞点和
有效性闪烁。旧的 `/camera/rgb.mjpeg` 与 `/camera/depth.mjpeg` 端点仅保留用于
手动诊断；页面不会在 WebRTC 失败时立即争用同一台相机。
相机采集由独立守护进程管理；librealsense 初始化失败时只替换相机子进程，
强制释放 UVC 句柄，网页会短暂等待后自动重连，不会中断机械臂控制服务。
深度流单独失败时会暂时保留 RGB，并以 `2–30 s` 退避间隔自动重试；网页保留
深度画面位置并显示恢复状态，成功后沿用现有 WebRTC 轨自动恢复预览。
姿态识别使用 Pose Landmarker Heavy 的 VIDEO 模式，以 15 FPS 与相机同步，并对
关键点应用 One Euro 时域滤波和 250 ms 短暂丢帧保持。

## 场景参考模型

3D 场景使用可编辑的独立桌面板作为空间标定参考，默认尺寸为 `2000 x 1000 mm`。
X5 底座后侧外缘与桌面长边齐平，夹爪朝 `+X` 进入桌面，故机械臂朝向与桌面短边
平行；可调整宽、深、厚、XYZ 与 Yaw，参数自动保存在当前浏览器。桌面使用本地缓存的 Poly Haven `Oak Veneer 01` 2K
颜色、法线和粗糙度 PBR 纹理（CC0，来源和许可记录在
`assets/arx_motion/textures/oak_veneer_01/SOURCE.txt`）。外置 D455 使用类似 Blender 相机对象的视锥、
RGB 光轴和六自由度外参编辑器，不再用物理外壳网格占据场景。初始外参依据一帧
640x480 RGB 画面和约 90 x 65 度 RGB 视场作粗略估计，仅用于开始正式手眼标定
前的场景对齐；最终外参仍应由标定板或已知三维对应点求解。

夹爪上的 D405 使用 Intel `realsense2_description 4.55.1` 提供的官方 STL，按
官方 URDF 的毫米比例、`camera_link` 和光学坐标偏移渲染。双相机外参启用后，
网页使用实测 `T_eef_camera` 把模型挂到 `eef_link`；没有已启用的眼在手外参时
不显示 D405，避免用猜测安装位姿误导轨迹观察。官方网格读取失败时只回退为
`42 x 42 x 23 mm` 尺寸外壳。

相机应接入 USB 3.x 端口。运行 `lsusb -t` 时，D455 与眼在手 D405 所在分支应显示
`5000M` 或更高，而不是 `480M`。当前机械臂按 librealsense 序列号固定绑定：
D455 为 `239222300444`，眼在手 D405 为 `230322274781`。需要更换硬件时可覆盖：

```bash
ARX5_REALSENSE_SERIAL=239222300444 \
ARX5_EYE_IN_HAND_SERIAL=230322274781 \
./run.sh components run deployment.tailnet
```

深度传感器默认使用 D455 的 `Default` preset。可按场景切换；`High Accuracy`
通常会明显降低有效覆盖率，因此不是默认值：

```bash
ARX5_DEPTH_PRESET=high-density ./run.sh components run deployment.tailnet
# 可选值：default、high-density、high-accuracy
```

眼在手 D405 默认以 `1280x720@30 FPS` 单源采集，WebRTC 媒体轨缩放为
`640x360@30 FPS` 预览；自动手眼标定复用相同的 `1280x720@30 FPS` 档位，并从该源
缩放引导画面，不会另开视频流。
D455 保持 `640x480@30 FPS`。

WebRTC 首帧冷启动最多等待 `25 s`。若浏览器未收到 D455 RGB 首帧，页面只会重建
WebRTC 信令/媒体 Peer（最多 3 次），不会释放或重启 D455/D405；最终错误会给出
ICE、候选对、RTP 字节/帧和解码帧诊断，主机终端同步打印采集就绪、SDP、ICE 和编码器
首帧时序。

V4L2 设备变量 `ARX5_RGB_CAMERA_DEVICE` 和 `ARX5_DEPTH_CAMERA_DEVICE` 只影响
MJPEG 回退端点。

## 机械臂主机

首次使用时安装后端和视觉依赖，并构建前端：

```bash
cd /home/r2/work/projects/embodied/QuestArmTeleop
./run.sh components setup analysis
./run.sh components setup web
./run.sh components setup frontend
./run.sh components run ui.build
```

之后恢复 CAN 并启动 Tailscale 控制台：

```bash
./run.sh components run diagnostic.can
./run.sh components run deployment.tailnet
```

命令会读取 `tailscale0` 的 IPv4 地址、只监听该地址、生成本地 CA/服务器
证书，并显示包含配对令牌的直接访问网址。普通 Wi-Fi/LAN 地址不会监听控制
端口。证书和令牌位于 `runtime/`，私钥权限为 `600`。

## Mac 首次信任证书

Mac 和机械臂都需要登录当前 tailnet。保持 `./run.sh components run deployment.tailnet` 运行，在
Mac 终端执行（IP 使用启动输出中的地址）：

```bash
curl -k https://100.112.78.10:8000/arx5-local-ca.crt \
  -o ~/Downloads/arx5-local-ca.crt
sudo security add-trusted-cert -d -r trustRoot \
  -k /Library/Keychains/System.keychain ~/Downloads/arx5-local-ca.crt
```

这一步只需执行一次。`curl -k` 仅用于下载公开的 CA 证书，服务器不会提供
CA 私钥。随后用 Chrome 打开启动命令显示的完整地址，例如：

```text
https://100.112.78.10:8000/?token=启动时显示的令牌
```

页面通过 HTTPS 打开；控制 API 使用同源 `/api/v1/control`，不再要求手机额外连接
`8766`。D455 RGB-D 仍由 `wss://主机:8767/api/v1/cameras/d455/webrtc`
完成需要画面时的 WebRTC 信令，媒体使用 WebRTC 加密传输。GR00T 采集页不连接
相机服务；录制命令只通过主机内部 `127.0.0.1:8768` 转发。Three.js、MediaPipe 运行库、
WASM 以及 Pose/Hand 任务模型均由前端构建或本地资产提供；固定版本 X5 STL 获取
超时会自动使用内置降级几何，避免控制页面卡在加载状态。以后
重新运行 `./run.sh components run deployment.tailnet` 时，
配对令牌保持不变，可以收藏完整网址。Tailscale IP 通常也会保持稳定。
如果此前已经为 `web-lan` 安装过同一个 ARX5 本地 CA，无需重复安装。

只在同一局域网内使用时，仍可启动 LAN 模式：

```bash
./run.sh components run deployment.lan
```

## localhost / SSH 备用模式

不能在 Mac 安装本地 CA 时，机械臂主机运行 `./run.sh components run ui.console`，Mac 运行：

```bash
ssh -N \
  -L 8000:127.0.0.1:8000 \
  -L 8765:127.0.0.1:8765 \
  -L 8767:127.0.0.1:8767 \
  r2@192.168.5.8
```

然后打开 `http://localhost:8000`。只想看摄像头和关键点时可运行
`./run.sh components run input.hand_service`，它不会启动机械臂控制服务。

## 第一次手动操作

1. 确认机械臂周围没有障碍；网页服务会在后台连接机械臂，连接不会触发回零或运动。
2. 等 3D 模型切换到六关节真实反馈姿态，先观察模型弯曲方向是否和真机一致。
3. 默认的“XYZ 末端”模式显示基座坐标系中的当前 TCP 和绝对目标 XYZ（mm）。
   TCP 目标处常驻类似 MoveIt/RViz Interactive Marker 的 6DoF 操作器。平移模式
   只显示红、绿、蓝三个世界坐标正向箭头，不显示负向箭头、平面块或中心自由拖动；
   拖动对应位置时以 `0.5 mm` 吸附。按 `R` 切换为绕
   夹爪局部 XYZ 的三个旋转环，直接修改 quaternion，按 `G` 返回平移。`Q/E` 仍
   可按 `5°` 微调局部 Roll，按住 Shift 时为 `1°`。`Esc` 恢复
   当前编辑阶段，`Enter` 只确认编辑，不执行真机。拖动期间每 `50 ms` 至多请求
   一次只读 IK 并连续刷新半透明机械臂；松开后再自动请求包含末端轨迹的正式预览。
   白色小点是实际 TCP，半透明夹爪是目标 Ghost。编辑完成后会请求 IK 预览；
   IK 成功才显示可执行状态，仍须点击“执行预览”才能移动机械臂。
   夹爪姿态由同一 TCP quaternion 和“旋转”模式控制；夹爪开闭是独立的一维
   开度。位置控制用于预张开，接触物体时使用限扭矩夹取，不使用位置闭合硬夹。
   “执行轨迹”默认选择“平滑曲线”，先按 100% 使用关节空间 Ruckig 生成一条规范曲线，
   再按速度档位做时间缩放和固定周期 Hermite 插值；若末端必须严格沿直线避障，切换为“末端直线”。
   机械臂速度使用档位滑条，页面每次加载都回到 `10%`，档位为
   `1%、2%、3%、4%、5%、10%、25%、50%、75%、100%`。`100%` 采用 SDK 公布的
   逐关节、末端和夹爪速度上限，其余档位从这一基准按百分比缩放，并增加慢速插值点。
   它控制关节、XYZ 和回参考。网页不再附加固定 XYZ 距离或单次关节
   角度上限；实际可达范围仍由当前姿态、软件限位、IK 和轨迹检查决定。
   目标姿态默认保持连接时的实际 TCP 姿态，面板同时显示进近向量和四元数。
   目标待验证时为琥珀色，IK 成功为绿色，失败为红色。目标
   拖动默认只修改前端 TargetPose；勾选“实时伺服”后才会以当前 TCP 目标连续发送
   伺服更新，松开鼠标后平滑停车。实时 Ghost 使用的 `preview_xyz_live` 只求解显示
   姿态，不会调用真实运动接口。
   `M` 只在 XYZ/单关节之间切换；夹爪是右侧控制器底部的紧凑展开工具。
   `D` 张开夹爪，`F` 闭合夹爪；第一次按 `N` 预览回参考 Ghost 和 TCP
   轨迹，第二次按 `N` 才执行并安全停止。真实回参考期间，锁定的 TCP 操作器
   会随反馈末端移动，结束或中止后同步到实际姿态。
4. 切换到“单关节”，点击模型连杆或右侧 `J1..J6` 选择关节。拖动绿色 3D 圆环、右侧圆形旋钮，
   或使用 `±1°/±5°`，只会改变琥珀色半透明预览；确认后点击“执行预览”。
   第一次执行会自动缓慢启用关节位置控制。
   `Space` 为全局急停，会取消当前拖动/预览并让机械臂进入阻尼；输入框编辑时不会抢占
   空格。快捷键提示集中在 3D 视图右上角。
   控制台会固定在当前浏览器可视区内，不产生整页滚动；窗口变窄或变矮时，
   摄像头会缩为浮动预览，次要说明和圆形旋钮会自动收起，3D 拖动与执行按钮始终保留。
5. 精确排查时展开“高级调试”，里面保留六个关节滑块和单关节回参考。
   3D 控制和滑块都覆盖完整软件关节范围，并始终避开上下限各 1° 的安全边界。
6. 夹爪控制位于末端与关节控制下方，可使用 `20/40/88 mm` 快捷开度；按住 `D/F` 可连续张开或闭合，松开即停。
   抓取物体使用限扭矩闭合，并用最低开度限制行程。N·m 是电机估算扭矩，不是标定后的
   指尖夹持力。
7. 正常结束点击“回参考并保持”，确认回程完成后再点“断开”或在终端
   `Ctrl+C`。若不需要回到参考姿态，可点“正常进入阻尼”再断开。动作之间
   不需要反复进入阻尼。

连接机械臂时，网页会把当时的六个关节角度和夹爪开度记录为“参考姿态”。
每个关节的“回参考”只返回这个软件参考，不会寻找机械零点，也不会写入电机校准。
回参考会恢复机械臂与夹爪的参考状态；夹爪“关闭”动作仍会将其送到 0 mm。
可用“记录当前为参考”随时更新参考姿态。
单关节回参考会在软件关节限位内直接生成到参考角度的完整轨迹，速度由机械臂
   速度档位滑条决定（`1%、2%、3%、4%、5%、10%、25%、50%、75%、100%`），
   不再按固定角度分段。
断开并重连后，会以新的连接姿态重新建立参考。

“回参考并保持”第一次点击只生成并显示完整的六关节余弦轨迹、半透明参考
姿态和 TCP 路径；再次点击确认后才一次协同移回软件参考姿态，并继续主动位置保持；
不再使用最多 16 次的 30° 分段循环。
回参考完成时会恢复记录的夹爪参考开度。终点最大残差会显示在
控制消息中；超过 `3°` 会报告具体关节和误差并进入故障阻尼。这个按钮用于
正常收尾，不等同于机械零点或校准。

`Ctrl+C`、红色“紧急阻尼”、浏览器意外断开或服务停止都会释放位置控制。
若当时正在运动，会立即进入故障阻尼；若处于空闲关节位置控制，则缓慢卸载增益
后进入阻尼。它们不会自动规划回参考运动，因为碰撞、夹住物体或姿态异常时，
继续运动反而可能扩大危险。网页 3D 预览和关节滑条只受软件关节范围约束。
高级点动每次上限为 `10°`，夹爪上限为
`88 mm / 0.25 N·m`，并且不提供回零或校准按钮。软件保护不能替代实体急停。

摄像头预览点击“开启”后，将姿态开关拨到左手或右手即可开始识别；页面会自动连接
数据桥。看到骨架和手部关键点即表示本地识别正常，视觉性能项的悬停说明会附带数据桥
连接状态。

XYZ 实现复用 `arx5-interface 0.1.3` 自带的 `Arx5Solver`（KDL LMA）。它用当前
关节角作为 IK 种子，并额外验证 IK 状态、FK 残差和软件限位。
每次执行都会检查
预览是否过期。“平滑曲线”先求解目标 IK，再离线生成同步的关节空间 Ruckig
轨迹，并验证全部采样点的限位、速度、加速度和 jerk；速度档位不会重新求解另一条路径。
“末端直线”在内存生成笛卡尔路径，以上
一个解为下一个 IK 种子，验证全部 waypoint 的限位、FK 残差和离散速度。
两种模式都在整条路径通过后才调用一次 `set_joint_traj`，执行期间继续反馈监控。

方块“一键叠放”会以当前选中方块为源、最近的稳定方块为目标。确认后先记录任务
起始参考位，再一次生成抓取、搬运、放置、撤离和回程的完整预览；执行阶段复用冻结的
轨迹，连续 waypoint 使用圆角过渡，不再逐阶段重新规划和停顿。夹紧、释放及最终回到
任务参考位仍保留反馈停稳检查，结束后继续主动保持。

参考实现为
[ARX5 SDK keyboard teleop](https://github.com/real-stanford/arx5-sdk/blob/8ccbf0aa8d4288f2721c41a735e5ac4c389bb13b/python/examples/keyboard_teleop.py)
和 [Cartesian waypoint scheduling](https://github.com/real-stanford/arx5-sdk/blob/8ccbf0aa8d4288f2721c41a735e5ac4c389bb13b/python/examples/cartesian_waypoint_scheduling.py)。

机械臂执行关节、回参考和夹爪运动时，后端每 `100 ms` 推送一次真实反馈；
浏览器在反馈点之间逐帧插值白色实机模型。它显示的是平滑后的真实反馈，不是
仅按时间播放的假动画；琥珀色模型表示普通预览目标，青蓝色模型表示实时伺服
已经接受的动态目标，红橙色模型表示鼠标已越过可达边界并保持在最后可达点。
