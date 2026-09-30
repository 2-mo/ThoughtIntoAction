# D455 RGB 方块感知

网页相机服务使用 D455 RGB 检测平放在桌面的黑色、白色或蓝色 `30 x 30 x 30 mm` 方块。
检测轮廓、中心、方向、基座坐标、Yaw、估计尺寸和跟踪状态叠加在 RGB 画面；
同一结果通过 WebRTC DataChannel 更新 Three.js 场景中的等尺寸方块。深度可用时，
服务会读取每个方块候选顶面中心的对齐深度并转换到基座系；只有该点接近预期顶面高度
时才用于多帧修正 `x/y`，同时显示深度与顶面高度残差。深度无效时自动使用
`T_base_camera` 和网页桌面顶面求交，不会中断检测。

## 候选选择

检测到多个候选时，视觉工具栏会显示方块切换按钮；稳定候选在 RGB 和三维桌面中为实线、
不透明显示，只有短时遮挡、忽略或过期状态才降低透明度，选中候选使用加粗边框。
可直接在 RGB 画面或 Three.js 桌面中点击稳定方块选择目标；抓取前选择源方块，
夹住后选择目标方块。抓取预览始终使用当前选择的候选。

## 分割与时序

检测使用固定相机标定后的桌面 ROI，并根据木纹桌面的实时颜色基线分割彩色方块，再用
30 mm 尺寸、近方形轮廓和时序连续性过滤桌边、线缆与工具。原始相机保持 30 FPS，网页
每 125 ms 读取一次最新缓存候选并更新三维场景；抓取动作只在生成预览、执行前和放置后
复核候选，绝不在机械臂运动中按视频帧改写轨迹。检测器结合上一帧位置、颜色和姿态先验
保持候选 ID 与基座位置稳定；没有桌面 ROI 时不会全局检测。
无需采集空桌面背景，开启 D455、启用固定相机标定后即可检测。保留的 DataChannel 诊断接口仍支持发送
`{"type":"cube_background_capture","frames":24}`，查询状态使用
`{"type":"cube_background_status"}`。

## 启动

正常启动前安装网页视觉依赖：

```bash
UV_PROJECT_ENVIRONMENT=.venv-web uv sync --only-group web
./arx5 web-tailnet
```

强制验证 RGB-only 路径时，在启动命令前设置环境变量；此模式不申请 D455 深度流，
网页会在深度区域显示 RGB-only 提示，但 RGB 检测、坐标叠加和三维方块继续运行：

```bash
ARX5_CAMERA_RGB_ONLY=1 ./arx5 web-tailnet
```

正常模式下深度配置或首帧失败会暂时切换到同一 RGB-only 路径，并以指数退避自动
重试深度流；恢复后网页无需重新连接。

## 抓取

视觉工具栏中的“抓取方块”第一次点击只完成连续 IK 安全预检并在 Three.js 中显示真实
TCP 路径，第二次点击才执行：预张开、接近、限扭矩夹紧、抬升并保持方块。执行前必须连接
机械臂、启用当前位置保持，并保持检测结果稳定；机械臂姿态或方块位置在预检后发生变化时，
动作包自动失效。

抓取几何、桌面边界和候选姿态枚举由
[`src/experiments/cube_pick_rotate_place/planner.py`](../src/experiments/cube_pick_rotate_place/planner.py)
单点定义，网页控制服务和终端实验都规划自同一份代码，不会独占启动第二个 D455。
终端实验的完整用法见
[src/experiments/cube_pick_rotate_place/README.md](../src/experiments/cube_pick_rotate_place/README.md)。
