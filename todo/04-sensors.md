# 传感器

每台相机由一个服务持有，其他功能作为消费者。

- [ ] **S-01** 合并相机服务 `P1`
  - 现状：
    - `src/robot_core/camera_service.py`（239 行）：ZMQ 传输，在 `:112` 用 `DeviceLease("camera:" + serial)` 加锁。
    - `src/arx5_backend/features/camera/service.py`（2592 行）：WebRTC，没有设备锁。两者可能同时打开同一台相机（未实测）。
  - 做法：保留带锁的服务作为唯一相机所有者；WebRTC 预览作为它的消费者。

- [ ] **S-02** 直接打开 RealSense 的位置 `P1`
  - 现状：
    - `src/hand_eye_calibration/core/camera.py`
    - `src/experiments/cube_pick_rotate_place/perception.py`
    - `src/gr_zeroshot/cameras.py`
    - `src/arx_tools/d405_profile_benchmark.py`
  - 做法：改为通过相机服务取帧；确需独占设备的诊断脚本必须先拿同一把锁。
  - 依赖：S-01。

- [ ] **S-03** 相机帧契约 `P2`
  - 做法：按 `docs/component-boundaries.md` 已设计的 CameraFrame 实现：camera / stream ID、帧序号、分辨率、编码、颜色顺序、内参与畸变、设备时间与主机时间。区分"查看最新画面"和"需要完整帧序列"两类订阅。

- [ ] **S-04** 相机启停与遥操会话解耦 `P2`
  - 现状：`camera_control.check_legacy_session()` 会查询遥操会话（`docs/component-boundaries.md` 已记录）。
  - 做法：改为通用的使用租约；采集、标定、策略各自声明占用。

- [ ] **S-05** 相机配置 `P2`
  - 做法：序列号、分辨率和帧率放平台配置（R-05）；`configs/local.toml` 已有 `[cameras.wrist_rgb]`、`[cameras.exterior_rgb]`，以此为准。
