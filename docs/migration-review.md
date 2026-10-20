# 桌面参考实现迁移审查（2026-09-08）

桌面工程仅只读检查；没有移动/修改其文件，没有复制旧 URDF、标定结果、设备序列号或整个控制器。以下能力在项目 `src/` 内独立实现，按 `robot_core`、`robot_control`、`handeye_calibration`、`robot_simulation` 与 `quest_teleop` 分包，以当前 X5-2025 hybrid 为唯一默认模型。见 [源码分层](../src/README.md)。

| 参考位置 | 检查结果 | 本项目处理 |
| --- | --- | --- |
| `arx5_test/src/frontend/src/features/scene/runtime.ts` | Three.js 显示与网页伺服、D405 安装偏移、旧 `JOINT_SPECS` 及网格形变交织，不能整文件套用到当前模型 | 提取深色/金属材质、灯光和木纹场景，替换公共 `Scene`；由当前 URDF/Pinocchio 输出网格世界变换，网页只读。未迁移旧关节参数、相机安装位置或网页运动控制；资产来源与许可随项目保存 |
| `reBotArm_control_py/example/sim/visualizer.py:193` | `plot_trajectory_line` 将 Nx3 数组直接传给要求 3xN 的 Meshcat `PointsGeometry`，与同文件 `draw_path` 不一致 | 当前统一 Three.js 场景通过 JSON 保留 Nx3 点序，测试 3/4/10 个点；不再使用 Meshcat 数组转置接口 |
| `reBotArm_control_py/reBotArm_control_py/kinematics/robot_model.py:74` | 限位数组用关节 ID 下标；固定基座六轴模型首轴错位、末轴可能越界 | 直接采用 Pinocchio 配置数组，测试首末轴边界；不复制硬编码轴变换 |
| `arx5_test/src/embodied/calibration/hand_eye/core/projection.py` | 把 modified Brown 与标准 Brown 统一套 OpenCV 公式，不适用于所有 RealSense 模型 | 支持 none/Brown/inverse Brown，后两种调用 librealsense 原生去投影；不支持的模型显式拒绝 |
| `arx5_test/src/embodied/calibration/hand_eye/core/targets.py:350` | 单平面 IPPE 只选最小重投影误差候选，未检查两个不同姿态几乎等价的情况 | 检查所有角点正深度、重投影误差和候选间歧义；记录原始角点/图像，重复采样可追溯 |
| `arx5_test/src/embodied/calibration/hand_eye/core/solver.py:285` | 鲁棒拟合剔除样本后未重新检查运动多样性；留一法跳过失败折可能让报告过于乐观 | 剔除后重新检查轴秩/跨度；留一法失败不当成通过；保留全部验证样本 |
| `arx5_test/src/embodied/calibration/hand_eye/core/solver.py:324` | 当前本机 OpenCV 5 不提供旧代码调用的 `calibrateRobotWorldHandEye` | 增加相对运动 SVD 求解，不降级/替换现有遥操环境；真实矩阵方向通过合成数据验证 |
| `arx5_test/src/embodied/calibration/hand_eye/collector.py` | 大型采集器依赖旧控制器、双相机假设与旧动作计划 | 拆为相机、目标检测、会话、求解、轨迹和 CLI；适配当前单 D435i；自动计划可先离线预览 |
| `arx5_test` 数据采集/导出模块 | 固定双相机、固定任务及阻尼示教的 `action[t]=state[t+1]` 不适合当前 Quest 指令遥操 | 本轮只复用静止多帧采样、元数据与持久化思路；训练 episode 和 action 定义后续单独接入 |

上表区分了可复现代码缺陷、兼容性问题与迁移时新增的质量检查；不据此宣称桌面工程的所有标定结果都错误。参考根目录为 `/home/r2/Desktop/arx5_test` 与 `/home/r2/Desktop/reBotArm_control_py`，这些路径仅出现在说明材料中。

当前 Stanford 适配器从原遥操脚本提取，现位于 `src/legacy/robot_control/hardware.py`；调用方直接导入该模块。沿用关节限速、0.05 rad 指令超前限制、夹爪行为和零附加前馈力矩。新增共享 CAN 所有权锁和部分初始化失败的清理；没有采用桌面 motorbridge/自动回零硬件层。

验证范围：合成真值/噪声/离群点/退化与独立验证；真实 Pinocchio FK/IK、插值速度/加速度与共享场景初始化；虚拟手臂中断/故障清理；原 presets A/Grip 回归；短时 D435i 视频流。没有执行真机自动标定。
