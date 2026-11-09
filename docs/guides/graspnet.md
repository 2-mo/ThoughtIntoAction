# GraspNet 远程推理

本机保留相机、手眼标定和运动内核，GPU 服务器只生成相机光学坐标系下的候选抓取。以下三个脚本均不向机械臂发送运动或夹爪命令。没有可用预测时保留空结果，不切换规则抓取。

## 服务器部署

本次目录为 `wzj@192.168.96.201:/home/wzj/project`。使用 Python 3.8 独立环境 `graspnet-env`、PyTorch `1.13.1+cu117`、CUDA 11.7、NumPy 1.23.5、Open3D 0.18.0；官方 README 的原始环境为 PyTorch 1.6，此处为编译环境调整。CUDA 算子直接编译官方源码，不更改网络、预测解码和碰撞筛选算法。

官方来源与本次固定版本：

- [graspnet-baseline](https://github.com/graspnet/graspnet-baseline)，`280c215129f759ed8649cb4e89fc5dfee55f4f80`。
- [graspnetAPI](https://github.com/graspnet/graspnetAPI)，`eb57dd2092d8dbe05312a29c3d0c22f3226efbfc`。
- [RealSense 预训练权重 checkpoint-rs.tar](https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view)，[官方百度网盘备用](https://pan.baidu.com/s/1Eme60l39tTZrilF0I86R5A)。放在服务器 `/home/wzj/project/weights/checkpoint-rs.tar`。

无须训练数据集。推理只加载官方 API 的 grasp 表示，跳过包顶层的完整数据集评测依赖。权重须来自上述可信来源；旧版 PyTorch checkpoint 使用 pickle。

已准备的独立环境缺少系统 ensurepip 时，通过项目目录内的 virtualenv bootstrap 创建，不修改系统 Python。依赖安装及算子构建命令（在服务器 `/home/wzj/project`）：

```bash
graspnet-env/bin/python -m pip install --timeout 60 \
  'setuptools<70' wheel ninja 'numpy==1.23.5' 'scipy==1.10.1' \
  'Pillow==10.4.0' tqdm 'open3d==0.18.0' \
  'opencv-python-headless==4.10.0.84' 'transforms3d==0.4.2' \
  'trimesh<5' 'Cython==0.29.37'
graspnet-env/bin/python -m pip install 'torch==1.13.1+cu117' \
  --index-url https://download.pytorch.org/whl/cu117
CUDA_VISIBLE_DEVICES=7 CUDA_HOME=/usr/local/cuda-11.7 \
  TORCH_CUDA_ARCH_LIST=8.6 MAX_JOBS=2 \
  graspnet-env/bin/python -m pip install --no-build-isolation --no-deps \
  ./graspnet-baseline/pointnet2 ./graspnet-baseline/knn
graspnet-env/bin/python -m pip install --no-build-isolation 'grasp-nms==1.0.2'
```

GPU 编号按实际空闲情况指定；`8.6` 是本服务器 A6000 的架构。已安装环境无须重复安装。实际软件版本、源码 revision 和权重 SHA256 随每次结果保存。

## 本机采集与调用

工作目录 `/home/guest/下载/ThoughtIntoAction-local`。左臂停稳、暂停左 Quest，使用与当前机器人模型匹配的有效腕部标定。目标 ROI 由当前画面指定，**不能沿用方块移动前的像素位置**；ROI 仅选择目标及周边点云，抓取方向和分数由网络预测。这尚不是任意物体自动分割流程。

```bash
PYTHONPATH=src .venv/bin/python -B scripts/graspnet/capture.py \
  outputs/diagnostics/graspnet-capture-001 --roi 45 340 145 435

.venv/bin/python -B scripts/graspnet/remote.py \
  outputs/diagnostics/graspnet-capture-001 \
  --host wzj@192.168.96.201 --remote-root /home/wzj/project --gpu 7
```

采集目录必须是新目录；生成 RGB、原始 uint16 深度、相机内参、深度比例、机器人前后状态、标定变换及 `input.npz`。拒绝运动中采集、模型不匹配、标定板明显移动、过期深度和 RGB/深度时间差超过 50 ms。深度按 RGB 畸变模型反投影，无效零深度不进入点云。

远程调用沿用 SSH/SCP 身份认证；已有多路复用连接时可加 `--control-path outputs/diagnostics/graspnet-ssh.sock`。不保存密码，不开放 HTTP 服务。服务器每次使用唯一运行目录，保留输入、脚本和输出；本机 `model/remote.log` 保留真实错误，失败不自动重试推理。

## 输出与执行边界

`model/grasps.npz` 保存 `raw_grasps` 和经过官方碰撞筛选、NMS、分数排序的 `grasps`。每行 17 个数：`score, width, height, depth, R(9, row-major), translation(3), object_id`。长度单位米，坐标为本次彩色相机光学系；抓取 x 轴为接近方向，y 轴为夹爪开合，z 轴为夹爪厚度方向。官方几何的指尖位于 `translation + depth * R[:, 0]`，不能把 translation 直接当作本机 TCP。

`grasps.json` 记录模型来源、随机种子 0、输入和权重 SHA256、源码 revision、依赖版本、真实推理时长及候选数量。本机验证输入 SHA256、坐标系、旋转矩阵、数值和尺寸，防止套用旧结果。

运动执行仍需另行完成：候选转入基坐标系、实物夹爪与网络夹爪的几何转换、70 mm 名义开口限制、桌面/夹指净空、IK 和运动路径检查。官方碰撞筛选使用通用夹爪，不能代替 RealMan 全臂碰撞检查。现有 RealMan 任意开口位置命令被控制器拒绝，已实测可用的是官方张开/闭合接口；不通过修改 `grasp=false` 绕过平台能力检查。

2026-11-09 已加载官方 RealSense epoch 18 权重，SHA256 为 `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`，在服务器 A6000 GPU 2 上完成真实推理。9151 点输入得到 530 个原始候选、73 个通用几何碰撞筛选/NMS 后候选，网络耗时约 3.61 秒。收紧 ROI 与板面高度分割另做了两个独立输入试验，所有输入和结果保存在 `outputs/diagnostics/graspnet-live-20261109-193453/`；高度分割属于本次工程预处理，不是模型自带物体分割。

这次位置的候选未通过执行约束：在夹爪开口和接近角度限制下选出的候选，两种对称方向均未通过 MoveL 预览；控制器 DH 离线 IK 检查也未收敛。因此没有发送真机运动或夹爪命令。模型推理成功不表示新位置抓取成功，仍需调整物体位置、重新采集并检查实物 TCP/夹爪几何。
