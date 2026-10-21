"""Platform commands by function: `./run.sh <group> <command> [args]`.

Each entry names an existing launcher command, script or component; the
implementations stay where they are. Targets:
  ("run", [command, *args])   launcher command (scripts/launch.py)
  ("bash", path, [*args])      repository script
  ("component", name)          configs/components/commands.json entry
  ("workspace", [*args])       tmux workbench (scripts/workspace/tmux_workspace.py)
"""

GROUPS = {
    "sim": ("MuJoCo 仿真工作台（Quest 遥操 + arx 内核 + 采集 + 网页）", {
        "start": ("启动并进入工作台；--detach 后台启动", ("workspace", ["start", "--mode", "simulation"])),
        "status": ("工作台各窗口状态", ("workspace", ["status"])),
        "stop": ("有序退出遥操与内核，保留网页和相机", ("workspace", ["stop"])),
        "shutdown": ("关闭工作台启动的全部服务", ("workspace", ["shutdown"])),
    }),
    "real": ("真机工作台（当前日常链路：机械臂服务 + 遥操 + 网页）", {
        "start": ("启动并进入工作台；首次需要在窗口 1 输入 sudo 密码", ("workspace", ["start", "--mode", "real"])),
        "status": ("工作台各窗口状态", ("workspace", ["status"])),
        "stop": ("有序退出遥操与机械臂服务", ("workspace", ["stop"])),
        "shutdown": ("关闭工作台启动的全部服务", ("workspace", ["shutdown"])),
    }),
    "kernel": ("arx-televr 控制内核（control/，唯一 SDK 所有者）", {
        "start": ("启动内核：--backend sim|real，--input quest|demo，--managed 接受多来源", ("run", ["control", "start"])),
        "viewer": ("附加只读 MuJoCo 窗口", ("run", ["control", "viewer"])),
        "observe": ("读取运行中内核的遥测：--run <RUN>", ("run", ["control", "observe"])),
        "verify": ("核对上游文件与本地补丁台账", ("run", ["control", "verify"])),
        "check": ("上游离线检查；--simulation 加跑 MuJoCo 链路", ("run", ["control", "check"])),
        "doctor": ("检查依赖与模型，不连接设备", ("run", ["control", "doctor"])),
        "setup": ("创建 Python 3.10 内核环境", ("run", ["control", "setup"])),
        "build-sdk": ("编译带夹爪修正的 SDK", ("run", ["control", "build-sdk"])),
        "profiles": ("工作台组合 profile：plugins | validate | start | status | stop", ("run", ["workbench"])),
    }),
    "arm": ("机械臂（真机）", {
        "service": ("旧机械臂服务（准备 CAN、阻尼待机、交互控制台）", ("run", ["arm-service", "--prepare-can", "--interactive"])),
        "check": ("只读服务反馈；--virtual 读取虚拟服务", ("run", ["check-arm"])),
        "home": ("回零", ("bash", "scripts/arm/home.sh", [])),
        "can-up": ("配置 CAN 接口", ("run", ["can-up"])),
        "can-status": ("CAN 接口状态", ("run", ["can-status"])),
        "setup-device": ("安装 USB-CAN udev 规则", ("run", ["setup-arm-device"])),
        "joint-test": ("关节检查，默认只读；--actuate 小幅动作", ("run", ["test-joint"])),
        "state": ("直连 SDK 读取状态（需独占 CAN）", ("component", "diagnostic.arm_state")),
        "gripper-position": ("夹爪位置测试", ("component", "gripper.position")),
        "gripper-torque": ("夹爪力矩测试", ("component", "gripper.torque")),
    }),
    "quest": ("Quest 头显与遥操", {
        "teleop": ("唤醒 Quest 并启动遥操（启动遥操.sh）", ("bash", "启动遥操.sh", [])),
        "check": ("XR 数据检查", ("run", ["check-xr"])),
        "service": ("启动 XRoboToolkit PC Service", ("run", ["start-xr-service"])),
        "usb": ("配置 USB 反向端口：prepare | status", ("run", ["quest-transport"])),
        "discovery": ("头显发现广播：on | off | status", ("run", ["quest-discovery"])),
        "battery": ("头显电量", ("run", ["quest-battery"])),
        "wake": ("唤醒与模拟佩戴", ("run", ["quest-wake"])),
        "visualize": ("浏览器查看手柄位姿", ("run", ["visualize-xr"])),
        "calibrate": ("手柄与工具方向标定", ("run", ["quest-calibrate"])),
    }),
    "phone": ("iPhone 遥操", {
        "run": ("接入运行中的内核：--run <RUN>", ("run", ["phone"])),
        "tools": ("手机协议工具集", ("component", "phone.tools")),
        "monitor": ("查看手机位姿流", ("component", "diagnostic.pose_monitor")),
        "simulate": ("模拟手机位姿流", ("component", "diagnostic.pose_simulator")),
        "loopback": ("本机 UDP 回环", ("component", "diagnostic.udp_loopback")),
        "network": ("网络连通与丢包", ("component", "diagnostic.network_test")),
        "latency": ("延迟报告", ("component", "diagnostic.latency_report")),
    }),
    "motion": ("动作（内核客户端）", {
        "run": ("--run <RUN> status | movej | movel | jog | gripper | teach | return | home | stop", ("run", ["motion"])),
        "console": ("浏览器动作编辑台", ("component", "motion.console")),
    }),
    "collect": ("数据采集", {
        "run": ("从运行中的内核采集：--run <RUN> --task <任务>", ("run", ["collect"])),
        "session": ("旧链路采集会话：status | demo | ...", ("run", ["session"])),
        "record": ("旧链路录制开始 / 结束", ("run", ["record"])),
        "groot": ("GR00T 双相机采集", ("component", "collection.groot")),
    }),
    "data": ("数据查看、回放与导出", {
        "review": ("查看、检查、回放与标注 episode", ("run", ["dataset"])),
        "runtime": ("内核 episode（schema 2）检查", ("run", ["runtime-data"])),
        "export": ("导出 LeRobot v2.1（首次自动安装导出环境）", ("bash", "scripts/datasets/convert.sh", [])),
        "export-setup": ("安装导出环境", ("bash", "scripts/datasets/setup_export.sh", [])),
    }),
    "policy": ("策略推理（真机）", {
        "pi05": ("π0.5 零样本", ("bash", "scripts/inference/start_pi05_zeroshot.sh", [])),
        "pi05-sft": ("π0.5 微调模型（PyTorch）", ("bash", "src/embodied/policy/pi05_sft/run.sh", [])),
        "pi05-sft-jax": ("π0.5 微调模型（JAX）", ("bash", "src/embodied/policy/pi05_sft/run_jax.sh", [])),
        "groot": ("GR00T 推理", ("component", "policy.groot")),
        "pregrasp": ("前往预抓取位", ("bash", "src/embodied/policy/pi05_sft/pregrasp.sh", [])),
        "stop": ("停止并恢复", ("bash", "scripts/inference/stop_pi05.sh", [])),
        "setup-pi05": ("安装零样本推理环境", ("bash", "src/embodied/policy/pi05_zeroshot/setup.sh", [])),
        "setup-sft": ("安装微调推理环境（PyTorch）", ("bash", "src/embodied/policy/pi05_sft/setup.sh", [])),
        "setup-sft-jax": ("安装微调推理环境（JAX）", ("bash", "src/embodied/policy/pi05_sft/setup_jax.sh", [])),
    }),
    "calib": ("标定", {
        "wrist": ("腕部 D435i 手眼：marker | plan | manual | auto | solve；--runtime <RUN> 经内核", ("run", ["handeye"])),
        "handeye": ("D455 / D405 双相机手眼", ("component", "calibration.handeye")),
        "gripper": ("夹爪零点", ("component", "calibration.gripper")),
        "payload": ("负载辨识", ("component", "calibration.payload")),
    }),
    "camera": ("相机", {
        "service": ("共享相机服务", ("run", ["camera-service"])),
        "check": ("相机检查", ("run", ["check-camera"])),
        "rgbd": ("D405 / D455 RGB-D 与 WebRTC", ("component", "camera.rgbd")),
        "d405": ("D405 配置基准", ("component", "diagnostic.d405")),
        "permissions": ("修复相机设备权限", ("bash", "scripts/setup/fix-camera-permissions.sh", [])),
    }),
    "web": ("网页", {
        "monitor": ("18081 监看页（含共享相机）", ("run", ["view-arm", "--camera"])),
        "control": ("监看页启停：start | stop | status | camera-on | camera-off", ("run", ["viewer-control"])),
        "console": ("apps/console 控制台", ("component", "ui.console")),
        "backend": ("控制台后端", ("component", "ui.backend")),
        "build": ("构建控制台前端", ("component", "ui.build")),
        "preview": ("控制台前端预览", ("component", "ui.preview")),
        "lan": ("局域网 HTTPS 发布", ("component", "deployment.lan")),
        "tailnet": ("Tailscale 发布", ("component", "deployment.tailnet")),
        "site": ("本地预览宣传网站（site/）", ("bash", "scripts/web/site.sh", [])),
    }),
    "task": ("任务程序", {
        "cube": ("方块抓取 / 旋转 / 堆叠", ("component", "task.cube")),
    }),
    "check": ("离线检查", {
        "all": ("全部检查组：--only <组>", ("run", ["check-all"])),
        "test": ("主环境 pytest", ("run", ["test"])),
        "deps": ("核对锁定的源码依赖", ("run", ["check-dependencies"])),
        "doctor": ("旧链路设备与环境诊断", ("run", ["doctor"])),
        "ports": ("端口与 socket：登记、占用进程、冲突", ("run", ["ports"])),
    }),
    "setup": ("安装与设备准备", {
        "deps": ("获取锁定的源码依赖", ("run", ["fetch-dependencies"])),
        "env": ("创建组件环境：analysis | web | phone | rebot | frontend | groot", ("run", ["components", "setup"])),
        "connections": ("准备 Quest USB 与 CAN", ("run", ["prepare-connections"])),
        "xr-sdk": ("重建 Quest 绑定", ("run", ["rebuild-xr-sdk"])),
    }),
}
