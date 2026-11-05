# 双臂控制台运行手册

从零把「两臂 + 三相机 + Quest 遥操」拉起来：**命令行只开控制台，其余全部在网页上点**。
链路细节与排障见 [双臂 VR 遥操链路](dual-arm-vr-pipeline.md)。

## 启动（命令行，只有这一步）

```bash
cd <仓库根目录>
export PATH=/home/arx/.local/node/bin:$PATH
export QUEST_BOOTSTRAP_PYTHON=.venv/bin/python
HOST_IP=$(hostname -I | awk '{print $1}')      # 本机实际 IP，每次按当前网卡取
./run.sh web console --host "$HOST_IP"
```

浏览器打开 `http://<本机实际 IP>:8000`（就是上面 `$HOST_IP` 的值）。IP 不要写死，换网/换机器时会变。

## 网页流程

| 顺序 | 位置 | 操作 |
|---|---|---|
| 1 | 总览 → CAN | 「建立 CAN 接口」→ 自动建 can0 / can1（不动电机） |
| 2 | 总览 → 设备 | 「启动相机服务」→ 三路相机 |
| 3 | 头显 | XRoboToolkit：USB 连接填 `127.0.0.1`（走 ADB 转发）；Wi-Fi 连接填**本机实际 IP**，再点 **Connect** |
| 4 | 总览 → VR | 「准备 VR 连接」→ 显示已连接 |
| 5 | 总览 → 系统电源 | 运行配置「真机 · 双臂 Quest」→「启动内核」→ 确认（会使能电机） |
| 6 | 控制页 | 「连接机械臂」→ 状态「只读 · VR 遥操」，3D 两台臂跟随 |
| 7 | 采集页 | 填任务 → 勾相机 → 固定时长/手动 → 开始 → 停止 |

控制页左下切「左臂 / 右臂」看关节卡片；采集页按臂显示 12 条关节曲线和两只夹爪开度，已录数据可逐条删除（两步确认）。

## 注意

- **先连头显，再起内核**（第 3 步早于第 5 步）。反了会 `InitialReadinessTimeout` 锁成 Disabled，按 A 无反应，只能重启内核。
- 按 A 切模式时**别碰 B**，否则被识别成 A+B 直接回 Home。
- 按 A 没反应、内核 `armed` 一直 false：多半是头显在**手部追踪**模式，按键恒为 0，关掉手部追踪再试。
- 内核由控制台托管，**关控制台会连带停内核**（走阻尼）。要看内核日志就留着控制台窗口。
- CAN 左右对应在 `configs/robots/arx5/dual.json`：`left=can1`、`right=can0`；反了就换过来。

## 备用：全命令行启动

```
./run.sh can-up --all                       # can0/can1
./run.sh camera-service                     # 相机服务
./run.sh quest-transport prepare            # 先让头显 Connect
./run.sh kernel profiles start --profile real-dual-quest
./run.sh web console --host "$(hostname -I | awk '{print $1}')"
```
