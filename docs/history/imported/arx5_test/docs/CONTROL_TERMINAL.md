# 控制终端

`./arx5 control` 启动长驻交互控制，动作之间不重复初始化 SDK，也不重复申请
`can0`。常用命令如下：

```text
state
move 6 10 3
pulse 6 30 5 5 0.5
grip 40 2
grip-force 0.15 0 5 0.5
damping
quit
```

夹爪采用混合策略：位置控制张开，限扭矩控制闭合，开度负责约束行程。这里的
N·m 是 SDK 根据电流估算的电机扭矩，不是标定后的指尖夹持力 N。当前设备实测
默认使用反向读数 `gripper_open_readout=-3.4`。若网页提示夹爪读数越界，先断开
机械臂，在网页夹爪面板按引导手动闭合、写零点、再手动张开；测得的全开读数会保存到
`calibrations/gripper/calibration.json`，供网页控制服务后续连接使用。

## 单次工具

不进入长驻终端时可直接运行：

```bash
./arx5 state          # 读取状态，不主动运动
./arx5 test-gripper   # 位置控制 0 → 40 mm → 0
./arx5 test-force     # 位置张开 20 mm，再以 0.15 N·m 限扭矩闭合
```

查看所有参数：

```bash
./arx5 help
./arx5 can-up --help
./arx5 test-force --help
```
