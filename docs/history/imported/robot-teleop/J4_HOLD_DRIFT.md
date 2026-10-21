# ARX5 J4 HOLD 漂移问题记录

日期：2026-08-09

## 现象

机械臂通过关节轨迹移动到 J1-J6 全零附近后，停止手机输入并启动 Ubuntu
后台。几秒后 J4 又向上翘到约 `+0.330548 rad`。

当时网页反馈为：

```text
runtime: disconnected
command: hold / pipeline hold
J4 position: +0.330548 rad
J4 torque:   +1.164835 Nm
servo_error: null
receiver_error: null
accepted_inputs: 0
```

## 原因

这不是一次新的零位标定，也不是软件限位触发。J4 的力矩读数说明机械臂确实承受
重力负载，但直接原因是 HOLD 目标处理错误：

1. 没有手机输入时，pipeline 产生 `HOLD`。
2. ARX5 backend 每个周期都把当前测量关节位置作为新的保持目标。
3. J4 在重力下发生微小漂移后，下一周期的目标也跟着漂移。
4. 这个过程持续累积，最后 J4 到达约 `+0.33 rad` 的重力平衡姿态。

因此，单次发送全零目标后立即进入阻尼，不能保证机械臂长期保持全零。

## 修复

提交：`7926933 fix: latch ARX5 hold position`

`Arx5Backend` 现在在第一次进入 HOLD 时锁存关节目标，后续 HOLD 周期持续发送同一
目标。收到新的关节运动、笛卡尔运动、HOME 或 STOP 命令时清除锁存，下一次 HOLD
再从新的当前姿态建立目标。

网页同时增加了：

- TCP 四元数的完整分量显示；
- Recent events / faults 的 `COPY` 按钮；
- 夹爪实时归一化位置和目标数值。

## 修复后状态

后台重新接管后，J4 保持在约 `-0.011 rad`，没有再次漂移到 `+0.33 rad`；后台运行
频率约 `200 Hz`，无 servo 或 receiver error。

最近一次反馈：

```text
J1  +0.00324 rad
J2  +0.00210 rad
J3  +0.00553 rad
J4  -0.01087 rad
J5  +0.00706 rad
J6  -0.00362 rad
```

## 后续注意

这份修复解决的是“无手机输入时保持目标漂移”，不是修改编码器 offset，也不是
证明厂家物理零位已经标定。若要再次执行全零动作，应使用带未来时间戳的
`set_joint_traj()`，以当前反馈为起点生成到全零的平滑轨迹；不要重复发送没有正确
时间戳的单点 `set_joint_cmd()`。

