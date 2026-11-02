# RM65-BI `.18` 接入

独立配置：`configs/robots/realman/rm65-18.json`。默认不带 `--config` 的命令仍选择 `.19`。

2026-11-02 只读核对 `192.168.1.18:8080`：型号RM65-BI，控制/规划V1.7.1，控制器算法1.5.5-c0d52d18；工具名 `work`，偏移 `[0,0,140000,0,0,0]`，工作坐标系 `Base`、位姿为零，错误列表 `[0]`。工具负载读数为0，不能据此认为夹爪没有重量；实际工具负载与安装朝向需现场核对，本次未修改。夹爪首次查询为全零、离线，不能视为已验证可控。初次连接8080曾被拒绝，稍后重试恢复；仅Ping可达不足以判断控制接口就绪。

使用实测DH与关节范围执行本地正逆解核对：模型位置误差约7.3微米、姿态误差约0.00128rad，当前姿态逆解通过，奇异区代码0。查询记录在 `outputs/realman/robot18-onboarding-readonly-20261102.json`。仅验证读取与当前点运动学一致性，未发送任何机械臂或夹爪运动。

平移缩放0.6、速度上限0.06m/s、半径0.15m、角速度上限0.2rad/s、角度范围0.5rad，与当前 `.19` 相同；夹爪分段默认50原始单位。XR基变换沿用 `.19` 的初始约定，需要通过 `.18` 现场预览核对；两台安装朝向不同就需要分别设置，不能由IP地址推断。通用保护和按键规则见 [平台说明](platform.md)。

工作目录 `/home/guest/下载/ThoughtIntoAction-local`：

```bash
# 以下均为只读，不发送运动：
./run.sh realman status --config configs/robots/realman/rm65-18.json
./run.sh realman check-kinematics --config configs/robots/realman/rm65-18.json
./run.sh realman gripper-status --config configs/robots/realman/rm65-18.json
./run.sh realman quest --config configs/robots/realman/rm65-18.json --position-only --duration 30
```

此单臂Quest入口目前仍绑定右手柄。不要同时启动分别指向 `.18` 与 `.19` 的两个右手柄执行会话；这不是左右手柄分别控制两台的双臂入口。
