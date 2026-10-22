# Reference7

本项目原创的七轴无夹爪教学机械臂，使用 MuJoCo 刚体动力学、关节力矩 PD 和重力补偿。
它用于检验插件接入及不同关节数的数据链路，不对应任何厂商实物，不代表真机性能。

- `reference7.xml`：七个旋转关节、七个力矩执行器、TCP site；2 ms 物理步长，默认 100 Hz 控制。
- `reference7.urdf`：同一关节链和 TCP，供平台 Pinocchio/Three.js 显示。显示几何为圆柱，MuJoCo 连杆为胶囊。
- 没有夹爪，数据契约明确为 `gripper: null`；不记录虚构第八维。
- 不提供硬件插件，不校验自碰撞；工作区检查仍由公共内核处理。

配置入口：`configs/robots/reference7/robot.json`。模型、关节顺序和频率由运行时描述；改变 URDF/XML 会改变模型身份。
