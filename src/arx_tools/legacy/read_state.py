"""Legacy one-shot state reader; prefer ``./arx5 state``."""

import time

import numpy as np
import arx5_interface as arx5


MODEL = "X5"
INTERFACE = "can0"

robot_config = arx5.RobotConfigFactory.get_instance().get_config(MODEL)

controller_config = (
    arx5.ControllerConfigFactory.get_instance()
    .get_config("joint_controller", robot_config.joint_dof)
)

# 后台进行 CAN 收发
controller_config.background_send_recv = True

print("正在连接 ARX X5……")
arm = arx5.Arx5JointController(
    robot_config,
    controller_config,
    INTERFACE,
)

try:
    time.sleep(1.0)

    state = arm.get_joint_state()

    print("关节位置（rad）：", state.pos())
    print("关节位置（deg）：", np.degrees(state.pos()))
    print("关节速度：", state.vel())
    print("夹爪位置：", state.gripper_pos)

    time.sleep(1.0)

finally:
    # 对象销毁时，X5 SDK 会进入阻尼状态
    del arm
