# 可替换组件

新 arx 运行使用共享 `embodied_contracts.registry.Registry`，合并 `configs/plugins.toml` 和 `control/config/components.json`；机械臂、VR、控制源、相机、规划、标定和导出接口见 [插件工作台](plugin-workbench.md)。下面保留旧服务入口的契约，不能将其 `arm` 插件直接当作新 `robot` 后端使用。

组件由 `configs/plugins.toml` 注册，按职责加载。列出注册项：`./run.sh plugins`。

| 类型 | 内置实现 | 加载位置 |
| --- | --- | --- |
| `arm` | `stanford`、`simulated` | `arm-service --backend NAME` |
| `arm` | `service` | Quest 控制器中的本机服务客户端 |
| `arm` | `arx2025` | 独立实验/诊断入口，不能作为主服务后端 |
| `input` | `quest`、`idle` | 虚拟控制 `--input`，真机遥操 `--input-plugin` |
| `telemetry` | `local`、`none` | 服务固定使用 local；回放和历史诊断可选 |
| `viewer` | `web` | 独立监看 `view-arm --viewer NAME` |
| `camera` | `realsense` | `cameras.wrist_rgb.driver` |

日常链路为控制客户端 → 机械臂服务 → 状态流 → 监看。控制端不创建设备后端或状态发布器；显示插件只在监看进程加载。启动流程见 [独立服务与监看](monitoring.md)。

## 工厂契约

实现不必继承公共基类，注册表只检查工厂与能力声明。

- 设备后端：`factory(settings=..., initial_q=None)`。六轴加夹爪反馈和指令采用 rad/m；提供 `set_control_enabled(bool)`、`set_motion_limit(speed)`、`set_to_damping()`、`close()`。硬件另需 `get_recording_snapshot()`，含反馈、SDK 输出及时间戳。可供独立服务加载的插件声明 `service_device` 能力。`initial_q` 仅用于虚拟后端。
- 服务客户端：`ArmClient` 读取反馈、申请/释放控制权、提交关节和夹爪目标；不导入设备 SDK。先设置六轴目标，再设置夹爪，合并成一条服务请求。
- 位姿输入：无参工厂，返回具名 xyz+xyzw 位姿、Grip/Trigger、按键/tracker 读取和 `close()`。`idle` 不按键、不跟随，用于虚拟流程演练。
- 状态输出：`factory(robot=..., source=...)`，提供 `update(actual, target=None)`、`path(name, points, color)`、`close()`。路径是 Nx3 米坐标；不得等待监看确认。主服务使用 `LocalPublisher.set_source()` 在控制权变化时更新来源。
- 显示：`factory(robot=..., port=0)`，返回 `accept(packet)`、`set_stream_error(message)`、`close()`。消费统一状态，不操作机械臂。
- 相机：`factory(serial=..., width=..., height=..., fps=...)`，提供上下文生命周期、`metadata` 和含图像及时间戳的 `read()`。

例如新增设备后端时，在 `robot_control/backends/` 实现上述服务契约，并注册：

```toml
[arm.my_device]
factory = "robot_control.backends.my_device:create"
hardware = true
capabilities = ["service_device", "recording"]
```

使用 `./run.sh arm-service --backend my_device` 选择；输入和监看无需修改。插件代码属于可信项目代码，服务不替代后端自己的限速、反馈检查及安全模式实现。

虚拟控制客户端明确要求虚拟服务；`simulation_only=true` 的输入不可用于主真机入口。真实采集不接受虚拟服务。手眼标定也通过同一个服务申请控制权，不创建第二个 SDK 控制器。

主控制入口不再提供 `--backend`、`--telemetry`、CAN 或 SDK preview 参数。后端和硬件参数由服务管理，监看独立选择显示插件；不保留旧参数转发。历史独立诊断入口仍保留其自身参数，但须遵守设备所有权锁。
