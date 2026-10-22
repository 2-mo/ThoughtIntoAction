# PhyAgentOS 的参考取舍

阅读日期：2026-10-22。依据为 PhyAgentOS-core 的固定提交 [`a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8`](https://github.com/PhyAgentOS/PhyAgentOS-core/tree/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8)，核对了实际 Python 实现、接口文档和 [MIT 许可证](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/LICENSE)。本次借鉴设计，没有安装它，也没有引入运行依赖或复制其控制框架。

## 值得借鉴的实现

| 实际实现 | 本项目采用或保留的设计 |
| --- | --- |
| [`SkillManifest` 与 `RuntimeProfile`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/skill_runtime/manifest.py#L114) 将版本、能力集合、资源和启动 profile 显式声明；未知 profile 报错 | 使用现有 `configs/plugins.json` 单目录。机器人 profile、模型和后端由设备模块提供；公共控制内核消费描述，不先执行某个品牌的 SDK/CAN 检查 |
| [`ForgeToolClient`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/forge/tool_client.py#L128) 区分 Query、Action、Session；接收动作和接收取消请求均不表示执行完成 | 状态查询、有限运动、持续采集/遥操各自明确生命周期。沿用 `Motion`、operation ID 与反馈完成判据，不重新搭建一套 HTTP 执行面 |
| [`RuntimeManager`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/skill_runtime/manager.py#L48) 将运行状态与实际能力就绪联合判断；[启动失败](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/skill_runtime/manager.py#L162) 清理已创建的 flow | 驱动负责 preflight、设备资源、打开与关闭；公共内核负责流程和错误释放。PID、socket 或模块导入成功不能单独证明设备可用 |
| [`ForgeSkillBinding`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/forge/binding.py#L53) 固定运行实例、profile、版本及能力规格哈希 | 保留 runtime/boot/model 身份与参数快照。进一步扩充插件版本记录时使用现有 run manifest；不把跨进程哈希查询塞入控制周期 |
| [`NodeLock`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/skill_runtime/manifest.py#L160) 固定产物平台、架构、入口和摘要；[Core 分发边界](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/docs/forge/UNIFIED_TOOL_API.md#L54) 不捆绑具体机器人实现 | 参考与运行依赖分离。安装项锁版本，受管工作区放 `outputs/dependencies/`；正式维护的 vendor 资源随所属模块保存来源和许可证 |

## 不照搬的部分

- **它不提供可直接获得的机器人驱动集合。** 该 Core 的 [`plugins/__init__.py`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/plugins/__init__.py) 仅声明模块，具体物理执行能力由独立 Skill Runtime 提供。参考它不等于本项目新增 Piper 或其他机械臂支持。
- **当前发现机制限定单个健康 Runtime。** [`discover_active_runtime`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/skill_runtime/integration.py#L238) 遇到多个活动 Runtime 会报错；本项目机械臂、相机、输入和模型推理需要并行组合，不采用这个限制。
- **Agent 层校验不能代替本地控制循环。** [`validate_tool`](https://github.com/PhyAgentOS/PhyAgentOS-core/blob/a6d3fc3da0b916b66cb7bd3f9fe45a4a5e049bd8/PhyAgentOS/forge/binding.py#L275) 会查询远端规格和上下文。本项目保留本地有界控制路径，不在高频伺服中引入网络请求。
- 现有 Registry、设备租约、进程管理和动作协议继续复用；仅为插件化不额外引入 Dora、资源下载服务或 LLM 工具系统。

## 本轮落点与边界

公共运行时归 `src/embodied/control/`；ARX profile、SDK、真机/仿真与型号运动学归 `devices/robots/arx5/`；Quest 输入及原生路由归 `devices/inputs/quest/`。环境声明、设备配置与运行产物分别归 `configs/`、`assets/robots/` 和 `outputs/`，详见 [源码导航](../../src/README.md) 和 [部件边界](component-boundaries.md)。

这是代码结构和契约层面的改造。X5 六轴是当前完整机械臂实现，七轴遥测 fixture 通过只说明协议能够表达七轴；新硬件仍需自己的 profile、驱动、模型、生命周期与实测验收。原有 LeRobot X5 导出器继续明确限定六轴，不能把协议扩展当成训练格式已经通用。
