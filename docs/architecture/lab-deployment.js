// Deployment annotations for the unified architecture, not a separate view.
// Router model and wired Mesh plan are supplied by the user, not live inventory.
window.LAB_DEPLOYMENT = {
  "cables": [
    {
      "from": "hub",
      "to": "mesh-main",
      "path": "M400 798V841"
    },
    {
      "from": "mesh-main",
      "to": "mesh-a",
      "path": "M510 865H760",
      "label": "有线回程",
      "x": 635,
      "y": 856
    },
    {
      "from": "mesh-main",
      "to": "mesh-b",
      "path": "M510 876H580V901H1350V865H1390",
      "label": "有线回程",
      "x": 1110,
      "y": 893
    },
    {
      "from": "mesh-a",
      "to": "compute",
      "path": "M810 841V820H740V798"
    },
    {
      "from": "mesh-a",
      "to": "arm-node",
      "path": "M930 841V820H1010V798"
    },
    {
      "from": "mesh-b",
      "to": "sensor-node",
      "path": "M1450 841V820H1280V798"
    },
    {
      "from": "mesh-b",
      "to": "mobile-node",
      "path": "M1550 841V820H1646V798"
    }
  ],
  "nodes": [
    {
      "id": "hub",
      "kind": "hub",
      "x": 290,
      "y": 742,
      "width": 220,
      "height": 56,
      "title": "Ubuntu 总台",
      "subtitle": "无 GPU · L1—L4",
      "description": "总台承载统一操作入口与协调服务。中央 Harness 维护任务、上下文与工具循环，通过网络调用模型服务，本机不需要加载大模型权重。",
      "responsibilities": [
        "承载网页、终端共用的 API、用户会话、任务状态、设备目录与授权；各入口按设备实例选择目标。",
        "登记节点身份、能力、端点、版本和健康状态。节点上线、服务可调用与取得控制权分别判断。",
        "保存任务元数据、审计与轨迹引用；原始视频可在设备侧缓存或进入独立存储，按需提供预览与检索。",
        "依赖正常的 CPU、内存、存储与网络；可先与操作电脑合用，之后也可独立托管。"
      ],
      "boundary": "目标部署。当前 ./run.sh 仍启动本机工作台；中央 Harness、跨主机目录与远程设备选择尚待实现，搬动网页文件不能完成迁移。",
      "modules": [
        "web-frontend",
        "terminal-interface",
        "application-api",
        "task-services",
        "decision",
        "device-directory",
        "resource-control",
        "security",
        "inference-service"
      ]
    },
    {
      "id": "compute",
      "kind": "compute",
      "x": 620,
      "y": 742,
      "width": 240,
      "height": 56,
      "title": "5090 算力节点",
      "subtitle": "L5 模型服务 / RTC",
      "description": "5090 及其他算力机器提供可替换的模型服务。总台负责选择和调用，模型进程独立维护权重、运行环境与资源占用。",
      "responsibilities": [
        "节点登记模型、版本、输入输出契约、端点和健康状态；是否能受理任务由实际资源状态决定。",
        "LLM / VLM 请求由 Harness 发起；在线策略按授权订阅设备观测，输出经控制会话进入设备侧运行时。",
        "连续策略独立推理，不逐帧经过中央 LLM；设备侧验证目标序号、时效、控制权与本地约束。",
        "算力节点失联时报告受影响任务；独立的设备查询与人工控制路径仍可使用。",
        "RTC 在兼容的扩散/流模型采样过程中利用动作前缀与延迟条件生成下一块；设备侧客户端反馈执行游标与未执行前缀。"
      ],
      "boundary": "已有模型客户端与独立环境基础；RTC 引导采样和跨机执行协同尚待实现。普通返回动作块的推理 API 不自动具备 RTC。",
      "modules": [
        "inference-service",
        "model-algorithms",
        "observation-scheduler",
        "action-chunk-executor"
      ]
    },
    {
      "id": "network",
      "kind": "service",
      "x": 42,
      "y": 752,
      "width": 218,
      "height": 56,
      "title": "平台节点接入",
      "subtitle": "认证 / 登记 / 健康",
      "description": "有线 Mesh 负责机器之间的网络连通；各设备和算力主机上的节点服务再向 Ubuntu 总台认证登记能力。这是应用层的接入协议，与路由器的 Mesh 配网分别维护。",
      "responsibilities": [
        "第一步采用明确的总台地址和节点配置，登记 host_id、device_id、能力、服务端点、版本与在线状态；协议发现仅作为后续扩展。",
        "登记成功后仍须验证端点可达。可直连时使用受保护端点，无法直连时另配认证双向通道或中继。",
        "任务协调与状态汇聚由总台负责；大流量观测及策略目标可在设备和算力节点间按授权传输，必要时走中继。",
        "各主机的 monotonic 时钟不能直接相减；协议需定义时间域、观测年龄、目标 TTL、序号与本地失效判断。",
        "连续目标过期由设备侧停止或保持；离散动作按其声明的失联策略处理。重连不自动取得控制权，也不重放结果未知的动作。"
      ],
      "boundary": "现有远程网页访问不等于多机设备接入。通用节点注册、能力路由、跨机认证与时效协议均属于目标能力。",
      "modules": [
        "network",
        "device-directory",
        "security",
        "resource-control",
        "operations"
      ]
    },
    {
      "id": "arm-node",
      "kind": "device",
      "x": 890,
      "y": 742,
      "width": 240,
      "height": 56,
      "title": "机械臂接入机",
      "subtitle": "L5 执行器 · L6—L7",
      "description": "与机械臂和相机实际连接的电脑持有设备连接，向总台发布能力。运动规划与周期控制继续使用本地运行时，不把高频控制循环搬到总台。",
      "responsibilities": [
        "节点接入服务处理远程身份、能力请求和状态发布，内部复用本机 ControlClient、运行时与驱动。",
        "机械臂 worker 在本地读取状态、仲裁来源、执行命令与约束并发布反馈；相机服务独立采集与发布观测。",
        "按设备实例保留唯一资源所有者；同一主机可以接多个设备，每个实例分别拥有身份、生命周期和控制权。",
        "总台或网络失联时，由本地执行期限与租约规则处理动作；重新连接先对账，不自动恢复旧动作。",
        "策略执行器与运动内核分开：前者缓冲并对齐动作块，经控制会话提交；后者保留本地周期控制、限位和停止。"
      ],
      "boundary": "本地机械臂与相机运行时已有基础；远程节点服务、同主机多实例管理及新版真机完整验收仍待完成。",
      "modules": [
        "observation-scheduler",
        "action-chunk-executor",
        "motion-kernel",
        "perception",
        "arm-driver",
        "resource-control"
      ]
    },
    {
      "id": "sensor-node",
      "kind": "device",
      "x": 1160,
      "y": 742,
      "width": 240,
      "height": 56,
      "title": "感知接入机",
      "subtitle": "L6—L7 · 相机 / 传感器",
      "description": "将分散在不同电脑上的相机与传感器接入同一个设备目录，消费者通过声明的流端点订阅数据。只读感知节点无需实现机器人动作接口。",
      "responsibilities": [
        "发布设备身份、流描述、时间来源、坐标系与质量信息；消费者检查观测新鲜度和时间对齐条件。",
        "多方共享观测，由本地服务统一持有硬件。视频与高频测量按带宽需求直接供给工作台或算力节点。",
        "本地采集缓存和上传任务显式管理容量与失败；大体量数据可放入独立存储，总台保存索引与任务关联。"
      ],
      "boundary": "已有本机相机工厂与共享帧机制；跨机器流服务、通用传感器驱动及多机时间质量管理尚待实现。",
      "modules": [
        "perception",
        "perception-drivers",
        "experiment-data",
        "network"
      ]
    },
    {
      "id": "mobile-node",
      "kind": "device",
      "x": 1430,
      "y": 742,
      "width": 432,
      "height": 56,
      "title": "移动 / 装置接入机",
      "subtitle": "L6—L7 · 底盘 / 实验设备",
      "description": "移动机器人可用自己的机载电脑作为接入节点，其他装置可经现有控制电脑或外接网关接入；不要求每个硬件直接运行总台客户端。",
      "responsibilities": [
        "按实际型号提供状态、导航、底盘运动或其他专用能力，保留自己的运行时和驱动。",
        "移动机器人可同时登记底盘、定位与相机能力。设备类别用于展示，调用方按能力契约选择目标。",
        "停止、导航取消、定位质量与断连行为在设备侧定义并逐项验收；不依赖总台持续刷新页面。",
        "图中实线示例为接入机的有线连接；移动机载电脑最后一跳可按场景使用 Wi-Fi，路由器间仍保持有线回程。"
      ],
      "boundary": "这是待接入的节点角色示例，尚未选定或验收移动机器人和通用实验装置；图中节点并非在线设备清单。",
      "modules": [
        "mobile-runtime",
        "actuator-plugins",
        "mobile-drivers",
        "perception",
        "resource-control"
      ]
    },
    {
      "id": "mesh-main",
      "kind": "router",
      "x": 290,
      "y": 841,
      "width": 220,
      "height": 48,
      "title": "Mesh 主节点",
      "subtitle": "网关 / 地址分配",
      "description": "按用户提供的同款 E2631 有线 Mesh 方案，主节点在本图中以三路 LAN 分别连接 Ubuntu 总台、Mesh 子节点 A 与 B。路由器负责网络，Ubuntu 总台上的服务负责任务和设备管理。",
      "responsibilities": [
        "沿用这批设备固件支持的原生 Mesh 组网方式；主节点 LAN 到子节点 WAN 用于有线回程，端口角色与配对步骤以实际固件说明为准。",
        "后台确认子节点已加入 Mesh 且回程为有线；各主机获得预期地址和网关，实际受保护服务端点可互通。",
        "总台、算力机和设备接入机保留稳定地址，Mesh 子节点不另行配置为独立 NAT 路由器。",
        "路由器组网、节点服务认证登记和设备控制权分别判断；Mesh 本身不加载设备插件，也不执行中央 Agent。"
      ],
      "boundary": "设备型号与接线为用户确认的规划输入，尚未现场核验固件、端口或带宽；不把灯色或“自组网 V2”作为通用配置要求。",
      "modules": [
        "network",
        "device-directory",
        "security",
        "operations"
      ]
    },
    {
      "id": "mesh-a",
      "kind": "router",
      "x": 760,
      "y": 841,
      "width": 220,
      "height": 48,
      "title": "Mesh 子节点 A",
      "subtitle": "计算与操作区",
      "description": "按用户提供的同款 E2631 有线 Mesh 方案，子节点 A 就近连接 5090 与机械臂接入机。路由器负责网络，Ubuntu 总台上的服务负责任务和设备管理。",
      "responsibilities": [
        "沿用这批设备固件支持的原生 Mesh 组网方式；主节点 LAN 到子节点 WAN 用于有线回程，端口角色与配对步骤以实际固件说明为准。",
        "后台确认子节点已加入 Mesh 且回程为有线；各主机获得预期地址和网关，实际受保护服务端点可互通。",
        "总台、算力机和设备接入机保留稳定地址，Mesh 子节点不另行配置为独立 NAT 路由器。",
        "路由器组网、节点服务认证登记和设备控制权分别判断；Mesh 本身不加载设备插件，也不执行中央 Agent。"
      ],
      "boundary": "设备型号与接线为用户确认的规划输入，尚未现场核验固件、端口或带宽；不把灯色或“自组网 V2”作为通用配置要求。",
      "modules": [
        "network",
        "device-directory",
        "security",
        "operations"
      ]
    },
    {
      "id": "mesh-b",
      "kind": "router",
      "x": 1390,
      "y": 841,
      "width": 220,
      "height": 48,
      "title": "Mesh 子节点 B",
      "subtitle": "感知与装置区",
      "description": "按用户提供的同款 E2631 有线 Mesh 方案，子节点 B 就近连接感知和其他设备接入机。路由器负责网络，Ubuntu 总台上的服务负责任务和设备管理。",
      "responsibilities": [
        "沿用这批设备固件支持的原生 Mesh 组网方式；主节点 LAN 到子节点 WAN 用于有线回程，端口角色与配对步骤以实际固件说明为准。",
        "后台确认子节点已加入 Mesh 且回程为有线；各主机获得预期地址和网关，实际受保护服务端点可互通。",
        "总台、算力机和设备接入机保留稳定地址，Mesh 子节点不另行配置为独立 NAT 路由器。",
        "路由器组网、节点服务认证登记和设备控制权分别判断；Mesh 本身不加载设备插件，也不执行中央 Agent。"
      ],
      "boundary": "设备型号与接线为用户确认的规划输入，尚未现场核验固件、端口或带宽；不把灯色或“自组网 V2”作为通用配置要求。",
      "modules": [
        "network",
        "device-directory",
        "security",
        "operations"
      ]
    }
  ]
};
