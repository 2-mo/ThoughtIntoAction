// Target deployment: physical connections and service calls are drawn separately.
// Router model and wired Mesh plan are supplied by the user, not live inventory.
window.LAB_DEPLOYMENT = {
  title: "知行有灵 · 有线 Mesh 与多机协作", width: 1900, height: 880,
  panels: [
    {"x":28,"y":86,"width":1234,"height":720,"title":"01  机器怎么接","subtitle":"实线表示网线；设备连接另标协议。路由器只负责联网。"},
    {"x":1286,"y":86,"width":586,"height":720,"title":"02  任务怎么走","subtitle":"箭头表示服务调用；实际通过左侧网络传输。"},
  ],
  cables: [
    {"from":"hub","to":"mesh-main","kind":"ethernet","path":"M412 210H612","label":"网线","x":512,"y":200},
    {"from":"mesh-main","to":"mesh-a","kind":"backhaul","path":"M690 242V302H407V332","label":"LAN → WAN · 有线回程","x":550,"y":292},
    {"from":"mesh-main","to":"mesh-b","kind":"backhaul","path":"M800 242V302H1007V332","label":"LAN → WAN · 有线回程","x":930,"y":292},
    {"from":"mesh-a","to":"compute","kind":"ethernet","path":"M350 398V429H197V460"},
    {"from":"mesh-a","to":"arm-node","kind":"ethernet","path":"M460 398V429H497V460"},
    {"from":"mesh-b","to":"sensor-node","kind":"ethernet","path":"M950 398V429H797V460"},
    {"from":"mesh-b","to":"mobile-node","kind":"ethernet","path":"M1060 398V429H1097V460"},
    {"from":"arm-node","to":"arm-hardware","kind":"device","path":"M497 590V650","label":"USB / CAN / 网口","x":497,"y":624},
    {"from":"sensor-node","to":"sensor-hardware","kind":"device","path":"M797 590V650","label":"USB / 网口 / 串口","x":797,"y":624},
    {"from":"mobile-node","to":"mobile-hardware","kind":"device","path":"M1097 590V650","label":"机载总线 / 厂商 API","x":1097,"y":624},
  ],
  hardware: [
    {"id":"arm-hardware","x":362,"y":650,"width":270,"height":60,"title":"机械臂 · 夹爪 · 相机","subtitle":"执行与感知设备"},
    {"id":"sensor-hardware","x":662,"y":650,"width":270,"height":60,"title":"相机 · 各类传感器","subtitle":"观测与测量设备"},
    {"id":"mobile-hardware","x":962,"y":650,"width":270,"height":60,"title":"移动底盘 · 实验装置","subtitle":"按型号提供控制接口"},
  ],
  workflow: {
    groups: [
      {"x":1378,"y":315,"width":440,"height":266,"title":"L5 推理部署与策略运行 · 跨主机"},
    ],
    nodes: [
      {"id":"hub","kind":"hub","x":1394,"y":176,"width":408,"height":82,"title":"Ubuntu 总台","subtitle":"统一入口 / 任务 / Agent Harness","footer":"任务编排与策略启停"},
      {"id":"inference-service","kind":"compute","x":1394,"y":350,"width":408,"height":78,"title":"算力端 · 模型服务与 RTC 推理","subtitle":"模型装配 / 前缀约束 / 下一动作块","footer":"RTC 采样需按模型适配"},
      {"id":"action-chunk-executor","kind":"compute","x":1394,"y":477,"width":408,"height":82,"title":"设备端 · 策略执行器","subtitle":"观测适配 / 异步调度 / 动作块缓冲","footer":"执行游标对齐，丢弃过期结果"},
      {"id":"motion-kernel","kind":"device","x":1394,"y":643,"width":408,"height":82,"title":"L6 设备运行时","subtitle":"感知发布 / 控制会话 / 运动约束","footer":"设备侧独立周期控制"},
    ],
    links: [
      {"kind":"model","path":"M1598 260V313","label":"模型调用 / 策略启停","x":1614,"y":291},
      {"kind":"task","path":"M1392 217H1340V684H1392","label":"人工 / 确定性技能 ↔ 状态与结果","x":1323,"y":452,"rotate":-90},
      {"kind":"policy","path":"M1598 430V475","label":"观测与前缀 ↑　↓ 新动作块","x":1598,"y":457,"center":true},
      {"kind":"target","path":"M1598 561V641","label":"控制目标 ↓ / 观测与反馈 ↑","x":1614,"y":613},
    ],
  },
  nodes: [
    {
      "id": "hub",
      "kind": "hub",
      "x": 62,
      "y": 152,
      "width": 350,
      "height": 136,
      "label": "总台主机 · L1—L4",
      "title": "Ubuntu 总台 · 无需 GPU",
      "subtitle": "统一入口与任务协调",
      "rows": [
        "工作台 / API / 任务管理",
        "中央 Harness / 设备目录"
      ],
      "footer": "按能力调用设备与模型",
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
      "x": 62,
      "y": 460,
      "width": 270,
      "height": 130,
      "label": "算力节点 · L5 服务端",
      "title": "5090 主机",
      "subtitle": "独立模型环境与权重",
      "rows": [
        "模型服务 / 动作块推理",
        "RTC 采样适配（待建设）"
      ],
      "footer": "可扩展多个算力节点",
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
      "x": 1310,
      "y": 744,
      "width": 538,
      "height": 54,
      "title": "平台节点接入",
      "subtitle": "认证 / 登记 / 健康 / 服务可达性",
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
      ],
      "label": "跨主机服务的共同前提",
      "footer": "Mesh 配网与平台节点登记分别完成"
    },
    {
      "id": "arm-node",
      "kind": "device",
      "x": 362,
      "y": 460,
      "width": 270,
      "height": 130,
      "label": "设备节点 A · L5 客户端 / L6—L7",
      "title": "机械臂接入机",
      "subtitle": "执行与感知可在同机",
      "rows": [
        "策略执行器 / 动作块缓冲",
        "运动内核 / 相机与驱动"
      ],
      "footer": "周期控制与约束在本机",
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
      "x": 662,
      "y": 460,
      "width": 270,
      "height": 130,
      "label": "设备节点 B · L6—L7",
      "title": "感知接入机",
      "subtitle": "独立采集与共享观测",
      "rows": [
        "相机 / 传感器运行时",
        "采集缓存 / 数据订阅"
      ],
      "footer": "一个服务持有，多方订阅",
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
      "x": 962,
      "y": 460,
      "width": 270,
      "height": 130,
      "label": "设备节点 C · L6—L7",
      "title": "移动 / 装置接入机",
      "subtitle": "机载电脑或外接控制机",
      "rows": [
        "导航 / 装置运行时",
        "状态反馈 / 专用驱动"
      ],
      "footer": "按能力独立接入",
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
      "x": 612,
      "y": 176,
      "width": 270,
      "height": 66,
      "label": "中兴巡天 · ZXHN E2631",
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
      "x": 272,
      "y": 332,
      "width": 270,
      "height": 66,
      "label": "中兴巡天 · ZXHN E2631",
      "title": "Mesh 子节点 A",
      "subtitle": "有线回程 · 计算与操作区",
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
      "x": 872,
      "y": 332,
      "width": 270,
      "height": 66,
      "label": "中兴巡天 · ZXHN E2631",
      "title": "Mesh 子节点 B",
      "subtitle": "有线回程 · 感知与装置区",
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
    },
  ]
};
