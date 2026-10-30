// Target deployment roles; these are not live nodes or implemented startup profiles.
window.LAB_DEPLOYMENT = {
  title: '知行有灵 · 实验室多机部署', width: 1800, height: 850,
  nodes: [
    {
      id: 'hub', kind: 'hub', x: 40, y: 76, width: 780, height: 256,
      label: 'CENTRAL HUB / L1—L4', title: '轻量总台 · 普通 CPU 主机',
      subtitle: '统一入口、任务编排与设备目录 · 无需本地 GPU',
      rows: ['工作台与应用服务', '中央 Agent Harness（编排）', '节点 / 设备 / 能力目录', '身份授权与任务记录'],
      footer: '模型调用交给算力节点，设备动作交给接入节点',
      description: '总台承载统一操作入口与协调服务。中央 Harness 维护任务、上下文与工具循环，通过网络调用模型服务，本机不需要加载大模型权重。',
      responsibilities: [
        '承载网页、终端共用的 API、用户会话、任务状态、设备目录与授权；各入口按设备实例选择目标。',
        '登记节点身份、能力、端点、版本和健康状态。节点上线、服务可调用与取得控制权分别判断。',
        '保存任务元数据、审计与轨迹引用；原始视频可在设备侧缓存或进入独立存储，按需提供预览与检索。',
        '依赖正常的 CPU、内存、存储与网络；可先与操作电脑合用，之后也可独立托管。'
      ],
      boundary: '目标部署。当前 ./run.sh 仍启动本机工作台；中央 Harness、跨主机目录与远程设备选择尚待实现，搬动网页文件不能完成迁移。',
      modules: ['web-frontend', 'terminal-interface', 'application-api', 'task-services', 'decision', 'device-directory', 'resource-control', 'security']
    },
    {
      id: 'compute', kind: 'compute', x: 1140, y: 76, width: 620, height: 256,
      label: 'COMPUTE NODES / 模型与算法服务', title: '算力节点 × N · 5090 等',
      subtitle: '模型环境与权重独立部署，按能力提供推理服务',
      rows: ['LLM / VLM · 语音与视觉推理', 'VLA 策略 · 独立推理循环'],
      footer: '算力与设备角色可以部署在同一台物理机器',
      description: '5090 及其他算力机器提供可替换的模型服务。总台负责选择和调用，模型进程独立维护权重、运行环境与资源占用。',
      responsibilities: [
        '节点登记模型、版本、输入输出契约、端点和健康状态；是否能受理任务由实际资源状态决定。',
        'LLM / VLM 请求由 Harness 发起；在线策略按授权订阅设备观测，输出经控制会话进入设备侧运行时。',
        '连续策略独立推理，不逐帧经过中央 LLM；设备侧验证目标序号、时效、控制权与本地约束。',
        '算力节点失联时报告受影响任务；独立的设备查询与人工控制路径仍可使用。'
      ],
      boundary: '已有 GR00T TCP 客户端基础，但默认仍拉起本机模型服务；π0.5 主要依赖本机子进程与文件。通用远程模型服务与算力目录尚待建设。',
      modules: ['model-algorithms', 'decision', 'resource-control', 'experiment-data']
    },
    {
      id: 'network', kind: 'network', x: 40, y: 390, width: 1720, height: 70,
      title: '实验室网络与节点接入', subtitle: '认证登记 · 能力发现 · 心跳与状态 · 可路由端点 / 双向通道',
      description: '各设备节点与算力节点主动向配置的总台地址认证登记。网络连通后，总台按能力定位服务；图中的网络带表示主机通信关系，不是所有消息必须经过的中心转发服务。',
      responsibilities: [
        '第一步采用明确的总台地址和节点配置，登记 host_id、device_id、能力、服务端点、版本与在线状态；协议发现仅作为后续扩展。',
        '登记成功后仍须验证端点可达。可直连时使用受保护端点，无法直连时另配认证双向通道或中继。',
        '任务协调与状态汇聚由总台负责；大流量观测及策略目标可在设备和算力节点间按授权传输，必要时走中继。',
        '各主机的 monotonic 时钟不能直接相减；协议需定义时间域、观测年龄、目标 TTL、序号与本地失效判断。',
        '连续目标过期由设备侧停止或保持；离散动作按其声明的失联策略处理。重连不自动取得控制权，也不重放结果未知的动作。'
      ],
      boundary: '现有远程网页访问不等于多机设备接入。通用节点注册、能力路由、跨机认证与时效协议均属于目标能力。',
      modules: ['network', 'device-directory', 'security', 'resource-control', 'operations']
    },
    {
      id: 'arm-node', kind: 'device', x: 40, y: 514, width: 540, height: 228,
      label: 'DEVICE NODE A / L5—L7', title: '机械臂工作站',
      subtitle: '一台接入机可同时管理执行与感知设备',
      rows: ['机械臂运动内核 + 机器人驱动', '相机 / 传感器运行时 + 采集驱动'],
      footer: 'USB / CAN / 网口 → 机械臂 · 夹爪 · 相机',
      description: '与机械臂和相机实际连接的电脑持有设备连接，向总台发布能力。运动规划与周期控制继续使用本地运行时，不把高频控制循环搬到总台。',
      responsibilities: [
        '节点接入服务处理远程身份、能力请求和状态发布，内部复用本机 ControlClient、运行时与驱动。',
        '机械臂 worker 在本地读取状态、仲裁来源、执行命令与约束并发布反馈；相机服务独立采集与发布观测。',
        '按设备实例保留唯一资源所有者；同一主机可以接多个设备，每个实例分别拥有身份、生命周期和控制权。',
        '总台或网络失联时，由本地执行期限与租约规则处理动作；重新连接先对账，不自动恢复旧动作。'
      ],
      boundary: '本地机械臂与相机运行时已有基础；远程节点服务、同主机多实例管理及新版真机完整验收仍待完成。',
      modules: ['motion-kernel', 'perception', 'arm-driver', 'perception-drivers', 'resource-control']
    },
    {
      id: 'sensor-node', kind: 'device', x: 630, y: 514, width: 540, height: 228,
      label: 'DEVICE NODE B / L5—L7', title: '感知工作站',
      subtitle: '独立接入相机与传感器，按流共享观测',
      rows: ['感知运行时 + 厂商采集驱动', '本地采集缓存 + 授权数据订阅'],
      footer: 'USB / 网口 / 串口 → 相机 · 各类传感器',
      description: '将分散在不同电脑上的相机与传感器接入同一个设备目录，消费者通过声明的流端点订阅数据。只读感知节点无需实现机器人动作接口。',
      responsibilities: [
        '发布设备身份、流描述、时间来源、坐标系与质量信息；消费者检查观测新鲜度和时间对齐条件。',
        '多方共享观测，由本地服务统一持有硬件。视频与高频测量按带宽需求直接供给工作台或算力节点。',
        '本地采集缓存和上传任务显式管理容量与失败；大体量数据可放入独立存储，总台保存索引与任务关联。'
      ],
      boundary: '已有本机相机工厂与共享帧机制；跨机器流服务、通用传感器驱动及多机时间质量管理尚待实现。',
      modules: ['perception', 'perception-drivers', 'experiment-data', 'network']
    },
    {
      id: 'mobile-node', kind: 'device', x: 1220, y: 514, width: 540, height: 228,
      label: 'DEVICE NODE C / L5—L7', title: '移动机器人 / 其他装置节点',
      subtitle: '机载电脑或外接网关，都按能力接入',
      rows: ['导航 / 装置运行时 + 专用驱动', '本地状态、执行约束与断连处置'],
      footer: '机载总线 / 厂商 API → 底盘 · 实验装置',
      description: '移动机器人可用自己的机载电脑作为接入节点，其他装置可经现有控制电脑或外接网关接入；不要求每个硬件直接运行总台客户端。',
      responsibilities: [
        '按实际型号提供状态、导航、底盘运动或其他专用能力，保留自己的运行时和驱动。',
        '移动机器人可同时登记底盘、定位与相机能力。设备类别用于展示，调用方按能力契约选择目标。',
        '停止、导航取消、定位质量与断连行为在设备侧定义并逐项验收；不依赖总台持续刷新页面。'
      ],
      boundary: '这是待接入的节点角色示例，尚未选定或验收移动机器人和通用实验装置；图中节点并非在线设备清单。',
      modules: ['mobile-runtime', 'actuator-plugins', 'mobile-drivers', 'perception', 'resource-control']
    }
  ]
};
