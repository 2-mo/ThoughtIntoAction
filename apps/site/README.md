# 项目展示站

`apps/site/` 是可独立托管的静态宣传网站，使用 HTML、CSS、原生 JavaScript 与本地 Three.js，无构建步骤、外部字体或 CDN 依赖。页面先展示平台全貌，再让读者逐层查看真实接口、实现与使用入口。内容规划与维护边界见 [DESIGN.md](DESIGN.md)。

## 启动

从仓库根目录运行，访问终端打印的地址：

```bash
./run.sh web site
```

也可以独立预览：

```bash
python3 -m http.server 18082 --directory apps/site
```

随后打开 <http://127.0.0.1:18082/>。Three.js 使用 ES module，应通过 HTTP 访问；文档与源码链接使用 GitHub 地址，无需服务器暴露整个仓库。

## 六段内容

| 段落 | 实际内容与交互 |
| --- | --- |
| 概览 `#overview` | 输入、控制、相机、数据与策略组成三维模块图。可拖动观察、点击模块，并切换动作控制 / 示教采集 / 策略执行三种连接示意 |
| 完整架构 `#architecture` | 九个模块按职责排列；点击查看职责、协作接口、当前实现与源码。三条调用路径和首屏的数据流选择同步，每个步骤链接具体实现 |
| 编程接口 `#api` | MoveJ、XYZ MoveL、夹爪、记点返回、只读采集五种示例；SIM / REAL 切换连接代码，动作内容不变；整段代码可复制 |
| 工作台 `#workbench` | 手动切换场景、姿态与关节遥测三张项目截图，同时更新说明及指南链接；均标注为静态截图 |
| 数据与策略 `#workflow` | 四个固定阶段：采集、检查与回放、导出与训练、策略执行。机器人与相机双时间线、episode 产物示意及相关指南说明数据如何汇合 |
| 现状与开始 `#start` | 仿真已贯通、新内核真机继续验收、新设备与入口按模块收敛；提供工作台、可视化仿真、示例目录命令与安装文档 |

架构包含 `applications / inputs / contracts / motion / data / policy / core / control / sensors`。控制模块路径是 `src/embodied/control/`，机器人型号实现位于 `devices/robots/`；所有插件由 `configs/plugins.json` 登记。它们按职责组织，不等于九个常驻服务，也不表示相机、数据导出或训练必须经过机械臂内核。内容分别对应 `src/embodied/` 的公共运行时、设备适配与领域模块，以及 `apps/` 界面。

三条调用路径保留实际差异：MoveJ 使用 Robot → 预览 / 执行 → Ruckig → worker → 到位；采集汇集只读 observer 与共享相机帧；OpenPI 策略通过独立模型环境输出流式 joint 目标，不经过 MoveJ 的整段规划链。Python 与动作 CLI 已共用 Robot，网页控制台仍有待收敛部分。

## 三维与页面交互

- 三维图使用 `inputs / control / sensors / data / policy` 五个节点；流动光点表示所选路径。场景注明“架构示意，非实时数据”，不连接设备或控制服务。
- 拖动场景改变视角；聚焦画布后可用方向键旋转、`R` 复位，也可点击“重置视角”。三维模块具有可聚焦的文字按钮，点击后更新架构详情与首屏说明。
- 首屏的“暂停动效”控制持续动画；系统设置减少动态效果时进入低动态模式，关闭持续流动和视差，保留手动选择与视角操作。
- Canvas 离开视口或标签页隐藏时停止持续渲染；CSS 时间线只在对应章节可见且页面可见时播放。像素比限制为 1.5，页面离开时释放场景资源。
- 架构按钮支持普通键盘操作；数据流、代码示例、SIM / REAL、截图选择组支持左右方向键与 Home / End。按钮、链接有可见焦点，复制结果有读屏提示。
- 滚动进入时使用淡入与小位移；顶部显示阅读进度。页面正常纵向阅读，不再提供旧版全屏章节翻页或演示模式。

## 降级与设备适配

WebGL 2 不可用或三维资源加载失败时保留首屏 SVG 静态拓扑；架构详情、调用路径、API 与截图切换继续由普通 JavaScript 工作。暂停动效按钮仍可控制时间线，仅隐藏三维视角重置按钮。

禁用 JavaScript 时，静态拓扑、完整模块概览、默认控制详情、MoveJ 仿真代码、第一张截图、数据流程与开始命令仍可阅读；`noscript` 补充真机连接及采集 / 策略路径。交互切换和自动复制不可用，可通过现有文档链接继续阅读。

窄屏改为纵向布局，架构详情在模块图下方，代码在自身区域横向滚动。文字按钮提供触屏入口，内容不依赖 hover。

## 文件与本地依赖

- [index.html](index.html)：正文、静态拓扑、默认内容与无 JavaScript 说明。
- [style.css](style.css)：视觉、响应式布局、低动态样式与时间线动画。
- [site.js](site.js)：模块数据、三条路径、API、截图、复制、阅读进度与场景装配。
- [scene.js](scene.js)：三维几何、路径信号、选择、视角、渲染生命周期。
- `assets/scene.jpg`、`pose.png`、`telemetry.png`：项目已有界面截图。
- `assets/vendor/three/`：Three.js **0.180.0** 的本地 `three.module.min.js` 与 `three.core.min.js`；MIT 许可证保存在 [LICENSE](assets/vendor/three/LICENSE)。沿用仓库监看页面已有版本，不运行时请求 CDN。

## 内容边界

`Robot.sim(viewer=True)` 创建带窗口的 MuJoCo 仿真；`Robot.real(run)` 只连接已有真机内核并检查后端类型，不启动或使能硬件。页面仅展示代码，不执行动作。XYZ 用 m，关节用 rad，当前 `speed / acceleration / jerk` 仍约束关节轨迹。

当前内核使用 X5 六轴模型；RealSense / synthetic、Quest / iPhone 是具体接入，新增机械臂、相机或输入设备仍需适配。`Robot.record()` 默认只采集运行轨迹，多相机示教通过采集入口配置。

统一内核已做保持与小幅动作现场测试；后续新版轨迹与终点保持完成离线 / 仿真验证，仍需真机及完整任务验收。legacy 兼容链路仍保留。250 Hz 是 worker 配置频率；截图、流动光点与仿真结果不能写成实机性能或验收结论。更新内容时对照 [架构概览](../../docs/architecture/overview.md) 与 [Robot 接口说明](../../docs/reference/robot-primitives.md)。
