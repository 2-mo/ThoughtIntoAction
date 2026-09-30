# 中文一屏架构图

用浏览器打开本目录的 [index.html](index.html)。首页是一张 16:9 架构海报，完整适应当前屏幕，不需要滚动或切页。点击部件查看职责与源码，顶部支持全屏和 3200 × 1800 PNG 导出。架构连线与文字由网页绘制，部件和界面图标统一使用 Font Awesome Free。页面读取本地 `catalog.js` 和 `vendor/fontawesome/icons.js`，无 CDN、网络依赖或设备连接。

总图按「控制源 → arx 原生控制核心 → X5 真机 / 仿真」组织，上方展示算法与工作流扩展，下方串起相机、观测、采集、导出。实线表示主线或数据流，虚线明确标记独立扩展与尚未全部接入的边界；B601 独立设备插件单独说明。

原来的 [详细导览](explorer.html) 从首页右上角「详细资料」进入，包含：

- **架构总览**：arx 工作台、迁入的独立操作与旧兼容入口；点击节点查看职责、源码位置和边界。
- **功能与插件**：按功能找代码，筛选真实注册表，搜索和复制组件命令。
- **采集与标定**：两条数据链路的 action 定义，以及两个手眼包当前各自负责的工作。
- **目录导航**：实际目录按角色分组，支持名称和中文职责搜索。
- **整理建议**：明确标记为后续建议，不代表已经实现的目录或接口。

当前行为以源码为准：`workbench` 显式使用 arx 工作台；裸 `run.sh` 和 `启动遥操.sh` 仍保留旧 presets 流程。此文档没有更改任何控制入口。

在仓库根目录更新目录与注册表快照：

```bash
python3 scripts/docs/build_architecture.py
```

生成器仅使用 Python 3.11+ 标准库，读取三个注册表、组件命令配置与目录列表，不导入插件或硬件 SDK。生成的 `catalog.js` 可随文档提交。职责说明与运行链路分别保存在 `index.html`（总图）和 `explorer.html`（详细索引）中，架构变化时需要同步更新。

图标采用 [Font Awesome Free 6.7.2](https://fontawesome.com/)（Fonticons, Inc.，SVG 图标为 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)）。仅打包 23 个使用到的图标，原始路径和 viewBox 保持不变，通过本地 symbol 按原比例渲染；海报导出内嵌这些 symbol，并保留署名。来源、文件哈希和完整许可见 [SOURCE.json](vendor/fontawesome/SOURCE.json) 与 [LICENSE.txt](vendor/fontawesome/LICENSE.txt)。

如需重新生成图标包，在已安装前端锁定依赖的仓库中执行：

```bash
python3 scripts/docs/build_architecture_icons.py
```

生成后浏览器不依赖 `node_modules`，两个页面共用这份本地图标包。

如果希望通过 HTTP 预览，在仓库根目录执行：

```bash
python3 -m http.server 8897 --bind 127.0.0.1
```

随后访问 <http://127.0.0.1:8897/docs/architecture/>，停止预览使用 Ctrl+C。此方式也支持网页里的相对源码和文档链接。

浏览器检查和详细导览截图保存在 `outputs/architecture/`；一屏布局截图、检查结果和海报导出位于 `outputs/architecture/poster/`，属于本机生成产物。
