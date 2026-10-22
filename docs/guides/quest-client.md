# Quest USB 自动连接客户端

这份补丁让 Quest 客户端启动后直接连接 `127.0.0.1:63901`。电脑上的接收服务或 USB 转发尚未就绪时，客户端以 1、2、4、8 秒的间隔持续重试，之后保持最多 8 秒一次；单次连接等待最多 5 秒。TCP 连接断开后也使用这个流程，无需重新输入 IP、点击 Connect。

**当前交付的是源码补丁与构建入口，尚未使用 Unity 构建、安装 APK 或完成头显端到端验证。** 现有官方 APK 不会因为修改本仓库而获得这些功能。电脑端的 USB 检测、授权等待、转发和启动应用属于工作台连接管理，客户端补丁负责头显内的默认地址与 TCP 重试。

## 来源与边界

- 官方项目：[XRoboToolkit-Unity-Client-Quest](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest/tree/07a28333341629008187e988b73b48bced316092)，固定提交 `07a28333341629008187e988b73b48bced316092`。
- Unity 版本：`2021.3.45f2`，来自该提交的 `ProjectSettings/ProjectVersion.txt`。
- 上游采用 MIT 许可证，版权为 `Copyright (c) 2025 XR Robotics`。完整许可与修改记录保存在 [`scripts/setup/quest-client`](../../scripts/setup/quest-client/)。
- 新应用包名为 `com.thoughtintoaction.quest`，显示名为 `Thought into Action Quest`。官方应用 `com.xrobotoolkit.client.quest` 可以保留，不需要卸载，也不使用其签名。
- 不改变发送跟踪数据的开关，不缓存并重放离线时的控制或姿态消息，不启动机器人。网络连通不等于已启用遥操作。
- `TcpHandler.Close()` 明确断开后不会自动重连；手动 Connect / Reconnect 或下次重新启动应用才重新启用连接。原 IP 对话框的 Close 仅关闭对话框，语义保持不变。

首次连接新电脑时，Quest 的 USB 调试信任仍需在头显内确认，可勾选“始终允许此计算机”。ADB 尚未授权时，电脑不能代为批准这一步，详见 [Android ADB 文档](https://developer.android.com/tools/adb#Enabling)。

## 准备与构建

仓库只保存小型补丁和构建脚本。完整官方源码缓存位于 `outputs/dependencies/quest-client.git`，应用补丁后的 Unity 工程位于 `outputs/build/quest-client/source`，没有 `third_party` 运行时依赖。

准备源码需要 Git 和 Python 3.12（与本项目一致）。以下命令显式允许下载固定上游版本；它不会运行 Unity 或连接设备：

```bash
./run.sh quest build-client prepare --fetch
```

已有包含该提交的本地官方仓库时，可以离线导出。脚本只读取该提交，不修改原仓库，也不包含其中未提交的改动：

```bash
./run.sh quest build-client prepare --source-checkout /path/to/XRoboToolkit-Unity-Client-Quest
```

准备过程核对关键文件的 Git blob，应用 TCP 补丁并把自动连接组件接入原有 `UIOperate.Awake`。组件通过该对象已有的序列化 `TcpHandler` 引用连接，不按场景物体名称猜测，也不新建第二套网络实例。生成的 `thought-into-action-source.json` 记录来源、补丁和受管文件摘要。

构建入口沿用[官方 README 明确要求的 Unity `2021.3.45f2`](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest/blob/07a28333341629008187e988b73b48bced316092/README.md)，需安装 Android Build Support（包括 SDK、NDK、OpenJDK）并完成 Unity 自身的许可配置。它检查实际 `Assets/Main.unity` 场景中的 `UIOperate` / `TcpHandler` 连接关系，目标产物是 ARM64、IL2CPP APK。

**上游构建配置有已知不一致，尚不能承诺这套配方直接成功：**[依赖清单](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest/blob/07a28333341629008187e988b73b48bced316092/Packages/manifest.json)锁定 Shader Graph `14.0.9`，而[该包的官方配置](https://github.com/Unity-Technologies/Graphics/blob/500ec217bc8e5ca139c3c699df7ee21ad9a813f9/Packages/com.unity.shadergraph/package.json)声明 Unity `2022.3`。同时，客户端 README 报告用 Unity `2022.3` 构建会偶发空指针崩溃。`v1.0.1` 对应提交 `05ad3f081e64c400d8eda34c0efdf711c47f6b3a` 也有相同的版本与依赖文件。本补丁保留官方要求，构建前显示此问题并写入来源记录；未擅自降级渲染包或声称迁移编辑器已解决问题。下面命令是待实际验证的构建入口：

```bash
./run.sh quest build-client build \
  --unity "$HOME/Unity/Hub/Editor/2021.3.45f2/Editor/Unity"
```

默认产物为 `outputs/build/quest-client/ThoughtIntoAction-Quest-USB.apk`；同目录的 `.build.log` 保存 Unity 日志，`.build.json` 只在构建成功且实际生成 APK 后记录摘要。构建使用本机默认调试签名，适用于本地验证；它不包含官方签名，也不自动安装。不要将本机签名文件放进仓库。

工程和 APK 已存在时，脚本拒绝覆盖。补丁更新后可指定新的 `--project-dir outputs/build/quest-client-v2/source`；构建时传入同一个目录并用 `--output` 指定新 APK 路径。

## 安装后验证

完成构建后，选定已授权 Quest 的序列号再安装。以下是操作示例，本次修改没有执行这些命令：

```bash
adb devices -l
adb -s QUEST_SERIAL install -r outputs/build/quest-client/ThoughtIntoAction-Quest-USB.apk
```

日常从项目目录执行 `./run.sh`，在工作台 VR 窗口查看客户端版本、USB 授权、转发和连接状态。电脑端应复用已有客户端，避免官方版与新包同时连接。只有显示“自动连接版”（包名 `com.thoughtintoaction.quest`）时，才能期待这份补丁的自动 Connect 行为。

头显验证需覆盖以下顺序；观察网络状态即可，不需要机器人运动：

1. 工作台先启动，头显稍后开机并连上 USB；确认无需输入地址和点击 Connect。
2. 接收服务晚于客户端启动；确认首次失败后仍能恢复连接。
3. 拔插 USB、使 TCP 连接中断再恢复；确认重新连接，且恢复后没有旧消息补发。
4. 主动调用 `Close()` 后等待；确认保持断开，手动 Reconnect 后才恢复。

新包也实现了可选的 Android 启动参数。默认无需传参；若需禁用本次自动连接，可以在**应用未运行时**启动：

```bash
adb -s QUEST_SERIAL shell am start \
  -n com.thoughtintoaction.quest/com.unity3d.player.UnityPlayerActivity \
  --es quest_server_ip 127.0.0.1 --ez quest_autoconnect false
```

这些参数只在新启动时读取：`quest_server_ip` 必须是 IPv4，`quest_autoconnect` 默认 `true`。它们由本补丁实现，官方 APK 不支持；不承诺对已运行应用追加 intent 会改变连接。用户手动填写其他有效 IPv4 后，重试仍使用该手动地址。

## 已验证与待验证

本仓库的轻量验证覆盖本地 Git 导出、源文件身份校验、补丁应用、不覆盖用户工程、篡改/过期补丁拒绝构建，以及模拟 Unity 构建成功却没有 APK 时不报成功。TCP 补丁另在固定上游原文件上检查正向与反向应用。

这些验证不能代替 Unity C# 编译、Android 打包与头显网络测试。上述安装后验证通过前，不应将该 APK 标为已完成真机验证。
