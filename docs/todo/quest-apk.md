# Quest 自动连接 APK

状态：**暂缓构建和发布，先跑通现有真机流程**。源码补丁和构建入口已完成；没有可安装的新 APK，没有发布 Release。

## 已完成

- [x] 固定官方源码版本，增加默认 `127.0.0.1` 自动连接和断线退避重试。
- [x] 独立应用包名 `com.thoughtintoaction.quest`，保留原官方客户端。
- [x] `./run.sh` 支持头显晚开机、等待授权、自动补转发与启动客户端。
- [x] 准备官方源码并实际应用补丁：`outputs/build/quest-client/source`；来源记录在工程内的 `thought-into-action-source.json`。
- [x] `./run.sh quest client install / package`：校验 APK 包名、SHA256 和来源记录，支持私有 GitHub Release 下载。12 项模拟测试通过；没有真实下载或安装。

## 待完成

- [ ] 在项目 `outputs/tools` 中配置 Unity 官方 CLI 和 Editor，不使用其他项目环境。
- [ ] 用户在官方浏览器页面完成一次 Unity 登录，按自身许可资格完成激活；不在聊天、仓库或日志记录账号密码、令牌、许可文件和签名私钥。
- [ ] 安装 Unity `2021.3.45f2`（changeset `88f88f591b2e`）及 Android 子模块。Editor 下载约 3.26 GB，完整 Android 构建环境还需额外空间。
- [ ] 实际构建并解决编译错误。保留官方 2021.3.45f2 配方；上游 Shader Graph 14.0.9 的版本声明不一致尚待实际构建验证，不直接升级到上游报告存在崩溃的 2022.3。
- [ ] 头显已授权后安装新包；验证先工作台后开机、首次连接失败、拔插重连和主动断开。安装新包后退出旧客户端，避免同时连接。
- [ ] 发布 GitHub Release：APK、SHA256 校验文件、来源与构建元数据；源码正常提交，二进制放 Release。未完成头显验证的版本标为预发布。
- [ ] 在另一台电脑按同一安装命令下载并核对，验证迁移流程。

## 下次从这里继续

现有源码不需要重新下载。构建与安装说明见 [Quest 客户端指南](../guides/quest-client.md)。

Unity 最新官方 CLI 已提供浏览器登录和 Personal 激活接口。后续先审阅官方安装脚本，再按实际安装版本的 `--help` 核对选项；不要沿用“Personal 只能通过 Hub 图形界面激活”的旧说法。设置 `UNITY_CLI_HOME` 指向项目目录，不改写 `HOME`。

参考顺序（本次尚未执行）：

```bash
unity auth login
unity license status
# 确认许可资格并阅读条款后，按官方 CLI 完成激活。
unity install 2021.3.45f2 --changeset 88f88f591b2e --module android --child-modules \
  --download-directory /path/to/ThoughtIntoAction/outputs/tools/unity/2021.3.45f2
./run.sh quest build-client build --unity /path/to/Editor/Unity
```

来源：[Unity CLI 安装](https://docs.unity.com/en-us/unity-cli/use-unity-cli)、[命令参考](https://docs.unity.com/en-us/unity-cli/unity-cli-reference)、[发布说明](https://docs.unity.com/en-us/unity-cli/release-notes)、[官方认证与许可说明](https://github.com/Unity-Technologies/skills/blob/main/skills/unity-cli/references/auth-license-cloud.md)、[Editor 版本](https://unity.com/releases/editor/whats-new/2021.3.45f2)。
