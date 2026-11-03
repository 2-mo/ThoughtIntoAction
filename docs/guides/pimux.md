# PiMux 终端工作台

2026-11-03 按用户要求，在当前 Linux 用户安装 [2-mo/pimux](https://github.com/2-mo/pimux) v0.8.0。安装包来自该版本 GitHub Release，已通过随包 SHA256SUMS 校验；原始安装包保存在 Git 忽略的 `outputs/installers/pimux-v0.8.0/`。

启动与版本检查：

```bash
pimux
pimux --version
```

当前终端若尚未读取新的 PATH，可直接执行 `/home/guest/.local/bin/pimux`；新登录终端自动加入用户命令目录。主界面按 n 创建会话；进入会话后 Ctrl-a 再按 h 打开菜单，Ctrl-a 再按 d 离开并保留任务。

安装使用官方 setup-tmux.sh，命令与配置分别位于用户的 `~/.local/bin`、`~/.config/pimux`；其 Python 依赖单独放在 `~/.local/share/pimux/venv`，不使用本项目环境。Starship v1.27.0 放在用户命令目录。安装器为 `.bashrc` 和 `.profile` 创建备份后追加用户命令目录的 PATH 设置，没有替换已有内容。

PiMux 默认使用独立的 pimux tmux server；本项目 `./run.sh` 的 emb 工作台保持原有控制入口，不依赖 PiMux。两套工具的快捷键不同：项目工作台通常为 Ctrl-b，PiMux 为 Ctrl-a。PiMux 是终端管理工具，不控制机器人，也不替代 VR 传输、设备后端或控制内核。

已验证 `pimux --version` 返回0.8.0、帮助可读、独立环境可导入 blessed 与 rich。未启动交互式 PiMux 会话，也未据此执行机器人动作。
