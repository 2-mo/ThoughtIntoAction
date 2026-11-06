# RealMan RM65

- [MuJoCo 仿真：无需 VR 的脚本演示、简化夹爪与 Quest 输入](simulation.md)
- [真机平台接入与 Quest 遥操](platform.md)
- [当前版本真机复测步骤与通过条件](retest.md)
- [第二台 RM65-BI `.18` 的独立配置与核对](robot18.md)

## 示教器远程访问（Windows）

两个 PowerShell 脚本分别连接现场的两台机械臂，只建立 SSH 端口转发，不发送机械臂指令。文件放在文档目录便于下载；必须在 Windows 本机执行，不是在 Linux SSH 终端执行。当前配置为 SSH 主机 `guest@100.99.53.55`，变化时修改脚本顶部的 `$SshTarget`。

| 脚本 | 机械臂地址 | Windows 浏览器地址 |
| --- | --- | --- |
| [connect-rm65-18.ps1](connect-rm65-18.ps1) | `192.168.1.18` | `http://127.0.0.1/#/login` |
| [connect-rm65-19.ps1](connect-rm65-19.ps1) | `192.168.1.19` | `http://127.0.0.2/#/login` |

## 运行

将需要的 `.ps1` 文件下载或复制到 Windows，例如 `C:\Users\ibing\Downloads`。在 Windows PowerShell 执行：

```powershell
cd "$HOME\Downloads"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\connect-rm65-18.ps1
```

连接另一台时，在另一个 PowerShell 窗口执行：

```powershell
cd "$HOME\Downloads"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\connect-rm65-19.ps1
```

`ExecutionPolicy Bypass` 仅作用于本次启动的 PowerShell 进程，不修改系统执行策略。脚本使用 Windows OpenSSH Client；若提示缺少 `ssh.exe`，先在 Windows 可选功能中安装 OpenSSH 客户端。

保持对应窗口打开，无输出且没有返回提示符通常是正常等待状态。浏览器手动输入表中地址，不使用旧的 `18090` 或应用自动生成的预览地址。默认账号 `user`、密码 `123` 仅适用于未修改凭据的设备。两个脚本可独立或同时运行，按 `Ctrl+C` 关闭对应隧道。

## 前提与排查

- Linux 必须能访问目标机械臂；现场当前使用 `eno1` 的附加地址 `192.168.1.100/24`。该地址为临时配置，重启会失效。若缺失，在 Linux 执行 `sudo ip address add 192.168.1.100/24 dev eno1`；已存在时不要重复添加，不删除原 SSH 地址或修改默认路由。
- 示教器网页、接口、实时反馈分别使用 `80`、`8090`、`8060`，三个端口都需转发。当前示教器会拼接接口端口，因此保留本地 HTTP 默认端口，用两个回环地址区分设备。
- `Address already in use` 或端口绑定失败：先关闭此前占用同一地址、端口的 SSH 隧道。不要与旧的通配地址转发混用。
- `No route to host`：SSH 收到了请求，但 Linux 无法连接该机械臂。检查机械臂电源、初始化、网线及 IP；SSH 保活无法修复这段链路。单台关机不影响另一台。
- 页面可打开但通信异常：确认运行的是包含三个端口的脚本，并强制刷新页面。仍异常时保留 SSH 报错及浏览器网络请求错误。

故障机械臂先只查看系统信息、完整错误码与版本，不执行运动、清除报警或设置零位。厂家示教器连通不代表本项目已完成 RealMan 驱动或 VR 联动适配。
