# USB-CAN 故障排查

网页提示 `Failed to open USB-CAN adapter with interface name can0`，且
`ip link show can0` 返回 `Device "can0" does not exist` 时，说明故障发生在
网页连接机械臂之前：Linux 尚未创建 SocketCAN 接口 `can0`。

本项目使用的 `MCS USB2CAN`（USB ID `16d0:117e`）是串口式 SLCAN 设备。
系统通常先将它识别为 `/dev/ttyACM0`，再由 `slcand` 创建 `can0`。因此，
`lsusb` 能看到设备但 `ip -details link show type can` 没有输出，通常表示 USB
设备已识别，但 SLCAN 桥尚未启动或在重新插拔后失效。运行：

```bash
cd /home/r2/work/projects/embodied/arx5_test
./arx5 can-up
```

脚本会通过 `/dev/serial/by-id/usb-ARX_USB2CAN_*-if00` 找到稳定的串口设备，
启动 `slcand` 并拉起 `can0`。随后验证：

```bash
ip -details link show can0
candump can0
```

确认 `can0` 存在后，再启动网页服务。如果 `can-up` 失败，可进一步检查：

```bash
lsusb
ls -l /dev/serial/by-id/
sudo dmesg | tail -n 50
```

普通用户直接运行 `dmesg` 出现 `Operation not permitted` 只是内核日志权限
限制，并不表示 USB-CAN 本身故障。

USB-CAN 重新插拔后只需再次运行 `./arx5 can-up`。脚本会找到稳定的 ARX
USB2CAN by-id，清理与 `can0` 对应的失效 `slcand`，重新创建接口并显示错误计数。

## 接口被占用

`/tmp/arx5-control-can0.lock` 防止两个本项目控制器同时占用 CAN。终端命令方式
标定前必须先关闭网页服务，释放 D455 和 `can0`；网页入口会自动完成这一段释放流程。

## 网页改动没有生效

控制台不使用 `?v=N` 版本号。修改 `src/frontend/src/` 后必须先运行
`cd src/frontend && npm run build`。控制 FastAPI 的同源静态服务对构建后的
`.js`/`.css` 发送 `no-cache, must-revalidate`，对 SPA 入口发送 `no-store`，
因此构建完成后普通刷新即可加载新版本。
