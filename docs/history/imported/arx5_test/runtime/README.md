# 运行状态

这里只保存可重新生成、无需人工管理的本机状态，不进入 Git：

```text
https/             本地 CA 和网页服务证书
cache/kdl-urdf/    自动生成的 KDL 兼容 URDF
web-pairing-token  网页控制配对令牌
web-lan.pid        服务运行期间的进程记录
```

标定、数据集和诊断输出分别位于 `calibrations/`、`~/work/data/arx5/` 和 `outputs/`。
