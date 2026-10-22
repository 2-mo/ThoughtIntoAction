# 原生 VR 依赖

`bin/rollio-device-xrobot` 和 `bin/rollio-teleop-router` 沿用已有验证产物，未重新编译。Linux x86_64，版本 `1.6.0-dev.38+g057d4142`；Python iceoryx2 固定为 0.9.1。

运行不需要解压源码。`rollio-source.tar.gz` 单独保存原生链路所需源码快照、原始 Cargo.lock、第三方许可证、哈希清单、重建脚本及完整来源说明，避免把依赖源码混在应用代码中。它不包含机械臂运行记录或虚拟环境。

在工程根目录校验交付二进制和源码包：

```bash
sha256sum --check native/SHA256SUMS
```

需要修改原生 VR 或重建时，解压至空目录：

```bash
mkdir -p work
# 源码位于 work/vendor/rollio-reference。
tar -xzf native/rollio-source.tar.gz -C work
python3 work/vendor/prepare_rebuild.py --destination work/native-build
cd work/native-build
cargo build -p rollio-device-xrobot -p rollio-teleop-router --bins
```

需要 Rust/Cargo、Clang/Clang++、LLVM ar、C++20 和 Linux 开发依赖。详细条件见解压后的 `work/docs/PROVENANCE.md`。准备脚本避免调用 Git；旧原始构建脚本中的版本信息逻辑仅在新构建树内被替换。

本次不验证可重现构建，源码快照和既有二进制不宣称逐字节对应。替换二进制前需重新验证 ABI 和完整仿真链路，并更新 SHA256SUMS。

PC Service 的 `libPXREARobotSDK.so` 是外部动态库，不包含在本包中；路径由 `config/vr.json` 或 `XROBOT_SDK_LIBRARY` 提供。根目录保留 GPLv3，其他许可证随各自源码保留。
