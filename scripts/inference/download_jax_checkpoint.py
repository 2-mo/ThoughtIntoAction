# pip install -U gcsfs tqdm -i https://pypi.tuna.tsinghua.edu.cn/simple
import os
import gcsfs
from tqdm import tqdm

bucket_path = "openpi-assets/checkpoints/pi05_base"
local_root = "./pi05_base"

# 每次读取 16 MiB
CHUNK_SIZE = 16 * 1024 * 1024

fs = gcsfs.GCSFileSystem(token="anon")

print("Scanning remote files...")
files = fs.find(bucket_path)

print(f"Found {len(files)} objects")

for i, remote in enumerate(files, 1):
    info = fs.info(remote)

    if info.get("type") == "directory":
        continue

    rel = os.path.relpath(remote, bucket_path)
    local = os.path.join(local_root, rel)

    os.makedirs(os.path.dirname(local), exist_ok=True)

    remote_size = info.get("size", 0)

    # 已经完整下载
    if os.path.exists(local):
        local_size = os.path.getsize(local)

        if local_size == remote_size:
            print(f"[{i}/{len(files)}] SKIP {rel}")
            continue

        # 本地文件比远程文件还大，说明不正常，重新下载
        if local_size > remote_size:
            print(f"[{i}/{len(files)}] RESTART {rel}")
            os.remove(local)
            local_size = 0
        else:
            print(
                f"[{i}/{len(files)}] RESUME {rel} "
                f"from {local_size / 1024**3:.2f} GiB"
            )
    else:
        local_size = 0
        print(
            f"[{i}/{len(files)}] DOWNLOAD {rel} "
            f"({remote_size / 1024**3:.2f} GiB)"
        )

    # 从远端断点位置继续读取
    with fs.open(remote, "rb") as src:
        src.seek(local_size)

        # 本地追加写入
        with open(local, "ab") as dst:
            with tqdm(
                total=remote_size,
                initial=local_size,
                unit="B",
                unit_scale=True,
                unit_divisor=1024,
                desc=rel,
                dynamic_ncols=True,
            ) as pbar:

                while True:
                    chunk = src.read(CHUNK_SIZE)

                    if not chunk:
                        break

                    dst.write(chunk)
                    pbar.update(len(chunk))

print("Done.")
