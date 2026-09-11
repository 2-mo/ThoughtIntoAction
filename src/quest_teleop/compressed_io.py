"""Lossless JSONL compression; legacy plain files remain readable."""
import gzip
import hashlib
from pathlib import Path
import shutil


def open_jsonl(path):
    path = Path(path)
    if path.is_file():
        return path.open("rt", encoding="utf-8")
    return gzip.open(str(path) + ".gz", "rt", encoding="utf-8")


def compress_jsonl(path):
    path = Path(path)
    target = Path(str(path) + ".gz")
    if not path.exists():
        if not target.is_file():
            raise FileNotFoundError(path)
        return target
    temporary = Path(str(target) + ".partial")
    source_hash = hashlib.sha256()
    with path.open("rb") as source, gzip.open(temporary, "wb", compresslevel=3) as dest:
        while chunk := source.read(1024 * 1024):
            source_hash.update(chunk)
            dest.write(chunk)
    restored_hash = hashlib.sha256()
    with gzip.open(temporary, "rb") as source:
        while chunk := source.read(1024 * 1024):
            restored_hash.update(chunk)
    if source_hash.digest() != restored_hash.digest():
        raise RuntimeError("Compressed JSONL verification failed")
    temporary.replace(target)
    path.unlink()
    return target
