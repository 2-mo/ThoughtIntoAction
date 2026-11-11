"""msgpack + numpy extension used by the π0.5 policy server (msgpack_numpy format).

Encoded shapes follow the server exactly:

    ndarray      {b"__ndarray__": True,  b"data": bytes, b"dtype": str, b"shape": tuple}
    numpy scalar {b"__npgeneric__": True, b"data": scalar, b"dtype": str}
"""
from __future__ import annotations

import numpy as np

try:  # prefer the reference implementation when the environment provides it
    import msgpack_numpy as _numpy_pack
except ImportError:  # pragma: no cover - the fallback below is exercised in tests
    _numpy_pack = None


def _encode(value):
    if isinstance(value, np.ndarray):
        return {b"__ndarray__": True, b"data": value.tobytes(), b"dtype": str(value.dtype).encode(),
                b"shape": value.shape}
    if isinstance(value, np.generic):
        return {b"__npgeneric__": True, b"data": value.item(), b"dtype": str(value.dtype).encode()}
    raise TypeError(f"cannot encode {type(value)!r}")


def _dtype(value):
    """msgpack_numpy writes the dtype as a str; our own encoder writes bytes."""
    return np.dtype(value.decode() if isinstance(value, bytes) else value)


def _decode(mapping):
    if mapping.get(b"__ndarray__") is True:
        array = np.frombuffer(mapping[b"data"], dtype=_dtype(mapping[b"dtype"]))
        return array.reshape(mapping[b"shape"]).copy()
    if mapping.get(b"__npgeneric__") is True:
        return _dtype(mapping[b"dtype"]).type(mapping[b"data"])
    return mapping


def packb(payload) -> bytes:
    import msgpack

    return msgpack.packb(payload, default=_encode, use_bin_type=True)


def unpackb(payload: bytes):
    import msgpack

    return msgpack.unpackb(payload, object_hook=_decode, raw=False)


def have_reference() -> bool:
    """True when msgpack_numpy is installed (the server's own encoder)."""
    return _numpy_pack is not None
