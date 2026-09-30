"""Batched CPU RGB decoding with a bounded number of process-pool results."""
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from itertools import groupby
import multiprocessing

import numpy as np

from episode_io.video import SequentialFrameReader, media_path


class RGBBatchDecoder:
    def __init__(self, backend):
        self.backend = backend
        self.video, self.path = None, None
        self.reader = SequentialFrameReader()
        if backend == 'torchcodec':
            try:
                import torch
                from torchcodec.decoders import VideoDecoder
            except (ImportError, RuntimeError) as exc:
                raise RuntimeError('TorchCodec unavailable: run scripts/datasets/setup_export.sh and install '
                    'FFmpeg shared libraries, or use --decoder opencv. ' + str(exc)) from exc
            # Each process gets one decoding thread; encoding has its own thread budget.
            torch.set_num_threads(1)
            self.decoder_type = VideoDecoder

    def read(self, root, frames):
        groups = []
        for video, selected in groupby(frames, key=lambda frame: frame.get('video')):
            selected = list(selected)
            if self.backend == 'torchcodec' and video is not None:
                path = media_path(root, selected[0])
                if path != self.path:
                    self.video = self.decoder_type(str(path), device='cpu',
                        dimension_order='NHWC', num_ffmpeg_threads=1)
                    self.path = path
                indices = [frame['video_frame_index'] for frame in selected]
                if any(type(index) is not int or index < 0 for index in indices):
                    raise ValueError('Invalid video frame index')
                groups.append(self.video.get_frames_at(indices=indices).data.numpy())
            else:
                pixels = []
                for frame in selected:
                    image = self.reader.read(root, frame)
                    if image is None:
                        raise ValueError('Cannot decode source RGB')
                    pixels.append(image[..., ::-1].copy())
                groups.append(np.stack(pixels))
        return groups[0] if len(groups) == 1 else np.concatenate(groups)

    def close(self):
        self.reader.close()
        self.video, self.path = None, None


_worker_decoder = None


def decode_batch(backend, root, frames):
    global _worker_decoder
    if _worker_decoder is None or _worker_decoder.backend != backend:
        if _worker_decoder is not None:
            _worker_decoder.close()
        _worker_decoder = RGBBatchDecoder(backend)
    return _worker_decoder.read(root, frames)


class BatchFrameReader:
    """Keep at most workers pending batches plus the currently consumed batch.

    Results are consumed in submission order, regardless of worker completion.
    The same pool/decoders are reused across episodes. workers=0 runs in-process.
    """

    def __init__(self, backend='torchcodec', workers=0, batch_size=32):
        if backend not in ('torchcodec', 'opencv') or workers < 0 or batch_size < 1:
            raise ValueError('Invalid decoder, worker count or batch size')
        self.backend, self.workers, self.batch_size = backend, workers, batch_size
        self.pool, self.decoder = None, None

    def __enter__(self):
        if self.workers:
            # Spawn is safe after PyTorch/FFmpeg have started native threads.
            self.pool = ProcessPoolExecutor(max_workers=self.workers,
                mp_context=multiprocessing.get_context('spawn'))
        else:
            self.decoder = RGBBatchDecoder(self.backend)
        return self

    def frames(self, root, frames):
        batches = (frames[start:start+self.batch_size] for start in range(0, len(frames), self.batch_size))
        if self.pool is None:
            for batch in batches:
                yield from self.decoder.read(root, batch)
            return
        pending = deque()
        try:
            for batch in batches:
                pending.append(self.pool.submit(decode_batch, self.backend, root, batch))
                if len(pending) == self.workers:
                    break
            while pending:
                pixels = pending.popleft().result()
                batch = next(batches, None)
                if batch is not None:
                    pending.append(self.pool.submit(decode_batch, self.backend, root, batch))
                yield from pixels
        finally:
            for future in pending:
                future.cancel()

    def __exit__(self, *exc):
        if self.pool is not None:
            self.pool.shutdown(wait=True, cancel_futures=True)
        if self.decoder is not None:
            self.decoder.close()
