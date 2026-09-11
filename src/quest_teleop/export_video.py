"""Stream RGB to H264 while retaining the pinned LeRobot statistics samples."""
from pathlib import Path


class StreamingVideoWriter:
    """LeRobot image-writer interface; bounded encoder buffers, no PNG round trip.

    LeRobot v2.1 computes image statistics from sample_indices(length). Only those
    lossless PNGs are needed. Its encoder reuses the already finalized MP4.
    """

    def __init__(self, path, shape, frame_count, fps, preset, threads):
        import av
        from lerobot.common.datasets.compute_stats import sample_indices

        self.count, self.expected = 0, frame_count
        self.samples = set(sample_indices(frame_count))
        self.shape = shape
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.container = av.open(str(self.path), 'w', options={'movflags': '+faststart'})
        try:
            self.stream = self.container.add_stream('h264', fps,
                options={'crf': '20', 'preset': preset, 'g': '2'})
            self.stream.height, self.stream.width = shape[:2]
            self.stream.pix_fmt = 'yuv420p'
            self.stream.codec_context.thread_count = threads
        except BaseException:
            self.stop()
            raise

    def save_image(self, image, fpath):
        import av
        from PIL import Image

        if self.container is None or self.count >= self.expected:
            raise ValueError('Unexpected frame after video completion')
        if image.shape != self.shape or str(image.dtype) != 'uint8':
            raise ValueError('RGB shape/dtype differs from episode metadata')
        for packet in self.stream.encode(av.VideoFrame.from_ndarray(image, format='rgb24')):
            self.container.mux(packet)
        if self.count in self.samples:
            # Preserve the exact upstream sampling/statistics, with cheap lossless PNGs.
            Image.fromarray(image).save(fpath, compress_level=1)
        self.count += 1

    def wait_until_done(self):
        if self.count != self.expected:
            raise ValueError(f'Expected {self.expected} video frames, received {self.count}')
        if self.container is not None:
            try:
                for packet in self.stream.encode():
                    self.container.mux(packet)
            finally:
                self.stop()

    def stop(self):
        if self.container is not None:
            container, self.container = self.container, None
            container.close()
