"""Timestamp-indexed RGB storage. Video PTS never replaces capture timestamps."""
from pathlib import Path
import subprocess


class RGBVideoWriter:
    def __init__(self, path, width, height, fps, crf=20):
        self.shape = (height, width, 3)
        self.stderr = Path(str(path) + '.ffmpeg.log').open('wb')
        self.process = subprocess.Popen(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', str(fps),
            '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', str(crf),
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-n', str(path)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.stderr)

    def write(self, image):
        if image.shape != self.shape or str(image.dtype) != 'uint8':
            raise ValueError('RGB video frame shape/dtype changed')
        self.process.stdin.write(image.tobytes())

    def close(self):
        try:
            self.process.stdin.close()
            if self.process.wait(timeout=30):
                raise RuntimeError('H.264 encoding failed; inspect episode ffmpeg log')
        except BaseException:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait()
            raise
        finally:
            self.stderr.close()


def media_path(root, row):
    root = Path(root).resolve()
    path = (root / row.get('video', row.get('image', ''))).resolve()
    if not path.is_relative_to(root):
        raise ValueError('Frame path leaves episode')
    return path


def read_frame(root, row):
    import cv2
    path = media_path(root, row)
    if 'video' not in row:
        return cv2.imread(str(path))
    index = row.get('video_frame_index')
    if type(index) is not int or index < 0:
        raise ValueError('Invalid video frame index')
    capture = cv2.VideoCapture(str(path))
    try:
        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, image = capture.read()
        return image if ok else None
    finally:
        capture.release()


class SequentialFrameReader:
    """Decode increasing frame indices once, retaining only the current image."""

    def __init__(self):
        self.capture = None
        self.path = None
        self.index = -1
        self.image = None

    def read(self, root, row):
        import cv2
        path = media_path(root, row)
        if 'video' not in row:
            return cv2.imread(str(path))
        index = row.get('video_frame_index')
        if type(index) is not int or index < 0:
            raise ValueError('Invalid video frame index')
        if path != self.path:
            self.close()
            self.capture = cv2.VideoCapture(str(path))
            self.path, self.index = path, -1
        if index < self.index:
            raise ValueError('Video frame indices must be ordered')
        while self.index < index:
            ok, self.image = self.capture.read()
            if not ok:
                raise ValueError(f'Cannot decode source RGB frame {index}: {path}')
            self.index += 1
        return self.image

    def close(self):
        if self.capture is not None:
            self.capture.release()
        self.capture, self.path, self.image = None, None, None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
