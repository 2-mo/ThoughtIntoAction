"""Recorded samples, read-only review, and a cancellable arm replay state machine."""
import base64
from bisect import bisect_right
import math
import time

import numpy as np

from .datasets import load_episode, image_path
from .video import media_path, read_frame
from robot_core.model import RobotModel
from robot_core.project import load_config, resolve_path


class RecordedEpisode:
    def __init__(self, path):
        self.root, self.meta, self.states, self.frames = load_episode(path)
        if self.meta.get('lifecycle') != 'complete' or self.meta.get('outcome') == 'discarded':
            raise ValueError('只能回放完整保存且未丢弃的数据')
        if len(self.states) < 2:
            raise ValueError('上一条缺少机械臂状态数据')
        model = RobotModel(resolve_path(load_config()['robot']['urdf']), eef=self.meta['robot']['eef_frame'])
        if model.metadata() != self.meta['robot']:
            raise ValueError('回放数据与当前机械臂模型不一致')
        stamps = [row['host_monotonic_ns'] for row in self.states]
        if any(type(t) is not int for t in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError('机械臂记录时间不连续递增')
        self.times = [(t - stamps[0]) / 1e9 for t in stamps]
        self.duration = self.times[-1]
        for row in self.states:
            model.joints(row['actual_pos'])
            grip = row['gripper_actual_pos']
            if not math.isfinite(grip) or not 0 <= grip <= .082:
                raise ValueError('记录的夹爪开度无效')
        self.frame_times = [(row['host_received_monotonic_ns'] - stamps[0]) / 1e9 for row in self.frames]
        if any(b <= a for a, b in zip(self.frame_times, self.frame_times[1:])):
            raise ValueError('视频记录时间不连续递增')
        for frame in self.frames:
            if not media_path(self.root, frame).is_file():
                raise ValueError('录制视频有缺失图片')
        self._image_cache = (None, None)
        self.path_points = [model.fk(row["actual_pos"])[:3, 3].tolist() for row in self.states]

    def row(self, seconds):
        index = max(0, min(len(self.times) - 1, bisect_right(self.times, seconds) - 1))
        return self.states[index], index

    def review(self, seconds):
        row, index = self.row(seconds)
        image = None
        if self.frames:
            frame = max(0, min(len(self.frames) - 1, bisect_right(self.frame_times, seconds) - 1))
            if self._image_cache[0] != frame:
                import cv2
                raw = read_frame(self.root, self.frames[frame])
                if raw is None:
                    raise ValueError('录制视频图片损坏')
                if 'video' in self.frames[frame]:
                    ok, buffer = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if not ok: raise ValueError('回放视频编码失败')
                    encoded = buffer.tobytes()
                else:
                    encoded = image_path(self.root, self.frames[frame]['image']).read_bytes()
                if len(encoded) > 5_500_000: raise ValueError('单帧图像过大，无法通过监看接口回放')
                mime = 'image/png' if encoded.startswith(b'\x89PNG') else 'image/jpeg'
                self._image_cache = (frame, base64.b64encode(encoded).decode(), mime)
            image = self._image_cache[1]
        start = max(0, bisect_right(self.times, seconds - 5) - 1)
        submission = row.get('submission') or {}
        history = [{'time': self.times[i] * 1000,
                    'values': [*(self.states[i].get('actual_torque') or [None] * 6), self.states[i].get('gripper_actual_torque')]}
                   for i in range(start, index + 1)]
        stride = max(1, (index + 1) // 500)
        return {'actual_vel': row.get('actual_vel'), 'target_vel': row.get('sdk_cmd_vel'),
                'actual_torque': row.get('actual_torque'), 'gripper_actual_torque': row.get('gripper_actual_torque'),
                'gripper_actual_vel': row.get('gripper_actual_vel'),
                'target_q': submission.get('pos'), 'target_gripper_m': submission.get('gripper_pos'),
                'gripper_max': self.meta.get('control', {}).get('gripper_width_m', .082),
                'torque_history': history, 'path_points': self.path_points[:index + 1:stride],
                'record_format': 'mcap',
                'depth_recorded': self.meta.get('depth_recorded', False),
                'episode': self.meta['id'], 'duration_s': self.duration, 'position_s': seconds,
                'sample_index': index, 'sample_count': len(self.states),
                'model_hash': self.meta['robot']['urdf_sha256'],
                'q': row['actual_pos'], 'gripper_m': row['gripper_actual_pos'], 'image_base64': image,
                'image_mime': self._image_cache[2] if image else None}


class WebReplay:
    def __init__(self, episode):
        self.episode = episode
        self.position = 0.
        self.started = time.monotonic()
        self.playing = True
        self.speed = 1.
        self.generation = time.monotonic_ns()

    def seconds(self):
        value = self.position + ((time.monotonic() - self.started) * self.speed if self.playing else 0)
        return min(self.episode.duration, value)

    def control(self, action, position=None, speed=None):
        value = self.seconds()
        if action == 'seek':
            if type(position) not in (int, float) or not math.isfinite(position):
                raise ValueError('无效回放时间')
            value = min(self.episode.duration, max(0., position))
        elif action == 'speed':
            if type(speed) not in (int, float) or speed not in (.25, .5, 1, 2):
                raise ValueError('回放倍速须为 0.25、0.5、1 或 2')
            self.speed = speed
        elif action == 'play':
            if value >= self.episode.duration: value = 0.
            self.playing = True
        elif action == 'pause':
            self.playing = False
        else:
            raise ValueError('未知网页回放操作')
        self.position, self.started = value, time.monotonic()

    def frame(self):
        position = self.seconds()
        if position >= self.episode.duration:
            self.position, self.playing = position, False
        return {**self.episode.review(position), 'playing': self.playing, 'speed': self.speed, 'generation': self.generation}


class RobotReplay:
    def __init__(self, episode, now=None):
        self.episode = episode
        self.phase = 'positioning'
        self.submitted = False
        self.last_index = None
        self.started = time.monotonic() if now is None else now
        self.position = 0.

    def status(self):
        return {'episode': self.episode.meta['id'], 'phase': self.phase,
                'position_s': self.position, 'duration_s': self.episode.duration}

    def tick(self, arm, now):
        state = arm.snapshot()
        if self.submitted and (state.get('table_safety') or {}).get('blocked'):
            raise RuntimeError('真机回放被桌面安全边界拦截')
        if self.phase == 'positioning':
            seconds = 0.
            if now - self.started > 60: raise RuntimeError('回放起点定位超时')
        else:
            seconds = min(self.episode.duration, now - self.started)
            if now - self.started > self.episode.duration + 60: raise RuntimeError('回放末点跟随超时')
        row, index = self.episode.row(seconds)
        error = np.max(np.abs(np.asarray(state['actual_pos']) - row['actual_pos']))
        settled = error < .03 and np.max(np.abs(state['actual_vel'])) < .03
        settled = settled and abs(state['gripper_actual_pos'] - row['gripper_actual_pos']) < .005
        trajectory = state.get('trajectory') or {}
        settled = settled and trajectory.get('finished', False) and not trajectory.get('stopping', True)
        if self.phase == 'positioning' and self.submitted and settled:
            self.phase, self.started = 'playing', now
        elif self.phase == 'playing' and seconds >= self.episode.duration and self.last_index == len(self.episode.states) - 1 and settled:
            self.phase = 'finished'
            return False
        arm.set_joint_positions(row['actual_pos'], profile='preset' if self.phase == 'positioning' else 'teleop')
        arm.set_catch_pos(row['gripper_actual_pos'])
        self.submitted = True
        self.last_index = index
        self.position = seconds
        return True
