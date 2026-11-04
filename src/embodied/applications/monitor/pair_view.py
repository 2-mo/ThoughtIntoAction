"""Combine two standard observer streams for display, never for control."""
import select
import time

import numpy as np

from embodied.contracts.runtime import RuntimeReader
from embodied.applications.workstation.pair_state import active_pair
from .realman_view import RealManView


class PairView:
    def __init__(self, *, finder=active_pair, reader_factory=RuntimeReader, model_factory=None):
        self.finder, self.reader_factory = finder, reader_factory
        self.identity = None
        self.readers, self.ticks = {}, {}
        self.discovery_at = 0.
        self.error = None
        self.sequence = 0
        self.view = RealManView(reader=self.snapshot, **({'model_factory': model_factory} if model_factory else {}))

    def close(self):
        for reader in self.readers.values():
            reader.close()
        self.readers.clear()
        self.ticks.clear()

    def snapshot(self):
        if self.identity is None or len(self.ticks) != 2:
            return None
        arms = []
        for side in ('left', 'right'):
            tick = self.ticks[side]
            stamp = tick['timing']['feedback_received_ns']
            if (time.monotonic_ns()-stamp)/1e9 > .15:
                return None
            feedback = tick['feedback']
            command = tick.get('command') or {}
            arms.append({'hand': side, 'host': side, 'q': feedback['q'],
                'dq': feedback.get('dq'), 'target_dq': command.get('dq'),
                'target_q_deg': np.rad2deg(command.get('q', feedback['q'])).tolist(),
                'feedback_at': stamp/1e9, 'fault': feedback.get('fault', ''),
                'gripper_width_m': feedback.get('gripper_width'),
                'gripper_target_m': command.get('gripper_width'),
                'run': self.identity['runs'][side]})
        quest = any(reader.descriptor.get('native') for reader in self.readers.values())
        return {'schema': 1, 'stream_id': self.identity['runtime_id'], 'sequence': self.sequence,
            'sample_monotonic_ns': min(int(a['feedback_at']*1e9) for a in arms),
            'arms': arms, 'xr_fresh': None, 'paused': None,
            'input': 'quest' if quest else 'idle',
            'reason': ('统一内核反馈；左X / 右A切换跟随，Grip相对重定位' if quest
                       else '统一内核反馈；无输入，使用网页 / 动作 / 策略'),
            'source_kind': 'hardware' if all(r.descriptor.get('backend') == 'real' for r in self.readers.values()) else 'virtual'}

    def step(self, scene):
        if time.monotonic() >= self.discovery_at:
            self.discovery_at = time.monotonic()+1.
            found = self.finder()
            if found != self.identity:
                self.close()
                self.identity = found
                self.view.latest = None
                self.view.stream_id = None
                if scene:
                    scene.set_model(None)
        if self.identity is None:
            return False
        try:
            for side, run in self.identity['runs'].items():
                if side not in self.readers:
                    self.readers[side] = self.reader_factory(run, 'latest', timeout=.1)
            # Drain ready messages without allowing a slow arm to block the
            # other observer. Freshness is checked separately for each side.
            for side, reader in self.readers.items():
                for _ in range(32):
                    if not select.select([reader.sock], [], [], 0)[0]:
                        break
                    tick = reader.read()
                    if tick.get('terminal') or tick.get('error'):
                        raise EOFError(tick.get('error') or f'{side}内核已退出')
                    if tick.get('feedback'):
                        self.ticks[side] = tick
                        self.sequence += 1
            if not self.view.step(scene):
                self.error = '等待双臂新鲜反馈；不会显示陈旧姿态'
                if scene:
                    scene.set_stream_error(self.error)
            else:
                self.error = None
        except (OSError, ValueError, RuntimeError, EOFError, KeyError, TypeError) as exc:
            self.close()
            self.view.step(scene)
            self.error = f'等待双臂内核反馈：{exc}'
            if scene:
                scene.set_stream_error(self.error)
        return True

    def monitor(self):
        if self.identity is None:
            return None
        payload = self.view.monitor() or {}
        payload.update(runtime_id=self.identity['runtime_id'], runtime_directory=None,
            runtime_monitor=True, connections={}, collection_count=None, recording=None,
            runtime_runs=self.identity['runs'], monitor_message=self.error or '双臂统一内核只读反馈；底座为示意布局',
            input_error='手柄状态请查看Quest面板；不由姿态反馈推断控制权', input=None,
            button_layout={
                'left': {'stick': 'L3', 'holds': [], 'keys': {'X': '跟随 / 保持', 'Y': '长按初始化张开'}},
                'right': {'stick': 'R3', 'holds': [], 'keys': {'A': '跟随 / 保持', 'B': '长按初始化张开'}}},
            collection={'state': '按侧采集', 'task': 'collect run --run <left/right RUN> --task <任务>', 'control_enabled': False},
            collection_hint='两侧分别使用现有采集/策略接口；尚无同步双臂策略协议。')
        hands, stamps, device_stamps = {}, [], []
        now = time.monotonic_ns()
        for side, tick in dict(self.ticks).items():
            inp = tick.get('input') or {}
            stamp = inp.get('sample_monotonic_ns')
            valid = stamp is not None and 0 <= now-stamp < 150_000_000 and not inp.get('error')
            hands[side] = inp.get('hands', {}).get(side) if valid else None
            if hands[side] is not None:
                stamps.append(stamp)
                device_stamps.append(inp.get('device_timestamp_ns', 0))
        if stamps:
            payload.update(input={'hands': hands, 'sample_monotonic_ns': min(stamps),
                'device_timestamp_ns': max(device_stamps)}, input_age_s=(now-min(stamps))/1e9,
                input_error=None)
        from embodied.applications.workstation.pair_state import selected_run
        from .kernel_view import collection_snapshot
        from embodied.core.project import PROJECT_ROOT, load_config, resolve_path
        from pathlib import Path
        import json
        selected = selected_run(self.identity)
        if selected:
            side, directory = selected
            try:
                identity = json.loads((directory/'runtime.json').read_text())
                roots = [PROJECT_ROOT/'outputs/datasets/runtime', resolve_path(load_config()['paths']['dataset_dir'])/'mujoco']
                payload.update(collection_snapshot(identity, roots))
                payload['selected_side'] = side
                payload['runtime_directory'] = str(directory)
                payload['collection_hint'] = '当前操作侧：' + side + '。总控 C 采集；[ / ] 切换。'
            except (OSError, ValueError, KeyError):
                pass
        return payload
