"""Browser hand-eye calibration around ManualEyeToHandCollector, for any controller (todo C-06).

The kernel console runs it with a KernelArm on its own lease. Stages, commands and messages are
those of the legacy console (`kernel/control.py`), so the page needs no change.
"""
from __future__ import annotations

import asyncio
import json
from pathlib import Path
import queue
import time
from typing import Any, Awaitable, Callable

from embodied.calibration.hand_eye.__main__ import load_config
from embodied.calibration.hand_eye.collector import ManualEyeToHandCollector
from embodied.calibration.hand_eye.core.storage import CalibrationSession
from embodied.calibration.hand_eye.core.validation import disable_result, enable_validated_result, validate_cached_result
from embodied.core.arx5_paths import CALIBRATIONS_ROOT, HANDEYE_ROOT

CONFIG = HANDEYE_ROOT / 'configs' / 'x5_d455_eye_to_hand.json'
OUTPUT_ROOT = CALIBRATIONS_ROOT / 'hand-eye' / 'eye-to-hand'
MODES = ('dual', 'eye_in_hand', 'eye_to_hand')
FINISHED = ('done', 'cancelled', 'incomplete', 'error')
STAGE_COMMANDS = {'ready': 'READY', 'board': 'BOARD', 'center': 'CENTER', 'plan': 'RUN', 'remove': 'REMOVE',
                  'cancel_remove': 'REMOVE'}
INITIAL_MESSAGES = {
    'dual': 'D405 桌面板阶段将进入自由阻尼；手动调整到画面可见后会自动锁定并采样。',
    'eye_in_hand': 'D405 桌面板阶段将进入自由阻尼；手动调整到画面可见后会自动锁定并采样。',
    'eye_to_hand': 'D455 独立标定：请将标定板放入夹爪；确认后会夹紧并进入 D455 安全中心规划。',
}
DONE_MESSAGES = {
    'dual': '双相机手眼标定完成；机械臂已回参考位并保持。',
    'eye_in_hand': 'D405 眼在手标定已独立保存；D455 无需参与本次流程。',
    'eye_to_hand': 'D455 外部相机标定已独立保存；D405 无需参与本次流程。',
}

Send = Callable[[dict[str, Any]], Awaitable[None]]


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def millimetres(section: Any, key: str) -> float | None:
    value = section.get(key) if isinstance(section, dict) else None
    return 1000.0 * value if isinstance(value, (int, float)) else None


def result_summary(result: dict[str, Any] | None = None) -> dict[str, Any] | None:
    result = result if result is not None else read_json(OUTPUT_ROOT / 'current.json')
    if not isinstance(result, dict):
        return None
    residual = result.get('residual') or {}
    stability = result.get('leave_one_out_stability') or {}
    if not isinstance(residual, dict) or not isinstance(stability, dict):
        return None
    hand = result.get('eye_in_hand') if isinstance(result.get('eye_in_hand'), dict) else {}
    external = result.get('eye_to_hand') if isinstance(result.get('eye_to_hand'), dict) else {}
    hand_residual, hand_stability = hand.get('residual'), hand.get('leave_one_out_stability')
    relocalization, hand_relocalization = external.get('independent_validation'), hand.get('independent_validation')
    get = lambda section, key: section.get(key) if isinstance(section, dict) else None
    return {
        'sample_count': result.get('sample_count'), 'best_method': result.get('best_method'),
        'created_at': result.get('created_at'),
        'closure_translation_mm': millimetres(residual, 'closure_translation_rms_m'),
        'closure_rotation_deg': residual.get('closure_rotation_rms_deg'),
        'stability_translation_mm': millimetres(stability, 'translation_rms_m'),
        'stability_rotation_deg': stability.get('rotation_rms_deg'),
        'dual': result.get('calibration_type') == 'dual_camera_hand_eye', 'sample_counts': result.get('sample_counts'),
        'hand_closure_translation_mm': millimetres(hand_residual, 'closure_translation_rms_m'),
        'hand_closure_rotation_deg': get(hand_residual, 'closure_rotation_rms_deg'),
        'hand_stability_translation_mm': millimetres(hand_stability, 'translation_rms_m'),
        'relocalization_translation_mm': millimetres(relocalization, 'closure_translation_rms_m'),
        'relocalization_rotation_deg': get(relocalization, 'closure_rotation_rms_deg'),
        'hand_relocalization_translation_mm': millimetres(hand_relocalization, 'closure_translation_rms_m'),
        'hand_relocalization_rotation_deg': get(hand_relocalization, 'closure_rotation_rms_deg'),
        'session': result.get('session'), 'phase': result.get('calibration_role'),
        'combined_profile_updated': result.get('combined_profile_updated'),
        'standalone_profile_updated': result.get('standalone_profile_updated'),
    }


def validation_state() -> dict[str, Any]:
    """Solve validation and activation of the cached result, independent of collection."""
    current = read_json(OUTPUT_ROOT / 'current.json')
    validation = read_json(OUTPUT_ROOT / 'validation.json')
    enabled = read_json(OUTPUT_ROOT / 'enabled.json')
    requested = enabled is not None and enabled.get('enabled') is True
    current_stamp = current.get('created_at') if current else None
    enabled_stamp = (enabled.get('validation', {}).get('source_created_at')
                     if requested and isinstance(enabled.get('validation'), dict) else None)
    matches = bool(requested and ((current_stamp is not None and enabled_stamp == current_stamp) or (
        current_stamp is None and current is not None and enabled.get('T_base_camera') == current.get('T_base_camera'))))
    reviews = current.get('operator_reviews', {}) if current else {}
    reviewed = bool(isinstance(reviews, dict) and all(
        isinstance(reviews.get(role), dict) and reviews[role].get('accepted') is True
        and isinstance(reviews[role].get('fit_sample_count'), int) and reviews[role]['fit_sample_count'] >= 20
        for role in ('eye_in_hand', 'eye_to_hand')))
    return {
        'available': (OUTPUT_ROOT / 'current.json').is_file(), 'validated': validation is not None,
        'passed': validation.get('passed') is True if validation else False, 'enabled': matches,
        'stale': requested and not matches, 'validation': validation, 'operator_reviewed': reviewed,
        'operator_override': bool(matches and enabled.get('operator_override') is True),
        'enabled_at': enabled.get('enabled_at') if enabled else None,
        'T_base_camera': enabled.get('T_base_camera') if matches else None,
        'T_eef_camera': enabled.get('T_eef_camera') if matches else None,
        'profile': (enabled.get('profile', 'eye-to-hand') if matches
                    else current.get('calibration_type', 'eye-to-hand') if current else 'eye-to-hand'),
    }


class HandEyeFlow:
    def __init__(self, state_payload: Callable[[], dict[str, Any]]) -> None:
        self.state_payload = state_payload
        self.state: dict[str, Any] | None = None
        self.task: asyncio.Task | None = None
        self.commands: queue.Queue[str] | None = None
        self.collector: ManualEyeToHandCollector | None = None
        self.controller = None

    @property
    def active(self) -> bool:
        if self.state is not None and self.state.get('stage') in FINISHED:
            return False
        return self.task is not None and not self.task.done()

    def calibration_state(self) -> dict[str, Any] | None:
        state = self.state
        if state is None:
            result = result_summary()
            return None if result is None else {
                'active': False, 'stage': 'cached', 'message': '已有手眼标定结果，可在采集窗口重新标定。',
                'progress': None, 'quality': None, 'return_to_reference': True,
                'return_reference_label': '网页参考位', 'result': result, 'error': None}
        return {'active': self.active, 'mode': state.get('mode', 'dual'),
                'return_to_reference': bool(state.get('return_to_reference', True)),
                'manual_ready': state.get('manual_ready', False),
                **{key: state.get(key) for key in ('stage', 'message', 'progress', 'quality', 'result', 'error',
                                                   'feedback', 'return_reference_label')}}

    async def send_state(self, send: Send) -> None:
        calibration = self.calibration_state()
        if calibration is not None:
            await send({'type': 'handeye_calibration', 'calibration': calibration, 'state': self.state_payload()})

    async def result(self, send: Send, request_id, action, *, ok=True, result=None, error=None) -> None:
        await send({'type': 'command_result', 'id': request_id, 'action': action, 'ok': ok, 'result': result,
                    'error': error, 'state': self.state_payload()})

    # Cached result: validate, enable, disable.

    async def file_action(self, send: Send, request_id, action, params) -> None:
        if action == 'handeye_validate' and self.active:
            await self.result(send, request_id, action, ok=False, error='标定流程正在进行，请完成或取消后再验证。')
            return
        override = params.get('operator_override', False)
        if not isinstance(override, bool):
            await self.result(send, request_id, action, ok=False, error='人工采用标记必须是布尔值。')
            return
        try:
            if action == 'handeye_validate':
                validate_cached_result(OUTPUT_ROOT)
            elif action == 'handeye_enable':
                enable_validated_result(OUTPUT_ROOT, operator_override=override)
            else:
                disable_result(OUTPUT_ROOT)
        except (OSError, RuntimeError, ValueError) as exc:
            await self.result(send, request_id, action, ok=False, error=str(exc))
            return
        await send({'type': 'handeye_validation', 'validation': validation_state()})
        await self.result(send, request_id, action, result=validation_state())

    # Collection.

    async def start(self, send: Send, request_id, params, controller, reference_q=None) -> str | None:
        """Start the collector on `controller`; returns an error message instead of starting."""
        mode = params.get('mode', 'dual')
        if mode not in MODES:
            return '手眼标定相机模式无效。'
        if self.active:
            return '已有标定正在进行。'
        if params.get('safety_confirmed') is not True:
            return '请先确认标定板、所选相机视野、工作区和实体急停均已准备好。'
        try:
            config = load_config(CONFIG)
            if isinstance(params.get('scene_hint'), dict):
                config['scene_hint'] = params['scene_hint']
            automatic = config.setdefault('automatic', {})
            back = params.get('return_to_reference_after_complete', automatic.get('return_to_reference_after_complete', True))
            if not isinstance(back, bool):
                raise ValueError('自动回参考位设置无效')
            automatic['return_to_reference_after_complete'] = back
            if back and reference_q is not None:
                automatic['return_reference_joint_position_rad'] = list(reference_q)
                automatic['return_reference_label'] = '网页参考位'
            session = CalibrationSession((Path(config['collection']['output_root']) / 'sessions'
                                          / time.strftime('%Y%m%d-%H%M%S')).resolve())
        except (OSError, RuntimeError, ValueError) as exc:
            return f'无法准备手眼标定：{exc}'
        # JPEG guidance and feedback are snapshots: keep at most one unsent item of each.
        events: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        latest: dict[str, dict[str, Any]] = {}
        loop = asyncio.get_running_loop()

        def enqueue(event):
            kind = str(event.get('type', ''))
            if kind in ('preview', 'feedback'):
                scheduled = kind in latest
                latest[kind] = event
                if not scheduled:
                    events.put_nowait({'type': 'latest_' + kind})
                return
            events.put_nowait(event)
        self.commands = queue.Queue()
        self.controller = controller
        self.collector = ManualEyeToHandCollector(
            config, session, controller=controller, commands=self.commands,
            on_stage=lambda stage: loop.call_soon_threadsafe(enqueue, {'type': 'stage', 'stage': stage}),
            on_event=lambda event: loop.call_soon_threadsafe(enqueue, event))
        minimum = int(config['quality'].get('min_samples', 10))
        target = max(minimum, int(automatic.get('target_valid_samples', minimum + 3)))
        attempts = max(1, int(automatic.get('max_pose_attempts', 72)))
        self.state = {
            'stage': 'board' if mode == 'eye_to_hand' else 'table_check', 'mode': mode,
            'progress': {'phase': 'eye_to_hand' if mode == 'eye_to_hand' else 'eye_in_hand', 'stage': 'preparing',
                         'attempted': 0, 'attempt_limit': attempts, 'current': 0, 'total': attempts, 'valid': 0,
                         'target': target, 'elapsed_s': 0.0, 'estimated_remaining_s': None, 'maximum_remaining_s': None},
            'message': INITIAL_MESSAGES[mode], 'error': None, 'result': None, 'request_id': request_id,
            'cancel_requested': False, 'log': [], 'feedback': None, 'quality': None, 'return_to_reference': back,
            'return_reference_label': '网页参考位' if reference_q is not None else '六关节零位'}
        await send({'type': 'command_started', 'id': request_id, 'action': 'handeye_calibration_start'})
        self.task = asyncio.create_task(self.run(send, request_id, events, latest))
        await self.send_state(send)
        return None

    async def run(self, send: Send, request_id, events, latest) -> None:
        state, collector = self.state, self.collector
        mode = state['mode']
        worker = asyncio.create_task(asyncio.to_thread(
            collector.run_automatic if mode == 'dual' else collector.run_automatic_phase, *(() if mode == 'dual' else (mode,))))
        label = state.get('return_reference_label', '参考位')
        messages = {
            'ready': 'D405 姿态已对齐桌面标定板；点击确认后生成自动采样计划。',
            'board': '请把标定板放入夹爪后确认；系统将夹紧它并直接开始 D455 标定。',
            'center': '夹爪已保持标定板；调整 D455 安全中心后确认。',
            'plan': '自动采样计划已生成并显示在仿真中；确认后机械臂才会执行。',
            'manual': '自动采样未达精度目标；已进入自由阻尼手动补采。',
            'review': '已完成目标有效姿态及独立重定位评估。误差指标仅供评审；可采用当前结果，或补采 4 组新的联合平移/旋转姿态。',
            'motion_hold': '轨迹未完全到位，控制器正保持当前位置；请检查工作区，可使用“取消标定并回零”或实体急停。',
            'remove': (f'采样完成；托住标定板后确认张开夹爪，随后自动回{label}。' if state.get('return_to_reference', True)
                       else '采样完成；托住标定板后确认张开夹爪，随后保持当前位置。'),
            'return_reference': f'标定板已移除，正在自动回{label}；夹爪保持张开。',
        }
        try:
            while not worker.done():
                try:
                    event = await asyncio.wait_for(events.get(), timeout=.2)
                except asyncio.TimeoutError:
                    continue
                kind = str(event.get('type', ''))
                if kind.startswith('latest_'):
                    kind = kind.removeprefix('latest_')
                    event = latest.pop(kind, None)
                    if event is None:
                        continue
                if kind == 'preview':
                    await send({'type': 'handeye_preview', 'preview': event['preview']})
                elif kind == 'plan':
                    state['plan'] = event['plan']
                    await send({'type': 'handeye_plan', 'plan': event['plan']})
                elif kind in ('feedback', 'progress', 'quality'):
                    if kind == 'feedback' or isinstance(event.get(kind), dict):
                        state[kind] = event[kind]
                        await self.send_state(send)
                elif kind == 'manual_status' and isinstance(event.get('manual'), dict):
                    state['manual_ready'] = bool(event['manual'].get('ready'))
                    if isinstance(event['manual'].get('message'), str) and event['manual']['message']:
                        state['message'] = event['manual']['message']
                    await self.send_state(send)
                elif event.get('stage') in messages:
                    state['stage'], state['message'] = event['stage'], messages[event['stage']]
                    await self.send_state(send)
            result = await worker
            if state.get('cancel_requested'):
                state['stage'], state['message'] = 'cancelled', '手眼标定已取消；控制器保持当前位置。'
            elif result is None:
                state['stage'], state['message'] = 'incomplete', '采样结束但未达到精度要求。'
            else:
                state['result'], state['stage'] = result_summary(result), 'done'
                state['message'] = DONE_MESSAGES.get(mode, '手眼标定完成；机械臂已回参考位并保持。')
            ok = state['stage'] == 'done'
            await self.send_state(send)
            await self.result(send, request_id, 'handeye_calibration_start', ok=ok, result=state.get('result'),
                              error=None if ok else state['message'])
        except BaseException as exc:
            state['stage'], state['error'] = 'error', str(exc)
            state['message'] = f'手眼标定失败：{exc}；控制器保持当前位置。'
            await self.send_state(send)
            await self.result(send, request_id, 'handeye_calibration_start', ok=False, error=state['message'])
        finally:
            self.commands = self.collector = None

    async def confirm(self, send: Send, request_id, params) -> None:
        state, commands, action = self.state, self.commands, 'handeye_calibration_confirm'
        if not self.active or state is None or commands is None:
            await self.result(send, request_id, action, ok=False, error='当前没有等待确认的手眼标定。')
            return
        stage = params.get('stage')
        choices = {'manual': ('manual_action', {'sample': 'SAMPLE', 'done': 'DONE', 'home': 'HOME'}, 'manual_working', {
                       'sample': '正在保持当前位置并采集手动样本。', 'done': '正在检查手动补采质量并进入独立验证。',
                       'home': '正在安全回零；完成后继续手动补采。'}, '手动补采操作无效。'),
                   'review': ('review_action', {'more': 'MORE', 'accept': 'ACCEPT'}, 'review_working', {
                       'more': '正在将本轮独立验证样本纳入拟合，补采 4 个高信息差异化姿态；随后使用全新独立姿态重新评估。',
                       'accept': '已采用当前人工评审结果，正在继续下一阶段。'}, '标定评审操作无效。')}
        if stage in choices and state.get('stage') == stage:
            key, values, working, messages, invalid = choices[stage]
            choice = params.get(key)
            if choice not in values:
                await self.result(send, request_id, action, ok=False, error=invalid)
                return
            state['stage'], state['message'] = working, messages[choice]
            commands.put(values[choice])
            await self.result(send, request_id, action, result={'stage': stage, key: choice})
        elif stage in STAGE_COMMANDS and stage == state.get('stage'):
            state['stage'], state['message'] = 'working', '已确认，标定继续执行并保持当前控制器。'
            commands.put(STAGE_COMMANDS[stage])
            await self.result(send, request_id, action, result={'stage': stage})
        else:
            await self.result(send, request_id, action, ok=False, error='标定流程阶段已变化，请按当前提示操作。')
            return
        await self.send_state(send)

    async def cancel(self, *, return_home: bool = False) -> None:
        task = self.task
        if task is None or task.done():
            return
        if self.state is not None:
            self.state.update(cancel_requested=True, stage='cancelling',
                              message='正在安全取消并回零。' if return_home else '正在由同一控制器安全回到阶段起点。')
        if self.commands is not None:
            self.commands.put('HOME' if return_home else 'CANCEL')
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=30.0)
        except asyncio.TimeoutError:
            # Never abandon the worker while a physical move may run.
            if self.state is not None:
                self.state['message'] = '取消仍在等待安全收尾；控制器保持当前位置。'

    def emergency(self) -> None:
        """From the event loop: stop the arm now, then let the collector unwind."""
        if self.active:
            if self.controller is not None:
                self.controller.emergency_damping('网页急停')
            if self.commands is not None:
                self.commands.put('EMERGENCY')
