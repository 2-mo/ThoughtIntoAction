"""Console backend on the arx kernel (todo C-05).

Speaks the console's WebSocket protocol (hello, state, command lifecycle, teleoperation and servo
streams) and reaches the arm only through one ControlSession. Joint, pose and park moves are kernel
previews that the kernel executes; the gripper, damping and live drags are short-TTL kernel
targets. Joint limits, the table plane, step limits and hold when targets stop are the kernel's;
the page attaches to a running kernel and never starts or stops it. Hand-eye calibration is the
`calib handeye` CLI; its saved results place the cameras in the scene. Payload / gripper
calibration, cube and browser collection are not on the kernel (`web legacy`, `collect run`) and
fail here with a message.
"""
from __future__ import annotations

import asyncio
import hmac
import json
import math
import time
import uuid
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlsplit

import numpy as np

from embodied import calibration
from embodied.contracts.runtime.control import ControlSession, live_runs
from embodied.core.arx5_paths import CONFIG_ROOT, PROJECT_ROOT
from legacy.arx5_control.teleoperation import load_teleoperation_config

RUNS = PROJECT_ROOT / 'control/run'
MOTION_PROFILE_REVISION = 6  # The page refuses other revisions.
SPEED_TIERS = (1, 2, 3, 4, 5, 10, 25, 50, 75, 100)
DEFAULT_SPEED = 10
MAX_JOINT_SPEED = .5  # rad/s at 100 %, the kernel planner's bound
MAX_GRIPPER_MM = 88.
GRIPPER_SPEED_MM_S = 40.  # gripper drag at 100 %
TTL_MS = 200
LEGACY_ONLY = {'payload_calibration': '负载标定', 'gripper_calibration': '夹爪标定', 'cube': '方块任务'}

COLLECTION = {'stage': 'unavailable', 'active': False, 'message': '内核采集：./run.sh collect run --run <RUN>'}


def rpy(q):
    """xyzw -> roll, pitch, yaw with R = Rz(yaw) Ry(pitch) Rx(roll), the scene's "ZYX" order."""
    x, y, z, w = q
    return [math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1., min(1., 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))]


def multiply(a, b):
    """Hamilton product a ⊗ b of xyzw quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def vector(value, size, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f'{name} 必须是 {size} 个有限数值。')
    return array


def unit(value):
    q = vector(value, 4, '四元数')
    if not .5 < np.linalg.norm(q) < 1.5:
        raise ValueError('四元数未归一化。')
    return (q / np.linalg.norm(q)).tolist()


def degrees(values):
    return np.degrees(np.asarray(values, dtype=float)).tolist()


def speed(params):
    percent = params.get('speed_percent', DEFAULT_SPEED)
    if isinstance(percent, bool) or percent not in SPEED_TIERS:
        raise ValueError(f'速度档位必须是 {list(SPEED_TIERS)} 之一。')
    return percent, max(.01, MAX_JOINT_SPEED * percent / 100)


def after_motion(params):
    mode = params.get('mode_after_motion', 'active')
    if mode not in ('active', 'damping'):
        raise ValueError('mode_after_motion 必须是 active 或 damping。')
    return mode


def joint_index(params):
    joint = params.get('joint')
    if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 6:
        raise ValueError('joint 必须是 1..6。')
    return joint - 1


def angle_deg(a, b):
    return math.degrees(2 * math.acos(min(1., abs(float(np.dot(a, b))))))


def hand_eye_state(cameras):
    """Saved hand-eye results in the page's legacy shape: fixed camera T_base_camera, wrist camera T_eef_camera."""
    found = {}
    for camera_id, options in cameras.items():
        result = calibration.load(camera_id, options.get('serial') or None)
        if result is not None:
            name = calibration.TRANSFORMS[result['mount']]
            found.setdefault(name, (camera_id, result))
    return {'available': bool(found), 'validated': bool(found), 'passed': bool(found), 'enabled': bool(found),
            'stale': False, 'operator_reviewed': True, 'profile': 'embodied.calibration',
            'cameras': {name: camera_id for name, (camera_id, _) in found.items()},
            **{name: result[name] for name, (_, result) in found.items()}}


class ArxConsole:
    """One browser owner, at most one kernel lease, one command at a time."""

    collection_camera = None

    def __init__(self, run: Path | None = None, token: str | None = None) -> None:
        self.run, self.token = run, token
        self.teleoperation_config = load_teleoperation_config(CONFIG_ROOT / 'teleoperation.json').web_payload()
        from embodied.core.project import load_config
        self.cameras = load_config()['cameras']
        self.started_at = time.time()
        self.owner = None
        self.session: ControlSession | None = None
        self.robot: dict[str, Any] = {}
        self.mode, self.fault, self.busy = 'disconnected', None, False
        self.task: asyncio.Task | None = None
        self.free_drive: asyncio.Task | None = None
        self.stream: dict[str, Any] | None = None
        self.reference = None
        self.preview: dict[str, Any] | None = None
        self.last_status = None
        self.handlers = {
            'connect': self.connect, 'disconnect': self.disconnect, 'activate': self.activate,
            'damping': self.damping, 'clear_fault': self.clear_fault, 'set_reference': self.set_reference,
            'preview_park': self.preview_park, 'park': self.park, 'preview_xyz': self.preview_xyz,
            'move_xyz': self.move_xyz, 'move_joint': self.move_joint, 'move_joint_target': self.move_joint,
            'reset_joint': self.move_joint, 'gripper_position': self.gripper, 'reset_gripper': self.gripper,
            'gripper_force': self.gripper,
        }

    async def startup(self) -> None:
        print(f'网页控制台：arx 内核模式（{self.run or "连接时选择最新运行中的内核"}）。', flush=True)

    async def shutdown(self) -> None:
        await self.disconnect({})

    # Kernel access; ControlSession calls block on a Unix socket, so they leave the event loop.

    async def call(self, function, *args, **kwargs):
        return await asyncio.to_thread(function, *args, **kwargs)

    def _target(self, action, **fields):
        return self.session.command(action, ttl_ms=TTL_MS, sent_monotonic_ns=time.monotonic_ns(), **fields)

    async def status(self):
        if self.session is None:
            return None
        try:
            status = await self.call(self.session.status)
            if self.session.error:
                raise RuntimeError(f'控制租约已失效：{self.session.error}')
            if status['stopped']:
                raise RuntimeError('内核已执行 stop 并锁存，需要重启内核。')
        except Exception as exc:
            self.enter_fault(str(exc))
            return None
        self.last_status = status
        return status

    def enter_fault(self, reason):
        if self.mode != 'fault':
            print(f'内核控制故障：{reason}', flush=True)
        self.mode, self.fault, self.stream = 'fault', reason, None
        if self.free_drive:
            self.free_drive.cancel()
            self.free_drive = None

    def require_active(self):
        if self.session is None:
            raise RuntimeError('机械臂未连接。')
        if self.mode != 'active':
            raise RuntimeError('请先切换到关节位置模式。' if self.mode == 'damping' else f'当前模式 {self.mode} 不能运动。')
        if self.stream is not None:
            raise RuntimeError('实时控制进行中。')

    # State

    def state_payload(self, status=None) -> dict[str, Any]:
        stream = self.stream
        teleoperation = stream is not None and stream['kind'] == 'teleoperation'
        payload: dict[str, Any] = {
            'connected': self.session is not None, 'mode': self.mode, 'busy': self.busy,
            'servo_active': stream is not None and not stream.get('preview_only'), 'servo_stopping': False,
            'servo_kind': None if stream is None else
            'teleoperation_preview' if stream.get('preview_only') else stream['kind'],
            'fault_reason': self.fault, 'control_stack': 'arx',
            'kernel_runtime_id': self.session.identity['runtime_id'] if self.session else None,
            'teleoperation_config': self.teleoperation_config,
            'teleoperation': {
                'active': teleoperation and not stream.get('preview_only'),
                'preview_only': teleoperation and bool(stream.get('preview_only')), 'stopping': False,
                'target_age_ms': 1000 * (time.monotonic() - stream['updated']) if teleoperation else None,
                'servo_period_ms': 1000 / self.robot['control_hz'] if self.robot else None,
                'gripper_contact_latched': False,
            },
            'payload_calibration': None, 'gripper_calibration': None,
            'handeye_calibration': None, 'handeye_validation': hand_eye_state(self.cameras),
            'cube_manipulation': {'stage': 'idle'}, 'collection': COLLECTION,
        }
        status = status if status is not None else self.last_status
        if self.session is None or status is None:
            return payload
        pose = status['pose']
        payload.update(
            joint_position_deg=degrees(status['q']), joint_velocity_rad_s=status['dq'],
            joint_min_deg=degrees(self.robot['joint_min']), joint_max_deg=degrees(self.robot['joint_max']),
            gripper_position_mm=1000 * status['gripper_width'],
            gripper_max_mm=min(MAX_GRIPPER_MM, 1000 * self.robot['gripper_max_width']),
            eef_pose_6d=[*pose[:3], *rpy(pose[3:])],
            state_age_ms=(time.monotonic_ns() - status['feedback_monotonic_ns']) / 1e6,
            kernel_phase=status['phase'],
        )
        if self.reference is not None:
            payload.update(reference_joint_position_deg=degrees(self.reference[0]),
                           reference_gripper_position_mm=1000 * self.reference[1])
        return payload

    # WebSocket

    async def send(self, websocket, event):
        await websocket.send(json.dumps(event, ensure_ascii=False, allow_nan=False))

    async def handle(self, websocket) -> None:
        query = parse_qs(urlsplit(websocket.request.path).query)
        token = query.get('token', [])
        if self.token is not None and not (len(token) == 1 and hmac.compare_digest(token[0], self.token)):
            await self.send(websocket, {'type': 'fatal', 'message': '缺少或配对令牌不正确。'})
            await websocket.close(code=1008, reason='invalid pairing token')
            return
        if self.owner is not None:
            if query.get('takeover') != ['1'] or self.busy or self.stream is not None:
                await self.send(websocket, {'type': 'fatal', 'message': '已有一个网页客户端占用控制服务。'})
                await websocket.close(code=1013, reason='control client already connected')
                return
            previous, self.owner = self.owner, websocket
            await previous.close(code=1012, reason='control session handed over')
        self.owner = websocket
        await self.send(websocket, {
            'type': 'hello', 'protocol': 1, 'control_stack': 'arx', 'service_started_at': self.started_at,
            'limits': {'motion_profile_revision': MOTION_PROFILE_REVISION, 'max_joint_step_deg': 10,
                       'max_gripper_width_mm': MAX_GRIPPER_MM, 'max_gripper_torque_nm': .25,
                       'min_speed_percent': SPEED_TIERS[0], 'max_speed_percent': SPEED_TIERS[-1],
                       'default_speed_percent': DEFAULT_SPEED, 'speed_tiers_percent': list(SPEED_TIERS)},
            'state': self.state_payload(await self.status())})
        try:
            async for raw in websocket:
                message = json.loads(raw)
                if message.get('type') == 'command':
                    await self.command(websocket, message.get('id'), message.get('action'), message.get('params') or {})
                elif message.get('type') in ('teleop', 'servo'):
                    await self.on_stream(websocket, message)
        finally:
            if self.owner is websocket:
                self.owner = None
                print('网页控制客户端断开；释放内核控制，内核保持当前位置。', flush=True)
                await self.disconnect({})

    async def reply(self, websocket, request_id, action, *, result=None, error=None):
        if error is not None:
            print(f'{action} 失败：{error}', flush=True)
        await self.send(websocket, {
            'type': 'command_result', 'id': request_id, 'action': action, 'ok': error is None,
            **({'error': error} if error is not None else {'result': result}),
            'state': self.state_payload(await self.status())})

    async def command(self, websocket, request_id, action, params):
        if action == 'state':
            await self.send(websocket, {'type': 'state', 'id': request_id, 'state': self.state_payload(await self.status())})
            return
        if action == 'emergency':
            await self.emergency()
            await self.reply(websocket, request_id, action)
            return
        handler = self.handlers.get(action)
        if handler is None:
            legacy = next((name for prefix, name in LEGACY_ONLY.items() if str(action).startswith(prefix)), None)
            await self.reply(websocket, request_id, action, error=(
                f'{legacy}尚未迁到 arx 内核，请暂用旧控制台。' if legacy else
                '手眼标定在终端运行：./run.sh calib handeye capture <相机> --auto；结果保存后页面自动显示相机位姿。'
                if str(action).startswith('handeye') else
                '浏览器采集已由内核采集替代：./run.sh collect run --run <RUN>。' if str(action).startswith('collection')
                else f'未知命令：{action}'))
            return
        if self.task is not None and not self.task.done():
            await self.reply(websocket, request_id, action, error='上一条命令尚未完成。')
            return
        await self.send(websocket, {'type': 'command_started', 'id': request_id, 'action': action})
        self.busy = True
        self.task = asyncio.create_task(self.run_command(websocket, request_id, action, handler, params))

    async def run_command(self, websocket, request_id, action, handler, params):
        try:
            result = await handler(params, websocket=websocket, request_id=request_id, action=action)
        except asyncio.CancelledError:
            self.busy = False
            await self.reply(websocket, request_id, action, error='已急停，命令取消；内核保持当前位置。')
        except Exception as exc:
            self.busy = False
            await self.reply(websocket, request_id, action, error=str(exc))
        else:
            self.busy = False
            await self.reply(websocket, request_id, action, result=result)

    # Session and modes

    async def connect(self, params, **_):
        if self.session is not None:
            return None
        runs = [Path(self.run)] if self.run else live_runs(RUNS)
        if not runs:
            raise RuntimeError('没有运行中的内核：先运行 ./run.sh kernel sim 或 ./run.sh kernel real。')
        session = ControlSession(runs[-1], source='console', purpose='operation')
        await self.call(session.__enter__)
        self.session, self.mode, self.fault, self.preview = session, 'active', None, None
        self.robot = (await self.call(session.describe))['robot']
        status = await self.status()
        if status is None:
            raise RuntimeError(self.fault)
        self.reference = (status['q'], status['gripper_width'])
        print(f'已接入内核 {session.identity["runtime_id"]}（{session.identity["backend"]}）。', flush=True)
        return None

    async def disconnect(self, params, **_):
        if self.task is not None and self.task is not asyncio.current_task():
            self.task.cancel()
        self.stop_free_drive()
        self.stream, self.preview = None, None
        if self.session is not None:
            session, self.session = self.session, None
            await self.call(session.close)  # Release; the kernel holds position.
        self.mode, self.fault, self.last_status = 'disconnected', None, None
        return None

    async def emergency(self):
        if self.task is not None and not self.task.done():
            self.task.cancel()
        self.stop_free_drive()
        self.stream = None
        if self.session is not None and self.mode != 'fault':
            try:
                await self.call(self.session.command, 'hold')
                self.mode = 'active'
            except Exception as exc:
                self.enter_fault(str(exc))

    async def activate(self, params=None, **_):
        if self.session is None:
            raise RuntimeError('机械臂未连接。')
        if self.mode == 'fault':
            raise RuntimeError(f'先清除故障：{self.fault}')
        self.stop_free_drive()
        await self.call(self.session.command, 'hold')
        self.mode = 'active'

    async def damping(self, params=None, **_):
        if self.session is None or self.mode == 'fault':
            raise RuntimeError('机械臂未连接或处于故障。')
        if self.stream is not None:
            raise RuntimeError('实时控制进行中。')
        await self.call(self._target, 'free-drive')
        self.mode = 'damping'
        if self.free_drive is None:
            self.free_drive = asyncio.create_task(self.keep_free_drive())

    async def keep_free_drive(self):
        try:
            while True:
                await asyncio.sleep(.1)
                await self.call(self._target, 'free-drive')
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.free_drive = None
            self.enter_fault(str(exc))

    def stop_free_drive(self):
        if self.free_drive is not None:
            self.free_drive.cancel()
            self.free_drive = None

    async def clear_fault(self, params, **_):
        await self.disconnect({})
        return await self.connect({})

    async def set_reference(self, params, **_):
        status = await self.status()
        if status is None:
            raise RuntimeError('机械臂未连接。')
        self.reference = (status['q'], status['gripper_width'])

    async def settle(self, params):
        if after_motion(params) == 'damping':
            await self.damping()

    # Previewed moves

    def describe_plan(self, plan):
        q = np.asarray(plan['q'])
        return {'start_joint_position_deg': degrees(q[0]), 'target_joint_position_deg': degrees(q[-1]),
                'max_joint_delta_deg': float(np.degrees(np.max(np.abs(q[-1] - q[0])))),
                'path_xyz_mm': [[1000 * v for v in pose[:3]] for pose in plan['tcp']],
                'trajectory_duration_s': float(plan['times'][-1])}

    async def execute(self, plan, websocket, request_id, action):
        operation = (await self.call(self.session.execute, plan))['operation_id']
        deadline = time.monotonic() + plan['times'][-1] + 10
        while True:
            status = await self.status()
            if status is None:
                raise RuntimeError(self.fault)
            entry = next((op for op in status['operations'] if op['operation_id'] == operation), None)
            if entry is not None and entry['status'] != 'running':
                if entry['status'] != 'completed':
                    raise RuntimeError(f'内核运动{entry["status"]}：{entry.get("reason") or ""}')
                return status
            if time.monotonic() > deadline:
                await self.call(self.session.command, 'cancel')
                raise TimeoutError('运动超时，已取消。')
            await self.send(websocket, {'type': 'motion_state', 'id': request_id, 'action': action,
                                        'state': self.state_payload(status)})
            await asyncio.sleep(.04)

    async def joint_motion(self, q, params, **context):
        self.require_active()
        _, rate = speed(params)
        plan = await self.call(self.session.preview, 'movej', q=[float(v) for v in q], speed=rate)
        status = await self.execute(plan, **context)
        await self.settle(params)
        return {'kind': 'joint_motion', 'final_error_deg': degrees(np.abs(np.subtract(status['q'], q)))}

    async def move_joint(self, params, **context):
        self.require_active()
        index = joint_index(params)
        status = await self.status()
        q = list(status['q'])
        if context['action'] == 'move_joint':
            delta = params.get('delta_deg')
            if isinstance(delta, bool) or not isinstance(delta, (int, float)) or not .1 <= abs(delta) <= 10:
                raise ValueError('delta_deg 必须在 ±0.1..10°。')
            q[index] += math.radians(delta)
        elif context['action'] == 'move_joint_target':
            target = params.get('target_deg')
            if isinstance(target, bool) or not isinstance(target, (int, float)) or not math.isfinite(target):
                raise ValueError('target_deg 必须是有限数值。')
            q[index] = math.radians(target)
        else:
            if self.reference is None:
                raise RuntimeError('尚未记录参考姿态。')
            if abs(math.degrees(self.reference[0][index] - q[index])) < .1:
                raise ValueError(f'J{index + 1} 已经位于参考角度附近。')
            q[index] = self.reference[0][index]
        return await self.joint_motion(q, params, **context)

    async def preview_park(self, params, **_):
        self.require_active()
        if self.reference is None:
            raise RuntimeError('尚未记录参考姿态。')
        percent, rate = speed(params)
        plan = await self.call(self.session.preview, 'movej', q=list(self.reference[0]), speed=rate)
        self.preview = {'action': 'park', 'speed': percent, 'plan': plan}
        return {'kind': 'park_preview', **self.describe_plan(plan)}

    async def park(self, params, **context):
        self.require_active()
        percent, _ = speed(params)
        preview, self.preview = self.preview, None
        if not preview or preview['action'] != 'park' or preview['speed'] != percent:
            raise RuntimeError('请先以相同速度预览回到参考姿态。')
        status = await self.execute(preview['plan'], **context)
        await self.settle(params)
        return {'kind': 'joint_motion',
                'final_error_deg': degrees(np.abs(np.subtract(status['q'], preview['plan']['q'][-1])))}

    async def preview_xyz(self, params, live=False, **_):
        self.require_active()
        percent, rate = speed(params)
        delta = vector([params.get('x_mm'), params.get('y_mm'), params.get('z_mm')], 3, 'x/y/z_mm')
        status = await self.status()
        start = np.asarray(status['pose'])
        orientation = (start[3:].tolist() if live or params.get('orientation_xyzw') is None
                       else unit(params['orientation_xyzw']))
        target = [*(start[:3] + delta / 1000).tolist(), *orientation]
        plan = await self.call(self.session.preview, 'movel', pose=target, speed=rate)
        end = np.asarray(plan['tcp'][-1])
        if not live:
            self.preview = {'action': 'xyz', 'speed': percent, 'delta': delta, 'orientation': orientation,
                            'plan': plan, 'target': target}
        return {'kind': 'cartesian_preview', 'delta_xyz_mm': delta.tolist(),
                'start_xyz_mm': (1000 * start[:3]).tolist(), 'target_xyz_mm': [1000 * v for v in target[:3]],
                'target_orientation_xyzw': orientation,
                'ik_position_error_mm': float(1000 * np.linalg.norm(end[:3] - target[:3])),
                'ik_orientation_error_deg': angle_deg(end[3:], orientation), 'path_mode': 'linear',
                **self.describe_plan(plan)}

    async def move_xyz(self, params, **context):
        self.require_active()
        percent, _ = speed(params)
        delta = vector([params.get('x_mm'), params.get('y_mm'), params.get('z_mm')], 3, 'x/y/z_mm')
        preview, self.preview = self.preview, None
        orientation = params.get('orientation_xyzw')
        if (not preview or preview['action'] != 'xyz' or preview['speed'] != percent
                or np.max(np.abs(preview['delta'] - delta)) > .01
                or (orientation is not None and angle_deg(unit(orientation), preview['orientation']) > .01)):
            raise RuntimeError('目标与预览不一致，请重新预览。')
        status = await self.execute(preview['plan'], **context)
        await self.settle(params)
        final, target = np.asarray(status['pose'][:3]), np.asarray(preview['target'][:3])
        return {'kind': 'cartesian_motion', 'target_xyz_mm': (1000 * target).tolist(),
                'final_xyz_mm': (1000 * final).tolist(),
                'final_position_error_mm': float(1000 * np.linalg.norm(final - target)),
                'final_joint_error_deg': degrees(np.abs(np.subtract(status['q'], preview['plan']['q'][-1])))}

    # Gripper: position targets streamed until reached or stalled; then the kernel holds the width.

    async def gripper(self, params, websocket, request_id, action):
        self.require_active()
        limit = min(MAX_GRIPPER_MM, 1000 * self.robot['gripper_max_width'])
        if action == 'gripper_position':
            speed(params)
            width = params.get('width_mm')
        elif action == 'reset_gripper':
            speed(params)
            width = 0.
        else:
            width = params.get('minimum_mm', 0.)
        if isinstance(width, bool) or not isinstance(width, (int, float)) or not 0 <= width <= limit:
            raise ValueError(f'夹爪宽度必须在 0..{limit:.0f} mm。')
        target = width / 1000
        best, since, deadline, reason = math.inf, time.monotonic(), time.monotonic() + 8, 'reached'
        while True:
            status = await self.status()
            if status is None:
                raise RuntimeError(self.fault)
            error = abs(status['gripper_width'] - target)
            if error < .0008:
                break
            if error < best - .0003:
                best, since = error, time.monotonic()
            elif time.monotonic() - since > .6:
                reason = 'stalled'
                break
            if time.monotonic() > deadline:
                raise TimeoutError('夹爪运动超时。')
            await self.call(self._target, 'gripper', gripper_width=target)
            await self.send(websocket, {'type': 'motion_state', 'id': request_id, 'action': action,
                                        'state': self.state_payload(status)})
            await asyncio.sleep(.04)
        await self.call(self.session.command, 'hold')
        await self.settle(params)
        final = 1000 * status['gripper_width']
        if action == 'gripper_force':
            return {'kind': 'gripper_force', 'final_width_mm': final, 'stop_reason': reason}
        return {'kind': 'gripper_position', 'final_width_mm': final, 'final_error_mm': abs(final - width)}

    # Streams: gesture teleoperation and drag servoing, each a kernel target with a short TTL.

    async def on_stream(self, websocket, message):
        kind = 'teleoperation' if message['type'] == 'teleop' else message.get('kind')
        action, sequence, params = message.get('action'), message.get('seq'), message.get('params') or {}
        stream = self.stream
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            return
        if action == 'update' and (stream is None or stream['kind'] != kind or sequence <= stream['seq']):
            return  # Stale or orphaned updates get no reply, as before.
        reply = {'type': 'teleop_state' if kind == 'teleoperation' else 'servo_state', 'seq': sequence,
                 'action': action, **({} if kind == 'teleoperation' else {'kind': kind})}
        try:
            if action == 'start':
                await self.start_stream(kind, sequence, params)
            elif action == 'update':
                if kind == 'teleoperation' and message.get('session_id') != stream['session_id']:
                    raise RuntimeError('遥操作会话已结束。')
                stream['seq'] = sequence
                await self.update_stream(stream, params)
            elif action == 'stop':
                if stream is not None and stream['kind'] == kind:
                    await self.stop_stream()
            else:
                raise ValueError(f'未知的流命令：{action}')
            reply['ok'] = True
        except Exception as exc:
            reply.update(ok=False, error=str(exc))
            if action == 'update' and self.stream is stream:
                await self.stop_stream()
        session_id = (self.stream or stream or {}).get('session_id') or message.get('session_id')
        if session_id:
            reply['session_id'] = session_id
        reply['state'] = self.state_payload(await self.status())
        await self.send(websocket, reply)

    async def start_stream(self, kind, sequence, params):
        if self.stream is not None:
            raise RuntimeError('已有实时控制会话。')
        if self.task is not None and not self.task.done():
            raise RuntimeError('上一条命令尚未完成。')
        if self.mode == 'damping':
            await self.activate()
        self.require_active()
        status = await self.status()
        stream = {'kind': kind, 'seq': sequence, 'pose': status['pose'], 'width': status['gripper_width'],
                  'updated': time.monotonic()}
        if kind == 'teleoperation':
            opening = params.get('open_width_mm', MAX_GRIPPER_MM)
            if isinstance(opening, bool) or not isinstance(opening, (int, float)) or not 0 <= opening <= MAX_GRIPPER_MM:
                raise ValueError('open_width_mm 无效。')
            stream.update(session_id=uuid.uuid4().hex, preview_only=bool(params.get('preview_only')),
                          open=min(opening / 1000, self.robot['gripper_max_width']))
        elif kind == 'cartesian':
            speed(params)
            stream['after'] = after_motion(params)
        elif kind == 'gripper':
            percent, _ = speed(params)
            minimum = params.get('minimum_mm', 0.)
            if isinstance(minimum, bool) or not isinstance(minimum, (int, float)) or not 0 <= minimum <= MAX_GRIPPER_MM:
                raise ValueError('minimum_mm 无效。')
            stream.update(after=after_motion(params), rate=GRIPPER_SPEED_MM_S * percent / 100 / 1000,
                          minimum=minimum / 1000, direction=0)
        else:
            raise ValueError(f'未知的伺服类型：{kind}')
        self.stream = stream

    async def update_stream(self, stream, params):
        now = time.monotonic()
        if stream['kind'] == 'teleoperation':
            delta = vector(params.get('delta_xyz_m'), 3, 'delta_xyz_m')
            rotation = unit(params.get('delta_orientation_xyzw'))
            if params.get('gripper_state') not in ('open', 'close'):
                raise ValueError('gripper_state 必须是 open 或 close。')
            pose = [*(np.asarray(stream['pose'][:3]) + delta).tolist(), *multiply(rotation, stream['pose'][3:])]
            if not stream['preview_only']:
                await self.call(self._target, 'cartesian', pose=pose,
                                gripper_width=stream['open'] if params['gripper_state'] == 'open' else 0.)
        elif stream['kind'] == 'cartesian':
            delta = vector([params.get('x_mm'), params.get('y_mm'), params.get('z_mm')], 3, 'x/y/z_mm')
            pose = [*(np.asarray(stream['pose'][:3]) + delta / 1000).tolist(), *stream['pose'][3:]]
            await self.call(self._target, 'cartesian', pose=pose)
        else:
            direction = params.get('direction', stream['direction'])
            if direction not in (-1, 0, 1):
                raise ValueError('direction 必须是 -1、0 或 1。')
            limit = self.robot['gripper_max_width']
            step = direction * stream['rate'] * min(.2, now - stream['updated'])
            stream.update(direction=direction, width=min(limit, max(stream['minimum'], stream['width'] + step)))
            await self.call(self._target, 'gripper', gripper_width=stream['width'])
        stream['updated'] = now

    async def stop_stream(self):
        stream, self.stream = self.stream, None
        if self.session is not None and self.mode != 'fault':
            await self.call(self.session.command, 'hold')
            if stream and stream.get('after') == 'damping':
                await self.damping()
