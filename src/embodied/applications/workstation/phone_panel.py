"""Phone pane uses the same receiver and status as the web monitor."""
from embodied.applications.workstation.status import Status
from embodied.devices.inputs import phone_service


def rows():
    state = phone_service.monitor()
    connected = state.get('connected', False)
    enabled = state.get('enabled', False)
    label = ('遥操开启 · 正在跟随' if state.get('active') else '遥操开启 · 松手保持') if enabled else '仅监看'
    details = [('手机', Status('已连接 · ' + label if connected else '等待 RobotTeleop 连接',
                              'active' if enabled else 'ready' if connected else 'waiting'))]
    if enabled:
        details.append(('停止操作', Status('S / 空格 停止遥操；手机也可点停止', 'active')))
    if state.get('mapping') == 'phone_calibrated':
        details.append(('空间方向', Status('已对齐 · 平移与旋转共用方向' if state.get('aligned')
            else '手机前端对准夹爪，按 C 一键对齐', 'ready' if state.get('aligned') else 'waiting')))
    if not state['available']:
        details.append(('接收服务', Status('未就绪 · 按 R 启动', 'waiting')))
    endpoints = [f'{host}:{state["port"]}' for host in state.get('addresses', [])]
    details.append(('App 服务器', ' / '.join(endpoints) or f'本机网络地址 · UDP {state["port"]}'))
    if state.get('peer'):
        details.append(('手机地址', str(state['peer'][0])))
    tracking = {'normal': '正常', 'limited': '受限', 'unavailable': '无追踪'}
    details += [('ARKit', Status(tracking.get(state.get('tracking'), '等待输入'),
                                'ready' if state.get('tracking') == 'normal' else 'waiting')),
                ('相对映射', f'位移 {state.get("position_scale", 1):.2f}× · 旋转 {state.get("rotation_scale", 1):.2f}×'),
                ('输入帧率', f'{state.get("fps", 0):.1f} Hz' if connected else '—'),
                ('数据年龄', f'{state["age_s"] * 1000:.0f} ms' if connected else '—'),
                ('控制权', Status(label, 'active') if enabled else '未接管'),
                ('Clutch', '按住' if state.get('clutch') else '松开'),
                ('位置 XYZ', ' / '.join(f'{v:.3f}' for v in state['position']) + ' m' if state.get('position') else '—')]
    if state.get('estop'):
        details.append(('停止锁定', Status('已锁存 · 在手机点恢复', 'error')))
    if state.get('target_limited'):
        details.append(('跟随', Status('目标超前，已丢弃多余位移；可反向移动或松开重新定位', 'waiting')))
    if state.get('preset'):
        preset = state['preset']
        details.append(('预设动作', Status(preset['message'],
            'active' if preset['phase'] in ('opening', 'moving') else 'ready' if preset['phase'] == 'completed' else 'waiting')))
    if state.get('error'):
        details.append(('提示', Status(state['error'], 'error')))
    return details, ['手机与电脑使用可互通的网络，在 RobotTeleop 连接设置填写上方地址。',
                     'T 启用手机遥操 · S / 空格 停止遥操并保持 · R 启动 / 刷新接收服务。',
                     'C 对齐方向：手机前端（后置相机朝向）与夹爪同向，松手按一次；不移动机械臂。',
                     '打开本窗口默认仅接收输入；手机晚打开也可连接。',
                     '网页与本窗口共用状态；数据年龄是收包距今，不是端到端延迟。']


def press(key):
    if key == 'c':
        from embodied.applications.workstation.bench import current_run
        found = current_run()
        if found is None:
            raise ValueError('先启动 robot 取得夹爪姿态')
        phone_service.command('align', runtime_id=found[1]['runtime_id'])
        return '方向已对齐；平移与旋转使用同一校准，按住 Clutch 跟随'
    if key == 't':
        from embodied.applications.workstation.bench import current_run
        found = current_run()
        if found is None:
            raise ValueError('先在总控启动 robot')
        phone_service.command('enable', runtime_id=found[1]['runtime_id'])
        return '手机已接管；松开后按住 Clutch 控制'
    if key in ('s', ' '):
        phone_service.command('disable')
        return '已停止手机遥操；继续接收输入'
    return None
