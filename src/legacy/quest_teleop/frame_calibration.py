"""Measure Quest-to-base directions in standby; never command robot motion."""
import argparse
import math
import time

import numpy as np

from embodied.core.session_protocol import request


def validate_rotation(value):
    rotation = np.asarray(value, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)):
        raise ValueError("标定必须是右手系正交旋转矩阵")
    if not np.allclose(rotation @ [0., 1., 0.], [0., 0., 1.], atol=1e-6):
        raise ValueError("标定必须保持 Quest 上方向对应机械臂竖直方向")
    return rotation


def heading_rotation(quaternion):
    """Controller local -Z points along base +X; preserve gravity, ignore tilt."""
    import meshcat.transformations as tf
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.) > .05:
        raise ValueError("无效手柄姿态")
    forward = -tf.quaternion_matrix(q[[3, 0, 1, 2]])[:3, 2].copy()
    forward[1] = 0.
    if np.linalg.norm(forward) < .5:
        raise ValueError("请将手柄前端大致水平指向机械臂正前方，不要朝上或朝下")
    forward /= np.linalg.norm(forward)
    up = np.array([0., 1., 0.])
    return validate_rotation(np.vstack((forward, np.cross(up, forward), up)))


def capture_heading(path, session_id):
    headings, stamps = [], set()
    for _ in range(12):
        status, _, stamp = standby_snapshot(path, session_id)
        basis = heading_rotation(status["state"]["input"]["hands"]["right"]["pose"][3:])
        if stamp > 0 and stamp not in stamps:
            headings.append(basis[0])
            stamps.add(stamp)
        time.sleep(.05)
    if len(headings) < 3:
        raise RuntimeError("Quest 时间戳未持续更新，请重新采样")
    forward = np.mean(headings, axis=0)
    forward /= np.linalg.norm(forward)
    if min(np.dot(h, forward) for h in headings) < math.cos(math.radians(5)):
        raise RuntimeError("手柄方向不稳定，请停稳后重试 c")
    return validate_rotation(np.vstack((forward, np.cross([0., 1., 0.], forward), [0., 1., 0.])))


def fit_directions(x_displacement, y_displacement):
    """Fit two measured base +X/+Y translations, preserving gravity (+Y -> +Z)."""
    directions = []
    for displacement in (x_displacement, y_displacement):
        vector = np.asarray(displacement, dtype=float)
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError("无效手柄位移")
        length = np.linalg.norm(vector)
        if length < .08:
            raise ValueError("移动距离不足 8 cm，请移动约 15–20 cm 后重试")
        if abs(vector[1]) / length > math.sin(math.radians(15)):
            raise ValueError("标定移动应保持水平，请重试")
        horizontal = vector.copy()
        horizontal[1] = 0.
        directions.append(horizontal / np.linalg.norm(horizontal))
    x, y = directions
    if abs(np.dot(x, y)) > math.sin(math.radians(15)):
        raise ValueError("两次移动方向不够垂直，请重新确认基座 +X／+Y")
    if np.dot(np.cross(x, y), [0, 1, 0]) <= 0:
        raise ValueError("方向左右颠倒：+Y 是沿基座 +X 看过去的左侧")
    source = np.column_stack((x, y, [0, 1, 0]))
    u, _, vt = np.linalg.svd(source.T)
    rotation = validate_rotation(u @ vt)
    errors = [math.degrees(math.acos(float(np.clip(np.dot(rotation @ vector, expected), -1, 1))))
              for vector, expected in zip(directions, ([1, 0, 0], [0, 1, 0]))]
    return rotation, errors


def standby_snapshot(path, session_id=None):
    status = request(path, "status", timeout=2)
    return validate_standby_status(status, session_id)


def validate_standby_status(status, session_id=None):
    if session_id is not None and status["session_id"] != session_id:
        raise RuntimeError("遥操已重启，请重新标定")
    state = status.get("state") or {}
    if state.get("control_enabled") is not False or status.get("recording") is not None:
        raise RuntimeError("请先返回待机、结束录制，再输入 c 标定")
    age = status.get("state_age_s")
    sample = state.get("input") or {}
    if (age is None or age > .5 or sample.get("error")
            or time.monotonic_ns() - sample.get("sample_monotonic_ns", 0) > 500_000_000):
        raise RuntimeError("手柄数据不新鲜，请检查 Quest 连接")
    hand = sample.get("hands", {}).get("right", {})
    pose = np.asarray(hand.get("pose", []), dtype=float)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise RuntimeError("缺少有效右手柄位姿")
    if hand.get("grip", 1) > .1 or hand.get("trigger", 1) > .1 or any(hand.get("buttons", {}).values()):
        raise RuntimeError("采样时请松开 Grip、Trigger 和按键")
    return status, pose[:3], sample.get("device_timestamp_ns", 0)


def capture(path, session_id):
    samples, stamps = [], set()
    for _ in range(12):
        _, position, stamp = standby_snapshot(path, session_id)
        if stamp > 0 and stamp not in stamps:
            samples.append(position)
            stamps.add(stamp)
        time.sleep(.05)
    if len(samples) < 3:
        raise RuntimeError("Quest 时间戳未持续更新，请重新采样")
    points = np.array(samples)
    if np.max(np.linalg.norm(points - points.mean(axis=0), axis=1)) > .01:
        raise RuntimeError("采样时手柄移动过大，请停稳后再按回车")
    return points.mean(axis=0)


def main(argv=None):
    from embodied.core.project import load_config, resolve_path
    from embodied.core.storage import save_json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse", action="store_true", help="Explicitly reuse heading; Quest must not have restarted/recentered")
    parser.add_argument("--socket", help="Local teleop session socket")
    args = parser.parse_args(argv)
    config = load_config()
    path = args.socket or resolve_path(config["runtime"]["socket"])
    status, _, _ = standby_snapshot(path)
    if status.get("rotation_frame") == "base":
        import json
        saved = resolve_path(config["paths"]["log_dir"]) / "last-controller-heading.json"
        session_id = status["session_id"]
        if args.reuse:
            rotation = validate_rotation(json.loads(saved.read_text())["quest_to_base_rotation"])
        else:
            print("右手柄前端指向机械臂正前方（零位时夹爪伸出的方向），保持不动，松开所有按键。")
            input("准备好后按回车（只采样方向，机械臂不运动）：")
            rotation = capture_heading(path, session_id)
        request(path, "input.calibrate", rotation=rotation.tolist(), session_id=session_id)
        save_json(saved, {"session_id": session_id, "method": "controller_forward_with_gravity",
                          "quest_to_base_rotation": rotation.tolist()})
        print("方向已对齐。下拨摇杆 1 秒启用，握 Grip 建立当前位置和姿态起点。")
        print("只重启电脑端且 Quest 未重启／重定位时，可输入 cr 复用；否则重新 c。")
        return
    if status.get("rotation_frame") != "calibrated":
        raise RuntimeError("请使用更新后的启动遥操.sh（--rotation-frame calibrated），再输入 c")
    session_id = status["session_id"]
    print("方向标定：机械臂保持待机，只移动右手柄，勿按手柄按键。")
    print("基座 +X：机械臂零位时夹爪伸出的方向；+Y：沿 +X 看过去的左侧；+Z：竖直向上。")
    print("以下前后左右按机械臂基座定义，与你站在哪一侧无关；手柄朝向不限。")
    positions = []
    for prompt in (
        "1/4 手柄放在方便移动的位置，停稳后按回车记录起点：",
        "2/4 从刚才起点沿基座 +X 水平移动约 15–20 cm，停稳后按回车：",
        "3/4 回到方便移动的位置，停稳后按回车记录第二个起点：",
        "4/4 从第二个起点沿基座 +Y 水平移动约 15–20 cm，停稳后按回车：",
    ):
        input(prompt)
        positions.append(capture(path, session_id))
    rotation, errors = fit_directions(positions[1] - positions[0], positions[3] - positions[2])
    # Preserve the measurements for inspection; never silently reload them after
    # a Quest recenter/restart, which may change the tracking coordinate frame.
    output = resolve_path(config["paths"]["log_dir"]) / f"frame-calibration-{time.time_ns()}.json"
    save_json(output, {"session_id": session_id, "method": "two_translations_with_gravity",
                       "positions_quest_m": [p.tolist() for p in positions],
                       "quest_to_base_rotation": rotation.tolist(), "fit_errors_deg": errors})
    request(path, "input.calibrate", rotation=rotation.tolist(), session_id=session_id)
    print(f"标定已应用，方向拟合误差 {errors[0]:.2f}° / {errors[1]:.2f}°；记录：{output}")
    print("现在可下拨右摇杆 1 秒启用控制。握 Grip 仅建立相对起点，不再重新猜测方向。")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, EOFError, KeyboardInterrupt) as exc:
        raise SystemExit(f"标定未完成：{exc}")
