"""录制已有运行实例中的 approach/retreat 示范；相机、角色与作者显式指定。"""
import argparse
import json
from pathlib import Path

from embodied import Robot
from embodied.applications.operations.recording import record_task
from embodied.data.export.lerobot import prepare_camera_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='已启动的 runtime 目录；本示例不启动硬件')
    parser.add_argument('--cameras', required=True, help='已配置的相机 ID，逗号分隔')
    parser.add_argument('--camera-roles', required=True, help='例如 {"base_0_rgb":"front_rgb"}')
    parser.add_argument('--annotator', required=True)
    parser.add_argument('--confidence', type=float, required=True, help='事件标注置信度；不是场景成功率')
    parser.add_argument('--output', type=Path, default=Path('outputs/datasets/runtime'))
    parser.add_argument('--offset-z', type=float, default=.02, help='相对当前 TCP 向上移动的米数，再返回')
    parser.add_argument('--export', type=Path, help='可选 LeRobot 输出目录；需在已有数据导出环境运行')
    args = parser.parse_args(argv)
    with Robot.connect(args.run) as robot:
        recording = record_task(robot, 'approach and retreat',
            cameras=[value.strip() for value in args.cameras.split(',')],
            camera_roles=json.loads(args.camera_roles), output=args.output,
            annotator=args.annotator, confidence=args.confidence)
        with recording as task:
            task.retreat(offset=(0., 0., args.offset_z))
            task.retreat(offset=(0., 0., -args.offset_z))
    _, _, report, _ = prepare_camera_episode(recording.path)
    print(json.dumps({'episode': str(recording.path), 'outcome': recording.manifest['outcome'],
        'annotation_revision': recording.annotation['revision'], 'export_preflight': report}, ensure_ascii=False, indent=2))
    if args.export is not None:
        from embodied.data.export.lerobot import export
        export(recording.path, args.export, repo_id='local/task_demonstrations')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
