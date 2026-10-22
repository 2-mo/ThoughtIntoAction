"""将已有任务事件标为可复核的技能区间；不操作机器人、不修改原始 episode。"""
import argparse
from pathlib import Path

from embodied.data.episode.annotations import annotate_task_events


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path, help='已结束的原始 episode 目录')
    parser.add_argument('events', type=Path, help='任务输出目录中的 events.jsonl')
    parser.add_argument('--annotator', required=True, help='标注人或事件来源名称')
    parser.add_argument('--confidence', type=float, required=True, help='显式给出标签置信度，0..1；不代表任务成功')
    parser.add_argument('--camera', help='帧索引所属相机；默认用 episode 的参考相机')
    parser.add_argument('--object', dest='object_name', help='明确已知的目标对象；未知时留空')
    args = parser.parse_args(argv)
    annotation = annotate_task_events(args.episode, args.events, annotator=args.annotator,
        confidence=args.confidence, camera_id=args.camera, object_name=args.object_name)
    print(f"已保存待复核标注 v{annotation['revision']}，共 {len(annotation['segments'])} 个区间。")
    print(args.episode / 'annotations' / f"skills.v{annotation['revision']:06d}.json")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
