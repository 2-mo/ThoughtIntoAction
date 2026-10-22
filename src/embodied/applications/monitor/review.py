"""Task/annotation views. No operation submission or raw recording mutation."""
import json
from pathlib import Path
import re


def read_json(path):
    if path.stat().st_size > 2_000_000:
        raise ValueError('Review document is too large')
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError('Review document must be an object')
    return value


class ReviewStore:
    def __init__(self, monitor):
        self.monitor = monitor

    def episodes(self):
        result = []
        for root in self.monitor.roots:
            root = Path(root).resolve()
            for path in root.glob('*/episode.json'):
                if not path.resolve().is_relative_to(root):
                    continue
                try:
                    manifest = read_json(path)
                    if manifest.get('schema_version') == 2 and manifest.get('lifecycle') == 'complete':
                        result.append({'id': path.parent.name, 'task': manifest.get('task'),
                                       'started_monotonic_ns': manifest.get('started_monotonic_ns'),
                                       'cameras': list(manifest.get('cameras') or
                                           ({'wrist_rgb': manifest['camera']} if manifest.get('camera') else {}))})
                except (OSError, ValueError, TypeError):
                    continue
        return sorted(result, key=lambda row: row['started_monotonic_ns'] or 0, reverse=True)

    def episode_path(self, episode_id):
        if not isinstance(episode_id, str) or not re.fullmatch(r'[A-Za-z0-9_.-]+', episode_id) or episode_id in ('.', '..'):
            raise ValueError('Invalid episode ID')
        matches = []
        for root in self.monitor.roots:
            root = Path(root).resolve()
            path = (root / episode_id).resolve()
            if path.is_relative_to(root) and (path / 'episode.json').is_file():
                manifest = read_json(path / 'episode.json')
                if manifest.get('schema_version') != 2 or manifest.get('lifecycle') != 'complete':
                    raise ValueError('Only complete runtime episodes can be reviewed')
                matches.append(path)
        if len(matches) != 1:
            raise ValueError('Episode is missing or its ID is ambiguous')
        return matches[0]

    def annotations(self, episode_id):
        from embodied.data.episode.annotations import read_annotations
        return read_annotations(self.episode_path(episode_id))

    def review(self, data):
        from embodied.data.episode.annotations import review_annotations
        return review_annotations(self.episode_path(data.get('episode_id')),
                                  reviewer=data.get('reviewer'), decision=data.get('decision'),
                                  expected_revision=data.get('expected_revision'), note=data.get('note', ''))

    def tasks(self):
        directory = self.monitor.directory
        if directory is None:
            return []
        root = (Path(directory) / 'tasks').resolve()
        result = []
        for path in root.glob('*/result.json'):
            if not path.resolve().is_relative_to(root):
                continue
            try:
                task = read_json(path)
                if task.get('schema_version') == 1:
                    result.append(task)
            except (OSError, ValueError, TypeError):
                continue
        return sorted(result, key=lambda row: row.get('started_unix_ns', 0), reverse=True)[:100]
