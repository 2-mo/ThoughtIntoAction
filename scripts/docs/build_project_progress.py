#!/usr/bin/env python3
"""Build an offline progress snapshot from one Git revision; no SDK or network."""
from __future__ import annotations

import fnmatch
import hashlib
import json
import re
import subprocess
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TASK = re.compile(r'^- \[([ xX])\] \*\*([A-Z]+-\d+)\*\* (.+)$', re.M)


def git(*args: str, root: Path = ROOT) -> str:
    return subprocess.check_output(['git', '-C', str(root), *args], text=True)


def parse_tasks(text: str, path: str) -> list[dict]:
    matches = list(TASK.finditer(text))
    tasks = []
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        body = text[match.end():end].strip()
        tasks.append({
            'id': match[2], 'title': match[3].strip(), 'closed': match[1].lower() == 'x',
            'path': path, 'line': text[:match.start()].count('\n') + 1, 'body': body,
        })
    return tasks


def assign_tasks(tasks: list[dict], modules: list[dict]) -> None:
    """Each formal task has one primary module; ambiguous mappings are an error."""
    seen = set()
    for task in tasks:
        if task['id'] in seen:
            raise ValueError(f"Duplicate task: {task['id']}")
        seen.add(task['id'])
        owners = [module['id'] for module in modules
                  if any(fnmatch.fnmatchcase(task['id'], pattern) for pattern in module['tasks'])]
        if len(owners) != 1:
            raise ValueError(f"Task {task['id']} requires one module, got {owners}")
        task['module'] = owners[0]


def modules_for_paths(paths: list[str], modules: list[dict], tasks: list[dict]) -> list[str]:
    """Use longest explicit path match, including historical aliases and deletions."""
    result = set()
    for path in paths:
        if path.startswith('todo/'):
            path = 'docs/plans/' + path.removeprefix('todo/')
        # A plan change belongs to the modules covered by that plan, not just docs.
        plan_modules = {task['module'] for task in tasks if task['path'] == path}
        if plan_modules:
            result.update(plan_modules)
            continue
        matches = [(len(prefix), module['id']) for module in modules for prefix in module['paths']
                   if (path.startswith(prefix) if prefix.endswith('/') else fnmatch.fnmatchcase(path, prefix))]
        if matches:
            length = max(item[0] for item in matches)
            result.update(module for size, module in matches if size == length)
    return sorted(result)


def parse_history(raw: str) -> list[dict]:
    commits = []
    for record in raw.split('\x1e'):
        if not record.strip('\x00\n'):
            continue
        sha, date, name, email, committer, subject, body, paths = record.split('\x00', 7)
        identity = email.strip().casefold() or f'name:{name}'
        commits.append({
            'sha': sha, 'date': date, 'author': name, 'committer': committer,
            'authorId': hashlib.sha256(identity.encode()).hexdigest()[:16],
            'subject': subject,
            'coauthors': list(dict.fromkeys(re.findall(r'^Co-authored-by:\s*(.*?)\s*<[^>]+>\s*$', body, re.I | re.M))),
            'taskIds': sorted(set(re.findall(r'\b[A-Z]+-\d+\b', subject + '\n' + body))),
            'files': sorted(set(path.lstrip('\n') for path in paths.split('\x00') if path.strip('\n'))),
        })
    return commits


def summarize(commits: list[dict], modules: list[dict], tasks: list[dict]) -> list[dict]:
    authors = {}
    for commit in commits:
        commit['modules'] = modules_for_paths(commit['files'], modules, tasks)
        author = authors.setdefault(commit['authorId'], {
            'id': commit['authorId'], 'name': commit['author'], 'aliases': [], 'commits': 0,
            'latest': commit['date'], 'modules': set(),
        })
        author['commits'] += 1
        if commit['author'] not in author['aliases']:
            author['aliases'].append(commit['author'])
        author['modules'].update(commit['modules'])
    for author in authors.values():
        author['modules'] = sorted(author['modules'])
    for module in modules:
        own_tasks = [task for task in tasks if task['module'] == module['id']]
        own_commits = [commit for commit in commits if module['id'] in commit['modules']]
        module['plan'] = {'closed': sum(task['closed'] for task in own_tasks), 'total': len(own_tasks)}
        module['commitCount'] = len(own_commits)
        module['authors'] = dict(Counter(commit['authorId'] for commit in own_commits))
        module['latest'] = own_commits[0]['date'] if own_commits else None
    return sorted(authors.values(), key=lambda author: (-author['commits'], author['name']))


def build_snapshot(root: Path = ROOT, ref: str = 'HEAD') -> dict:
    head = git('rev-parse', '--verify', f'{ref}^{{commit}}', root=root).strip()
    modules = json.loads((root / 'docs/architecture/modules.json').read_text())
    readme = git('show', f'{head}:docs/plans/README.md', root=root)
    # Only numbered plans linked by the index; framework and redundancy are separate.
    plan_files = sorted(set(re.findall(r'\]\((\d{2}-[\w-]+\.md)\)', readme)))
    tasks = []
    for name in plan_files:
        path = f'docs/plans/{name}'
        tasks.extend(parse_tasks(git('show', f'{head}:{path}', root=root), path))
    assign_tasks(tasks, modules)
    raw = git('log', head, '--no-merges', '--date-order', '--no-renames', '--name-only', '-z',
              '--format=%x1e%H%x00%aI%x00%an%x00%ae%x00%cn%x00%s%x00%b%x00', root=root)
    commits = parse_history(raw)
    # Author timestamps may not be monotonic in Git's topological traversal.
    commits.sort(key=lambda commit: datetime.fromisoformat(commit['date']), reverse=True)
    authors = summarize(commits, modules, tasks)
    return {
        'schema': 1, 'generatedAt': datetime.now(timezone.utc).isoformat(),
        'head': head, 'branch': git('rev-parse', '--abbrev-ref', ref, root=root).strip(),
        'headDate': git('show', '-s', '--format=%cI', head, root=root).strip(),
        'mergeCount': int(git('rev-list', '--count', '--merges', head, root=root)),
        'shallow': git('rev-parse', '--is-shallow-repository', root=root).strip() == 'true',
        'modules': modules, 'tasks': tasks, 'commits': commits, 'authors': authors,
        'planFiles': [f'docs/plans/{name}' for name in plan_files],
    }


def main(ref: str = 'HEAD') -> None:
    data = build_snapshot(ref=ref)
    output = ROOT / 'docs/architecture/progress.js'
    output.write_text('// Generated from committed Git history; run scripts/docs/build_architecture.py.\n'
                      + 'window.PROJECT_PROGRESS = '
                      + json.dumps(data, ensure_ascii=False, separators=(',', ':')) + ';\n')
    print(f'{output.relative_to(ROOT)}: {len(data["commits"])} commits, '
          f'{len(data["tasks"])} tasks, {len(data["authors"])} author identities; {data["head"][:8]}')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ref', default='HEAD', help='Git commit or branch to snapshot (default: HEAD)')
    main(parser.parse_args().ref)
