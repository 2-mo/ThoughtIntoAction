"""Regression checks for the documentation dashboard's evidence accounting."""
import importlib.util
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / 'scripts/docs/build_project_progress.py'
SPEC = importlib.util.spec_from_file_location('project_progress', PATH)
progress = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(progress)


def test_plan_checkbox_is_not_inferred_from_body():
    text = ('# Plan\n- [ ] **K-01** Already partly implemented `P1`\n'
            '  - 已做：仿真通过；完成：验收标准\n'
            '- [x] **K-02** Closed in plan\n  - 未真机验证\n')
    tasks = progress.parse_tasks(text, 'docs/plans/01-control-kernel.md')
    assert [task['closed'] for task in tasks] == [False, True]
    assert tasks[0]['line'] == 2
    assert '未真机验证' not in tasks[0]['body']
    assert '未真机验证' in tasks[1]['body']


def test_task_assignment_rejects_double_counting_and_missing_owners():
    tasks = [{'id': 'K-01'}]
    with pytest.raises(ValueError, match='requires one module'):
        progress.assign_tasks(tasks, [{'id':'a', 'tasks':['K-*']}, {'id':'b', 'tasks':['K-01']}])
    with pytest.raises(ValueError, match='requires one module'):
        progress.assign_tasks(tasks, [{'id':'a', 'tasks':['D-*']}])
    with pytest.raises(ValueError, match='Duplicate task'):
        progress.assign_tasks([{'id':'K-01'}, {'id':'K-01'}], [{'id':'a', 'tasks':['K-*']}])


def test_same_email_aliases_coauthors_and_cross_module_commits():
    def record(sha, name, email, paths, body=''):
        return '\x1e' + '\x00'.join([sha, '2026-10-23T06:36:00+08:00', name, email,
                                     name, 'Change K-01', body, '\x00\n' + '\x00'.join(paths) + '\x00'])
    raw = record('a'*40, 'Current name', 'Person@example.test', ['src/new/a.py','src/data/b.py'],
                 'Co-authored-by: Collaborator <co@example.test>\n')
    raw += record('b'*40, 'Old name', 'person@example.test', ['src/old/deleted.py'])
    commits = progress.parse_history(raw)
    modules = [
        {'id':'runtime','paths':['src/new/','src/old/'],'tasks':['K-*']},
        {'id':'data','paths':['src/data/'],'tasks':['D-*']},
    ]
    tasks = [{'id':'K-01','path':'plan.md','module':'runtime','closed':False}]
    authors = progress.summarize(commits, modules, tasks)
    assert len(authors) == 1
    assert authors[0]['commits'] == 2
    assert authors[0]['aliases'] == ['Current name','Old name']
    assert commits[0]['coauthors'] == ['Collaborator']
    assert commits[0]['files'] == ['src/data/b.py','src/new/a.py']
    assert commits[0]['modules'] == ['data','runtime']
    assert commits[0]['taskIds'] == ['K-01']
    assert [module['commitCount'] for module in modules] == [2,1]
    assert '@' not in str(commits) and '@' not in str(authors)


def test_plan_path_mapping_and_longest_prefix():
    modules = [{'id':'docs','paths':['docs/']}, {'id':'data','paths':['src/data/']},
               {'id':'export','paths':['src/data/export/']}]
    tasks = [{'module':'data','path':'docs/plans/data.md'}, {'module':'export','path':'docs/plans/data.md'}]
    assert progress.modules_for_paths(['docs/plans/data.md'], modules, tasks) == ['data','export']
    assert progress.modules_for_paths(['todo/data.md'], modules, tasks) == ['data','export']
    assert progress.modules_for_paths(['src/data/export/run.py'], modules, tasks) == ['export']
    assert progress.modules_for_paths(['unmapped.txt'], modules, tasks) == []


def test_file_mapping_does_not_match_same_name_prefix():
    modules = [{'id':'entry','paths':['run.sh']}, {'id':'checks','paths':['scripts/check*']}]
    assert progress.modules_for_paths(['run.sh.backup'], modules, []) == []
    assert progress.modules_for_paths(['run.sh'], modules, []) == ['entry']
    assert progress.modules_for_paths(['scripts/check_data.py'], modules, []) == ['checks']
